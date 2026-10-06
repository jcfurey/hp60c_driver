// Copyright 2026 jcfurey
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hp60c_driver/hp60c_node.hpp"

#include <time.h>
#include <turbojpeg.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#ifdef HP60C_WITH_CUDA
#include "hp60c_driver/cuda_registrar.hpp"
#endif
#include "hp60c_driver/v4l2_capture.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hp60c_driver
{

namespace
{

constexpr std::uint16_t kVid = 0x3482;
constexpr std::uint16_t kPid = 0x6723;
constexpr int kStreamHeight = 642;   // the composite mode's nominal height
constexpr int kSampleEvery = 20;     // frames between exposure readouts (~1 s)

sensor_msgs::msg::CameraInfo make_info(const Intrinsics & k, const std::string & frame_id)
{
  sensor_msgs::msg::CameraInfo ci;
  ci.header.frame_id = frame_id;
  ci.width = kWidth;
  ci.height = kHeight;
  ci.distortion_model = "plumb_bob";
  ci.d = {0.0, 0.0, 0.0, 0.0, 0.0};   // the calibration block's distortion slots are zero
  ci.k = {k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0};
  ci.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  ci.p = {k.fx, 0.0, k.cx, 0.0, 0.0, k.fy, k.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return ci;
}

rcl_interfaces::msg::ParameterDescriptor param_desc(
  const std::string & text, bool read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = text;
  d.read_only = read_only;
  return d;
}

rcl_interfaces::msg::ParameterDescriptor read_only(const std::string & text)
{
  return param_desc(text + " Read-only: set it at startup.", true);
}

rcl_interfaces::msg::ParameterDescriptor int_range(
  const std::string & text, std::int64_t from, std::int64_t to)
{
  auto d = param_desc(text);
  rcl_interfaces::msg::IntegerRange r;
  r.from_value = from;
  r.to_value = to;
  r.step = 1;
  d.integer_range.push_back(r);
  return d;
}

rcl_interfaces::msg::ParameterDescriptor float_range(
  const std::string & text, double from, double to)
{
  auto d = param_desc(text);
  rcl_interfaces::msg::FloatingPointRange r;
  r.from_value = from;
  r.to_value = to;
  r.step = 0.0;
  d.floating_point_range.push_back(r);
  return d;
}

// A device value the parameter's descriptor accepts: inside the range, on a
// step, and a valid menu entry.
std::int64_t sanitize(const ControlInfo & c, std::int64_t v)
{
  if (c.is_menu()) {
    if (c.has_menu_entry(v) || c.menu.empty()) {
      return v;
    }
    return c.has_menu_entry(c.default_value) ? c.default_value : c.menu.front().first;
  }
  if (c.is_bool() || c.minimum > c.maximum) {
    return v;
  }
  v = std::clamp(v, c.minimum, c.maximum);
  const auto step = static_cast<std::int64_t>(std::max<std::uint64_t>(c.step, 1));
  return v == c.maximum ? v : c.minimum + (v - c.minimum) / step * step;
}

std::uint16_t * pixels(const std::unique_ptr<sensor_msgs::msg::Image> & m)
{
  return m ? reinterpret_cast<std::uint16_t *>(m->data.data()) : nullptr;
}

const ControlSpec * find_spec(std::uint32_t id)
{
  for (const auto & s : control_specs()) {
    if (s.id == id) {
      return &s;
    }
  }
  return nullptr;
}

}  // namespace

Hp60cNode::Hp60cNode(const rclcpp::NodeOptions & options)
: Node("hp60c", options)
{
  // Registered first so that values given at startup are validated too.
  on_set_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) {return validate_parameters(p);});
  post_set_handle_ = add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &) {settings_dirty_ = true;});
  declare_parameters();

  // REP 2003: drivers publish with the system-default QoS, i.e. reliable and
  // volatile. A reliable publisher serves both reliable and best-effort
  // (SensorDataQoS) subscribers, while a best-effort one is invisible to
  // reliable subscribers such as RViz's defaults. The keep-last depth is
  // explicit because intra-process comms rejects the system default (0); 2
  // keeps a slow subscriber from building a backlog. qos_overrides.<topic>.*
  // parameters adjust depth, history and reliability per topic.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
  if (get_parameter("best_effort").as_bool()) {
    qos.best_effort();
  }
  rclcpp::PublisherOptions po;
  po.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  color_pub_ = create_publisher<Image>("color/image_raw", qos, po);
  color_jpeg_pub_ = create_publisher<CompressedImage>("color/image_raw/compressed", qos, po);
  color_info_pub_ = create_publisher<CameraInfo>("color/camera_info", qos, po);
  depth_pub_ = create_publisher<Image>("depth/image_raw", qos, po);
  depth_info_pub_ = create_publisher<CameraInfo>("depth/camera_info", qos, po);
  aligned_pub_ = create_publisher<Image>("aligned_depth_to_color/image_raw", qos, po);
  aligned_info_pub_ = create_publisher<CameraInfo>("aligned_depth_to_color/camera_info", qos, po);
  // Filtered depth sits beside the raw topic, sharing its camera_info.
  depth_filtered_pub_ = create_publisher<Image>("depth/image_filtered", qos, po);
  aligned_filtered_pub_ = create_publisher<Image>("aligned_depth_to_color/image_filtered", qos, po);
  tf_static_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(1).reliable().transient_local());

  tj_ = tjInitDecompress();
  if (!tj_) {
    throw std::runtime_error("tjInitDecompress failed");
  }
  depth_mm_.resize(static_cast<std::size_t>(kWidth) * kHeight);
  filtered_mm_.resize(depth_mm_.size());
  luma_.resize(static_cast<std::size_t>(kWidth / 8) * (kHeight / 8));
  apply_filter_settings();

  const bool use_cuda = get_parameter("use_cuda").as_bool();
#ifdef HP60C_WITH_CUDA
  if (use_cuda) {
    try {
      cuda_ = std::make_shared<CudaRegistrar>();
      RCLCPP_INFO(get_logger(), "Depth unpacking and registration on CUDA");
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "CUDA unavailable (%s); using the CPU path", e.what());
    }
  }
#else
  if (use_cuda) {
    RCLCPP_INFO(get_logger(), "Built without CUDA; depth registration runs on the CPU");
  }
#endif
  stats_.backend = cuda_ ? "CUDA" : "CPU";
  stats_.exposure_mode = get_parameter("color.exposure_mode").as_string();

  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID("HP60C (USB 3482:6723)");
  diagnostics_->add("Camera", this, &Hp60cNode::diagnose);
  rate_ = std::make_unique<diagnostic_updater::FrequencyStatus>(
    diagnostic_updater::FrequencyStatusParam(&min_rate_, &max_rate_, 0.1, 10),
    "Frame rate", get_clock());
  diagnostics_->add(*rate_);

  if (publish_tf_ && !base_frame_.empty()) {
    publish_static_tf(nullptr, now());   // the body frame needs no calibration
  }
  worker_ = std::thread([this] {run();});
}

Hp60cNode::~Hp60cNode()
{
  stop_ = true;
  if (worker_.joinable()) {
    worker_.join();
  }
  if (tj_) {
    tjDestroy(tj_);
  }
}

// ---------------------------------------------------------------- parameters

void Hp60cNode::declare_parameters()
{
  device_param_ = declare_parameter<std::string>(
    "device", "", read_only(
      "V4L2 device, e.g. /dev/video0. Empty: find the camera by its USB id 3482:6723."));
  color_frame_ = declare_parameter<std::string>(
    "color_frame_id", "hp60c_color_optical_frame",
    read_only("Optical frame of the colour camera, and of the aligned depth."));
  depth_frame_ = declare_parameter<std::string>(
    "depth_frame_id", "hp60c_depth_optical_frame",
    read_only("Optical frame of the depth camera."));
  base_frame_ = declare_parameter<std::string>(
    "base_frame_id", "", read_only(
      "If set, also publish a static TF from this body frame (REP 103: x forward, z up), "
      "placed at the colour camera, to color_frame_id. Leave empty when a URDF already "
      "places the optical frame."));
  publish_tf_ = declare_parameter<bool>(
    "publish_tf", true, read_only("Publish the static TF between the camera frames."));
  declare_parameter<bool>(
    "best_effort", false, read_only(
      "Publish images best-effort instead of reliable. Reliable serves both kinds of "
      "subscriber (REP 2003); qos_overrides.<topic>.publisher.* tune each topic."));
  declare_parameter<bool>(
    "use_cuda", true, read_only(
      "Unpack and register depth on the GPU, if built with CUDA and one is usable."));

  DepthFilterParams fp;
  declare_parameter<bool>(
    "filter.enabled", true, param_desc("Publish the */image_filtered topics."));
  declare_parameter<bool>(
    "filter.temporal", fp.temporal,
    param_desc("Temporal stage: a per-pixel moving average that resets on real change."));
  declare_parameter<double>(
    "filter.temporal_alpha", fp.alpha,
    float_range("Weight of the newest frame in the temporal average.", 0.0, 1.0));
  declare_parameter<double>(
    "filter.temporal_reset_fraction", fp.reset_fraction,
    float_range("Restart a pixel's average when it changes by more than this fraction.", 0.0, 1.0));
  declare_parameter<bool>(
    "filter.spatial", fp.spatial, param_desc("Spatial stage: 3x3 median over valid neighbours."));

  auto mode = param_desc(
    "auto: the camera's own auto exposure. manual: color.exposure_us and color.gain as set. "
    "software: the driver's auto exposure and auto gain (color.software_ae.*), which "
    "lengthen exposure first and raise gain only at the exposure limit.");
  mode.additional_constraints = "auto, manual or software";
  declare_parameter<std::string>("color.exposure_mode", "auto", mode);
  AutoExposureParams ae;
  declare_parameter<double>(
    "color.software_ae.target_brightness", ae.target,
    float_range("Mean image brightness (0-255) the software auto exposure aims for.", 10, 245));
  declare_parameter<double>(
    "color.software_ae.tolerance", ae.tolerance, float_range(
      "No adjustment while the brightness is within this fraction of the target.", 0.02, 0.5));
  declare_parameter<std::int64_t>(
    "color.software_ae.min_exposure_us", 0, int_range(
      "Shortest exposure the software auto exposure uses, us. 0: the camera's minimum.",
      0, 10000000));
  declare_parameter<std::int64_t>(
    "color.software_ae.max_exposure_us", 30000, int_range(
      "Longest exposure the software auto exposure uses, us: bounds motion blur. Above the "
      "frame period (~40 ms) the frame rate drops. 0: the camera's maximum.", 0, 10000000));
  declare_parameter<std::int64_t>(
    "color.software_ae.min_gain", -1, int_range(
      "Lowest gain the software auto exposure uses. -1: the camera's minimum.", -1, 65535));
  declare_parameter<std::int64_t>(
    "color.software_ae.max_gain", -1, int_range(
      "Highest gain the software auto exposure uses; equal to min_gain turns auto gain off. "
      "-1: the camera's maximum.", -1, 65535));
}

// The camera's own controls become parameters once it is first seen, with its
// ranges and its current values as defaults (values given at startup win).
void Hp60cNode::declare_control_parameters(V4l2Capture & cap)
{
  // Log everything the camera offers, exposed or not.
  for (const auto & [id, c] : controls_) {
    std::optional<std::int64_t> cur;
    try {
      cur = cap.get_control(id);
    } catch (const std::system_error &) {
    }
    const ControlSpec * spec = find_spec(id);
    const std::string where = spec ? std::string(" -> color.") + spec->param :
      id == exposure_auto_id() ? " -> color.exposure_mode" : " (not exposed)";
    RCLCPP_INFO(get_logger(), "Camera control %s%s", describe(c, cur).c_str(), where.c_str());
  }
  {
    std::lock_guard<std::mutex> lock(control_params_mutex_);
    for (const auto & spec : control_specs()) {
      const auto it = controls_.find(spec.id);
      if (it != controls_.end()) {
        control_params_[std::string("color.") + spec.param] = {it->second, spec};
      }
    }
  }
  // Only this thread writes control_params_, so it can be read unlocked here.
  std::vector<std::string> failed;
  for (const auto & [name, cp] : control_params_) {
    const ControlInfo & c = cp.info;
    const std::int64_t scale = cp.spec.scale;
    std::int64_t cur = c.default_value;
    try {
      cur = cap.get_control(c.id);
    } catch (const std::system_error &) {
    }
    cur = sanitize(c, cur);
    auto d = param_desc(cp.spec.description);
    d.additional_constraints = "Camera: " + describe(c, std::nullopt);
    d.read_only = c.read_only();
    if (!c.is_bool() && c.minimum <= c.maximum) {
      rcl_interfaces::msg::IntegerRange r;
      r.from_value = c.minimum * scale;
      r.to_value = c.maximum * scale;
      const std::uint64_t step = c.is_menu() ? 1 : std::max<std::uint64_t>(c.step, 1);
      r.step = step * static_cast<std::uint64_t>(scale);
      d.integer_range.push_back(r);
    }
    for (const bool ignore_override : {false, true}) {
      try {
        if (c.is_bool()) {
          declare_parameter<bool>(name, cur != 0, d, ignore_override);
        } else {
          declare_parameter<std::int64_t>(name, cur * scale, d, ignore_override);
        }
        break;
      } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
        RCLCPP_WARN(get_logger(), "Ignoring the given %s: %s", name.c_str(), e.what());
      } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        RCLCPP_WARN(get_logger(), "Ignoring the given %s: %s", name.c_str(), e.what());
      }
      if (ignore_override) {
        failed.push_back(name);
      }
    }
  }
  if (!failed.empty()) {
    std::lock_guard<std::mutex> lock(control_params_mutex_);
    for (const auto & name : failed) {
      control_params_.erase(name);
    }
  }
  if (!controls_.count(exposure_auto_id())) {
    RCLCPP_WARN(
      get_logger(), "The camera reports no exposure mode control: color.exposure_mode "
      "auto leaves exposure to the camera, manual/software need color.exposure_us.");
  }
  controls_declared_ = true;
}

rcl_interfaces::msg::SetParametersResult Hp60cNode::validate_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult r;
  r.successful = true;
  for (const auto & p : params) {
    const std::string & name = p.get_name();
    if (name == "color.exposure_mode") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
        !parse_exposure_mode(p.as_string()))
      {
        r.successful = false;
        r.reason = "color.exposure_mode must be auto, manual or software";
      }
      continue;
    }
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
      continue;
    }
    std::lock_guard<std::mutex> lock(control_params_mutex_);
    const auto it = control_params_.find(name);
    if (it == control_params_.end() || !it->second.info.is_menu()) {
      continue;
    }
    const auto & c = it->second.info;
    if (!c.has_menu_entry(p.as_int() / it->second.spec.scale)) {
      r.successful = false;
      r.reason = name + " must be one of the camera's entries: " + describe(c, std::nullopt);
    }
  }
  return r;
}

// -------------------------------------------------------------- worker thread

bool Hp60cNode::running()
{
  return !stop_ && rclcpp::ok(get_node_base_interface()->get_context());
}

// Open the camera, stream, and on any failure retry every 2 s until shutdown.
void Hp60cNode::run()
{
  while (running()) {
    const std::string dev =
      device_param_.empty() ? find_video_device(kVid, kPid) : device_param_;
    if (dev.empty()) {
      update_stats(
        [](Stats & s) {
          s.streaming = false;
          s.device.clear();
          s.error = "No HP60C video device (USB 3482:6723)";
        });
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "No HP60C (3482:6723) video node. If the camera is plugged in, another driver "
        "(e.g. the vendor ascamera SDK) has detached uvcvideo; re-plug it or re-authorize "
        "its USB device to get /dev/video* back.");
      sleep_while_running(std::chrono::seconds(2));
      continue;
    }
    try {
      V4l2Capture cap(dev, kWidth, kStreamHeight);
      on_connect(cap, dev);
      int timeouts = 0;
      while (running()) {
        if (settings_dirty_.exchange(false)) {
          apply_settings(cap);
        }
        const bool got = cap.grab(
          500, [this, &cap](const CapturedFrame & frame) {on_frame(cap, frame);});
        timeouts = got ? 0 : timeouts + 1;
        if (timeouts >= 6) {
          throw std::runtime_error("no frames for 3 s");
        }
      }
    } catch (const std::exception & e) {
      update_stats(
        [&e](Stats & s) {
          s.streaming = false;
          s.error = e.what();
        });
      RCLCPP_ERROR(get_logger(), "Camera error on %s: %s; retrying", dev.c_str(), e.what());
      sleep_while_running(std::chrono::seconds(2));
    }
  }
  update_stats([](Stats & s) {s.streaming = false;});
}

template<class Duration>
void Hp60cNode::sleep_while_running(Duration d)
{
  const auto until = std::chrono::steady_clock::now() + d;
  while (running() && std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void Hp60cNode::on_connect(V4l2Capture & cap, const std::string & dev)
{
  RCLCPP_INFO(get_logger(), "Streaming from %s", dev.c_str());
  controls_.clear();
  for (auto & c : cap.query_controls()) {
    const auto id = c.id;
    controls_.emplace(id, std::move(c));
  }
  if (!controls_declared_) {
    declare_control_parameters(cap);
  }
  // A re-plugged camera starts from its power-on settings: write them all.
  written_.clear();
  refused_.clear();
  ae_active_ = false;
  last_sequence_.reset();
  frames_since_sample_ = 0;
  settings_dirty_ = false;
  apply_settings(cap);
  update_stats(
    [&dev](Stats & s) {
      s.device = dev;
      s.streaming = true;
      s.error.clear();
      ++s.connects;
    });
}

// Bring the camera, filter and software auto exposure in line with the
// parameters. Switches go first: the camera refuses a value (EACCES) that an
// automatic mode owns.
void Hp60cNode::apply_settings(V4l2Capture & cap)
{
  apply_filter_settings();

  const auto mode = parse_exposure_mode(get_parameter("color.exposure_mode").as_string())
    .value_or(ExposureMode::kAuto);
  // Report on a change of mode or a fresh connection, not on every parameter set.
  const bool announce = mode != exposure_mode_ || written_.empty();
  if (announce) {
    RCLCPP_INFO(get_logger(), "Exposure mode: %s", to_string(mode));
  }
  exposure_mode_ = mode;
  update_stats([mode](Stats & s) {s.exposure_mode = to_string(mode);});
  if (const auto it = controls_.find(exposure_auto_id()); it != controls_.end()) {
    if (const auto v = exposure_auto_value(mode, it->second)) {
      write_control(cap, it->first, *v);
    } else if (announce) {
      RCLCPP_WARN(
        get_logger(), "The camera's exposure modes offer nothing for color.exposure_mode %s",
        to_string(mode));
    }
  }

  const bool auto_wb = has_parameter("color.auto_white_balance") &&
    get_parameter("color.auto_white_balance").as_bool();
  for (const auto & spec : control_specs()) {
    const std::string name = std::string("color.") + spec.param;
    const auto it = controls_.find(spec.id);
    if (it == controls_.end() || it->second.read_only() || !has_parameter(name)) {
      continue;
    }
    bool apply = true;
    switch (spec.gate) {
      case Gate::kAlways: break;
      case Gate::kManualExposure: apply = mode == ExposureMode::kManual; break;
      case Gate::kNotSoftwareExposure: apply = mode != ExposureMode::kSoftware; break;
      case Gate::kManualWhiteBalance: apply = !auto_wb; break;
    }
    if (apply) {
      const auto p = get_parameter(name);
      write_control(
        cap, spec.id, p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL ?
        static_cast<std::int64_t>(p.as_bool()) : p.as_int() / spec.scale);
    }
  }
  apply_software_ae_settings(cap, announce);
}

void Hp60cNode::apply_filter_settings()
{
  filter_enabled_ = get_parameter("filter.enabled").as_bool();
  DepthFilterParams fp;
  fp.temporal = get_parameter("filter.temporal").as_bool();
  fp.alpha = get_parameter("filter.temporal_alpha").as_double();
  fp.reset_fraction = get_parameter("filter.temporal_reset_fraction").as_double();
  fp.spatial = get_parameter("filter.spatial").as_bool();
  if (!filter_ || fp.temporal != filter_params_.temporal || fp.alpha != filter_params_.alpha ||
    fp.reset_fraction != filter_params_.reset_fraction || fp.spatial != filter_params_.spatial)
  {
    filter_params_ = fp;
    filter_ = std::make_unique<DepthFilter>(kWidth, kHeight, fp);
    last_filtered_us_ = 0;
  }
}

void Hp60cNode::apply_software_ae_settings(V4l2Capture & cap, bool announce)
{
  if (exposure_mode_ != ExposureMode::kSoftware) {
    ae_active_ = false;
    update_stats([](Stats & s) {s.brightness.reset();});
    return;
  }
  const auto e = controls_.find(exposure_absolute_id());
  if (e == controls_.end() || e->second.read_only()) {
    if (announce) {
      RCLCPP_WARN(
        get_logger(), "color.exposure_mode software needs a settable exposure time, and "
        "this camera offers none; exposure is left to the camera");
    }
    ae_active_ = false;
    return;
  }
  const ControlInfo & ec = e->second;
  AutoExposureParams p;
  p.target = get_parameter("color.software_ae.target_brightness").as_double();
  p.tolerance = get_parameter("color.software_ae.tolerance").as_double();
  const auto min_us = get_parameter("color.software_ae.min_exposure_us").as_int();
  const auto max_us = get_parameter("color.software_ae.max_exposure_us").as_int();
  p.min_exposure = min_us > 0 ? std::clamp((min_us + 99) / 100, ec.minimum, ec.maximum) :
    ec.minimum;
  p.max_exposure = max_us > 0 ? std::clamp(max_us / 100, ec.minimum, ec.maximum) : ec.maximum;
  const auto g = controls_.find(gain_id());
  ae_has_gain_ = g != controls_.end() && !g->second.read_only();
  if (ae_has_gain_) {
    const ControlInfo & gc = g->second;
    const auto min_gain = get_parameter("color.software_ae.min_gain").as_int();
    const auto max_gain = get_parameter("color.software_ae.max_gain").as_int();
    p.min_gain = min_gain < 0 ? gc.minimum : std::clamp(min_gain, gc.minimum, gc.maximum);
    p.max_gain = max_gain < 0 ? gc.maximum : std::clamp(max_gain, gc.minimum, gc.maximum);
    p.gain_unity = gc.minimum;
    // Assume the camera's full gain range is ~8x; the loop corrects the error.
    p.gain_per_x = std::max(1.0, static_cast<double>(gc.maximum - gc.minimum) / 7.0);
  }
  ae_.set_params(p);
  if (!ae_active_) {
    // Start from where the camera is (after its own auto exposure, if it ran).
    std::int64_t exposure = ec.default_value;
    std::int64_t gain = ae_has_gain_ ? g->second.default_value : 0;
    try {
      exposure = cap.get_control(ec.id);
      if (ae_has_gain_) {
        gain = cap.get_control(gain_id());
      }
    } catch (const std::system_error &) {
    }
    ae_.reset(exposure, gain);
    ae_active_ = true;
    const auto & q = ae_.params();
    RCLCPP_INFO(
      get_logger(), "Software auto exposure: target %.0f, exposure %s..%s us, gain %s..%s",
      q.target, std::to_string(q.min_exposure * 100).c_str(),
      std::to_string(q.max_exposure * 100).c_str(),
      std::to_string(q.min_gain).c_str(), std::to_string(q.max_gain).c_str());
  }
  write_control(cap, exposure_absolute_id(), ae_.exposure());
  if (ae_has_gain_) {
    write_control(cap, gain_id(), ae_.gain());
  }
}

// Set a control unless it already holds `value`. A refusal is logged once per
// value and doesn't stop the stream.
bool Hp60cNode::write_control(V4l2Capture & cap, std::uint32_t id, std::int64_t value)
{
  const auto w = written_.find(id);
  if (w != written_.end() && w->second == value) {
    return true;
  }
  try {
    cap.set_control(id, value);
  } catch (const std::system_error & e) {
    written_.erase(id);
    if (refused_.emplace(id, value).second) {
      const auto it = controls_.find(id);
      RCLCPP_WARN(
        get_logger(), "The camera refused %s = %s: %s",
        it == controls_.end() ? "a control" : it->second.name.c_str(),
        std::to_string(value).c_str(), e.what());
    }
    return false;
  }
  written_[id] = value;
  for (const auto dependent : controls_gated_by(id)) {
    written_.erase(dependent);
  }
  return true;
}

// The image's acquisition time (sensor_msgs/Image: "time of image
// acquisition"): uvcvideo's timestamp for the frame, moved onto the node's
// clock. That takes the USB transfer time of each ~630 KB frame out of the
// stamp. Falls back to the arrival time if the driver gave none.
rclcpp::Time Hp60cNode::capture_stamp(const CapturedFrame & frame)
{
  const rclcpp::Time arrival = now();
  if (frame.monotonic_ns < 0) {
    return arrival;
  }
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const std::int64_t age =
    static_cast<std::int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec - frame.monotonic_ns;
  if (age < 0 || age > 1000000000 || age > arrival.nanoseconds()) {
    return arrival;   // implausible, or a clock (sim time) that just started
  }
  return arrival - rclcpp::Duration::from_nanoseconds(age);
}

void Hp60cNode::on_frame(V4l2Capture & cap, const CapturedFrame & frame)
{
  std::uint64_t lost = 0;
  if (last_sequence_) {
    const std::uint32_t gap = frame.sequence - *last_sequence_;   // wraps correctly
    if (gap > 1 && gap < 1000) {
      lost = gap - 1;
    }
  }
  last_sequence_ = frame.sequence;

  FrameView f;
  try {
    f = parse_frame(frame.data, frame.size);
  } catch (const FormatError & e) {
    update_stats(
      [lost](Stats & s) {
        ++s.bad_frames;
        s.host_dropped += lost;
      });
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping frame: %s", e.what());
    return;
  }
  const rclcpp::Time stamp = capture_stamp(frame);
  rate_->tick();
  update_stats(
    [lost](Stats & s) {
      ++s.frames;
      s.host_dropped += lost;
    });
  if (publish_tf_ && !tf_sent_) {
    publish_static_tf(&f.calib, stamp);
    tf_sent_ = true;
  }

  if (color_jpeg_pub_->get_subscription_count() > 0) {
    auto msg = std::make_unique<CompressedImage>();
    msg->header.stamp = stamp;
    msg->header.frame_id = color_frame_;
    msg->format = "jpeg";
    msg->data.assign(f.jpeg, f.jpeg + f.jpeg_len);
    color_jpeg_pub_->publish(std::move(msg));
  }
  if (color_pub_->get_subscription_count() > 0) {
    auto msg = std::make_unique<Image>();
    msg->header.stamp = stamp;
    msg->header.frame_id = color_frame_;
    msg->width = kWidth;
    msg->height = kHeight;
    msg->encoding = "rgb8";
    msg->step = kWidth * 3;
    msg->data.resize(static_cast<std::size_t>(msg->step) * kHeight);
    if (tjDecompress2(
        tj_, f.jpeg, static_cast<unsigned long>(f.jpeg_len),   // NOLINT(runtime/int)
        msg->data.data(), kWidth, 0, kHeight, TJPF_RGB, TJFLAG_FASTDCT) != 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "JPEG decode failed: %s", tjGetErrorStr2(tj_));
    } else {
      color_pub_->publish(std::move(msg));
    }
  }
  publish_info(color_info_pub_, f.calib.colour, color_frame_, stamp);

  DepthOut o;
  o.depth = depth_pub_->get_subscription_count() > 0 ? mono16(depth_frame_, stamp) : nullptr;
  o.aligned = aligned_pub_->get_subscription_count() > 0 ? mono16(color_frame_, stamp) : nullptr;
  if (filter_enabled_) {
    o.depth_f = depth_filtered_pub_->get_subscription_count() > 0 ?
      mono16(depth_frame_, stamp) : nullptr;
    o.aligned_f = aligned_filtered_pub_->get_subscription_count() > 0 ?
      mono16(color_frame_, stamp) : nullptr;
  }
  if (o.depth || o.aligned || o.depth_f || o.aligned_f) {
    compute_depth(f, o);
  }
  if (o.depth) {
    depth_pub_->publish(std::move(o.depth));
  }
  if (o.depth_f) {
    depth_filtered_pub_->publish(std::move(o.depth_f));
  }
  publish_info(depth_info_pub_, f.calib.depth, depth_frame_, stamp);
  if (o.aligned) {
    aligned_pub_->publish(std::move(o.aligned));
  }
  if (o.aligned_f) {
    aligned_filtered_pub_->publish(std::move(o.aligned_f));
  }
  publish_info(aligned_info_pub_, f.calib.colour, color_frame_, stamp);

  // Exposure work last, so it never delays the images.
  if (ae_active_) {
    run_software_ae(cap, f);
  } else if (++frames_since_sample_ >= kSampleEvery) {
    frames_since_sample_ = 0;
    sample_exposure(cap);
  }
}

// Mean luma of a 1/8-scale greyscale decode: libjpeg-turbo then uses only each
// 8x8 block's DC term and skips colour conversion. Huffman decoding remains, so
// it costs about 60% of a full decode (0.7 vs 1.2 ms for a 60 KB frame on x86).
bool Hp60cNode::mean_luma(const FrameView & f, double & out)
{
  constexpr int w = kWidth / 8;
  constexpr int h = kHeight / 8;
  if (tjDecompress2(
      tj_, f.jpeg, static_cast<unsigned long>(f.jpeg_len),   // NOLINT(runtime/int)
      luma_.data(), w, 0, h, TJPF_GRAY, TJFLAG_FASTDCT) != 0)
  {
    return false;
  }
  std::uint64_t sum = 0;
  for (const std::uint8_t v : luma_) {
    sum += v;
  }
  out = static_cast<double>(sum) / static_cast<double>(luma_.size());
  return true;
}

void Hp60cNode::run_software_ae(V4l2Capture & cap, const FrameView & f)
{
  double luma = 0.0;
  if (!mean_luma(f, luma)) {
    return;
  }
  if (ae_.update(luma)) {
    write_control(cap, exposure_absolute_id(), ae_.exposure());
    if (ae_has_gain_) {
      write_control(cap, gain_id(), ae_.gain());
    }
  }
  update_stats(
    [this, luma](Stats & s) {
      s.brightness = luma;
      s.exposure_us = ae_.exposure() * 100;
      s.gain = ae_has_gain_ ? std::optional<std::int64_t>(ae_.gain()) : std::nullopt;
    });
}

// Read back exposure and gain for /diagnostics, so the camera's own auto
// exposure can be watched too.
void Hp60cNode::sample_exposure(V4l2Capture & cap)
{
  std::optional<std::int64_t> exposure_us, gain;
  try {
    if (controls_.count(exposure_absolute_id())) {
      exposure_us = cap.get_control(exposure_absolute_id()) * 100;
    }
    if (controls_.count(gain_id())) {
      gain = cap.get_control(gain_id());
    }
  } catch (const std::system_error &) {
  }
  update_stats(
    [exposure_us, gain](Stats & s) {
      s.exposure_us = exposure_us;
      s.gain = gain;
    });
}

// Fill whichever outputs are wanted. Unpacking and registration run on CUDA
// when available, else on the CPU; a CUDA failure mid-run drops back to the
// CPU for good rather than losing frames. Filtering runs on the CPU.
void Hp60cNode::compute_depth(const FrameView & f, DepthOut & o)
{
  const bool want_filtered = o.depth_f || o.aligned_f;
  // Host-side unpacked depth, needed for filtering and for the CPU path.
  std::uint16_t * mm = o.depth ? pixels(o.depth) : depth_mm_.data();
  bool have_mm = false;
#ifdef HP60C_WITH_CUDA
  if (cuda_) {
    try {
      cuda_->process(
        f.depth_raw, f.calib, (o.depth || want_filtered) ? mm : nullptr, pixels(o.aligned));
      have_mm = o.depth || want_filtered;
      if (o.aligned_f) {
        run_filter(f, mm, o);
        cuda_->register_mm(pixels(o.depth_f) ? pixels(o.depth_f) : filtered_mm_.data(),
          f.calib, pixels(o.aligned_f));
      } else if (o.depth_f) {
        run_filter(f, mm, o);
      }
      return;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "CUDA failed (%s); switching to the CPU path", e.what());
      cuda_.reset();
      update_stats([](Stats & s) {s.backend = "CPU (CUDA failed)";});
    }
  }
#endif
  if (!have_mm) {
    depth_to_mm(f.depth_raw, mm);
  }
  if (o.aligned) {
    register_to_colour(mm, f.calib, pixels(o.aligned));
  }
  if (want_filtered) {
    run_filter(f, mm, o);
    if (o.aligned_f) {
      register_to_colour(
        pixels(o.depth_f) ? pixels(o.depth_f) : filtered_mm_.data(), f.calib,
        pixels(o.aligned_f));
    }
  }
}

// Filter mm into depth_f (or scratch). The temporal stage needs consecutive
// frames, so its history is dropped when the device timestamps show a gap
// longer than the camera's own cadence (one skipped cycle is ~81 ms).
void Hp60cNode::run_filter(const FrameView & f, const std::uint16_t * mm, DepthOut & o)
{
  if (last_filtered_us_ == 0 || f.device_stamp_us - last_filtered_us_ > 150000) {
    filter_->reset();
  }
  last_filtered_us_ = f.device_stamp_us;
  filter_->apply(mm, o.depth_f ? pixels(o.depth_f) : filtered_mm_.data());
}

std::unique_ptr<sensor_msgs::msg::Image> Hp60cNode::mono16(
  const std::string & frame, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<Image>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame;
  msg->width = kWidth;
  msg->height = kHeight;
  msg->encoding = "16UC1";
  msg->is_bigendian = 0;
  msg->step = kWidth * 2;
  msg->data.resize(static_cast<std::size_t>(msg->step) * kHeight);
  return msg;
}

void Hp60cNode::publish_info(
  const rclcpp::Publisher<CameraInfo>::SharedPtr & pub, const Intrinsics & k,
  const std::string & frame, const rclcpp::Time & stamp)
{
  if (pub->get_subscription_count() == 0) {
    return;
  }
  auto msg = std::make_unique<CameraInfo>(make_info(k, frame));
  msg->header.stamp = stamp;
  pub->publish(std::move(msg));
}

// Static TF, all in one message (/tf_static keeps only a publisher's last):
//  - base_frame_id -> colour optical frame, if base_frame_id is set: the REP
//    103 optical rotation (z forward, x right, y down) at the colour camera;
//  - colour optical -> depth optical, once the calibration is known, so a
//    point p in the depth frame maps to R p + t in the colour frame.
void Hp60cNode::publish_static_tf(const Calibration * c, const rclcpp::Time & stamp)
{
  tf2_msgs::msg::TFMessage msg;
  if (!base_frame_.empty()) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = base_frame_;
    tf.child_frame_id = color_frame_;
    // Columns: the optical x, y, z axes in the body frame.
    const auto q = rotation_to_quaternion({0, 0, 1, -1, 0, 0, 0, -1, 0});
    tf.transform.rotation.x = q[0];
    tf.transform.rotation.y = q[1];
    tf.transform.rotation.z = q[2];
    tf.transform.rotation.w = q[3];
    msg.transforms.push_back(tf);
  }
  if (c) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = color_frame_;
    tf.child_frame_id = depth_frame_;
    tf.transform.translation.x = c->t_mm[0] / 1000.0;
    tf.transform.translation.y = c->t_mm[1] / 1000.0;
    tf.transform.translation.z = c->t_mm[2] / 1000.0;
    const auto q = rotation_to_quaternion(c->R);
    tf.transform.rotation.x = q[0];
    tf.transform.rotation.y = q[1];
    tf.transform.rotation.z = q[2];
    tf.transform.rotation.w = q[3];
    msg.transforms.push_back(tf);
    RCLCPP_INFO(
      get_logger(), "Calibration: depth fx %.1f, colour fx %.1f, baseline %.2f mm",
      c->depth.fx, c->colour.fx,
      std::sqrt(c->t_mm[0] * c->t_mm[0] + c->t_mm[1] * c->t_mm[1] + c->t_mm[2] * c->t_mm[2]));
  }
  if (!msg.transforms.empty()) {
    tf_static_pub_->publish(msg);
  }
}

// ---------------------------------------------------------------- diagnostics

void Hp60cNode::diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  std::lock_guard<std::mutex> lock(stats_mutex_);
  const Stats & s = stats_;
  if (s.streaming) {
    stat.summary(DiagnosticStatus::OK, "Streaming");
  } else {
    stat.summary(DiagnosticStatus::ERROR, s.error.empty() ? "Not connected" : s.error);
  }
  stat.add("Device", s.device.empty() ? "(none)" : s.device);
  stat.add("Frames", s.frames);
  stat.add("Frames lost on the host", s.host_dropped);
  stat.add("Malformed frames", s.bad_frames);
  stat.add("Connections", s.connects);
  stat.add("Depth backend", s.backend);
  stat.add("Exposure mode", s.exposure_mode);
  if (s.exposure_us) {
    stat.add("Exposure (us)", *s.exposure_us);
  }
  if (s.gain) {
    stat.add("Gain", *s.gain);
  }
  if (s.brightness) {
    stat.addf("Mean brightness", "%.1f", *s.brightness);
  }
}

}  // namespace hp60c_driver

RCLCPP_COMPONENTS_REGISTER_NODE(hp60c_driver::Hp60cNode)
