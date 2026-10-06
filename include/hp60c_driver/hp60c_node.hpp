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

// ROS 2 component for the HP60C. Reads the composite UVC stream (see
// docs/FRAME_FORMAT.md) and publishes, each only while someone subscribes:
//
//   color/image_raw                   sensor_msgs/Image rgb8
//   color/image_raw/compressed        sensor_msgs/CompressedImage, the camera's own
//                                     JPEG passed through (no decode)
//   depth/image_raw                   16UC1 mm, depth camera
//   aligned_depth_to_color/image_raw  16UC1 mm, registered to the colour camera
//   depth/image_filtered              the two depth streams again, noise-filtered
//   aligned_depth_to_color/image_filtered  (temporal + spatial; filter.* params)
//   */camera_info                     from the per-unit calibration in the stream
//
// plus a static TF colour optical frame -> depth optical frame (and optionally
// base_frame_id -> colour optical frame), and /diagnostics.
//
// The camera's UVC image controls (exposure, gain, white balance, ...) are
// parameters under color.*, declared when the camera first connects, with the
// camera's own ranges and current values. color.exposure_mode picks the
// camera's auto exposure, manual, or the driver's own auto exposure + gain.
//
// All device access happens on one worker thread. Parameter callbacks only
// validate and flag a change; the worker applies it between frames, and again
// after every reconnect.

#ifndef HP60C_DRIVER__HP60C_NODE_HPP_
#define HP60C_DRIVER__HP60C_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/update_functions.hpp"
#include "hp60c_driver/auto_exposure.hpp"
#include "hp60c_driver/camera_controls.hpp"
#include "hp60c_driver/depth_filter.hpp"
#include "hp60c_driver/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace hp60c_driver
{

class CudaRegistrar;   // cuda_registrar.hpp; only defined in CUDA builds
class V4l2Capture;
struct CapturedFrame;

class Hp60cNode : public rclcpp::Node
{
public:
  explicit Hp60cNode(const rclcpp::NodeOptions & options);
  ~Hp60cNode() override;

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using CompressedImage = sensor_msgs::msg::CompressedImage;

  struct DepthOut
  {
    std::unique_ptr<Image> depth, aligned, depth_f, aligned_f;   // null = not wanted
  };

  // What /diagnostics reports; written by the worker, read by the updater.
  struct Stats
  {
    std::string device;
    bool streaming{false};
    std::string error;
    std::uint64_t frames{0};
    std::uint64_t host_dropped{0};
    std::uint64_t bad_frames{0};
    std::uint64_t connects{0};
    std::string backend;
    std::string exposure_mode;
    std::optional<std::int64_t> exposure_us;
    std::optional<std::int64_t> gain;
    std::optional<double> brightness;
  };

  // A device control behind a color.* parameter.
  struct ControlParam
  {
    ControlInfo info;
    ControlSpec spec;
  };

  // Parameters.
  void declare_parameters();
  void declare_control_parameters(V4l2Capture & cap);
  rcl_interfaces::msg::SetParametersResult validate_parameters(
    const std::vector<rclcpp::Parameter> & params);

  // Worker thread.
  bool running();
  void run();
  template<class Duration>
  void sleep_while_running(Duration d);
  void on_connect(V4l2Capture & cap, const std::string & dev);
  void apply_settings(V4l2Capture & cap);
  void apply_filter_settings();
  void apply_software_ae_settings(V4l2Capture & cap, bool announce);
  bool write_control(V4l2Capture & cap, std::uint32_t id, std::int64_t value);
  void on_frame(V4l2Capture & cap, const CapturedFrame & frame);
  rclcpp::Time capture_stamp(const CapturedFrame & frame);
  void run_software_ae(V4l2Capture & cap, const FrameView & f);
  bool mean_luma(const FrameView & f, double & out);
  void sample_exposure(V4l2Capture & cap);
  void compute_depth(const FrameView & f, DepthOut & o);
  void run_filter(const FrameView & f, const std::uint16_t * mm, DepthOut & o);
  std::unique_ptr<Image> mono16(const std::string & frame, const rclcpp::Time & stamp);
  void publish_info(
    const rclcpp::Publisher<CameraInfo>::SharedPtr & pub, const Intrinsics & k,
    const std::string & frame, const rclcpp::Time & stamp);
  void publish_static_tf(const Calibration * c, const rclcpp::Time & stamp);

  // Diagnostics.
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat);
  template<class F>
  void update_stats(F && f)
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    f(stats_);
  }

  // Static parameters (read-only).
  std::string device_param_;
  std::string color_frame_;
  std::string depth_frame_;
  std::string base_frame_;
  bool publish_tf_{true};

  rclcpp::Publisher<Image>::SharedPtr color_pub_, depth_pub_, aligned_pub_;
  rclcpp::Publisher<Image>::SharedPtr depth_filtered_pub_, aligned_filtered_pub_;
  rclcpp::Publisher<CompressedImage>::SharedPtr color_jpeg_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr color_info_pub_, depth_info_pub_, aligned_info_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;

  // Worker-thread state.
  void * tj_{nullptr};   // tjhandle
  std::vector<std::uint16_t> depth_mm_, filtered_mm_;
  std::vector<std::uint8_t> luma_;
  bool filter_enabled_{true};
  DepthFilterParams filter_params_;
  std::unique_ptr<DepthFilter> filter_;
  std::uint64_t last_filtered_us_{0};
  std::shared_ptr<CudaRegistrar> cuda_;   // null: CPU path
  std::map<std::uint32_t, ControlInfo> controls_;   // the connected camera's
  std::map<std::uint32_t, std::int64_t> written_;   // last value set per control
  std::set<std::pair<std::uint32_t, std::int64_t>> refused_;   // logged once each
  bool controls_declared_{false};
  ExposureMode exposure_mode_{ExposureMode::kAuto};
  AutoExposure ae_;
  bool ae_active_{false};
  bool ae_has_gain_{false};
  std::optional<std::uint32_t> last_sequence_;
  std::uint64_t frames_since_sample_{0};
  bool tf_sent_{false};

  // Shared with parameter callbacks.
  std::mutex control_params_mutex_;
  std::map<std::string, ControlParam> control_params_;   // by parameter name
  std::atomic<bool> settings_dirty_{true};

  // Diagnostics.
  std::mutex stats_mutex_;
  Stats stats_;
  double min_rate_{15.0}, max_rate_{30.0};
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::unique_ptr<diagnostic_updater::FrequencyStatus> rate_;

  std::atomic<bool> stop_{false};
  std::thread worker_;
};

}  // namespace hp60c_driver

#endif  // HP60C_DRIVER__HP60C_NODE_HPP_
