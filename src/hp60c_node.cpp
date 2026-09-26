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
//   */camera_info                     from the per-unit calibration in the stream
//
// plus a static TF colour optical frame -> depth optical frame.

#include <turbojpeg.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hp60c_driver/frame.hpp"
#include "hp60c_driver/v4l2_capture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace hp60c_driver
{

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

namespace
{

constexpr std::uint16_t kVid = 0x3482;
constexpr std::uint16_t kPid = 0x6723;
constexpr int kStreamHeight = 642;   // the composite mode's nominal height

CameraInfo make_info(const Intrinsics & k, const std::string & frame_id)
{
  CameraInfo ci;
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

}  // namespace

class Hp60cNode : public rclcpp::Node
{
public:
  explicit Hp60cNode(const rclcpp::NodeOptions & options)
  : Node("hp60c", options),
    device_param_(declare_parameter<std::string>("device", "")),
    color_frame_(declare_parameter<std::string>("color_frame_id", "hp60c_color_optical_frame")),
    depth_frame_(declare_parameter<std::string>("depth_frame_id", "hp60c_depth_optical_frame")),
    publish_tf_(declare_parameter<bool>("publish_tf", true))
  {
    // Reliable by default: a reliable publisher serves both reliable and
    // best-effort subscribers, while a best-effort one is invisible to reliable
    // subscribers (e.g. RViz's defaults). Depth 2 keeps a slow subscriber from
    // building a backlog.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
    if (declare_parameter<bool>("best_effort", false)) {
      qos.best_effort();
    }
    color_pub_ = create_publisher<Image>("color/image_raw", qos);
    color_jpeg_pub_ = create_publisher<CompressedImage>("color/image_raw/compressed", qos);
    color_info_pub_ = create_publisher<CameraInfo>("color/camera_info", qos);
    depth_pub_ = create_publisher<Image>("depth/image_raw", qos);
    depth_info_pub_ = create_publisher<CameraInfo>("depth/camera_info", qos);
    aligned_pub_ = create_publisher<Image>("aligned_depth_to_color/image_raw", qos);
    aligned_info_pub_ = create_publisher<CameraInfo>("aligned_depth_to_color/camera_info", qos);
    tf_static_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(1).reliable().transient_local());

    tj_ = tjInitDecompress();
    if (!tj_) {
      throw std::runtime_error("tjInitDecompress failed");
    }
    depth_mm_.resize(static_cast<std::size_t>(kWidth) * kHeight);
    worker_ = std::thread([this] {run();});
  }

  ~Hp60cNode() override
  {
    stop_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
    if (tj_) {
      tjDestroy(tj_);
    }
  }

private:
  // Open the camera, stream, and on any failure retry every 2 s until shutdown.
  void run()
  {
    while (!stop_ && rclcpp::ok()) {
      std::string dev = device_param_.empty() ? find_video_device(kVid, kPid) : device_param_;
      if (dev.empty()) {
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
        RCLCPP_INFO(get_logger(), "Streaming from %s", dev.c_str());
        int timeouts = 0;
        while (!stop_ && rclcpp::ok()) {
          const bool got = cap.grab(
            500, [this](const std::uint8_t * data, std::size_t len) {on_buffer(data, len);});
          timeouts = got ? 0 : timeouts + 1;
          if (timeouts >= 6) {
            throw std::runtime_error("no frames for 3 s");
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Camera error on %s: %s; retrying", dev.c_str(), e.what());
        sleep_while_running(std::chrono::seconds(2));
      }
    }
  }

  template<class Duration>
  void sleep_while_running(Duration d)
  {
    const auto until = std::chrono::steady_clock::now() + d;
    while (!stop_ && rclcpp::ok() && std::chrono::steady_clock::now() < until) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void on_buffer(const std::uint8_t * data, std::size_t len)
  {
    FrameView f;
    try {
      f = parse_frame(data, len);
    } catch (const FormatError & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping frame: %s", e.what());
      return;
    }
    const rclcpp::Time stamp = now();
    if (publish_tf_ && !tf_sent_) {
      publish_extrinsics(f.calib, stamp);
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

    const bool want_depth = depth_pub_->get_subscription_count() > 0;
    const bool want_aligned = aligned_pub_->get_subscription_count() > 0;
    if (want_depth || want_aligned) {
      depth_to_mm(f.depth_raw, depth_mm_.data());
    }
    if (want_depth) {
      depth_pub_->publish(mono16(depth_mm_.data(), depth_frame_, stamp));
    }
    publish_info(depth_info_pub_, f.calib.depth, depth_frame_, stamp);
    if (want_aligned) {
      auto msg = mono16(nullptr, color_frame_, stamp);
      register_to_colour(
        depth_mm_.data(), f.calib, reinterpret_cast<std::uint16_t *>(msg->data.data()));
      aligned_pub_->publish(std::move(msg));
    }
    publish_info(aligned_info_pub_, f.calib.colour, color_frame_, stamp);
  }

  std::unique_ptr<Image> mono16(
    const std::uint16_t * src, const std::string & frame, const rclcpp::Time & stamp)
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
    if (src) {
      std::memcpy(msg->data.data(), src, msg->data.size());
    }
    return msg;
  }

  void publish_info(
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

  // Static TF: parent = colour optical frame, child = depth optical frame, so a
  // point p in the depth frame maps to R p + t in the colour frame.
  void publish_extrinsics(const Calibration & c, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = color_frame_;
    tf.child_frame_id = depth_frame_;
    tf.transform.translation.x = c.t_mm[0] / 1000.0;
    tf.transform.translation.y = c.t_mm[1] / 1000.0;
    tf.transform.translation.z = c.t_mm[2] / 1000.0;
    const auto q = rotation_to_quaternion(c.R);
    tf.transform.rotation.x = q[0];
    tf.transform.rotation.y = q[1];
    tf.transform.rotation.z = q[2];
    tf.transform.rotation.w = q[3];
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(tf);
    tf_static_pub_->publish(msg);
    tf_sent_ = true;
    RCLCPP_INFO(
      get_logger(), "Calibration: depth fx %.1f, colour fx %.1f, baseline %.2f mm",
      c.depth.fx, c.colour.fx,
      std::sqrt(c.t_mm[0] * c.t_mm[0] + c.t_mm[1] * c.t_mm[1] + c.t_mm[2] * c.t_mm[2]));
  }

  const std::string device_param_;
  const std::string color_frame_;
  const std::string depth_frame_;
  const bool publish_tf_;

  rclcpp::Publisher<Image>::SharedPtr color_pub_, depth_pub_, aligned_pub_;
  rclcpp::Publisher<CompressedImage>::SharedPtr color_jpeg_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr color_info_pub_, depth_info_pub_, aligned_info_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;

  tjhandle tj_{nullptr};
  std::vector<std::uint16_t> depth_mm_;
  bool tf_sent_{false};
  std::atomic<bool> stop_{false};
  std::thread worker_;
};

}  // namespace hp60c_driver

RCLCPP_COMPONENTS_REGISTER_NODE(hp60c_driver::Hp60cNode)
