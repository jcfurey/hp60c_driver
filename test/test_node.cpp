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

// The node's ROS interface, in-process and with no camera: its device is
// /dev/null, which it can open but never stream from, so it keeps retrying.

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hp60c_driver/hp60c_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;
using hp60c_driver::Hp60cNode;

class NodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<Hp60cNode> make(std::vector<rclcpp::Parameter> overrides = {})
  {
    overrides.emplace_back("device", "/dev/null");
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    return std::make_shared<Hp60cNode>(options);
  }

  // Spin both nodes until `done` or the timeout.
  static bool spin_until(
    const rclcpp::Node::SharedPtr & a, const rclcpp::Node::SharedPtr & b,
    const std::function<bool()> & done, std::chrono::seconds timeout = 10s)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(a);
    exec.add_node(b);
    const auto until = std::chrono::steady_clock::now() + timeout;
    while (!done() && std::chrono::steady_clock::now() < until) {
      exec.spin_some(100ms);
    }
    return done();
  }
};

TEST_F(NodeTest, DeclaresDescribedParameters)
{
  const auto node = make();
  for (const char * name : {"device", "color_frame_id", "depth_frame_id", "base_frame_id",
      "publish_tf", "best_effort", "use_cuda"})
  {
    ASSERT_TRUE(node->has_parameter(name)) << name;
    const auto d = node->describe_parameter(name);
    EXPECT_FALSE(d.description.empty()) << name;
    EXPECT_TRUE(d.read_only) << name;
  }
  for (const char * name : {"filter.enabled", "filter.temporal", "filter.temporal_alpha",
      "filter.temporal_reset_fraction", "filter.spatial", "color.exposure_mode",
      "color.software_ae.target_brightness", "color.software_ae.tolerance",
      "color.software_ae.min_exposure_us", "color.software_ae.max_exposure_us",
      "color.software_ae.min_gain", "color.software_ae.max_gain"})
  {
    ASSERT_TRUE(node->has_parameter(name)) << name;
    const auto d = node->describe_parameter(name);
    EXPECT_FALSE(d.description.empty()) << name;
    EXPECT_FALSE(d.read_only) << name;
  }
  // The camera's own controls appear only once a camera has been seen.
  EXPECT_FALSE(node->has_parameter("color.gain"));
  // Standard QoS overrides on the image topics.
  EXPECT_TRUE(node->has_parameter("qos_overrides./color/image_raw.publisher.reliability"));
  EXPECT_TRUE(node->has_parameter("qos_overrides./depth/image_raw.publisher.depth"));
}

TEST_F(NodeTest, StaticParametersAreReadOnly)
{
  const auto node = make();
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("device", "/dev/video9")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("use_cuda", false)).successful);
  EXPECT_EQ(node->get_parameter("device").as_string(), "/dev/null");
}

TEST_F(NodeTest, ValidatesExposureMode)
{
  const auto node = make();
  EXPECT_EQ(node->get_parameter("color.exposure_mode").as_string(), "auto");
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("color.exposure_mode", "software")).successful);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("color.exposure_mode", "manual")).successful);
  const auto bad = node->set_parameter(rclcpp::Parameter("color.exposure_mode", "sometimes"));
  EXPECT_FALSE(bad.successful);
  EXPECT_NE(bad.reason.find("auto, manual or software"), std::string::npos);
  EXPECT_EQ(node->get_parameter("color.exposure_mode").as_string(), "manual");
}

TEST_F(NodeTest, RejectsAnInvalidStartupValue)
{
  EXPECT_THROW(
    make({rclcpp::Parameter("color.exposure_mode", "sometimes")}),
    rclcpp::exceptions::InvalidParameterValueException);
}

TEST_F(NodeTest, RangesAreEnforced)
{
  const auto node = make();
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("filter.temporal_alpha", 0.5)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("filter.temporal_alpha", 1.5)).successful);
  EXPECT_FALSE(
    node->set_parameter(rclcpp::Parameter("color.software_ae.target_brightness", 300.0))
    .successful);
  EXPECT_TRUE(
    node->set_parameter(rclcpp::Parameter("color.software_ae.max_gain", int64_t{40})).successful);
}

TEST_F(NodeTest, ReportsTheMissingCameraOnDiagnostics)
{
  const auto node = make();
  const auto listener = std::make_shared<rclcpp::Node>("diagnostics_listener");
  bool seen = false;
  auto sub = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10, [&seen](const diagnostic_msgs::msg::DiagnosticArray & msg) {
      for (const auto & s : msg.status) {
        if (s.hardware_id == "HP60C (USB 3482:6723)" &&
        s.name.find("Camera") != std::string::npos &&
        s.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR)
        {
          seen = true;
        }
      }
    });
  EXPECT_TRUE(spin_until(node, listener, [&seen] {return seen;}));
}

TEST_F(NodeTest, PublishesTheOptionalBodyFrame)
{
  const auto node = make({rclcpp::Parameter("base_frame_id", "hp60c_link")});
  const auto listener = std::make_shared<rclcpp::Node>("tf_listener");
  geometry_msgs::msg::TransformStamped got;
  bool seen = false;
  auto sub = listener->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(10).reliable().transient_local(),
    [&](const tf2_msgs::msg::TFMessage & msg) {
      for (const auto & t : msg.transforms) {
        if (t.header.frame_id == "hp60c_link") {
          got = t;
          seen = true;
        }
      }
    });
  ASSERT_TRUE(spin_until(node, listener, [&seen] {return seen;}));
  EXPECT_EQ(got.child_frame_id, "hp60c_color_optical_frame");
  // REP 103: optical z forward, x right, y down in a body frame x forward, z up.
  EXPECT_NEAR(got.transform.rotation.x, -0.5, 1e-9);
  EXPECT_NEAR(got.transform.rotation.y, 0.5, 1e-9);
  EXPECT_NEAR(got.transform.rotation.z, -0.5, 1e-9);
  EXPECT_NEAR(got.transform.rotation.w, 0.5, 1e-9);
}

TEST_F(NodeTest, ShutsDownPromptly)
{
  auto node = make();
  std::this_thread::sleep_for(300ms);   // let the worker get into its retry loop
  const auto start = std::chrono::steady_clock::now();
  node.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
}
