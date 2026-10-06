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

#include <gtest/gtest.h>
#include <linux/videodev2.h>

#include <set>
#include <string>
#include <vector>

#include "hp60c_driver/camera_controls.hpp"

namespace hp = hp60c_driver;

namespace
{

hp::ControlInfo exposure_menu(std::initializer_list<std::int64_t> entries)
{
  hp::ControlInfo c;
  c.id = V4L2_CID_EXPOSURE_AUTO;
  c.name = "Auto Exposure";
  c.type = V4L2_CTRL_TYPE_MENU;
  c.minimum = 0;
  c.maximum = 3;
  for (const auto v : entries) {
    c.menu.emplace_back(v, "entry " + std::to_string(v));
  }
  return c;
}

}  // namespace

TEST(CameraControls, ParsesExposureModes)
{
  EXPECT_EQ(hp::parse_exposure_mode("auto"), hp::ExposureMode::kAuto);
  EXPECT_EQ(hp::parse_exposure_mode("manual"), hp::ExposureMode::kManual);
  EXPECT_EQ(hp::parse_exposure_mode("software"), hp::ExposureMode::kSoftware);
  EXPECT_FALSE(hp::parse_exposure_mode("Auto"));
  EXPECT_FALSE(hp::parse_exposure_mode(""));
  for (const auto m : {hp::ExposureMode::kAuto, hp::ExposureMode::kManual,
      hp::ExposureMode::kSoftware})
  {
    EXPECT_EQ(hp::parse_exposure_mode(hp::to_string(m)), m);
  }
}

TEST(CameraControls, AutoPrefersAperturePriority)
{
  // A typical UVC camera: manual (1) and aperture priority (3) only.
  const auto uvc = exposure_menu({V4L2_EXPOSURE_MANUAL, V4L2_EXPOSURE_APERTURE_PRIORITY});
  EXPECT_EQ(
    hp::exposure_auto_value(hp::ExposureMode::kAuto, uvc), V4L2_EXPOSURE_APERTURE_PRIORITY);
  EXPECT_EQ(hp::exposure_auto_value(hp::ExposureMode::kManual, uvc), V4L2_EXPOSURE_MANUAL);
  EXPECT_EQ(hp::exposure_auto_value(hp::ExposureMode::kSoftware, uvc), V4L2_EXPOSURE_MANUAL);

  const auto full = exposure_menu({V4L2_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL});
  EXPECT_EQ(hp::exposure_auto_value(hp::ExposureMode::kAuto, full), V4L2_EXPOSURE_AUTO);

  const auto auto_only = exposure_menu({V4L2_EXPOSURE_APERTURE_PRIORITY});
  EXPECT_FALSE(hp::exposure_auto_value(hp::ExposureMode::kManual, auto_only));
}

TEST(CameraControls, SpecsAreUniqueAndSwitchesComeFirst)
{
  std::set<std::uint32_t> ids;
  std::set<std::string> names;
  int index = 0, awb = -1, wb_temp = -1;
  for (const auto & s : hp::control_specs()) {
    EXPECT_TRUE(ids.insert(s.id).second) << s.param;
    EXPECT_TRUE(names.insert(s.param).second) << s.param;
    EXPECT_GT(s.scale, 0) << s.param;
    EXPECT_NE(s.id, hp::exposure_auto_id()) << "exposure mode is color.exposure_mode";
    if (s.id == V4L2_CID_AUTO_WHITE_BALANCE) {
      awb = index;
    }
    if (s.id == V4L2_CID_WHITE_BALANCE_TEMPERATURE) {
      wb_temp = index;
      EXPECT_EQ(s.gate, hp::Gate::kManualWhiteBalance);
    }
    if (s.id == hp::exposure_absolute_id()) {
      EXPECT_EQ(s.gate, hp::Gate::kManualExposure);
      EXPECT_EQ(s.scale, 100);   // UVC exposure is in 100 us units
    }
    if (s.id == hp::gain_id()) {
      EXPECT_EQ(s.gate, hp::Gate::kNotSoftwareExposure);
    }
    ++index;
  }
  ASSERT_GE(awb, 0);
  EXPECT_LT(awb, wb_temp);
}

TEST(CameraControls, AutoModesInvalidateTheValuesTheyOwn)
{
  EXPECT_EQ(
    hp::controls_gated_by(hp::exposure_auto_id()),
    (std::vector<std::uint32_t>{hp::gain_id(), hp::exposure_absolute_id()}));
  EXPECT_EQ(
    hp::controls_gated_by(V4L2_CID_AUTO_WHITE_BALANCE),
    std::vector<std::uint32_t>{V4L2_CID_WHITE_BALANCE_TEMPERATURE});
  EXPECT_TRUE(hp::controls_gated_by(hp::gain_id()).empty());
}

TEST(CameraControls, DescribesControls)
{
  hp::ControlInfo gain;
  gain.id = V4L2_CID_GAIN;
  gain.name = "Gain";
  gain.type = V4L2_CTRL_TYPE_INTEGER;
  gain.minimum = 0;
  gain.maximum = 100;
  gain.default_value = 32;
  EXPECT_EQ(
    hp::describe(gain, 40), "Gain [0x00980913]: int 0..100 step 1, default 32 (current 40)");

  auto menu = exposure_menu({1, 3});
  menu.default_value = 3;
  menu.flags = V4L2_CTRL_FLAG_READ_ONLY;
  EXPECT_EQ(
    hp::describe(menu, std::nullopt),
    "Auto Exposure [0x009a0901]: menu {1: entry 1, 3: entry 3}, default 3, read-only");
  EXPECT_TRUE(menu.is_menu());
  EXPECT_TRUE(menu.read_only());
  EXPECT_TRUE(menu.has_menu_entry(3));
  EXPECT_FALSE(menu.has_menu_entry(2));
}
