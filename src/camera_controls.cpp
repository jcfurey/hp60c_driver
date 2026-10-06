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

#include "hp60c_driver/camera_controls.hpp"

#include <linux/videodev2.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

namespace hp60c_driver
{

bool ControlInfo::is_bool() const
{
  return type == V4L2_CTRL_TYPE_BOOLEAN;
}

bool ControlInfo::is_menu() const
{
  return type == V4L2_CTRL_TYPE_MENU || type == V4L2_CTRL_TYPE_INTEGER_MENU;
}

bool ControlInfo::read_only() const
{
  return (flags & V4L2_CTRL_FLAG_READ_ONLY) != 0;
}

bool ControlInfo::has_menu_entry(std::int64_t value) const
{
  return std::any_of(
    menu.begin(), menu.end(), [value](const auto & e) {return e.first == value;});
}

std::optional<ExposureMode> parse_exposure_mode(const std::string & s)
{
  if (s == "auto") {
    return ExposureMode::kAuto;
  }
  if (s == "manual") {
    return ExposureMode::kManual;
  }
  if (s == "software") {
    return ExposureMode::kSoftware;
  }
  return std::nullopt;
}

const char * to_string(ExposureMode mode)
{
  switch (mode) {
    case ExposureMode::kAuto: return "auto";
    case ExposureMode::kManual: return "manual";
    case ExposureMode::kSoftware: return "software";
  }
  return "?";
}

const std::vector<ControlSpec> & control_specs()
{
  static const std::vector<ControlSpec> specs = {
    {V4L2_CID_AUTO_WHITE_BALANCE, "auto_white_balance",
      "The camera's automatic white balance.", 1, Gate::kAlways},
    {V4L2_CID_EXPOSURE_AUTO_PRIORITY, "exposure_dynamic_framerate",
      "Let the camera's auto exposure lower the frame rate in the dark.", 1, Gate::kAlways},
    {V4L2_CID_POWER_LINE_FREQUENCY, "power_line_frequency",
      "Anti-flicker for mains lighting.", 1, Gate::kAlways},
    {V4L2_CID_BRIGHTNESS, "brightness", "Brightness (camera units).", 1, Gate::kAlways},
    {V4L2_CID_CONTRAST, "contrast", "Contrast (camera units).", 1, Gate::kAlways},
    {V4L2_CID_SATURATION, "saturation", "Colour saturation (camera units).", 1, Gate::kAlways},
    {V4L2_CID_HUE, "hue", "Hue (camera units).", 1, Gate::kAlways},
    {V4L2_CID_GAMMA, "gamma", "Gamma (camera units).", 1, Gate::kAlways},
    {V4L2_CID_SHARPNESS, "sharpness", "Sharpness (camera units).", 1, Gate::kAlways},
    {V4L2_CID_BACKLIGHT_COMPENSATION, "backlight_compensation",
      "Backlight compensation (camera units).", 1, Gate::kAlways},
    {V4L2_CID_WHITE_BALANCE_TEMPERATURE, "white_balance_temperature",
      "White balance in kelvin. Applied while auto_white_balance is false.", 1,
      Gate::kManualWhiteBalance},
    {V4L2_CID_GAIN, "gain",
      "Sensor gain (camera units). Applied in exposure_mode auto and manual; "
      "software mode drives it itself.", 1, Gate::kNotSoftwareExposure},
    {V4L2_CID_EXPOSURE_ABSOLUTE, "exposure_us",
      "Exposure time in microseconds (UVC steps of 100 us). Applied in exposure_mode "
      "manual; software mode drives it itself.", 100, Gate::kManualExposure},
  };
  return specs;
}

std::uint32_t exposure_auto_id()
{
  return V4L2_CID_EXPOSURE_AUTO;
}

std::uint32_t exposure_absolute_id()
{
  return V4L2_CID_EXPOSURE_ABSOLUTE;
}

std::uint32_t gain_id()
{
  return V4L2_CID_GAIN;
}

std::vector<std::uint32_t> controls_gated_by(std::uint32_t switch_id)
{
  std::vector<std::uint32_t> ids;
  for (const auto & s : control_specs()) {
    // A camera's auto exposure may move gain too, not just exposure time.
    const bool exposure = s.gate == Gate::kManualExposure || s.gate == Gate::kNotSoftwareExposure;
    if ((switch_id == V4L2_CID_EXPOSURE_AUTO && exposure) ||
      (switch_id == V4L2_CID_AUTO_WHITE_BALANCE && s.gate == Gate::kManualWhiteBalance))
    {
      ids.push_back(s.id);
    }
  }
  return ids;
}

std::optional<std::int64_t> exposure_auto_value(ExposureMode mode, const ControlInfo & c)
{
  if (mode == ExposureMode::kAuto) {
    for (const std::int64_t v : {V4L2_EXPOSURE_APERTURE_PRIORITY, V4L2_EXPOSURE_AUTO}) {
      if (c.has_menu_entry(v)) {
        return v;
      }
    }
    return std::nullopt;
  }
  if (c.has_menu_entry(V4L2_EXPOSURE_MANUAL)) {
    return V4L2_EXPOSURE_MANUAL;
  }
  return std::nullopt;
}

std::string describe(const ControlInfo & c, std::optional<std::int64_t> current)
{
  char head[96];
  std::snprintf(head, sizeof head, "%s [0x%08x]: ", c.name.c_str(), c.id);
  std::string s = head;
  if (c.is_bool()) {
    s += "bool";
  } else if (c.is_menu()) {
    s += "menu {";
    for (std::size_t i = 0; i < c.menu.size(); ++i) {
      s += (i ? ", " : "") + std::to_string(c.menu[i].first) + ": " + c.menu[i].second;
    }
    s += "}";
  } else {
    s += "int " + std::to_string(c.minimum) + ".." + std::to_string(c.maximum) +
      " step " + std::to_string(c.step);
  }
  s += ", default " + std::to_string(c.default_value);
  if (current) {
    s += " (current " + std::to_string(*current) + ")";
  }
  if (c.read_only()) {
    s += ", read-only";
  }
  return s;
}

}  // namespace hp60c_driver
