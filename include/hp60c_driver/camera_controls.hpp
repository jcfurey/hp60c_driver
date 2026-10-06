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

// The camera's UVC image controls (exposure, gain, white balance, ...) as the
// kernel's V4L2 interface reports them, and which of them the node exposes as
// ROS parameters. ROS-free, so it can be unit-tested without a camera.
//
// Parameter names are fixed per V4L2 control ID, not derived from the kernel's
// labels: those changed between kernel versions ("Exposure, Auto" became
// "Auto Exposure"), and a parameter API must not.

#ifndef HP60C_DRIVER__CAMERA_CONTROLS_HPP_
#define HP60C_DRIVER__CAMERA_CONTROLS_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace hp60c_driver
{

// One V4L2 control as the device reports it (VIDIOC_QUERY_EXT_CTRL).
struct ControlInfo
{
  std::uint32_t id{0};
  std::string name;              // the kernel's label, e.g. "Gain"
  std::uint32_t type{0};         // V4L2_CTRL_TYPE_*
  std::int64_t minimum{0};
  std::int64_t maximum{0};
  std::uint64_t step{1};
  std::int64_t default_value{0};
  std::uint32_t flags{0};        // V4L2_CTRL_FLAG_*
  std::vector<std::pair<std::int64_t, std::string>> menu;   // menu types: the valid entries

  bool is_bool() const;
  bool is_menu() const;
  bool read_only() const;
  bool has_menu_entry(std::int64_t value) const;
};

// How the colour camera's exposure is controlled (parameter color.exposure_mode).
enum class ExposureMode
{
  kAuto,       // the camera's own auto exposure
  kManual,     // color.exposure_us and color.gain as set
  kSoftware,   // the driver's auto exposure + auto gain (AutoExposure) drives them
};

std::optional<ExposureMode> parse_exposure_mode(const std::string & s);
const char * to_string(ExposureMode mode);

// What decides whether a control's parameter is written to the camera.
enum class Gate
{
  kAlways,
  kManualExposure,       // only in manual exposure mode (the camera's AE owns it otherwise)
  kNotSoftwareExposure,  // in auto and manual mode; the driver's AE owns it in software mode
  kManualWhiteBalance,   // only while color.auto_white_balance is false
};

// A standard UVC control exposed as the ROS parameter "color.<param>".
struct ControlSpec
{
  std::uint32_t id;          // V4L2_CID_*
  const char * param;        // parameter name below "color."
  const char * description;
  std::int64_t scale;        // parameter value = device value * scale
  Gate gate;
};

// The exposed controls, in the order they are written: switches before the
// values they gate. The exposure mode itself (V4L2_CID_EXPOSURE_AUTO) is
// driven by color.exposure_mode and written before all of these.
const std::vector<ControlSpec> & control_specs();

// V4L2 control IDs the node drives directly.
std::uint32_t exposure_auto_id();       // V4L2_CID_EXPOSURE_AUTO (menu)
std::uint32_t exposure_absolute_id();   // V4L2_CID_EXPOSURE_ABSOLUTE, 100 us units
std::uint32_t gain_id();                // V4L2_CID_GAIN

// Controls whose value an automatic mode may change behind the driver's back,
// so they must be written again after `switch_id` is: exposure time and gain
// after the exposure mode, white balance temperature after auto white balance.
std::vector<std::uint32_t> controls_gated_by(std::uint32_t switch_id);

// The V4L2_CID_EXPOSURE_AUTO menu value that selects `mode`, or nullopt if the
// camera's menu has no such entry. Auto prefers aperture priority (UVC's "auto
// exposure time, manual iris", what webcams implement) over full auto. Manual
// and software both need the camera's manual mode.
std::optional<std::int64_t> exposure_auto_value(ExposureMode mode, const ControlInfo & c);

// One human-readable line for the startup log, e.g.
// "Gain [0x00980913]: int 0..100 step 1, default 32 (current 32)".
std::string describe(const ControlInfo & c, std::optional<std::int64_t> current);

}  // namespace hp60c_driver

#endif  // HP60C_DRIVER__CAMERA_CONTROLS_HPP_
