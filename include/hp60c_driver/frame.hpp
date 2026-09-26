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

// Parsing of the HP60C 640x642 composite MJPG buffer. Pure C++, no ROS, so it
// can be unit-tested without a camera. The format is specified in
// docs/FRAME_FORMAT.md; section references below point there.

#ifndef HP60C_DRIVER__FRAME_HPP_
#define HP60C_DRIVER__FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace hp60c_driver
{

constexpr std::size_t kPrefixBytes = 2560;
constexpr int kWidth = 640;              // colour, and depth once rotated upright
constexpr int kHeight = 480;
constexpr std::size_t kDepthBytes = static_cast<std::size_t>(kWidth) * kHeight * 2;
constexpr std::size_t kTrailerBytes = 512;

struct Intrinsics
{
  double fx{0}, fy{0}, cx{0}, cy{0};
};

// Static calibration, prefix bytes 0-1023. Extrinsics map a point from the
// depth camera to the colour camera: P_colour = R * P_depth + t (t in mm).
struct Calibration
{
  Intrinsics depth;
  Intrinsics colour;
  std::array<double, 9> R{};   // row-major
  std::array<double, 3> t_mm{};
};

// One parsed buffer. Views point into the caller's buffer, which must outlive it.
struct FrameView
{
  Calibration calib;
  std::uint64_t device_stamp_us{0};
  const std::uint8_t * jpeg{nullptr};
  std::size_t jpeg_len{0};
  const std::uint8_t * depth_raw{nullptr};   // kDepthBytes, stored rotated
};

class FormatError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Parse one V4L2 buffer. Throws FormatError if it does not match the spec.
FrameView parse_frame(const std::uint8_t * buf, std::size_t len);

// Index just past the EOI of the JPEG starting at buf[start] (walks the
// entropy-coded data; a stray FF D9 in the depth payload must not end it).
std::size_t jpeg_end(const std::uint8_t * buf, std::size_t len, std::size_t start);

// Upright 640x480 depth in millimetres (0 = no measurement) from the stored
// 480x640 raw block: out(r, c) = raw(639 - c, r) >> 4.
void depth_to_mm(const std::uint8_t * depth_raw, std::uint16_t * out);

// Reproject depth (mm, depth camera, 640x480) into the colour camera's pixels.
// Each depth pixel fills every colour pixel centre its footprint covers, and
// the nearest surface wins. `out` is 640x480 and fully overwritten.
void register_to_colour(
  const std::uint16_t * depth_mm, const Calibration & calib, std::uint16_t * out);

// Unit quaternion (x, y, z, w) of a row-major rotation matrix.
std::array<double, 4> rotation_to_quaternion(const std::array<double, 9> & R);

}  // namespace hp60c_driver

#endif  // HP60C_DRIVER__FRAME_HPP_
