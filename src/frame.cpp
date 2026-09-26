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

#include "hp60c_driver/frame.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

namespace hp60c_driver
{

namespace
{

float read_f32(const std::uint8_t * p)
{
  float v;
  std::memcpy(&v, p, sizeof v);   // the camera is little-endian, as is aarch64/x86
  return v;
}

std::uint32_t read_u32(const std::uint8_t * p)
{
  std::uint32_t v;
  std::memcpy(&v, p, sizeof v);
  return v;
}

double word_f32(const std::uint8_t * prefix, std::size_t word)
{
  return static_cast<double>(read_f32(prefix + 4 * word));
}

}  // namespace

std::size_t jpeg_end(const std::uint8_t * buf, std::size_t len, std::size_t start)
{
  if (start + 4 > len || buf[start] != 0xFF || buf[start + 1] != 0xD8) {
    throw FormatError("no JPEG SOI where expected");
  }
  // Walk marker segments up to SOS.
  std::size_t i = start + 2;
  for (;; ) {
    if (i + 4 > len || buf[i] != 0xFF) {
      throw FormatError("malformed JPEG header");
    }
    const std::uint8_t marker = buf[i + 1];
    const std::size_t seg = (static_cast<std::size_t>(buf[i + 2]) << 8) | buf[i + 3];
    i += 2 + seg;
    if (marker == 0xDA) {
      break;   // entropy-coded data follows the SOS header
    }
  }
  // Entropy-coded data: FF 00 is a stuffed byte, FF D0..D7 a restart marker.
  for (; i + 1 < len; ++i) {
    if (buf[i] != 0xFF) {
      continue;
    }
    const std::uint8_t next = buf[i + 1];
    if (next == 0x00 || (next >= 0xD0 && next <= 0xD7)) {
      ++i;
      continue;
    }
    if (next == 0xD9) {
      return i + 2;
    }
    throw FormatError("unexpected marker inside JPEG scan data");
  }
  throw FormatError("JPEG not terminated");
}

FrameView parse_frame(const std::uint8_t * buf, std::size_t len)
{
  if (len < kPrefixBytes + kDepthBytes + kTrailerBytes) {
    throw FormatError("buffer too short: " + std::to_string(len) + " bytes");
  }
  FrameView f;
  const std::uint8_t * pre = buf;

  f.calib.depth = {word_f32(pre, 2), word_f32(pre, 3), word_f32(pre, 4), word_f32(pre, 5)};
  f.calib.colour = {word_f32(pre, 11), word_f32(pre, 12), word_f32(pre, 13), word_f32(pre, 14)};
  for (std::size_t k = 0; k < 9; ++k) {
    f.calib.R[k] = word_f32(pre, 20 + k);
  }
  for (std::size_t k = 0; k < 3; ++k) {
    f.calib.t_mm[k] = word_f32(pre, 29 + k);
  }
  f.device_stamp_us = read_u32(pre + 4 * 256) |
    (static_cast<std::uint64_t>(read_u32(pre + 4 * 257)) << 32);
  const std::size_t declared_jpeg = read_u32(pre + 4 * 268);

  const std::size_t end = jpeg_end(buf, len, kPrefixBytes);
  f.jpeg = buf + kPrefixBytes;
  f.jpeg_len = end - kPrefixBytes;
  if (f.jpeg_len != declared_jpeg) {
    throw FormatError(
            "JPEG is " + std::to_string(f.jpeg_len) + " bytes but the header says " +
            std::to_string(declared_jpeg));
  }
  if (end + kDepthBytes + kTrailerBytes > len) {
    throw FormatError("buffer ends inside the depth block");
  }
  f.depth_raw = buf + end;
  if (!(f.calib.depth.fx > 0 && f.calib.colour.fx > 0)) {
    throw FormatError("calibration block missing (non-positive focal length)");
  }
  return f;
}

void depth_to_mm(const std::uint8_t * depth_raw, std::uint16_t * out)
{
  // Stored as 640 rows x 480 columns; upright (r, c) = stored (639 - c, r).
  constexpr int stored_w = kHeight;   // 480
  for (int c = 0; c < kWidth; ++c) {
    const std::uint8_t * row = depth_raw + 2 * static_cast<std::size_t>(kWidth - 1 - c) * stored_w;
    for (int r = 0; r < kHeight; ++r) {
      const std::uint16_t v = static_cast<std::uint16_t>(row[2 * r] | (row[2 * r + 1] << 8));
      out[static_cast<std::size_t>(r) * kWidth + c] = static_cast<std::uint16_t>(v >> 4);
    }
  }
}

void register_to_colour(
  const std::uint16_t * depth_mm, const Calibration & calib, std::uint16_t * out)
{
  // Each depth pixel is splatted over its whole footprint, not just its centre:
  // the colour camera's focal length is ~1.33x the depth camera's, so its grid
  // is denser, and centre-only projection leaves ~40% of the covered pixels
  // empty. The footprint is the depth pixel's 1x1 square scaled by
  // (colour f / depth f) * (Z / z_colour), i.e. exact for R = I. The HP60C's
  // R is within a few degrees of identity (2.3 deg roll on the unit tested),
  // so the rotated square's bounding box is at most ~4% larger than this.
  // Nearest surface wins.
  // 0 marks "no value yet" (it is also the published "no measurement").
  const std::size_t n = static_cast<std::size_t>(kWidth) * kHeight;
  std::fill(out, out + n, std::uint16_t{0});
  const auto & R = calib.R;
  const auto & t = calib.t_mm;
  const Intrinsics & d = calib.depth;
  const Intrinsics & k = calib.colour;
  const double sx = 0.5 * k.fx / d.fx;
  const double sy = 0.5 * k.fy / d.fy;
  // Normalised rays: X = rx[u] * Z, Y = ry[v] * Z.
  std::array<double, kWidth> rx;
  std::array<double, kHeight> ry;
  for (int u = 0; u < kWidth; ++u) {
    rx[u] = (u - d.cx) / d.fx;
  }
  for (int v = 0; v < kHeight; ++v) {
    ry[v] = (v - d.cy) / d.fy;
  }
  for (int v = 0; v < kHeight; ++v) {
    // Row terms of R * (rx, ry, 1): only the rx part varies along the row.
    const double ax = R[1] * ry[v] + R[2];
    const double ay = R[4] * ry[v] + R[5];
    const double az = R[7] * ry[v] + R[8];
    const std::uint16_t * src = depth_mm + static_cast<std::size_t>(v) * kWidth;
    for (int u = 0; u < kWidth; ++u) {
      const std::uint16_t z = src[u];
      if (z == 0) {
        continue;
      }
      const double Z = z;
      const double zc = (R[6] * rx[u] + az) * Z + t[2];
      if (zc < 0.5 || zc >= 65535.5) {
        continue;   // behind the camera, or not representable in uint16 mm
      }
      const double inv = 1.0 / zc;
      const double xc = (R[0] * rx[u] + ax) * Z + t[0];
      const double yc = (R[3] * rx[u] + ay) * Z + t[1];
      const double uf = xc * inv * k.fx + k.cx;
      const double vf = yc * inv * k.fy + k.cy;
      const double hx = sx * Z * inv;
      const double hy = sy * Z * inv;
      // Colour pixel centres inside the footprint, clipped to the image. The
      // floating-point clip also keeps the int conversion in range.
      const double c0 = std::max(std::ceil(uf - hx), 0.0);
      const double c1 = std::min(std::floor(uf + hx), kWidth - 1.0);
      const double r0 = std::max(std::ceil(vf - hy), 0.0);
      const double r1 = std::min(std::floor(vf + hy), kHeight - 1.0);
      if (!(c0 <= c1 && r0 <= r1)) {
        continue;
      }
      const auto zv = static_cast<std::uint16_t>(zc + 0.5);
      for (int r = static_cast<int>(r0); r <= static_cast<int>(r1); ++r) {
        std::uint16_t * row = out + static_cast<std::size_t>(r) * kWidth;
        for (int c = static_cast<int>(c0); c <= static_cast<int>(c1); ++c) {
          if (row[c] == 0 || zv < row[c]) {
            row[c] = zv;
          }
        }
      }
    }
  }
}

std::array<double, 4> rotation_to_quaternion(const std::array<double, 9> & m)
{
  // Shepperd's method: pick the largest diagonal term for numerical stability.
  const double tr = m[0] + m[4] + m[8];
  double x, y, z, w;
  if (tr > 0) {
    const double s = std::sqrt(tr + 1.0) * 2;
    w = 0.25 * s;
    x = (m[7] - m[5]) / s;
    y = (m[2] - m[6]) / s;
    z = (m[3] - m[1]) / s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2;
    w = (m[7] - m[5]) / s;
    x = 0.25 * s;
    y = (m[1] + m[3]) / s;
    z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2;
    w = (m[2] - m[6]) / s;
    x = (m[1] + m[3]) / s;
    y = 0.25 * s;
    z = (m[5] + m[7]) / s;
  } else {
    const double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2;
    w = (m[3] - m[1]) / s;
    x = (m[2] + m[6]) / s;
    y = (m[5] + m[7]) / s;
    z = 0.25 * s;
  }
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  return {x / norm, y / norm, z / norm, w / norm};
}

}  // namespace hp60c_driver
