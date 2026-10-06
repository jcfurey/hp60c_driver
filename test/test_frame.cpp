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

// Format tests on synthetic buffers built from docs/FRAME_FORMAT.md. No real
// captures here: they are images of real rooms and must not be committed.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "hp60c_driver/frame.hpp"

namespace hp = hp60c_driver;

namespace
{

void put_f32(std::vector<std::uint8_t> & b, std::size_t word, float v)
{
  std::memcpy(b.data() + 4 * word, &v, 4);
}

void put_u32(std::vector<std::uint8_t> & b, std::size_t word, std::uint32_t v)
{
  std::memcpy(b.data() + 4 * word, &v, 4);
}

// Minimal marker structure: SOI, DQT stub, SOS, scan data containing a stuffed
// byte, a restart marker and nothing else, EOI. Enough for the marker walker.
std::vector<std::uint8_t> fake_jpeg()
{
  return {0xFF, 0xD8,
    0xFF, 0xDB, 0x00, 0x04, 0x00, 0x00,
    0xFF, 0xDA, 0x00, 0x04, 0x00, 0x00,
    0x12, 0xFF, 0x00, 0x34, 0xFF, 0xD3, 0x56,
    0xFF, 0xD9};
}

// Stored depth value at stored (row, col) of the 640x480 (rows x cols) block.
std::uint16_t stored_value(int row, int col)
{
  return static_cast<std::uint16_t>(((row * 7 + col * 3) % 4000 + 1) << 4 | (row & 0xF));
}

std::vector<std::uint8_t> make_buffer(bool poison_depth_with_eoi = false)
{
  std::vector<std::uint8_t> b(hp::kPrefixBytes, 0);
  put_f32(b, 2, 443.5F);
  put_f32(b, 3, 443.25F);
  put_f32(b, 4, 321.75F);
  put_f32(b, 5, 238.25F);
  put_f32(b, 11, 590.25F);
  put_f32(b, 12, 589.75F);
  put_f32(b, 13, 332.75F);
  put_f32(b, 14, 232.25F);
  const float R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (int k = 0; k < 9; ++k) {
    put_f32(b, 20 + k, R[k]);
  }
  put_f32(b, 29, -12.0F);
  put_u32(b, 256, 0x89ABCDEFu);
  put_u32(b, 257, 0x1u);
  const auto jpeg = fake_jpeg();
  put_u32(b, 268, static_cast<std::uint32_t>(jpeg.size()));
  b.insert(b.end(), jpeg.begin(), jpeg.end());
  for (int row = 0; row < 640; ++row) {
    for (int col = 0; col < 480; ++col) {
      const std::uint16_t v = stored_value(row, col);
      b.push_back(static_cast<std::uint8_t>(v & 0xFF));
      b.push_back(static_cast<std::uint8_t>(v >> 8));
    }
  }
  if (poison_depth_with_eoi) {   // an FF D9 pair inside the depth block
    const std::size_t at = hp::kPrefixBytes + jpeg.size() + 100;
    b[at] = 0xFF;
    b[at + 1] = 0xD9;
  }
  b.insert(b.end(), hp::kTrailerBytes, 0);
  return b;
}

}  // namespace

TEST(Frame, ParsesLayoutAndCalibration)
{
  const auto b = make_buffer();
  const auto f = hp::parse_frame(b.data(), b.size());
  EXPECT_EQ(f.jpeg, b.data() + hp::kPrefixBytes);
  EXPECT_EQ(f.jpeg_len, fake_jpeg().size());
  EXPECT_EQ(f.depth_raw, f.jpeg + f.jpeg_len);
  EXPECT_DOUBLE_EQ(f.calib.depth.fx, 443.5);
  EXPECT_DOUBLE_EQ(f.calib.depth.cy, 238.25);
  EXPECT_DOUBLE_EQ(f.calib.colour.fx, 590.25);
  EXPECT_DOUBLE_EQ(f.calib.colour.cx, 332.75);
  EXPECT_DOUBLE_EQ(f.calib.R[4], 1.0);
  EXPECT_DOUBLE_EQ(f.calib.t_mm[0], -12.0);
  EXPECT_EQ(f.device_stamp_us, 0x189ABCDEFull);
}

TEST(Frame, JpegEndIgnoresEoiInsideDepth)
{
  const auto b = make_buffer(true);
  const auto f = hp::parse_frame(b.data(), b.size());
  EXPECT_EQ(f.jpeg_len, fake_jpeg().size());
}

TEST(Frame, RejectsHeaderLengthMismatch)
{
  auto b = make_buffer();
  put_u32(b, 268, 999);
  EXPECT_THROW(hp::parse_frame(b.data(), b.size()), hp::FormatError);
}

TEST(Frame, RejectsShortBuffer)
{
  auto b = make_buffer();
  b.resize(b.size() - hp::kTrailerBytes - 1);
  EXPECT_THROW(hp::parse_frame(b.data(), b.size()), hp::FormatError);
}

TEST(Frame, DepthIsRotatedUprightAndShifted)
{
  const auto b = make_buffer();
  const auto f = hp::parse_frame(b.data(), b.size());
  std::vector<std::uint16_t> mm(static_cast<std::size_t>(hp::kWidth) * hp::kHeight);
  hp::depth_to_mm(f.depth_raw, mm.data());
  // upright (r, c) = round(stored (639 - c, r) / 16), i.e. numpy np.rot90(stored, 3)
  for (int r : {0, 1, 240, 479}) {
    for (int c : {0, 1, 320, 639}) {
      EXPECT_EQ(
        mm[static_cast<std::size_t>(r) * hp::kWidth + c], (stored_value(639 - c, r) + 8) >> 4)
        << "r=" << r << " c=" << c;
    }
  }
}

TEST(Frame, DepthRoundsSixteenthsToNearestMillimetre)
{
  // Raw depth is in 1/16 mm: 1000 mm + 7/16 rounds down, + 8/16 rounds up.
  std::vector<std::uint8_t> raw(hp::kDepthBytes, 0);
  auto put = [&raw](int stored_row, int stored_col, std::uint16_t v) {
      const std::size_t s = static_cast<std::size_t>(stored_row) * 480 + stored_col;
      raw[2 * s] = static_cast<std::uint8_t>(v & 0xFF);
      raw[2 * s + 1] = static_cast<std::uint8_t>(v >> 8);
    };
  put(639, 0, 1000 * 16 + 7);    // upright (0, 0)
  put(638, 0, 1000 * 16 + 8);    // upright (0, 1)
  put(637, 0, 0xFFFF);           // upright (0, 2): top of range, must not wrap
  std::vector<std::uint16_t> mm(static_cast<std::size_t>(hp::kWidth) * hp::kHeight);
  hp::depth_to_mm(raw.data(), mm.data());
  EXPECT_EQ(mm[0], 1000);
  EXPECT_EQ(mm[1], 1001);
  EXPECT_EQ(mm[2], 4096);
}

TEST(Frame, RegistrationWithIdentityIsNoOp)
{
  hp::Calibration c;
  c.depth = c.colour = {500.0, 500.0, 319.5, 239.5};
  c.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::vector<std::uint16_t> in(static_cast<std::size_t>(hp::kWidth) * hp::kHeight, 0);
  in[100 * hp::kWidth + 200] = 1500;
  in[300 * hp::kWidth + 600] = 800;
  std::vector<std::uint16_t> out(in.size(), 7);
  hp::register_to_colour(in.data(), c, out.data());
  EXPECT_EQ(out, in);
}

TEST(Frame, RegistrationShiftsByBaselineAndKeepsNearest)
{
  hp::Calibration c;
  c.depth = c.colour = {500.0, 500.0, 319.5, 239.5};
  c.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  c.t_mm = {-8.0, 0.0, 0.0};
  // Shift in pixels = fx * t_x / Z: -4 px at 1000 mm, -2 px at 2000 mm, so
  // columns 330 (near) and 328 (far) both land on column 326.
  std::vector<std::uint16_t> in(static_cast<std::size_t>(hp::kWidth) * hp::kHeight, 0);
  const int v = 240;
  in[v * hp::kWidth + 330] = 1000;
  in[v * hp::kWidth + 328] = 2000;
  std::vector<std::uint16_t> out(in.size());
  hp::register_to_colour(in.data(), c, out.data());
  EXPECT_EQ(out[v * hp::kWidth + 326], 1000);   // the nearer surface wins
  EXPECT_EQ(out[v * hp::kWidth + 328], 0);
  EXPECT_EQ(out[v * hp::kWidth + 330], 0);
}

TEST(Frame, RegistrationFillsMagnifiedFootprint)
{
  // Colour focal length 1.5x the depth one: a single depth pixel at the
  // principal point covers 1.5 colour pixels, so a 2x2 block must be filled
  // with no holes between neighbouring depth pixels.
  hp::Calibration c;
  c.depth = {400.0, 400.0, 320.0, 240.0};
  c.colour = {600.0, 600.0, 320.0, 240.0};
  c.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::vector<std::uint16_t> in(static_cast<std::size_t>(hp::kWidth) * hp::kHeight, 0);
  for (int v = 200; v < 280; ++v) {
    for (int u = 280; u < 360; ++u) {
      in[static_cast<std::size_t>(v) * hp::kWidth + u] = 1200;
    }
  }
  std::vector<std::uint16_t> out(in.size());
  hp::register_to_colour(in.data(), c, out.data());
  // The 80x80 depth patch magnifies to ~120x120; its interior must be solid.
  for (int v = 190; v < 290; ++v) {
    for (int u = 270; u < 370; ++u) {
      ASSERT_EQ(out[static_cast<std::size_t>(v) * hp::kWidth + u], 1200) << u << "," << v;
    }
  }
}

TEST(Frame, QuaternionFromRotation)
{
  const double a = 0.04;   // about z
  const std::array<double, 9> R = {std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0,
    1};
  const auto q = hp::rotation_to_quaternion(R);
  EXPECT_NEAR(q[0], 0.0, 1e-12);
  EXPECT_NEAR(q[1], 0.0, 1e-12);
  EXPECT_NEAR(q[2], std::sin(a / 2), 1e-12);
  EXPECT_NEAR(q[3], std::cos(a / 2), 1e-12);
}

TEST(Frame, QuaternionHasNonNegativeW)
{
  // The camera body -> optical frame rotation; Shepperd's last branch yields -q.
  const auto q = hp::rotation_to_quaternion({0, 0, 1, -1, 0, 0, 0, -1, 0});
  EXPECT_NEAR(q[0], -0.5, 1e-12);
  EXPECT_NEAR(q[1], 0.5, 1e-12);
  EXPECT_NEAR(q[2], -0.5, 1e-12);
  EXPECT_NEAR(q[3], 0.5, 1e-12);
}
