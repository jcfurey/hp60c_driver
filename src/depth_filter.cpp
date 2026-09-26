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

#include "hp60c_driver/depth_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace hp60c_driver
{

DepthFilter::DepthFilter(int width, int height, const DepthFilterParams & params)
: w_(width), h_(height), p_(params),
  avg_(static_cast<std::size_t>(width) * height, 0.0F),
  tmp_(static_cast<std::size_t>(width) * height, 0)
{
}

void DepthFilter::reset()
{
  std::fill(avg_.begin(), avg_.end(), 0.0F);
}

void DepthFilter::apply(const std::uint16_t * in, std::uint16_t * out)
{
  const std::size_t n = static_cast<std::size_t>(w_) * h_;
  // Stage 1 writes to tmp_ when spatial filtering follows, else straight to out.
  std::uint16_t * stage1 = p_.spatial ? tmp_.data() : out;

  if (p_.temporal) {
    const float a = static_cast<float>(p_.alpha);
    const float thr = static_cast<float>(p_.reset_fraction);
    for (std::size_t i = 0; i < n; ++i) {
      const std::uint16_t z = in[i];
      float & m = avg_[i];
      if (z == 0) {
        m = 0.0F;           // no measurement: drop the history, output invalid
        stage1[i] = 0;
        continue;
      }
      const float zf = z;
      if (m == 0.0F || std::fabs(zf - m) > thr * m) {
        m = zf;             // first sample, or a real change: follow it at once
      } else {
        m += a * (zf - m);
      }
      stage1[i] = static_cast<std::uint16_t>(m + 0.5F);
    }
  } else if (stage1 != in) {
    std::memcpy(stage1, in, n * sizeof(std::uint16_t));
  }

  if (!p_.spatial) {
    return;
  }
  // 3x3 median over valid neighbours, only at valid pixels (holes stay holes).
  // Borders use the general routine; the interior runs spatial_row(), which
  // the compiler vectorises.
  for (int u = 0; u < w_; ++u) {
    out[u] = stage1[u] ? median_valid(stage1, u, 0) : 0;
    const std::size_t last = static_cast<std::size_t>(h_ - 1) * w_ + u;
    out[last] = stage1[last] ? median_valid(stage1, u, h_ - 1) : 0;
  }
  for (int v = 1; v < h_ - 1; ++v) {
    const std::size_t row = static_cast<std::size_t>(v) * w_;
    out[row] = stage1[row] ? median_valid(stage1, 0, v) : 0;
    out[row + w_ - 1] = stage1[row + w_ - 1] ? median_valid(stage1, w_ - 1, v) : 0;
    spatial_row(stage1 + row - w_, stage1 + row, stage1 + row + w_, out + row, w_);
  }
}

namespace
{

inline void cswap(std::uint16_t & a, std::uint16_t & b)
{
  const std::uint16_t lo = a < b ? a : b;
  b = a < b ? b : a;
  a = lo;
}

}  // namespace

void DepthFilter::spatial_row(
  const std::uint16_t * a, const std::uint16_t * b, const std::uint16_t * c,
  std::uint16_t * out, int w)
{
  // For each interior pixel, sort its 3x3 window with a 25-comparator network
  // (verified exhaustively via the 0-1 principle). Holes (0) sort to the
  // front, so with z zeros the median of the valid values is element
  // z + (9 - z) / 2, the same one median_valid() picks. Everything is
  // branch-free min/max/select, so the loop vectorises (NEON / SSE).
  for (int u = 1; u < w - 1; ++u) {
    std::uint16_t p0 = a[u - 1], p1 = a[u], p2 = a[u + 1];
    std::uint16_t p3 = b[u - 1], p4 = b[u], p5 = b[u + 1];
    std::uint16_t p6 = c[u - 1], p7 = c[u], p8 = c[u + 1];
    const std::uint16_t centre = p4;
    const int z = (p0 == 0) + (p1 == 0) + (p2 == 0) + (p3 == 0) + (p4 == 0) + (p5 == 0) +
      (p6 == 0) + (p7 == 0) + (p8 == 0);
    cswap(p0, p3); cswap(p1, p7); cswap(p2, p5); cswap(p4, p8);
    cswap(p0, p7); cswap(p2, p4); cswap(p3, p8); cswap(p5, p6);
    cswap(p0, p2); cswap(p1, p3); cswap(p4, p5); cswap(p7, p8);
    cswap(p1, p4); cswap(p3, p6); cswap(p5, p7);
    cswap(p0, p1); cswap(p2, p4); cswap(p3, p5); cswap(p6, p8);
    cswap(p2, p3); cswap(p4, p5); cswap(p6, p7);
    cswap(p1, p2); cswap(p3, p4); cswap(p5, p6);
    // z: 0 -> p4; 1,2 -> p5; 3,4 -> p6; 5,6 -> p7; 7,8 -> p8
    const std::uint16_t med = z < 1 ? p4 : z < 3 ? p5 : z < 5 ? p6 : z < 7 ? p7 : p8;
    out[u] = centre ? med : 0;
  }
}

std::uint16_t DepthFilter::median_valid(const std::uint16_t * img, int u, int v) const
{
  std::array<std::uint16_t, 9> win;
  int k = 0;
  for (int dv = -1; dv <= 1; ++dv) {
    const int vv = v + dv;
    if (vv < 0 || vv >= h_) {
      continue;
    }
    for (int du = -1; du <= 1; ++du) {
      const int uu = u + du;
      if (uu < 0 || uu >= w_) {
        continue;
      }
      const std::uint16_t z = img[static_cast<std::size_t>(vv) * w_ + uu];
      if (z != 0) {
        win[k++] = z;
      }
    }
  }
  std::nth_element(win.begin(), win.begin() + k / 2, win.begin() + k);
  return win[k / 2];
}

}  // namespace hp60c_driver
