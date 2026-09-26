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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "hp60c_driver/depth_filter.hpp"

namespace hp = hp60c_driver;

namespace
{

constexpr int W = 64, H = 48;

hp::DepthFilterParams only_temporal()
{
  hp::DepthFilterParams p;
  p.spatial = false;
  return p;
}

hp::DepthFilterParams only_spatial()
{
  hp::DepthFilterParams p;
  p.temporal = false;
  return p;
}

double stddev(const std::vector<double> & x)
{
  double m = 0, v = 0;
  for (double a : x) {m += a;}
  m /= x.size();
  for (double a : x) {v += (a - m) * (a - m);}
  return std::sqrt(v / x.size());
}

}  // namespace

TEST(DepthFilter, TemporalReducesNoiseOnStaticScene)
{
  hp::DepthFilter f(W, H, only_temporal());
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, 20.0);   // ~ the camera's jitter
  std::vector<std::uint16_t> in(W * H), out(W * H);
  std::vector<double> raw_px, filt_px;
  for (int t = 0; t < 200; ++t) {
    for (auto & z : in) {
      z = static_cast<std::uint16_t>(std::lround(3000 + noise(rng)));
    }
    f.apply(in.data(), out.data());
    if (t >= 20) {   // after settling
      raw_px.push_back(in[100]);
      filt_px.push_back(out[100]);
    }
  }
  EXPECT_LT(stddev(filt_px), 0.6 * stddev(raw_px));
}

TEST(DepthFilter, TemporalFollowsARealChangeImmediately)
{
  hp::DepthFilter f(W, H, only_temporal());
  std::vector<std::uint16_t> in(W * H, 2000), out(W * H);
  for (int t = 0; t < 10; ++t) {
    f.apply(in.data(), out.data());
  }
  std::fill(in.begin(), in.end(), 1000);   // an object steps in: 50% change
  f.apply(in.data(), out.data());
  EXPECT_EQ(out[0], 1000);                 // no smearing from the old 2000
}

TEST(DepthFilter, NeverFillsHoles)
{
  hp::DepthFilter f(W, H, hp::DepthFilterParams{});
  std::vector<std::uint16_t> in(W * H, 1500), out(W * H);
  f.apply(in.data(), out.data());
  in[10 * W + 10] = 0;                     // a hole, surrounded by valid depth
  f.apply(in.data(), out.data());
  EXPECT_EQ(out[10 * W + 10], 0);
  in[10 * W + 10] = 1500;                  // and it comes back without ghosting
  f.apply(in.data(), out.data());
  EXPECT_EQ(out[10 * W + 10], 1500);
}

TEST(DepthFilter, SpatialRemovesSaltButKeepsEdges)
{
  hp::DepthFilter f(W, H, only_spatial());
  std::vector<std::uint16_t> in(W * H), out(W * H);
  for (int v = 0; v < H; ++v) {
    for (int u = 0; u < W; ++u) {
      in[v * W + u] = u < W / 2 ? 1000 : 3000;   // a vertical depth edge
    }
  }
  in[20 * W + 10] = 2500;                  // salt pixel in the near region
  f.apply(in.data(), out.data());
  EXPECT_EQ(out[20 * W + 10], 1000);
  for (int v = 1; v < H - 1; ++v) {        // edge columns keep their sides
    EXPECT_EQ(out[v * W + W / 2 - 1], 1000);
    EXPECT_EQ(out[v * W + W / 2], 3000);
  }
}

TEST(DepthFilter, ResetDropsHistory)
{
  hp::DepthFilter f(W, H, only_temporal());
  std::vector<std::uint16_t> in(W * H, 2000), out(W * H);
  f.apply(in.data(), out.data());
  std::fill(in.begin(), in.end(), 2040);   // within the reset band: would be averaged
  f.reset();
  f.apply(in.data(), out.data());
  EXPECT_EQ(out[0], 2040);
}

TEST(DepthFilter, SpatialMatchesBruteForceMedianWithHoles)
{
  // The interior uses a vectorised sorting network; borders a general path.
  // Both must equal the plain definition: median (element k/2 of the sorted
  // k valid values) of each valid pixel's 3x3 neighbourhood.
  hp::DepthFilter f(W, H, only_spatial());
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> val(1, 4000), hole(0, 9);
  std::vector<std::uint16_t> in(W * H), out(W * H);
  for (int round = 0; round < 20; ++round) {
    for (auto & z : in) {
      z = hole(rng) < 4 ? 0 : static_cast<std::uint16_t>(val(rng));   // ~40% holes
    }
    f.apply(in.data(), out.data());
    for (int v = 0; v < H; ++v) {
      for (int u = 0; u < W; ++u) {
        std::uint16_t expect = 0;
        if (in[v * W + u] != 0) {
          std::vector<std::uint16_t> nb;
          for (int dv = -1; dv <= 1; ++dv) {
            for (int du = -1; du <= 1; ++du) {
              const int vv = v + dv, uu = u + du;
              if (vv >= 0 && vv < H && uu >= 0 && uu < W && in[vv * W + uu] != 0) {
                nb.push_back(in[vv * W + uu]);
              }
            }
          }
          std::sort(nb.begin(), nb.end());
          expect = nb[nb.size() / 2];
        }
        ASSERT_EQ(out[v * W + u], expect) << "u=" << u << " v=" << v << " round=" << round;
      }
    }
  }
}
