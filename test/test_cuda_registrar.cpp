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

// The CUDA backend must reproduce the CPU path (frame.cpp). Built only with
// CUDA; skipped at runtime if no GPU is present.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <random>
#include <vector>

#include "hp60c_driver/cuda_registrar.hpp"
#include "hp60c_driver/frame.hpp"

namespace hp = hp60c_driver;

TEST(CudaRegistrar, MatchesCpuPath)
{
  std::unique_ptr<hp::CudaRegistrar> cuda;
  try {
    cuda = std::make_unique<hp::CudaRegistrar>();
  } catch (const std::exception & e) {
    GTEST_SKIP() << e.what();
  }
  // Realistic calibration (values of the unit this was developed on) and a
  // random-but-smooth depth field with holes, stored in the camera's layout.
  hp::Calibration c;
  c.depth = {443.635, 443.320, 321.799, 238.200};
  c.colour = {590.229, 589.842, 332.799, 232.275};
  c.R = {0.999155, -0.040317, -0.0080155, 0.0403505, 0.999177, 0.0040715,
    0.0078448, -0.0043915, 0.99996};
  c.t_mm = {-12.0357, -0.253644, -0.0430599};
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> noise(0, 15), hole(0, 9);
  std::vector<std::uint8_t> raw(hp::kDepthBytes);
  for (int row = 0; row < 640; ++row) {
    for (int col = 0; col < 480; ++col) {
      int mm = 600 + (row * 5 + col * 3) % 3500;
      if (hole(rng) == 0) {
        mm = 0;
      }
      const auto v = static_cast<std::uint16_t>((mm << 4) | noise(rng));
      const std::size_t s = static_cast<std::size_t>(row) * 480 + col;
      raw[2 * s] = static_cast<std::uint8_t>(v & 0xFF);
      raw[2 * s + 1] = static_cast<std::uint8_t>(v >> 8);
    }
  }
  const std::size_t n = static_cast<std::size_t>(hp::kWidth) * hp::kHeight;
  std::vector<std::uint16_t> cpu_depth(n), cpu_aligned(n), gpu_depth(n), gpu_aligned(n);
  hp::depth_to_mm(raw.data(), cpu_depth.data());
  hp::register_to_colour(cpu_depth.data(), c, cpu_aligned.data());
  cuda->process(raw.data(), c, gpu_depth.data(), gpu_aligned.data());

  EXPECT_EQ(gpu_depth, cpu_depth);   // pure integer work: must be identical
  // Registration is floating point; FMA contraction may differ between nvcc
  // and the host compiler, flipping a rounding at a footprint edge now and
  // then. Allow a vanishing fraction of such pixels, and never more than 1 mm.
  std::size_t differ = 0, filled = 0;
  for (std::size_t i = 0; i < n; ++i) {
    filled += cpu_aligned[i] != 0;
    if (gpu_aligned[i] != cpu_aligned[i]) {
      ++differ;
      if (gpu_aligned[i] != 0 && cpu_aligned[i] != 0) {
        EXPECT_LE(std::abs(gpu_aligned[i] - cpu_aligned[i]), 1) << "pixel " << i;
      }
    }
  }
  EXPECT_GT(filled, n / 2);
  EXPECT_LT(differ, n / 1000) << differ << " of " << n << " pixels differ";

  // Only-depth and only-aligned requests.
  std::vector<std::uint16_t> only(n, 7);
  cuda->process(raw.data(), c, only.data(), nullptr);
  EXPECT_EQ(only, cpu_depth);
  cuda->process(raw.data(), c, nullptr, only.data());
  EXPECT_EQ(only, gpu_aligned);
}
