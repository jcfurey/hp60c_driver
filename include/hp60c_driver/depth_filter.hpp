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

// Depth noise filtering, ROS-free. Raw HP60C depth jitters ~21 mm frame to
// frame on a static scene (docs/FRAME_FORMAT.md); this smooths it without
// inventing data: invalid (0) pixels stay invalid, and holes are never filled.
//
//   1. temporal: per-pixel exponential moving average, reset whenever the new
//      value differs from the average by more than a relative threshold, so a
//      real change (motion, an edge sweeping past) passes through at once
//      instead of smearing;
//   2. spatial: 3x3 median over the valid neighbours of each valid pixel.

#ifndef HP60C_DRIVER__DEPTH_FILTER_HPP_
#define HP60C_DRIVER__DEPTH_FILTER_HPP_

#include <cstdint>
#include <vector>

namespace hp60c_driver
{

struct DepthFilterParams
{
  bool temporal{true};
  double alpha{0.4};          // weight of the new sample in the moving average
  double reset_fraction{0.03};  // reset if |new - avg| > fraction * avg
  bool spatial{true};
};

class DepthFilter
{
public:
  DepthFilter(int width, int height, const DepthFilterParams & params);

  // in/out: width x height uint16 mm, 0 = no measurement. May not alias.
  void apply(const std::uint16_t * in, std::uint16_t * out);

  // Forget the temporal history (e.g. after a gap in the stream).
  void reset();

private:
  std::uint16_t median_valid(const std::uint16_t * img, int u, int v) const;
  static void spatial_row(
    const std::uint16_t * a, const std::uint16_t * b, const std::uint16_t * c,
    std::uint16_t * out, int w);

  int w_, h_;
  DepthFilterParams p_;
  std::vector<float> avg_;             // 0 = no history
  std::vector<std::uint16_t> tmp_;
};

}  // namespace hp60c_driver

#endif  // HP60C_DRIVER__DEPTH_FILTER_HPP_
