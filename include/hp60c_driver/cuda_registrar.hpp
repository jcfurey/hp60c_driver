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

// Optional CUDA backend for the per-pixel depth work: unpacking the rotated raw
// block (depth_to_mm) and registering it into the colour camera
// (register_to_colour). Same maths as the CPU versions in frame.cpp, in double
// precision, so results match them. Only built when CUDA is available
// (HP60C_WITH_CUDA); callers fall back to the CPU path otherwise.

#ifndef HP60C_DRIVER__CUDA_REGISTRAR_HPP_
#define HP60C_DRIVER__CUDA_REGISTRAR_HPP_

#include <cstdint>

#include "hp60c_driver/frame.hpp"

namespace hp60c_driver
{

class CudaRegistrar
{
public:
  // Throws std::runtime_error if no usable CUDA device is present.
  CudaRegistrar();
  ~CudaRegistrar();
  CudaRegistrar(const CudaRegistrar &) = delete;
  CudaRegistrar & operator=(const CudaRegistrar &) = delete;

  // depth_raw: the kDepthBytes stored block. Either output may be null; each
  // non-null one is a 640x480 buffer that is fully overwritten.
  void process(
    const std::uint8_t * depth_raw, const Calibration & calib,
    std::uint16_t * depth_mm_out, std::uint16_t * aligned_out);

private:
  std::uint8_t * d_raw_{nullptr};
  std::uint16_t * d_depth_{nullptr};
  std::uint32_t * d_zbuf_{nullptr};
  std::uint16_t * d_aligned_{nullptr};
};

}  // namespace hp60c_driver

#endif  // HP60C_DRIVER__CUDA_REGISTRAR_HPP_
