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

#include "hp60c_driver/cuda_registrar.hpp"

#include <cuda_runtime.h>

#include <stdexcept>
#include <string>

namespace hp60c_driver
{

namespace
{

void check(cudaError_t e, const char * what)
{
  if (e != cudaSuccess) {
    throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
  }
}

constexpr int kPixels = kWidth * kHeight;
constexpr std::uint32_t kEmpty = 0xFFFFFFFFu;

// upright (r, c) = stored (639 - c, r) >> 4; stored rows are 480 wide.
__global__ void unpack_kernel(const std::uint8_t * raw, std::uint16_t * out)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= kPixels) {
    return;
  }
  const int r = i / kWidth;
  const int c = i % kWidth;
  const int s = (kWidth - 1 - c) * kHeight + r;
  const std::uint16_t v = static_cast<std::uint16_t>(raw[2 * s] | (raw[2 * s + 1] << 8));
  out[i] = static_cast<std::uint16_t>(v >> 4);
}

struct Params
{
  double R[9];
  double t[3];
  double dfx, dfy, dcx, dcy;
  double cfx, cfy, ccx, ccy;
};

// Mirrors register_to_colour() in frame.cpp: splat each depth pixel's footprint,
// nearest surface wins (atomicMin on a 32-bit z-buffer).
__global__ void splat_kernel(const std::uint16_t * depth, Params p, std::uint32_t * zbuf)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= kPixels) {
    return;
  }
  const std::uint16_t z = depth[i];
  if (z == 0) {
    return;
  }
  const int v = i / kWidth;
  const int u = i % kWidth;
  const double Z = z;
  const double rx = (u - p.dcx) / p.dfx;
  const double ry = (v - p.dcy) / p.dfy;
  const double zc = (p.R[6] * rx + (p.R[7] * ry + p.R[8])) * Z + p.t[2];
  if (zc < 0.5 || zc >= 65535.5) {
    return;
  }
  const double inv = 1.0 / zc;
  const double xc = (p.R[0] * rx + (p.R[1] * ry + p.R[2])) * Z + p.t[0];
  const double yc = (p.R[3] * rx + (p.R[4] * ry + p.R[5])) * Z + p.t[1];
  const double uf = xc * inv * p.cfx + p.ccx;
  const double vf = yc * inv * p.cfy + p.ccy;
  const double hx = 0.5 * p.cfx / p.dfx * Z * inv;
  const double hy = 0.5 * p.cfy / p.dfy * Z * inv;
  const double c0 = fmax(ceil(uf - hx), 0.0);
  const double c1 = fmin(floor(uf + hx), kWidth - 1.0);
  const double r0 = fmax(ceil(vf - hy), 0.0);
  const double r1 = fmin(floor(vf + hy), kHeight - 1.0);
  if (!(c0 <= c1 && r0 <= r1)) {
    return;
  }
  const auto zv = static_cast<std::uint32_t>(zc + 0.5);
  for (int r = static_cast<int>(r0); r <= static_cast<int>(r1); ++r) {
    for (int c = static_cast<int>(c0); c <= static_cast<int>(c1); ++c) {
      atomicMin(&zbuf[r * kWidth + c], zv);
    }
  }
}

__global__ void finalize_kernel(const std::uint32_t * zbuf, std::uint16_t * out)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < kPixels) {
    out[i] = zbuf[i] == kEmpty ? 0 : static_cast<std::uint16_t>(zbuf[i]);
  }
}

constexpr int kBlock = 256;
constexpr int kGrid = (kPixels + kBlock - 1) / kBlock;

}  // namespace

CudaRegistrar::CudaRegistrar()
{
  int n = 0;
  if (cudaGetDeviceCount(&n) != cudaSuccess || n == 0) {
    throw std::runtime_error("no CUDA device");
  }
  try {
    check(cudaMalloc(&d_raw_, kDepthBytes), "cudaMalloc raw");
    check(cudaMalloc(&d_depth_, kPixels * sizeof(std::uint16_t)), "cudaMalloc depth");
    check(cudaMalloc(&d_zbuf_, kPixels * sizeof(std::uint32_t)), "cudaMalloc zbuf");
    check(cudaMalloc(&d_aligned_, kPixels * sizeof(std::uint16_t)), "cudaMalloc aligned");
  } catch (...) {
    cudaFree(d_raw_);
    cudaFree(d_depth_);
    cudaFree(d_zbuf_);
    cudaFree(d_aligned_);
    throw;
  }
}

CudaRegistrar::~CudaRegistrar()
{
  cudaFree(d_raw_);
  cudaFree(d_depth_);
  cudaFree(d_zbuf_);
  cudaFree(d_aligned_);
}

void CudaRegistrar::process(
  const std::uint8_t * depth_raw, const Calibration & calib,
  std::uint16_t * depth_mm_out, std::uint16_t * aligned_out)
{
  if (!depth_mm_out && !aligned_out) {
    return;
  }
  check(cudaMemcpy(d_raw_, depth_raw, kDepthBytes, cudaMemcpyHostToDevice), "upload");
  unpack_kernel<<<kGrid, kBlock>>>(d_raw_, d_depth_);
  check(cudaGetLastError(), "unpack_kernel");
  if (aligned_out) {
    Params p{};
    for (int k = 0; k < 9; ++k) {
      p.R[k] = calib.R[k];
    }
    for (int k = 0; k < 3; ++k) {
      p.t[k] = calib.t_mm[k];
    }
    p.dfx = calib.depth.fx;
    p.dfy = calib.depth.fy;
    p.dcx = calib.depth.cx;
    p.dcy = calib.depth.cy;
    p.cfx = calib.colour.fx;
    p.cfy = calib.colour.fy;
    p.ccx = calib.colour.cx;
    p.ccy = calib.colour.cy;
    check(cudaMemset(d_zbuf_, 0xFF, kPixels * sizeof(std::uint32_t)), "clear zbuf");
    splat_kernel<<<kGrid, kBlock>>>(d_depth_, p, d_zbuf_);
    check(cudaGetLastError(), "splat_kernel");
    finalize_kernel<<<kGrid, kBlock>>>(d_zbuf_, d_aligned_);
    check(cudaGetLastError(), "finalize_kernel");
    check(
      cudaMemcpy(
        aligned_out, d_aligned_, kPixels * sizeof(std::uint16_t),
        cudaMemcpyDeviceToHost), "download aligned");
  }
  if (depth_mm_out) {
    check(
      cudaMemcpy(
        depth_mm_out, d_depth_, kPixels * sizeof(std::uint16_t),
        cudaMemcpyDeviceToHost), "download depth");
  }
}

}  // namespace hp60c_driver
