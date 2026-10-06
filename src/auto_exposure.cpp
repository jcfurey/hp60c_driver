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

#include "hp60c_driver/auto_exposure.hpp"

#include <algorithm>
#include <cmath>

namespace hp60c_driver
{

namespace
{

constexpr double kMinStepScale = 1.0 / 16;
constexpr double kMaxStepFactor = 4.0;   // per step, either way

}  // namespace

AutoExposure::AutoExposure(const AutoExposureParams & params)
{
  set_params(params);
}

bool AutoExposure::set_params(const AutoExposureParams & params)
{
  p_ = params;
  p_.min_exposure = std::max<std::int64_t>(p_.min_exposure, 1);
  p_.max_exposure = std::max(p_.max_exposure, p_.min_exposure);
  p_.max_gain = std::max(p_.max_gain, p_.min_gain);
  p_.gain_per_x = std::max(p_.gain_per_x, 1e-6);
  step_scale_ = 1.0;
  last_dir_ = 0;
  return clamp_values();
}

void AutoExposure::reset(std::int64_t exposure, std::int64_t gain)
{
  exposure_ = exposure;
  gain_ = gain;
  clamp_values();
  wait_ = 0;
  last_dir_ = 0;
  step_scale_ = 1.0;
}

bool AutoExposure::clamp_values()
{
  const auto e = std::clamp(exposure_, p_.min_exposure, p_.max_exposure);
  const auto g = std::clamp(gain_, p_.min_gain, p_.max_gain);
  const bool changed = e != exposure_ || g != gain_;
  exposure_ = e;
  gain_ = g;
  return changed;
}

double AutoExposure::gain_factor(std::int64_t g) const
{
  return 1.0 + static_cast<double>(g - p_.gain_unity) / p_.gain_per_x;
}

// New exposure for a brightness factor; `factor` is left as what remains.
std::int64_t AutoExposure::scale_exposure(double & factor) const
{
  const auto e = std::clamp<std::int64_t>(
    std::llround(static_cast<double>(exposure_) * factor), p_.min_exposure, p_.max_exposure);
  factor *= static_cast<double>(exposure_) / static_cast<double>(e);
  return e;
}

// New gain for a brightness factor; `factor` is left as what remains.
std::int64_t AutoExposure::scale_gain(double & factor) const
{
  const double from = gain_factor(gain_);
  const double want = std::max(from * factor, gain_factor(p_.min_gain));
  const auto g = std::clamp<std::int64_t>(
    p_.gain_unity + std::llround((want - 1.0) * p_.gain_per_x), p_.min_gain, p_.max_gain);
  factor *= from / gain_factor(g);
  return g;
}

bool AutoExposure::update(double luma)
{
  if (wait_ > 0) {
    --wait_;
    return false;
  }
  const double err = std::max(luma, 1.0) / p_.target;   // > 1: too bright
  if (std::fabs(err - 1.0) <= p_.tolerance) {
    last_dir_ = 0;
    return false;
  }
  const int dir = err < 1.0 ? 1 : -1;
  if (last_dir_ == -dir) {
    step_scale_ = std::max(step_scale_ * 0.5, kMinStepScale);   // overshot
  } else if (last_dir_ == dir) {
    step_scale_ = std::min(step_scale_ * 2.0, 1.0);
  }
  last_dir_ = dir;
  double factor = std::clamp(
    std::pow(1.0 / err, p_.damping * step_scale_), 1.0 / kMaxStepFactor, kMaxStepFactor);

  std::int64_t e = exposure_;
  std::int64_t g = gain_;
  if (dir > 0) {
    e = scale_exposure(factor);
    if (e == p_.max_exposure && factor > 1.0) {
      g = scale_gain(factor);
    }
  } else {
    g = scale_gain(factor);
    if (g == p_.min_gain && factor < 1.0) {
      e = scale_exposure(factor);
    }
  }
  if (e == exposure_ && g == gain_) {
    return false;   // at a limit, or the step is below one camera unit
  }
  exposure_ = e;
  gain_ = g;
  wait_ = p_.settle_frames;
  return true;
}

}  // namespace hp60c_driver
