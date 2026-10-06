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

// The software auto exposure against simulated cameras: brightness =
// scene * exposure * gain, saturating at 255, with control latency, noise and
// either a linear or a dB gain response (the controller only models linear).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <random>
#include <utility>

#include "hp60c_driver/auto_exposure.hpp"

namespace hp = hp60c_driver;

namespace
{

struct Camera
{
  double scene{1.0};        // luma per exposure unit at 1x gain
  bool db_gain{false};      // gain units are dB rather than linear steps
  int latency{2};           // frames before a written setting shows
  double noise{0.0};        // relative, uniform
  std::deque<std::pair<std::int64_t, std::int64_t>> pipe;
  std::mt19937 rng{42};

  double amplification(std::int64_t g) const
  {
    return db_gain ? std::pow(10.0, g / 20.0) : 1.0 + g / 16.0;
  }

  // One frame's mean luma for the settings written so far.
  double frame(std::int64_t exposure, std::int64_t gain)
  {
    pipe.emplace_back(exposure, gain);
    while (static_cast<int>(pipe.size()) > latency + 1) {
      pipe.pop_front();
    }
    const auto [e, g] = pipe.front();
    std::uniform_real_distribution<double> u(-noise, noise);
    return std::min(255.0, scene * e * amplification(g) * (1.0 + u(rng)));
  }
};

hp::AutoExposureParams params()
{
  hp::AutoExposureParams p;
  p.min_exposure = 1;
  p.max_exposure = 300;   // 30 ms in 100 us units
  p.min_gain = 0;
  p.max_gain = 100;
  p.gain_unity = 0;
  p.gain_per_x = 100.0 / 7;
  return p;
}

struct Outcome
{
  double luma{0};
  int changes_late{0};      // changes in the last quarter of the run
  bool limits_ok{true};
  bool exposure_first{true};
};

Outcome simulate(hp::AutoExposure & ae, Camera & cam, int frames)
{
  Outcome r;
  const auto & p = ae.params();
  for (int i = 0; i < frames; ++i) {
    r.luma = cam.frame(ae.exposure(), ae.gain());
    if (ae.update(r.luma) && i >= frames * 3 / 4) {
      ++r.changes_late;
    }
    r.limits_ok = r.limits_ok && ae.exposure() >= p.min_exposure &&
      ae.exposure() <= p.max_exposure && ae.gain() >= p.min_gain && ae.gain() <= p.max_gain;
    if (ae.gain() > p.min_gain && ae.exposure() < p.max_exposure) {
      r.exposure_first = false;
    }
  }
  return r;
}

double settled_luma(hp::AutoExposure & ae, Camera & cam)
{
  cam.noise = 0.0;
  for (int i = 0; i < cam.latency + 1; ++i) {
    cam.frame(ae.exposure(), ae.gain());
  }
  return cam.frame(ae.exposure(), ae.gain());
}

}  // namespace

TEST(AutoExposure, BrightSceneShortensExposureAndKeepsGainLow)
{
  hp::AutoExposure ae(params());
  ae.reset(200, 50);
  Camera cam;
  cam.scene = 5.0;   // target needs exposure ~22
  const Outcome r = simulate(ae, cam, 200);
  EXPECT_TRUE(r.limits_ok);
  EXPECT_EQ(ae.gain(), 0);
  EXPECT_NEAR(settled_luma(ae, cam), 110.0, 11.0);
  EXPECT_EQ(r.changes_late, 0);
}

TEST(AutoExposure, DarkSceneUsesFullExposureBeforeGain)
{
  hp::AutoExposure ae(params());
  ae.reset(10, 0);
  Camera cam;
  cam.scene = 0.15;   // 300 units give 45: needs ~2.4x gain
  const Outcome r = simulate(ae, cam, 300);
  EXPECT_TRUE(r.limits_ok);
  EXPECT_TRUE(r.exposure_first);
  EXPECT_EQ(ae.exposure(), 300);
  EXPECT_GT(ae.gain(), 0);
  EXPECT_NEAR(settled_luma(ae, cam), 110.0, 11.0);
  EXPECT_EQ(r.changes_late, 0);
}

TEST(AutoExposure, SettlesWithDbGainItDoesNotModel)
{
  auto p = params();
  p.max_gain = 48;   // dB
  p.gain_per_x = 48.0 / 7;
  hp::AutoExposure ae(p);
  ae.reset(300, 0);
  Camera cam;
  cam.db_gain = true;
  cam.scene = 0.02;   // 300 units give 6: needs ~25 dB
  const Outcome r = simulate(ae, cam, 600);
  EXPECT_TRUE(r.limits_ok);
  EXPECT_NEAR(settled_luma(ae, cam), 110.0, 11.0);
  EXPECT_EQ(r.changes_late, 0);
}

TEST(AutoExposure, IgnoresNoiseInsideTolerance)
{
  hp::AutoExposure ae(params());
  Camera cam;
  cam.scene = 1.0;
  ae.reset(110, 0);   // already on target
  cam.noise = 0.05;
  const Outcome r = simulate(ae, cam, 400);
  EXPECT_EQ(ae.exposure(), 110);
  EXPECT_EQ(r.changes_late, 0);
}

TEST(AutoExposure, TracksASceneChange)
{
  hp::AutoExposure ae(params());
  ae.reset(100, 0);
  Camera cam;
  cam.scene = 1.0;
  simulate(ae, cam, 100);
  cam.scene = 0.25;   // lights dimmed 4x
  const Outcome r = simulate(ae, cam, 200);
  EXPECT_TRUE(r.limits_ok);
  EXPECT_NEAR(settled_luma(ae, cam), 110.0, 11.0);
  EXPECT_EQ(r.changes_late, 0);
}

TEST(AutoExposure, FixedGainWhenMinEqualsMax)
{
  auto p = params();
  p.min_gain = p.max_gain = 20;
  hp::AutoExposure ae(p);
  ae.reset(10, 0);
  EXPECT_EQ(ae.gain(), 20);   // clamped into the limits
  Camera cam;
  cam.scene = 0.01;   // far too dark even at full exposure
  const Outcome r = simulate(ae, cam, 200);
  EXPECT_TRUE(r.limits_ok);
  EXPECT_EQ(ae.gain(), 20);
  EXPECT_EQ(ae.exposure(), 300);
}

TEST(AutoExposure, WaitsForTheCameraAfterAChange)
{
  auto p = params();
  p.settle_frames = 3;
  hp::AutoExposure ae(p);
  ae.reset(100, 0);
  ASSERT_TRUE(ae.update(20.0));   // too dark: changes at once
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(ae.update(20.0));
  }
  EXPECT_TRUE(ae.update(20.0));
}

TEST(AutoExposure, SetParamsClampsCurrentValues)
{
  hp::AutoExposure ae(params());
  ae.reset(250, 80);
  auto p = params();
  p.max_exposure = 100;
  p.max_gain = 40;
  EXPECT_TRUE(ae.set_params(p));
  EXPECT_EQ(ae.exposure(), 100);
  EXPECT_EQ(ae.gain(), 40);
  EXPECT_FALSE(ae.set_params(p));
}
