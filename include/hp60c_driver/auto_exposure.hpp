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

// Driver-side auto exposure and auto gain (color.exposure_mode: software).
// UVC has no auto-gain or ISO control, and a camera's own auto exposure can't
// be bounded, so this closes the loop on the image itself: each frame's mean
// brightness drives the camera's manual exposure and gain controls. ROS-free.
//
// Exposure is preferred over gain, since gain amplifies noise: brightening
// raises exposure to its limit before touching gain; darkening lowers gain to
// its minimum before shortening exposure. Steps correct a fixed fraction of
// the error in log space, so they are proportional to how far off the image
// is. Gain's effect differs between cameras (linear or dB), so it is only a
// model; if a step overshoots, the step size halves until the loop settles.

#ifndef HP60C_DRIVER__AUTO_EXPOSURE_HPP_
#define HP60C_DRIVER__AUTO_EXPOSURE_HPP_

#include <cstdint>

namespace hp60c_driver
{

struct AutoExposureParams
{
  double target{110.0};      // wanted mean brightness (luma), 0-255
  double tolerance{0.1};     // no change while within target * (1 +- tolerance)
  double damping{0.6};       // fraction of the log error corrected per step
  int settle_frames{3};      // frames ignored after a change: UVC controls lag
  // Limits, in camera units. A stage whose min equals its max stays fixed, so
  // min_gain == max_gain turns auto gain off.
  std::int64_t min_exposure{1};
  std::int64_t max_exposure{1};
  std::int64_t min_gain{0};
  std::int64_t max_gain{0};
  // Gain model: gain g amplifies by 1 + (g - gain_unity) / gain_per_x.
  std::int64_t gain_unity{0};   // the camera's lowest gain
  double gain_per_x{1.0};       // camera units per 1x of extra amplification
};

class AutoExposure
{
public:
  explicit AutoExposure(const AutoExposureParams & params = {});

  // New parameters. Current values are clamped into the new limits; returns
  // true if that changed them.
  bool set_params(const AutoExposureParams & params);
  const AutoExposureParams & params() const {return p_;}

  // Start from the camera's current values (clamped to the limits).
  void reset(std::int64_t exposure, std::int64_t gain);

  // Feed one frame's mean luma (0-255). Returns true if exposure() or gain()
  // changed and should be written to the camera.
  bool update(double luma);

  std::int64_t exposure() const {return exposure_;}
  std::int64_t gain() const {return gain_;}

private:
  double gain_factor(std::int64_t g) const;
  std::int64_t scale_exposure(double & factor) const;
  std::int64_t scale_gain(double & factor) const;
  bool clamp_values();

  AutoExposureParams p_;
  std::int64_t exposure_{1};
  std::int64_t gain_{0};
  int wait_{0};               // frames left to settle
  int last_dir_{0};           // +1 brightened, -1 darkened, 0 in tolerance
  double step_scale_{1.0};    // shrinks after an overshoot
};

}  // namespace hp60c_driver

#endif  // HP60C_DRIVER__AUTO_EXPOSURE_HPP_
