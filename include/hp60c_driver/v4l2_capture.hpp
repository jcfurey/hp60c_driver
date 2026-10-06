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

// Minimal V4L2 streaming capture (mmap buffers) for the HP60C's composite
// 640x642 MJPG mode, plus the camera's UVC controls. Plain kernel uvcvideo: no
// libusb, no vendor library.

#ifndef HP60C_DRIVER__V4L2_CAPTURE_HPP_
#define HP60C_DRIVER__V4L2_CAPTURE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "hp60c_driver/camera_controls.hpp"

namespace hp60c_driver
{

// /dev/videoN of the first capture node whose USB device matches vid:pid, or ""
// if none. A missing node usually means another driver detached uvcvideo from
// the camera (the vendor SDK does this); re-enumerating the USB device fixes it.
std::string find_video_device(std::uint16_t vid, std::uint16_t pid);

// One dequeued buffer. `data` is only valid inside the grab() callback.
struct CapturedFrame
{
  const std::uint8_t * data{nullptr};
  std::size_t size{0};
  // When uvcvideo saw the frame start, on CLOCK_MONOTONIC (refined from the
  // camera's own clock where it sends one), or -1 if the driver gave no
  // monotonic timestamp.
  std::int64_t monotonic_ns{-1};
  // uvcvideo's frame counter. It counts every frame the camera sent, so a gap
  // means frames were lost on the host.
  std::uint32_t sequence{0};
};

class V4l2Capture
{
public:
  V4l2Capture(const std::string & device, int width, int height, unsigned buffer_count = 4);
  ~V4l2Capture();
  V4l2Capture(const V4l2Capture &) = delete;
  V4l2Capture & operator=(const V4l2Capture &) = delete;

  // Wait up to timeout_ms for a buffer; if one arrives, call fn with it and
  // hand the buffer back to the driver. Returns false on timeout.
  bool grab(int timeout_ms, const std::function<void(const CapturedFrame &)> & fn);

  // The device's controls, skipping disabled ones and class headings.
  std::vector<ControlInfo> query_controls() const;
  // Current value of a control. Throws std::system_error on failure.
  std::int64_t get_control(std::uint32_t id) const;
  // Throws std::system_error on failure (e.g. EACCES while an auto mode owns it).
  void set_control(std::uint32_t id, std::int64_t value);

private:
  void cleanup();

  struct Buffer
  {
    void * start{nullptr};
    std::size_t length{0};
  };
  int fd_{-1};
  std::vector<Buffer> buffers_;
  bool streaming_{false};
};

}  // namespace hp60c_driver

#endif  // HP60C_DRIVER__V4L2_CAPTURE_HPP_
