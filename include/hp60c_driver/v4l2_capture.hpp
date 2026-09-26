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
// 640x642 MJPG mode. Plain kernel uvcvideo: no libusb, no vendor library.

#ifndef HP60C_DRIVER__V4L2_CAPTURE_HPP_
#define HP60C_DRIVER__V4L2_CAPTURE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace hp60c_driver
{

// /dev/videoN of the first capture node whose USB device matches vid:pid, or ""
// if none. A missing node usually means another driver detached uvcvideo from
// the camera (the vendor SDK does this); re-enumerating the USB device fixes it.
std::string find_video_device(std::uint16_t vid, std::uint16_t pid);

class V4l2Capture
{
public:
  V4l2Capture(const std::string & device, int width, int height, unsigned buffer_count = 4);
  ~V4l2Capture();
  V4l2Capture(const V4l2Capture &) = delete;
  V4l2Capture & operator=(const V4l2Capture &) = delete;

  // Wait up to timeout_ms for a buffer; if one arrives, call fn(data, bytesused)
  // and hand the buffer back to the driver. Returns false on timeout.
  bool grab(int timeout_ms, const std::function<void(const std::uint8_t *, std::size_t)> & fn);

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
