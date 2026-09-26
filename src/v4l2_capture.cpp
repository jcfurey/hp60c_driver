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

#include "hp60c_driver/v4l2_capture.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <system_error>

namespace hp60c_driver
{

namespace
{

int xioctl(int fd, unsigned long req, void * arg)   // NOLINT(runtime/int): ioctl's type
{
  int r;
  do {
    r = ioctl(fd, req, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

[[noreturn]] void fail(const std::string & what)
{
  throw std::system_error(errno, std::generic_category(), what);
}

std::string read_hex_attr(const std::filesystem::path & p)
{
  std::ifstream f(p);
  std::string s;
  f >> s;
  return s;
}

bool is_capture_node(const std::string & dev)
{
  const int fd = ::open(dev.c_str(), O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    return false;
  }
  v4l2_capability cap{};
  const bool ok = xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 &&
    (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) && (cap.device_caps & V4L2_CAP_STREAMING);
  ::close(fd);
  return ok;
}

}  // namespace

std::string find_video_device(std::uint16_t vid, std::uint16_t pid)
{
  namespace fs = std::filesystem;
  char want_vid[5], want_pid[5];
  std::snprintf(want_vid, sizeof want_vid, "%04x", vid);
  std::snprintf(want_pid, sizeof want_pid, "%04x", pid);

  const fs::path root{"/sys/class/video4linux"};
  std::error_code ec;
  std::vector<std::string> names;
  for (const auto & e : fs::directory_iterator(root, ec)) {
    names.push_back(e.path().filename().string());
  }
  std::sort(names.begin(), names.end());   // video0 before video1
  for (const auto & name : names) {
    // .../videoN/device -> the USB interface; its parent holds idVendor/idProduct.
    const fs::path usb_if = fs::canonical(root / name / "device", ec);
    if (ec) {
      continue;
    }
    const fs::path usb_dev = usb_if.parent_path();
    if (read_hex_attr(usb_dev / "idVendor") == want_vid &&
      read_hex_attr(usb_dev / "idProduct") == want_pid)
    {
      const std::string dev = "/dev/" + name;
      if (is_capture_node(dev)) {
        return dev;
      }
    }
  }
  return "";
}

V4l2Capture::V4l2Capture(
  const std::string & device, int width, int height, unsigned buffer_count)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    fail("open " + device);
  }
  try {
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<__u32>(width);
    fmt.fmt.pix.height = static_cast<__u32>(height);
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
      fail("VIDIOC_S_FMT");
    }
    if (fmt.fmt.pix.width != static_cast<__u32>(width) ||
      fmt.fmt.pix.height != static_cast<__u32>(height) ||
      fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
    {
      errno = EINVAL;
      fail("camera refused MJPG " + std::to_string(width) + "x" + std::to_string(height));
    }

    v4l2_requestbuffers req{};
    req.count = buffer_count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
      fail("VIDIOC_REQBUFS");
    }
    for (unsigned i = 0; i < req.count; ++i) {
      v4l2_buffer b{};
      b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      b.memory = V4L2_MEMORY_MMAP;
      b.index = i;
      if (xioctl(fd_, VIDIOC_QUERYBUF, &b) < 0) {
        fail("VIDIOC_QUERYBUF");
      }
      void * p = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, b.m.offset);
      if (p == MAP_FAILED) {
        fail("mmap");
      }
      buffers_.push_back({p, b.length});
      if (xioctl(fd_, VIDIOC_QBUF, &b) < 0) {
        fail("VIDIOC_QBUF");
      }
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      fail("VIDIOC_STREAMON");
    }
    streaming_ = true;
  } catch (...) {
    cleanup();   // the destructor does not run when a constructor throws
    throw;
  }
}

V4l2Capture::~V4l2Capture()
{
  cleanup();
}

void V4l2Capture::cleanup()
{
  if (fd_ < 0) {
    return;
  }
  if (streaming_) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;
  }
  for (auto & b : buffers_) {
    munmap(b.start, b.length);
  }
  buffers_.clear();
  ::close(fd_);
  fd_ = -1;
}

bool V4l2Capture::grab(
  int timeout_ms, const std::function<void(const std::uint8_t *, std::size_t)> & fn)
{
  pollfd pfd{fd_, POLLIN, 0};
  const int r = poll(&pfd, 1, timeout_ms);
  if (r < 0) {
    if (errno == EINTR) {
      return false;
    }
    fail("poll");
  }
  if (r == 0) {
    return false;
  }
  v4l2_buffer b{};
  b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  b.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_DQBUF, &b) < 0) {
    if (errno == EAGAIN) {
      return false;
    }
    fail("VIDIOC_DQBUF");
  }
  struct Requeue
  {
    int fd;
    v4l2_buffer * b;
    ~Requeue() {xioctl(fd, VIDIOC_QBUF, b);}
  } requeue{fd_, &b};
  fn(static_cast<const std::uint8_t *>(buffers_[b.index].start), b.bytesused);
  return true;
}

}  // namespace hp60c_driver
