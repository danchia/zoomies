#include "hw/js.h"

#include <fcntl.h>
#include <linux/joystick.h>
#include <stdlib.h>
#include <unistd.h>

#include "spdlog/spdlog.h"

JS::JS() {
  fd_ = open("/dev/input/js0", O_RDONLY);
  if (fd_ < 0) {
    spdlog::warn("error joystick {}", fd_);
    fd_ = 0;
  }

  reader_ = std::thread([this] { ReadLoop(); });
}

JS::~JS() {
  mu_.lock();
  done_.store(true, std::memory_order_relaxed);
  mu_.unlock();
  reader_.join();
  if (fd_) {
    close(fd_);
  }
}

void JS::ReadLoop() {
  State s;
  if (fd_ <= 0) return;

  struct js_event e;
  while (read(fd_, &e, sizeof(e)) == sizeof(e) ||
         done_.load(std::memory_order_relaxed)) {
    if (done_.load(std::memory_order_relaxed)) return;

    if (e.type & JS_EVENT_AXIS) {
      if (e.number == 4) {
        s.accel = (e.value / 32767.0f) / 2.0f + 0.5f;
      }
      if (e.number == 0) {
        s.steer = e.value / -32767.0f;
        bool neg = s.steer < 0.0f;
        s.steer = pow(abs(s.steer), 1.5);
        if (neg) s.steer = -s.steer;
      }
    }
    if (e.type & JS_EVENT_BUTTON) {
      if (e.number == 0) {
        s.a_btn = e.value > 0;
      }
      if (e.number == 4) {
        s.y_btn = e.value > 0;
      }
    }

    std::lock_guard l(mu_);
    s_ = s;
  }
}

JS::State JS::Poll() {
  State s;
  if (fd_ <= 0) return s;

  {
    std::lock_guard l(mu_);
    s = s_;
  }

  return s;
}