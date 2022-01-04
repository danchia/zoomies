#include "hw/js.h"

#include <fcntl.h>
#include <linux/joystick.h>
#include <stdlib.h>
#include <unistd.h>

#include "spdlog/spdlog.h"

JS::JS() {
  fd_ = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
  if (fd_ < 0) {
    spdlog::warn("error joystick {}", fd_);
    fd_ = 0;
  }
}

JS::~JS() {
  if (fd_) {
    close(fd_);
  }
}

JS::State JS::Poll() {
  State s;
  if (fd_ <= 0) return s;

  struct js_event e;
  while (read(fd_, &e, sizeof(e)) == sizeof(e)) {
    if (e.type & JS_EVENT_AXIS) {
      if (e.number == 4) {
        accel_ = (e.value / 32767.0f) / 2.0f + 0.5f;
      }
      if (e.number == 0) {
        steer_ = e.value / -32767.0f;
        bool neg = steer_ < 0.0f;
        steer_ = pow(abs(steer_), 1.5);
        if (neg) steer_ = -steer_;
      }
    }
    if (e.type & JS_EVENT_BUTTON) {
      if (e.number == 0) {
        a_btn_ = e.value > 0;
      }
      if (e.number == 4) {
        y_btn_ = e.value > 0;
      }
    }
  }

  s.accel = accel_;
  s.steer = steer_;
  s.a_btn = a_btn_;
  s.y_btn = y_btn_;
  return s;
}