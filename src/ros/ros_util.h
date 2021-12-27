#pragma once

#include "inttypes.h"
#include "ros/ros_types.h"

inline builtins__Time MicrosToRos(int64_t t) {
  builtins__Time r;
  r.sec() = t / 1000000;
  r.nsec() = (t % 1000000) * 1000;
  return r;
}