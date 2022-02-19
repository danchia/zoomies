#pragma once

#include <inttypes.h>

#include "ros/builtins.pb.h"

int64_t RosToMicros(const ros::Time& t) {
  return t.sec() * 1000000 + t.nsec() / 1000;
}