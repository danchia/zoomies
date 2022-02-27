#pragma once

#include <inttypes.h>

#include "ros/builtins.pb.h"
#include "ros/geometry_msgs/Quaternion.pb.h"

inline int64_t RosToMicros(const ros::Time& t) {
  return t.sec() * 1000000 + t.nsec() / 1000;
}

inline ros::geometry_msgs::Quaternion HeadingToQuat(float theta) {
  // Rotation about z-axis.
  ros::geometry_msgs::Quaternion q;
  q.set_w(std::cos(0.5 * theta));
  q.set_x(0);
  q.set_y(0);
  q.set_z(std::sin(0.5 * theta));
  return q;
}

inline ros::Time MicrosToRos(int64_t t) {
  ros::Time r;
  r.set_sec(t / 1000000);
  r.set_nsec((t % 1000000) * 1000);
  return r;
}