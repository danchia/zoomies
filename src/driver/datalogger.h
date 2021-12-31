#pragma once

#include <inttypes.h>

#include <Eigen/Dense>
#include <mutex>
#include <string_view>

#include "ros/ros_writer.h"
#include "track/track.h"

class Datalogger {
 public:
  Datalogger(std::string_view path);

  void LogVideoFrame(int64_t t_us, int width, int height, uint8_t* data,
                     int len);
  void LogIMU(int64_t t_us, const Eigen::Vector3f& linear_accel,
              const Eigen::Vector3f& angular_vel);
  void LogDesiredTwist(int64_t t_us, float linear_velocity,
                       float angular_velocity);
  void LogActualTwist(int64_t t_us, float linear_velocity,
                      float angular_velocity);
  void LogGlobalPose(int64_t t_us, float x, float y, float theta);
  void LogEscSteer(int64_t t_us, float esc, float steer);

  void LogRacingPath(int64_t t_us,
                     const std::vector<RacingPath::PathPoint>& path);
  void LogRacingPathClosestPt(int64_t t_us, float car_x, float car_y, float px,
                              float py, bool is_right);

 private:
  int img_topic_;
  int imu_topic_;
  int desired_twist_topic_;
  int actual_twist_topic_;
  int esc_topic_;
  int steer_topic_;
  int global_pose_topic_;
  int racing_path_topic_;
  int racing_path_closet_pt_topic_;

  std::mutex mu_;
  RosWriter ros_writer_;
};