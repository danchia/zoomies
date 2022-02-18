#pragma once

#include <inttypes.h>

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>

#include "mcap/writer.h"
#include "ros/builtins.pb.h"
#include "ros/geometry_msgs/PoseStamped.pb.h"
#include "ros/geometry_msgs/Quaternion.pb.h"
#include "ros/sensor_msgs/Image.pb.h"
#include "ros/visualization_msgs/ImageMarker.pb.h"
#include "track/track.h"

class Datalogger {
 public:
  Datalogger(const std::string& path);

  void LogVideoFrame(int64_t t_us, const ros::sensor_msgs::Image& m);
  void LogGlobalPose(int64_t t_us, const ros::geometry_msgs::PoseStamped& m);
  void LogRacingPath(int64_t t_us,
                     const std::vector<RacingPath::PathPoint>& path);
  void LogRacingPathClosestPt(int64_t t_us, float car_x, float car_y, float px,
                              float py, bool is_right);

 private:
  int img_topic_;
  int global_pose_topic_;
  int racing_path_topic_;
  int racing_path_closet_pt_topic_;

  std::mutex mu_;
  std::unique_ptr<McapLogWriter> writer_;
};

ros::geometry_msgs::Quaternion HeadingToQuat(float theta);

ros::Time MicrosToRos(int64_t t);