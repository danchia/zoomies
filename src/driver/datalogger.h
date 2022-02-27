#pragma once

#include <inttypes.h>

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "google/protobuf/message_lite.h"
#include "mcap/writer.h"
#include "proto/proto_util.h"
#include "ros/geometry_msgs/PoseStamped.pb.h"
#include "ros/sensor_msgs/Image.pb.h"
#include "ros/visualization_msgs/ImageMarker.pb.h"
#include "track/track.h"
#include "zoomies/zoomies.pb.h"

class Datalogger {
 public:
  Datalogger(const std::string& path);
  ~Datalogger();

  void LogVideoFrame(int64_t t_us, const ros::sensor_msgs::Image& m);
  void LogGlobalPose(int64_t t_us, const ros::geometry_msgs::PoseStamped& m);
  void LogRacingPath(int64_t t_us,
                     const std::vector<RacingPath::PathPoint>& path);
  void LogRacingPathClosestPt(int64_t t_us, float car_x, float car_y, float px,
                              float py, bool is_right);
  void LogDriverLog(int64_t t_us, const zoomies::DriverLog& m);

 private:
  struct Msg {
    int chan_id;
    int64_t t_us;
    std::string data;
  };
  void QMsg(int chan_id, int64_t t_us, const google::protobuf::MessageLite& m);
  void WriterLoop();

  int img_topic_;
  int global_pose_topic_;
  int racing_path_topic_;
  int racing_path_closet_pt_topic_;
  int driver_log_topic_;

  std::unique_ptr<McapLogWriter> writer_;

  std::thread writer_thread_;
  std::mutex mu_;
  std::vector<Msg> msgs_;
  int size_ = 0;
  bool done_ = false;
};