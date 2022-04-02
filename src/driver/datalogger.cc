#include "driver/datalogger.h"

#include <unistd.h>

#include "ros/nav_msgs/Path.pb.h"
#include "ros/std_msgs/ColorRGBA.pb.h"
#include "ros/visualization_msgs/Marker.pb.h"
#include "spdlog/spdlog.h"

namespace {
constexpr int kMaxBacklog = 100 * 1024 * 1024;
}

Datalogger::Datalogger(const std::string& path)
    : writer_(McapLogWriter::Make(path)) {
  img_topic_ = writer_->AddChannel("/camera1/raw",
                                   ros::sensor_msgs::Image::descriptor());
  global_pose_topic_ = writer_->AddChannel(
      "/global_pose/pose_only", ros::geometry_msgs::PoseStamped::descriptor());
  racing_path_topic_ = writer_->AddChannel("/motion_planner/path",
                                           ros::nav_msgs::Path::descriptor());
  racing_path_closet_pt_topic_ =
      writer_->AddChannel("/motion_planner/closest_point",
                          ros::visualization_msgs::Marker::descriptor());
  motion_plan_topic_ = writer_->AddChannel("/motion_planner/plan",
                                           zoomies::MotionPlan::descriptor());
  driver_log_topic_ =
      writer_->AddChannel("/driver/state", zoomies::DriverLog::descriptor());

  writer_thread_ = std::thread([this] { WriterLoop(); });
}

Datalogger::~Datalogger() {
  mu_.lock();
  done_ = true;
  mu_.unlock();

  writer_thread_.join();
}

void Datalogger::WriterLoop() {
  while (true) {
    bool done;
    int size;
    std::vector<Msg> msgs;

    {
      std::lock_guard l(mu_);
      done = done_;
      using std::swap;
      swap(msgs, msgs_);
      size = size_;
      size_ = 0;
    }
    if (size > kMaxBacklog) {
      spdlog::warn("Logging backlog, msgs may have been dropped");
    }
    for (const auto& m : msgs) {
      writer_->WriteMsg(m.chan_id, m.t_us, m.data);
    }
    if (done) return;

    usleep(50000);
  }
}

void Datalogger::QMsg(int chan_id, int64_t t_us,
                      const google::protobuf::MessageLite& m) {
  Msg msg;
  msg.chan_id = chan_id;
  msg.t_us = t_us;
  m.SerializeToString(&msg.data);

  std::lock_guard l(mu_);
  if (size_ > kMaxBacklog) return;
  size_ += msg.data.size();
  msgs_.push_back(std::move(msg));
}

void Datalogger::LogVideoFrame(int64_t t_us, const ros::sensor_msgs::Image& m) {
  QMsg(img_topic_, t_us, m);
}

void Datalogger::LogGlobalPose(int64_t t_us,
                               const ros::geometry_msgs::PoseStamped& m) {
  QMsg(global_pose_topic_, t_us, m);
}

void Datalogger::LogDriverLog(int64_t t_us, const zoomies::DriverLog& m) {
  QMsg(driver_log_topic_, t_us, m);
}

void Datalogger::LogRacingPath(int64_t t_us,
                               const std::vector<RacingPath::PathPoint>& path) {
  ros::nav_msgs::Path m;
  *m.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
  m.mutable_header()->set_frame_id("/map");

  for (const auto& p : path) {
    auto& pose = *m.add_poses();
    auto& pos = *pose.mutable_pose()->mutable_position();
    pos.set_x(p.x);
    pos.set_y(p.y);
    *pose.mutable_pose()->mutable_orientation() = HeadingToQuat(p.heading);
  }

  QMsg(racing_path_topic_, t_us, m);
}

void Datalogger::LogRacingPathClosestPt(int64_t t_us, float car_x, float car_y,
                                        float px, float py, bool is_right) {
  ros::visualization_msgs::Marker m;
  *m.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
  m.mutable_header()->set_frame_id("/map");

  m.set_ns("motion_plan");
  m.set_id(0);

  m.mutable_pose()->mutable_position();
  m.mutable_pose()->mutable_orientation();
  m.mutable_scale();
  m.mutable_color();

  m.set_type(4);  // line strip
  m.set_action(0);

  m.mutable_scale()->set_x(0.1);
  m.mutable_scale()->set_y(0.1);
  m.mutable_scale()->set_z(0.1);

  ros::std_msgs::ColorRGBA color;
  color.set_a(1.0f);
  if (is_right) {
    color.set_r(1.0f);
  } else {
    color.set_g(1.0f);
  }
  {
    auto& car = *m.add_points();
    *m.add_colors() = color;
    car.set_x(car_x);
    car.set_y(car_y);
  }

  {
    auto& p = *m.add_points();
    *m.add_colors() = color;
    p.set_x(px);
    p.set_y(py);
  }

  QMsg(racing_path_closet_pt_topic_, t_us, m);
}

void Datalogger::LogMotionPlan(int64_t t_us, const zoomies::MotionPlan& m) {
  QMsg(motion_plan_topic_, t_us, m);
}