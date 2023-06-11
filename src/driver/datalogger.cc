#include "driver/datalogger.h"

#include <unistd.h>

#include "ros/nav_msgs/Path.pb.h"
#include "ros/sensor_msgs/PointCloud2.pb.h"
#include "ros/std_msgs/ColorRGBA.pb.h"
#include "ros/tf2_msgs/TFMessage.pb.h"
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

  tf_topic_ =
      writer_->AddChannel("/tf", ros::tf2_msgs::TFMessage::descriptor());

  landmarks_topic_ = writer_->AddChannel(
      "/localization/landmarks", ros::sensor_msgs::PointCloud2::descriptor());

  map_topic_ = writer_->AddChannel("/localization/map",
                                   ros::sensor_msgs::PointCloud2::descriptor());

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

void Datalogger::LogMap(int64_t t_us, const std::vector<Eigen::Vector3f>& map) {
  ros::sensor_msgs::PointCloud2 pt_cloud;
  *pt_cloud.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
  pt_cloud.mutable_header()->set_frame_id("/map");
  pt_cloud.set_height(1);

  auto* field = pt_cloud.add_fields();
  field->set_name("x");
  field->set_offset(0);
  field->set_datatype(7);
  field->set_count(1);

  field = pt_cloud.add_fields();
  field->set_name("y");
  field->set_offset(4);
  field->set_datatype(7);
  field->set_count(1);

  field = pt_cloud.add_fields();
  field->set_name("z");
  field->set_offset(8);
  field->set_datatype(7);
  field->set_count(1);

  pt_cloud.set_is_bigendian(false);
  pt_cloud.set_point_step(12);
  pt_cloud.set_is_dense(true);

  pt_cloud.set_width(map.size());
  int nbytes = 12 * map.size();
  pt_cloud.set_row_step(nbytes);
  pt_cloud.mutable_data()->resize(nbytes);
  memcpy(pt_cloud.mutable_data()->data(), map.data(), nbytes);

  QMsg(map_topic_, t_us, pt_cloud);
}

void Datalogger::LogLandmarks(int64_t t_us,
                              const std::vector<Eigen::Vector3f>& landmarks) {
  ros::sensor_msgs::PointCloud2 pt_cloud;
  *pt_cloud.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
  pt_cloud.mutable_header()->set_frame_id("/base_link");
  pt_cloud.set_height(1);

  auto* field = pt_cloud.add_fields();
  field->set_name("x");
  field->set_offset(0);
  field->set_datatype(7);
  field->set_count(1);

  field = pt_cloud.add_fields();
  field->set_name("y");
  field->set_offset(4);
  field->set_datatype(7);
  field->set_count(1);

  field = pt_cloud.add_fields();
  field->set_name("z");
  field->set_offset(8);
  field->set_datatype(7);
  field->set_count(1);

  pt_cloud.set_is_bigendian(false);
  pt_cloud.set_point_step(12);
  pt_cloud.set_is_dense(true);

  pt_cloud.set_width(landmarks.size());
  int nbytes = 12 * landmarks.size();
  pt_cloud.set_row_step(nbytes);
  pt_cloud.mutable_data()->resize(nbytes);
  memcpy(pt_cloud.mutable_data()->data(), landmarks.data(), nbytes);

  QMsg(landmarks_topic_, t_us, pt_cloud);
}

void Datalogger::LogGlobalPose(int64_t t_us,
                               const ros::geometry_msgs::PoseStamped& m) {
  ros::tf2_msgs::TFMessage tf;
  auto& transform = *tf.add_transforms();
  *transform.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
  transform.mutable_header()->set_frame_id("/map");
  transform.set_child_frame_id("/base_link");
  transform.mutable_transform()->mutable_translation()->set_x(
      m.pose().position().x());
  transform.mutable_transform()->mutable_translation()->set_y(
      m.pose().position().y());
  *transform.mutable_transform()->mutable_rotation() = m.pose().orientation();
  QMsg(tf_topic_, t_us, tf);

  QMsg(global_pose_topic_, t_us, m);
}

void Datalogger::LogDriverLog(int64_t t_us, const zoomies::DriverLog& m) {
  QMsg(driver_log_topic_, t_us, m);
}

void Datalogger::LogRacingPath(int64_t t_us,
                               const std::vector<RacingPath::PathPoint>& path) {
  ros::nav_msgs::Path m;
  auto stamp = MicrosToRos(t_us);
  *m.mutable_header()->mutable_stamp() = stamp;
  m.mutable_header()->set_frame_id("/map");

  for (const auto& p : path) {
    auto& pose = *m.add_poses();
    *pose.mutable_header()->mutable_stamp() = stamp;
    pose.mutable_header()->set_frame_id("/map");
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
