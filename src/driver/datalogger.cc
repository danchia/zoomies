#include "driver/datalogger.h"

#include "ros/nav_msgs/Path.pb.h"
#include "ros/std_msgs/ColorRGBA.pb.h"
#include "ros/visualization_msgs/Marker.pb.h"

ros::geometry_msgs::Quaternion HeadingToQuat(float theta) {
  // Rotation about z-axis.
  ros::geometry_msgs::Quaternion q;
  q.set_w(std::cos(0.5 * theta));
  q.set_x(0);
  q.set_y(0);
  q.set_z(std::sin(0.5 * theta));
  return q;
}

ros::Time MicrosToRos(int64_t t) {
  ros::Time r;
  r.set_sec(t / 1000000);
  r.set_nsec((t % 1000000) * 1000);
  return r;
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
                          ros::visualization_msgs::ImageMarker::descriptor());
  driver_log_topic_ =
      writer_->AddChannel("/driver/state", zoomies::DriverLog::descriptor());
}

void Datalogger::LogVideoFrame(int64_t t_us, const ros::sensor_msgs::Image& m) {
  std::lock_guard<std::mutex> l(mu_);
  writer_->Write(img_topic_, t_us, m);
}

void Datalogger::LogGlobalPose(int64_t t_us,
                               const ros::geometry_msgs::PoseStamped& m) {
  std::lock_guard<std::mutex> l(mu_);
  writer_->Write(global_pose_topic_, t_us, m);
}

void Datalogger::LogDriverLog(int64_t t_us, const zoomies::DriverLog& m) {
  std::lock_guard<std::mutex> l(mu_);
  writer_->Write(driver_log_topic_, t_us, m);
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

  std::lock_guard<std::mutex> l(mu_);
  writer_->Write(racing_path_topic_, t_us, m);
}

void Datalogger::LogRacingPathClosestPt(int64_t t_us, float car_x, float car_y,
                                        float px, float py, bool is_right) {
  ros::visualization_msgs::Marker m;
  *m.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
  m.mutable_header()->set_frame_id("/map");

  m.set_ns("motion_plan");
  m.set_id(0);

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

  std::lock_guard<std::mutex> l(mu_);
  writer_->Write(racing_path_closet_pt_topic_, t_us, m);
}