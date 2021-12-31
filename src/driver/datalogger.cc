#include "driver/datalogger.h"

#include "ros/ros_types.h"
#include "ros/ros_util.h"

namespace {
geometry_msgs__Quaternion HeadingToQuat(float theta) {
  // Rotation about z-axis.
  geometry_msgs__Quaternion q;
  q.w(std::cos(0.5 * theta));
  q.x(0);
  q.y(0);
  q.z(std::sin(0.5 * theta));
  return q;
}
}  // namespace

Datalogger::Datalogger(std::string_view path) : ros_writer_(path) {
  img_topic_ =
      ros_writer_.AddConnection("/camera1/raw", "sensor_msgs/msg/Image");
  imu_topic_ = ros_writer_.AddConnection("/imu", "sensor_msgs/msg/Imu");
  desired_twist_topic_ = ros_writer_.AddConnection(
      "/desired_twist", "geometry_msgs/msg/TwistStamped");
  actual_twist_topic_ = ros_writer_.AddConnection(
      "/actual_twist", "geometry_msgs/msg/TwistWithCovarianceStamped");
  global_pose_topic_ = ros_writer_.AddConnection(
      "/global_pose/pose_only", "geometry_msgs/msg/PoseStamped");
  esc_topic_ = ros_writer_.AddConnection("/hw/esc", "std_msgs/msg/Float32");
  steer_topic_ = ros_writer_.AddConnection("/hw/steer", "std_msgs/msg/Float32");
  racing_path_topic_ =
      ros_writer_.AddConnection("/motion_planner/path", "nav_msgs/msg/Path");
  racing_path_closet_pt_topic_ = ros_writer_.AddConnection(
      "/motion_planner/closest_point", "visualization_msgs/msg/Marker");
}

void Datalogger::LogVideoFrame(int64_t t_us, int width, int height,
                               uint8_t* data, int len) {
  // A bit hacky, but we'll send the whole YUV420 image which technically isn't
  // supported, but we'll advertise it as a smaller mono8 image.
  sensor_msgs__Image ros_img;
  ros_img.header().stamp() = MicrosToRos(t_us);
  ros_img.header().frame_id("/camera");
  ros_img.height(height);
  ros_img.width(width);
  ros_img.encoding("mono8");
  ros_img.is_bigendian(false);
  ros_img.step(width);
  ros_img.data().resize(len);
  memcpy(ros_img.data().data(), data, len);  // unfortunate copy

  std::lock_guard<std::mutex> l(mu_);
  ros_writer_.Write(img_topic_, t_us, ros_img);
}

void Datalogger::LogIMU(int64_t t_us, const Eigen::Vector3f& linear_accel,
                        const Eigen::Vector3f& angular_vel) {
  sensor_msgs__Imu m;

  m.header().stamp() = MicrosToRos(t_us);
  m.header().frame_id("/base_link");
  m.orientation().w(1);

  m.angular_velocity().x(angular_vel.x());
  m.angular_velocity().y(angular_vel.y());
  m.angular_velocity().z(angular_vel.z());

  m.linear_acceleration().x(linear_accel.x());
  m.linear_acceleration().y(linear_accel.y());
  m.linear_acceleration().z(linear_accel.z());

  std::lock_guard<std::mutex> l(mu_);
  ros_writer_.Write(imu_topic_, t_us, m);
}

void Datalogger::LogDesiredTwist(int64_t t_us, float linear_velocity,
                                 float angular_velocity) {
  geometry_msgs__TwistStamped m;

  m.header().stamp() = MicrosToRos(t_us);
  m.header().frame_id("/base_link");

  m.twist().linear().x(linear_velocity);
  m.twist().angular().z(angular_velocity);

  std::lock_guard<std::mutex> l(mu_);
  ros_writer_.Write(desired_twist_topic_, t_us, m);
}

void Datalogger::LogActualTwist(int64_t t_us, float linear_velocity,
                                float angular_velocity) {
  geometry_msgs__TwistWithCovarianceStamped m;

  m.header().stamp() = MicrosToRos(t_us);
  m.header().frame_id("/base_link");

  m.twist().twist().linear().x(linear_velocity);
  m.twist().twist().angular().z(angular_velocity);

  std::lock_guard<std::mutex> l(mu_);
  ros_writer_.Write(actual_twist_topic_, t_us, m);
}

void Datalogger::LogGlobalPose(int64_t t_us, float x, float y, float theta) {
  geometry_msgs__PoseStamped m;

  m.header().stamp() = MicrosToRos(t_us);
  m.header().frame_id("/map");

  m.pose().position().x(x);
  m.pose().position().y(y);

  m.pose().orientation(HeadingToQuat(theta));

  std::lock_guard<std::mutex> l(mu_);
  ros_writer_.Write(global_pose_topic_, t_us, m);
}

void Datalogger::LogEscSteer(int64_t t_us, float esc, float steer) {
  std_msgs__Float32 esc_m, steer_m;
  esc_m.data(esc);
  steer_m.data(steer);

  std::lock_guard<std::mutex> l(mu_);
  ros_writer_.Write(esc_topic_, t_us, esc_m);
  ros_writer_.Write(steer_topic_, t_us, steer_m);
}

void Datalogger::LogRacingPath(int64_t t_us,
                               const std::vector<RacingPath::PathPoint>& path) {
  nav_msgs__Path m;
  m.header().stamp() = MicrosToRos(t_us);
  m.header().frame_id("/map");

  for (const auto& p : path) {
    auto& pose = m.poses().emplace_back();
    auto& pos = pose.pose().position();
    pos.x(p.x);
    pos.y(p.y);
    pose.pose().orientation(HeadingToQuat(p.heading));
  }

  std::lock_guard<std::mutex> l(mu_);
  ros_writer_.Write(racing_path_topic_, t_us, m);
}

void Datalogger::LogRacingPathClosestPt(int64_t t_us, float car_x, float car_y,
                                        float px, float py, bool is_right) {
  visualization_msgs__Marker m;
  m.header().stamp() = MicrosToRos(t_us);
  m.header().frame_id("/map");

  m.ns("motion_plan");
  m.id(0);

  m.type(4);  // line strip
  m.action(0);

  m.scale().x(0.1);
  m.scale().y(0.1);
  m.scale().z(0.1);

  std_msgs__ColorRGBA color;
  color.a(1.0f);
  if (is_right) {
    color.r(1.0f);
  } else {
    color.g(1.0f);
  }
  {
    auto& car = m.points().emplace_back();
    m.colors().push_back(color);
    car.x(car_x);
    car.y(car_y);
  }

  {
    auto& p = m.points().emplace_back();
    m.colors().push_back(color);
    p.x(px);
    p.y(py);
  }

  std::lock_guard<std::mutex> l(mu_);
  ros_writer_.Write(racing_path_closet_pt_topic_, t_us, m);
}