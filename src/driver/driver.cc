#include "driver/driver.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "common/clock.h"
#include "hw/hw.h"
#include "ros/geometry_msgs/PoseStamped.pb.h"
#include "spdlog/spdlog.h"
#include "zoomies/zoomies.pb.h"

namespace {

constexpr bool kManualDrive = false;
constexpr int64_t kLoopPeriodMicros = 10000;
// pi * 62.7e-3 (wheel diameter) * 0.5 (belt ratio, 17t) * 27/90 / 3 =
constexpr float kMetersPerTick = 0.00984889296f;

constexpr float kMaxSteerDelta = 0.44;  // ~25 deg

constexpr float kCGtoFrontAxle = 0.137f;
constexpr float kFrontToRearLength = 0.265f;

constexpr int kVideoWidth = 640;
constexpr int kVideoHeight = 480;

constexpr bool kLogVideo = true;
constexpr int kVideoLogInterval = 10;

float MotorTickPeriodToMetersPerSecond(uint16_t p) {
  if (p == 0) return 0.0f;
  return (1e6 * kMetersPerTick) / p;
}

}  // namespace

Driver::Driver(Datalogger& datalogger, RacingPath& racing_path,
               Localizer& localizer)
    : datalogger_(datalogger),
      racing_path_(racing_path),
      localizer_(localizer) {
  assert(std::atomic_is_lock_free(&ticks_));

  ticks_.store(0, std::memory_order_relaxed);
}

Driver::~Driver() {}

// 30Hz update rate.
void Driver::OnCameraTick(int64_t t_us, uint8_t* buf, int len) {
  int64_t loop_tick = ticks_.load(std::memory_order_relaxed);
  if (loop_tick <= 0) return;

  ++vid_frame_;

  if (kLogVideo && vid_frame_ % kVideoLogInterval == 0) {
    img_msg_.Clear();
    // A bit hacky, but we'll send the whole YUV420 image which technically
    // isn't supported, but we'll advertise it as a smaller mono8 image.
    // sensor_msgs__Image ros_img;
    *img_msg_.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
    img_msg_.mutable_header()->set_frame_id("/camera");
    img_msg_.set_height(kVideoHeight);
    img_msg_.set_width(kVideoWidth);
    img_msg_.set_encoding("mono8");
    img_msg_.set_is_bigendian(false);
    img_msg_.set_step(kVideoWidth);
    img_msg_.mutable_data()->resize(len);
    memcpy(img_msg_.mutable_data()->data(), buf, len);  // unfortunate copy
    datalogger_.LogVideoFrame(t_us, img_msg_);
  }
  localizer_.OnVideoFrame(t_us, buf);
}

Driver::ControlOutput Driver::OnControlTick(int64_t t_us,
                                            const HWSensorReading& reading,
                                            const JS::State& js_state) {
  float desired_angular_velocity = 0.0f;

  int64_t loop_tick = ticks_.fetch_add(1, std::memory_order_relaxed) + 1;

  if (loop_tick == 1) {
    datalogger_.LogRacingPath(t_us, racing_path_.path_points());
    prev_reading_ = reading;
  }
  if (loop_tick % 100 == 0) {
    datalogger_.LogRacingPath(t_us, racing_path_.path_points());
  }

  uint16_t wheel_delta = reading.motor_ticks - prev_reading_.motor_ticks;

  State state;
  state.t_micros = t_us;
  state.dist_delta = wheel_delta * kMetersPerTick;
  state.fwd_vel = MotorTickPeriodToMetersPerSecond(reading.motor_period);
  if (state.fwd_vel > 50.0) state.fwd_vel = prev_state_.fwd_vel;
  state.angular_vel = reading.gyro.z();
  state.total_distance = prev_state_.total_distance + state.dist_delta;

  float dt = (state.t_micros - prev_state_.t_micros) * 1e-6;
  if (dt > 0.1f) {
    spdlog::warn("control loop stall: {} s", dt);
  }
  float heading_delta = state.angular_vel * dt;
  state.heading = prev_state_.heading + (heading_delta / 2.0f);
  state.x = prev_state_.x + cos(state.heading) * state.fwd_vel * dt;
  state.y = prev_state_.y + sin(state.heading) * state.fwd_vel * dt;
  state.heading += (heading_delta / 2.0f);

  const float stddev_dist = std::max(0.1f * state.dist_delta, 0.03f * 0.01f);
  const float stddev_heading =
      std::max(fabsf(0.3f * heading_delta), 0.1f * 0.01f);
  std::optional<Localizer::SyncResult> localizer_result =
      localizer_.ControlSync(
          t_us, Eigen::Vector3f{state.x, state.y, state.heading},
          state.dist_delta, heading_delta, stddev_dist, stddev_heading);

  if (localizer_result.has_value()) {
    state.x += localizer_result->correction.x();
    state.y += localizer_result->correction.y();
    state.heading += localizer_result->correction.z();
  }
  state.heading = normAngle(state.heading);

  if (kManualDrive) {
    state.desired_fwd_vel_ = js_state.accel > 0 ? js_state.accel * 1.5f : 0.0f;
  } else {
    // if (prev_state_.racing_path_dist_ + state.dist_delta >
    //     racing_path_.total_length()) {
    //   return Done();
    // }
    DoFollowRacingPath(t_us, dt, state);
  }

  float esc = CalculateLongitudinalControl(state);
  float steer = CalculateLateralControl(state);

  if (kManualDrive) {
    steer = js_state.steer;
  }

  // Done with iteration, overwrite previous states.
  prev_reading_ = reading;
  prev_state_ = state;

  // Log all ze things...
  ros::geometry_msgs::PoseStamped pose;
  *pose.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
  pose.mutable_header()->set_frame_id("/map");
  pose.mutable_pose()->mutable_position()->set_x(state.x);
  pose.mutable_pose()->mutable_position()->set_y(state.y);
  *pose.mutable_pose()->mutable_orientation() = HeadingToQuat(state.heading);
  datalogger_.LogGlobalPose(t_us, pose);

  zoomies::DriverLog driver_log;
  driver_log.set_t_us(t_us);
  driver_log.set_delta_t_s(dt);
  driver_log.set_linear_velocity(state.fwd_vel);
  driver_log.set_angular_velocity(state.angular_vel);
  driver_log.set_desired_linear_velocity(state.desired_fwd_vel_);
  driver_log.set_desired_angular_velocity(state.desired_angular_vel_);
  driver_log.set_heading(state.heading);
  driver_log.set_x(state.x);
  driver_log.set_y(state.y);
  driver_log.set_total_distance(state.total_distance);
  driver_log.set_racing_path_dist(state.racing_path_dist_);
  driver_log.set_dist_delta(state.dist_delta);
  driver_log.set_heading_delta(heading_delta);
  driver_log.set_dist_stddev(stddev_dist);
  driver_log.set_heading_stddev(stddev_heading);
  driver_log.set_esc(esc);
  driver_log.set_steer(steer);
  driver_log.mutable_imu_accel()->set_x(reading.accel.x());
  driver_log.mutable_imu_accel()->set_y(reading.accel.y());
  driver_log.mutable_imu_accel()->set_z(reading.accel.z());
  driver_log.mutable_imu_rotation()->set_x(reading.gyro.x());
  driver_log.mutable_imu_rotation()->set_y(reading.gyro.y());
  driver_log.mutable_imu_rotation()->set_z(reading.gyro.z());
  if (localizer_result.has_value()) {
    driver_log.mutable_localizer_correction()->set_x(
        localizer_result->correction.x());
    driver_log.mutable_localizer_correction()->set_y(
        localizer_result->correction.y());
    driver_log.mutable_localizer_correction()->set_z(
        localizer_result->correction.z());
    driver_log.mutable_localizer_variance()->set_x(
        localizer_result->variance.x());
    driver_log.mutable_localizer_variance()->set_y(
        localizer_result->variance.y());
    driver_log.mutable_localizer_variance()->set_z(
        localizer_result->variance.z());
  }
  datalogger_.LogDriverLog(t_us, driver_log);

  return ControlOutput{
      .done = false,
      .led = 3,
      .esc = esc,
      .steer = steer,
  };
}

namespace {
// h2 releative to h1. Positive means H2 is left turn relative to H1.
// hand turn.
double HeadingDiff(double h1, double h2) {
  double left = h2 - h1;
  double right = h1 - h2;
  if (left < 0) left += 2.0 * M_PI;
  if (right < 0) right += 2.0 * M_PI;

  return left < right ? left : -right;
}

}  // namespace

void Driver::DoFollowRacingPath(int64_t t_us, float dt, State& state) {
  float s_guess = prev_state_.racing_path_dist_ + state.dist_delta;

  // For the Stanley controller, have to translate CG coordinates into front
  // wheel coordinates. We'll just use the for speed profile too, out of
  // convenience.
  float fx = state.x + cos(state.heading) * kCGtoFrontAxle;
  float fy = state.y + sin(state.heading) * kCGtoFrontAxle;
  auto path_info = racing_path_.GetPathInfo(s_guess, fx, fy);
  datalogger_.LogRacingPathClosestPt(t_us, fx, fy, path_info.closest_x,
                                     path_info.closest_y,
                                     path_info.dist_to_closest >= 0.0f);

  state.racing_path_dist_ = path_info.s;
  state.desired_fwd_vel_ =
      std::min(path_info.velocity,
               prev_state_.desired_fwd_vel_ + racing_path_.max_accel() * dt);

  float lane_gain = 1.5f;
  // At the start, side-by-side with other car, don't lane keep so aggressively.
  if (state.total_distance <= 4.0f) {
    lane_gain /= 10.0f;
  }
  float delta_heading = HeadingDiff(state.heading, path_info.heading);
  float lane =
      atan2f32(lane_gain * path_info.dist_to_closest, state.desired_fwd_vel_);
  float delta = delta_heading + lane;
  delta = std::clamp(delta, -kMaxSteerDelta, kMaxSteerDelta);

  // tan delta = L/R = L*w / V
  // w = (tan delta)*V/L
  state.desired_angular_vel_ =
      tanf32(delta) * state.desired_fwd_vel_ / kFrontToRearLength;

  zoomies::MotionPlan plan;
  plan.set_path_velocity(path_info.velocity);
  plan.set_path_heading(path_info.heading);
  plan.set_path_dist_to_closest(path_info.dist_to_closest);
  plan.set_desired_linear_velocity(state.desired_fwd_vel_);
  plan.set_lane_gain(lane_gain);
  plan.set_delta_heading(delta_heading);
  plan.set_lane_delta(lane);
  plan.set_delta(delta);
  plan.set_desired_angular_velocity(state.desired_angular_vel_);
  datalogger_.LogMotionPlan(t_us, plan);
}

float Driver::CalculateLongitudinalControl(State& state) {
  // Consider it a decel event if we're slow at >= 0.5m/s^2, which at 100Hz is a
  // delta_v of 0.005m/s.
  bool is_decel =
      state.desired_fwd_vel_ + 0.005f < prev_state_.desired_fwd_vel_;

  float e = state.desired_fwd_vel_ - state.fwd_vel;
  fwd_vel_accel_e_i_ = 0.8f * fwd_vel_accel_e_i_ + e;
  // Accel: 7m/s esc: 0.8.
  // Decel: 4.47m/s esc: 0.11
  float feedforward = is_decel ? powf(state.desired_fwd_vel_, 1.3) * 0.02f
                               : powf(state.desired_fwd_vel_, 1.3) * 0.055f;

  float Kp = 0.15f;
  float Ki = 0.08f;
  float Kd = 0.04f;
  float esc = feedforward + e * Kp + fwd_vel_accel_e_i_ * Ki -
              Kd * (state.fwd_vel - prev_state_.fwd_vel);
  return std::clamp(esc, -1.0f, 1.0f);
}

float Driver::CalculateLateralControl(State& state) {
  float e = state.desired_angular_vel_ - state.angular_vel;
  angular_vel_e_i_ = 0.8f * angular_vel_e_i_ + e;

  // Fitted data gives: 3E-03 + 0.516x + -0.0673x^2
  float ov = (state.desired_fwd_vel_ > 0.1f)
                 ? state.desired_angular_vel_ / state.desired_fwd_vel_
                 : 0.0f;
  float feedforward = 0.516f * ov - 0.0673f * ov * ov;
  float Kp = 0.025f;
  float Ki = 0.0006f;
  float Kd = 0.005f;
  float steer =
      feedforward + Kp * e + Ki * angular_vel_e_i_ -
      Kd * (state.desired_angular_vel_ - prev_state_.desired_angular_vel_);
  return std::clamp(steer, -1.0f, 1.0f);
}

Driver::ControlOutput Driver::Done() {
  // stop the camera
  ticks_.store(-1, std::memory_order_relaxed);
  return ControlOutput{.done = true};
}
