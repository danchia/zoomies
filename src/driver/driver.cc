#include "driver/driver.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "common/clock.h"
#include "hw/hw.h"
#include "spdlog/spdlog.h"

namespace {

constexpr bool kManualDrive = false;
constexpr int64_t kLoopPeriodMicros = 10000;
// pi * 62.7e-3 (wheel diameter) * 0.5 (belt ratio, 17t) * 25/90 / 3 =
constexpr float kMetersPerTick = 0.00911934534f;

constexpr int kVideoWidth = 640;
constexpr int kVideoHeight = 480;

float MotorTickPeriodToMetersPerSecond(uint16_t p) {
  if (p == 0) return 0.0f;
  return (1e6 * kMetersPerTick) / p;
}

}  // namespace

Driver::Driver(Datalogger& datalogger) : datalogger_(datalogger) {
  assert(std::atomic_is_lock_free(&ticks_));

  ticks_.store(0, std::memory_order_relaxed);
}

Driver::~Driver() {}

// 30Hz update rate.
void Driver::OnCameraTick(int64_t t_us, uint8_t* buf, int len) {
  int64_t loop_tick = ticks_.load(std::memory_order_relaxed);
  if (loop_tick <= 0) return;

  datalogger_.LogVideoFrame(t_us, kVideoWidth, kVideoHeight, buf, len);
}

Driver::ControlOutput Driver::OnControlTick(int64_t t_us,
                                            const HWSensorReading& reading,
                                            const JS::State& js_state) {
  float desired_fwd_velocity = 0.0f;
  float desired_angular_velocity = 0.0f;

  int64_t loop_tick = ticks_.fetch_add(1, std::memory_order_relaxed) + 1;

  if (loop_tick == 1) {
    prev_reading_ = reading;
  }

  uint16_t wheel_delta = reading.motor_ticks - prev_reading_.motor_ticks;
  float dist_delta = wheel_delta * kMetersPerTick;

  State state;
  state.t_micros = t_us;
  state.fwd_vel = MotorTickPeriodToMetersPerSecond(reading.motor_period);
  if (state.fwd_vel > 50.0) state.fwd_vel = prev_state_.fwd_vel;
  state.rot_vel = reading.gyro.z();
  state.total_distance = prev_state_.total_distance + dist_delta;

  float dt = (state.t_micros - prev_state_.t_micros) * 1e-6;
  state.heading = prev_state_.heading + state.rot_vel * dt;
  state.x = prev_state_.x + cos(state.heading) * state.fwd_vel * dt;
  state.y = prev_state_.y + sin(state.heading) * state.fwd_vel * dt;

  prev_reading_ = reading;
  prev_state_ = state;

  if (loop_tick < 80) {
    desired_fwd_velocity = (2.0 * loop_tick) / 80;
  } else if (loop_tick < 160) {
    desired_fwd_velocity = 2.0 - ((2.0 * (loop_tick - 80)) / 80);
  } else {
    return Done();
  }

  float steer = 0.0f;
  if (kManualDrive) {
    desired_fwd_velocity = js_state.accel > 0 ? js_state.accel * 1.5f : 0.0f;
    steer = js_state.steer;
  }

  // longitudinal PID
  float e = desired_fwd_velocity - state.fwd_vel;
  vel_e_i_ = 0.9 * vel_e_i_ + e;
  float esc = e * 0.2 + vel_e_i_ * 0.06;

  // Log all ze things...
  datalogger_.LogIMU(t_us, reading.accel, reading.gyro);
  datalogger_.LogDesiredTwist(t_us, desired_fwd_velocity, 0.0f);
  datalogger_.LogActualTwist(t_us, state.fwd_vel, state.rot_vel);
  datalogger_.LogEscSteer(t_us, esc, steer);
  datalogger_.LogGlobalPose(t_us, state.x, state.y, state.heading);

  return ControlOutput{
      .done = false,
      .led = 3,
      .esc = esc,
      .steer = steer,
  };
}

Driver::ControlOutput Driver::Done() {
  // stop the camera
  ticks_.store(-1, std::memory_order_relaxed);
  return ControlOutput{.done = true};
}