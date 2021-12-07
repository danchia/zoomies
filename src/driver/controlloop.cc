#include "driver/controlloop.h"

#include <inttypes.h>
#include <unistd.h>

#include <cstdio>

#include "common/clock.h"
#include "hw/hw.h"
#include "spdlog/spdlog.h"

namespace {

constexpr int64_t kLoopPeriodMicros = 10000;
constexpr float kSteerTrim = -0.02f;
// pi * 62.7e-3 (wheel diameter) * 0.5 (belt ratio, 17t) * 25/90 / 3 =
constexpr float kMetersPerTick = 0.00911934534f;

struct State {
  int64_t t_micros = 0;

  float fwd_vel = 0.0f;  // m/s
  float rot_vel = 0.0f;  // rad/s

  float heading = 0.0f;
  float x = 0.0f;
  float y = 0.0f;

  float total_distance = 0.0f;
};

float MotorTickPeriodToMetersPerSecond(uint16_t p) {
  if (p == 0) return 0.0f;
  return (10000 * kMetersPerTick) / p;
}

}  // namespace

bool RunControlLoop(HW& hw) {
  int64_t loop_ticks = 0;
  HWSensorReading prev_reading, reading;

  int64_t target_ftime = kLoopPeriodMicros;
  int64_t current_loop_time = 0;

  if (!hw.ReadSensors(&prev_reading)) {
    return false;
  }

  Clock loop_clock;
  State prev_state;

  while (true) {
    ++loop_ticks;

    int64_t now = loop_clock.ElapsedMicros();
    while (target_ftime < now) {
      target_ftime += kLoopPeriodMicros;
    }
    int64_t sleep_duration = target_ftime - now;
    if (sleep_duration > 0) {
      usleep(sleep_duration);
    }
    now = loop_clock.ElapsedMicros();

    // State update
    if (!hw.ReadSensors(&reading)) {
      return false;
    }

    uint16_t wheel_delta = reading.motor_ticks - prev_reading.motor_ticks;
    float dist_delta = wheel_delta * kMetersPerTick;

    State state;
    state.t_micros = now;
    state.fwd_vel = MotorTickPeriodToMetersPerSecond(reading.motor_period);
    if (state.fwd_vel > 50.0) state.fwd_vel = prev_state.fwd_vel;
    state.rot_vel = reading.gyro.z();
    state.total_distance = prev_state.total_distance + dist_delta;

    float dt = (state.t_micros - prev_state.t_micros) * 1e-6;
    state.heading += state.rot_vel * dt;
    state.x += sin(state.heading) * state.fwd_vel * dt;
    state.y += cos(state.heading) * state.fwd_vel * dt;

    prev_reading = reading;
    prev_state = state;

    // PID

    hw.SetLedSpeedSteering(3, 0.0, 0.0f + kSteerTrim);

    // // TODO: remove
    // if (loop_ticks > 50) {
    //   return true;
    // }
    if (loop_ticks % 100 == 0) {
      spdlog::info("motor: {} cum, {} period", reading.motor_ticks,
                   reading.motor_period);
      spdlog::info("gyro: {:.3f} {:.3f} {:.3f}", reading.gyro.x(),
                   reading.gyro.y(), reading.gyro.z());
      spdlog::info("accel: {:.3f} {:.3f}, {:.3f}", reading.accel.x(),
                   reading.accel.y(), reading.accel.z());
    }
  }
}