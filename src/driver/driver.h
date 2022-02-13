#pragma once

#include <inttypes.h>
#include <stdio.h>

#include <atomic>

#include "driver/datalogger.h"
#include "hw/hw.h"
#include "hw/js.h"
#include "localization/localizer.h"
#include "track/track.h"

class Driver {
 public:
  struct ControlOutput {
    bool done = false;

    uint8_t led;
    float esc;
    float steer;
  };

  Driver(Datalogger& datalogger, RacingPath& racing_path, Localizer& localizer);
  ~Driver();

  ControlOutput OnControlTick(int64_t t_us, const HWSensorReading& reading,
                              const JS::State& js_state);
  void OnCameraTick(int64_t t_us, uint8_t* buf, int len);

 private:
  struct State {
    int64_t t_micros = 0;

    // Actual velocities.
    float fwd_vel = 0.0f;      // m/s
    float angular_vel = 0.0f;  // rad/s

    float heading = 0.0f;
    float x = 0.0f;
    float y = 0.0f;

    float total_distance = 0.0f;
    float dist_delta = 0.0f;
    float racing_path_dist_ = 0.0f;  // may wrap

    float desired_fwd_vel_ = 0.0f;
    float desired_angular_vel_ = 0.0f;
  };

  ControlOutput Done();

  void DoFollowRacingPath(int64_t t_us, State& state);
  float CalculateLongitudinalControl(State& state);
  float CalculateLateralControl(State& state);

  std::atomic<int64_t> ticks_;
  Datalogger& datalogger_;
  RacingPath& racing_path_;
  Localizer& localizer_;

  HWSensorReading prev_reading_;
  State prev_state_;

  float fwd_vel_accel_e_i_ = 0.0f;
  float fwd_vel_decel_e_i_ = 0.0f;
  float angular_vel_e_i_ = 0.0f;
  float prev_fwd_e_ = 0.0f;
};
