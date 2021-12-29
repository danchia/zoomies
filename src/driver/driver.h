#pragma once

#include <inttypes.h>
#include <stdio.h>

#include <atomic>

#include "driver/datalogger.h"
#include "hw/hw.h"
#include "hw/js.h"

class Driver {
 public:
  struct ControlOutput {
    bool done = false;

    uint8_t led;
    float esc;
    float steer;
  };

  Driver(Datalogger& datalogger);
  ~Driver();

  ControlOutput OnControlTick(int64_t t_us, const HWSensorReading& reading,
                              const JS::State& js_state);
  void OnCameraTick(int64_t t_us, uint8_t* buf, int len);

 private:
  ControlOutput Done();

  struct State {
    int64_t t_micros = 0;

    float fwd_vel = 0.0f;  // m/s
    float rot_vel = 0.0f;  // rad/s

    float heading = 0.0f;
    float x = 0.0f;
    float y = 0.0f;

    float total_distance = 0.0f;
  };

  std::atomic<int64_t> ticks_;
  Datalogger& datalogger_;

  HWSensorReading prev_reading_;
  State prev_state_;

  float vel_e_i_ = 0.0f;
};
