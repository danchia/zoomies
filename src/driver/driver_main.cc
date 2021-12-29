#include <stdlib.h>
#include <unistd.h>

#include "common/clock.h"
#include "driver/datalogger.h"
#include "driver/driver.h"
#include "hw/camera.h"
#include "hw/hw.h"
#include "hw/js.h"
#include "spdlog/spdlog.h"

namespace {

constexpr float kSteerTrim = -0.018f;
constexpr bool kSkipWaitJs = true;
constexpr int64_t kLoopPeriodMicros = 10000;

}  // namespace

int main() {
  spdlog::info("Initializing...\n");
  HW hw;
  Datalogger datalogger("datalog");
  Driver driver(datalogger);
  JS js;
  Clock clock;
  Camera cam(640, 480, 30);

  cam.Start([&driver, &clock](uint8_t* buf, int len) {
    driver.OnCameraTick(clock.ElapsedMicros(), buf, len);
  });

  spdlog::info("Init done! Waiting for camera warmup\n");
  usleep(3000000);  // Give camera 3s to autoexpose etc.

  spdlog::info("Camera wait done\n");

  while (true) {
    hw.SetLedSpeedSteering(2, 0.0f, kSteerTrim);
    auto s = js.Poll();
    if (s.a_btn || kSkipWaitJs) break;
  }

  spdlog::info("Starting program\n");

  // run the control loop
  int64_t target_ftime = kLoopPeriodMicros;
  int64_t current_loop_time = clock.ElapsedMicros();

  HWSensorReading reading;
  while (true) {
    int64_t now = clock.ElapsedMicros();
    while (target_ftime < now) {
      target_ftime += kLoopPeriodMicros;
    }
    int64_t sleep_duration = target_ftime - now;
    if (sleep_duration > 0) {
      usleep(sleep_duration);
    }
    now = clock.ElapsedMicros();

    if (!hw.ReadSensors(&reading)) {
      spdlog::warn("Failed to read sensors! Skipping control loop");
      continue;
    }
    auto js_state = js.Poll();

    auto control_out = driver.OnControlTick(now, reading, js_state);

    if (control_out.done) {
      break;
    }
    hw.SetLedSpeedSteering(control_out.led, control_out.esc,
                           control_out.steer + kSteerTrim);
  }

  spdlog::info("Stopping car\n");

  // Stop the car.
  for (int i = 0; i < 200; ++i) {
    hw.SetLedSpeedSteering(2, -0.3f, kSteerTrim);
    usleep(10000);
  }

  return 0;
}
