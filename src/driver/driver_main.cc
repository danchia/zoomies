#include <stdlib.h>
#include <unistd.h>

#include "common/clock.h"
#include "driver/datalogger.h"
#include "driver/driver.h"
#include "hw/camera.h"
#include "hw/hw.h"
#include "hw/js.h"
#include "spdlog/spdlog.h"
#include "track/track.h"

namespace {

constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;
constexpr float kCeilHeight = 2.5f;

constexpr float kSteerTrim = 0.0f;
constexpr bool kSkipWaitJs = false;
constexpr int64_t kLoopPeriodMicros = 10000;

}  // namespace

int main() {
  spdlog::info("Initializing...\n");
  HW hw;
  Datalogger datalogger("log.mcap");
  RacingPath racing_path("track.bin");
  Localizer localizer({
      .img_width = kImageWidth,
      .img_height = kImageHeight,
      .ceil_height = kCeilHeight,
      .camera_model_path = "camera_lut.bin",
      .ceil_mask_path = "ceil_mask.bin",
      .map_path = "map.txt",
  });
  Driver driver(datalogger, racing_path, localizer);
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
    if (js_state.y_btn) {
      spdlog::warn("Detected e-stop.\n");
      break;
    }

    auto control_out = driver.OnControlTick(now, reading, js_state);

    if (control_out.done) {
      break;
    }
    hw.SetLedSpeedSteering(control_out.led, control_out.esc,
                           control_out.steer + kSteerTrim);
  }

  spdlog::info("Stopping car\n");

  cam.Stop();

  // Stop the car.
  for (int i = 0; i < 200; ++i) {
    hw.SetLedSpeedSteering(2, -0.3f, kSteerTrim);
    usleep(10000);
  }

  return 0;
}
