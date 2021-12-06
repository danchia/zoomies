#include <unistd.h>

#include <cstdio>

#include "hw/hw.h"

constexpr float kSteerTrim = -0.02f;
HW* g_hw;

int main() {
  printf("Initializing...\n");
  g_hw = new HW();
  printf("Init done!\n");

  int loop_ticks = 0;
  HWSensorReading reading;
  while (true) {
    ++loop_ticks;

    g_hw->SetLedSpeedSteering(3, 0.2, 0.0f + kSteerTrim);
    usleep(10000);

    if (!g_hw->ReadSensors(&reading)) {
      return 1;
    }
    if (loop_ticks > 50) {
      return 0;
    }
    if (loop_ticks % 100 == 0) {
      printf("motor: %d cum, %d period\n", reading.motor_ticks,
             reading.motor_period);
      printf("gyro: %.3f %.3f %.3f\n", reading.gyro.x(), reading.gyro.y(),
             reading.gyro.z());
      printf("accel: %.3f %.3f %.3f\n", reading.accel.x(), reading.accel.y(),
             reading.accel.z());
    }
  }

  return 0;
}
