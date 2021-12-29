#include <stdlib.h>
#include <unistd.h>

#include "hw/hw.h"
#include "hw/js.h"
#include "spdlog/spdlog.h"

int main() {
  spdlog::info("Initializing...\n");
  HW hw;
  JS js;

  spdlog::info("At zero...\n");
  while (true) {
    hw.SetLedSpeedSteering(2, 0.0f, 0.0f);
    usleep(10000);
    auto s = js.Poll();
    if (s.a_btn) break;
  }

  while (true) {
    auto s = js.Poll();
    if (!s.a_btn) break;
  }

  spdlog::info("At fwd...\n");
  while (true) {
    hw.SetLedSpeedSteering(2, 1.0f, 0.0f);
    usleep(10000);
    auto s = js.Poll();
    if (s.a_btn) break;
  }

  while (true) {
    auto s = js.Poll();
    if (!s.a_btn) break;
  }

  spdlog::info("At reverse...\n");
  while (true) {
    hw.SetLedSpeedSteering(2, -1.0f, 0.0f);
    usleep(10000);
    auto s = js.Poll();
    if (s.a_btn) break;
  }

  return 0;
}