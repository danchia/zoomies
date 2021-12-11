#include <unistd.h>

#include "driver/controlloop.h"
#include "hw/camera.h"
#include "hw/hw.h"
#include "hw/js.h"

int main() {
  printf("Initializing...\n");
  HW hw;
  printf("Init done!\n");

  Driver driver;
  JS js;

  Camera cam(640, 480, 30);
  cam.Start(
      [&driver](uint8_t* buf, int len) { driver.OnCameraTick(buf, len); });

  if (!driver.RunControlLoop(hw, js)) {
    return -1;
  }

  return 0;
}
