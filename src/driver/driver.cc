#include <unistd.h>

#include "driver/controlloop.h"
#include "hw/camera.h"
#include "hw/hw.h"

int main() {
  printf("Initializing...\n");
  HW hw;
  printf("Init done!\n");

  Driver driver;

  Camera cam(640, 480, 30);
  cam.Start(
      [&driver](uint8_t* buf, int len) { driver.OnCameraTick(buf, len); });

  if (!driver.RunControlLoop(hw)) {
    return -1;
  }

  return 0;
}
