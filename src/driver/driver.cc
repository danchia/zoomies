#include <unistd.h>

#include "driver/controlloop.h"
#include "hw/hw.h"

int main() {
  printf("Initializing...\n");
  HW hw;
  printf("Init done!\n");

  if (!RunControlLoop(hw)) {
    return -1;
  }

  return 0;
}
