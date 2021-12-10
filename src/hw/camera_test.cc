#include "hw/camera.h"

#include "spdlog/spdlog.h"

void Callback(uint8_t* buf, int len) { spdlog::info("Got callback!"); }

int main() {
  Camera cam(1280, 720, 30);
  cam.Start(Callback);
  usleep(1000000);
  cam.Stop();
}