#pragma once

#include <inttypes.h>

#include <functional>

#include "interface/mmal/mmal.h"

class Camera {
 public:
  Camera(uint32_t width, uint32_t height, uint32_t fps);

  ~Camera() {
    if (camera_ != nullptr) {
      Stop();
    }
  }

  bool Start(std::function<void(uint8_t* buf, int len)> cb);
  void Stop();
  void BufferCallback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);

 private:
  uint32_t width_;
  uint32_t height_;
  uint32_t fps_;

  MMAL_COMPONENT_T* camera_ = nullptr;
  MMAL_POOL_T* pool_ = nullptr;
  std::function<void(uint8_t* buf, int len)> cb_;
};