#pragma once

#include <inttypes.h>
#include <stdio.h>

#include <atomic>

#include "hw/hw.h"

class Driver {
 public:
  Driver();
  ~Driver();

  bool RunControlLoop(HW& hw);
  void OnCameraTick(uint8_t* buf, int len);

 private:
  std::atomic<int64_t> ticks_;
  FILE* vid_file_ = nullptr;
};
