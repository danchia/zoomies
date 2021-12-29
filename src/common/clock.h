#pragma once

#include <inttypes.h>

class Clock {
 public:
  Clock();
  void Reset();
  int64_t ElapsedMicros() const;

 private:
  int64_t start_micros_;
};