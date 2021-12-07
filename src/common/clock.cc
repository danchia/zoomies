#include "common/clock.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

namespace {
int64_t MonoClockNow() {
  struct timespec t;
  if (clock_gettime(CLOCK_MONOTONIC, &t) == -1) {
    perror("clock_gettime");
    exit(1);
  }

  return ((int64_t)t.tv_sec) * 1e6 + t.tv_nsec / 1e3;
}
}  // namespace

Clock::Clock() : start_micros_(MonoClockNow()) {}

void Clock::Reset() { start_micros_ = MonoClockNow(); }

int64_t Clock::ElapsedMicros() { return MonoClockNow() - start_micros_; }
