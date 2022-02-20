#pragma once

#include <fcntl.h>

#include <atomic>
#include <mutex>
#include <thread>

class JS {
 public:
  struct State {
    float accel = 0.0f;
    float steer = 0.0f;
    bool a_btn = false;
    bool y_btn = false;
  };

  JS();
  ~JS();

  State Poll();
  void ReadLoop();

 private:
  std::thread reader_;
  int fd_ = 0;
  std::atomic<bool> done_ = false;

  std::mutex mu_;
  State s_;
};