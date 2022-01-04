#pragma once

#include <fcntl.h>

class JS {
 public:
  struct State {
    float accel;
    float steer;
    bool a_btn;
    bool y_btn;
  };

  JS();
  ~JS();

  State Poll();

 private:
  int fd_ = 0;
  float accel_ = 0.0f;
  float steer_ = 0.0f;
  bool a_btn_ = false;
  bool y_btn_ = false;
};