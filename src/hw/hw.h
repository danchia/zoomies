#pragma once
#include <stdint.h>

#include "Eigen/Dense"

struct HWSensorReading {
  uint16_t motor_ticks;
  uint16_t motor_period;
  Eigen::Vector3f accel;  // in g
  Eigen::Vector3f gyro;   // rad/s
};

class HW {
 public:
  HW();

  void SetLedSpeedSteering(uint8_t led, float speed, float steering);
  bool ReadSensors(HWSensorReading* values);

 private:
  int i2c_file_;
};