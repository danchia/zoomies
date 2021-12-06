#include "hw/hw.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cstring>

namespace {

constexpr int kHatAddr = 0x75;
constexpr int kIMUAddr = 0x69;

constexpr uint8_t ACCEL_SHIFT = 3;

bool I2CWrite(int fd, uint8_t addr, uint8_t reg, size_t len, uint8_t *buf) {
  uint8_t write_buf[33];
  if (len > 32) {
    perror("i2c write too large");
  }
  write_buf[0] = reg;
  std::memcpy(write_buf + 1, buf, len);
  struct i2c_rdwr_ioctl_data packet;
  struct i2c_msg message;

  packet.msgs = &message;
  packet.nmsgs = 1;

  message.addr = addr;
  message.flags = 0;
  message.buf = write_buf;
  message.len = len + 1;

  if (ioctl(fd, I2C_RDWR, &packet) < 0) {
    perror("i2c write");
    return false;
  }
  return true;
}

bool I2CRead(int fd, uint8_t addr, uint8_t reg, size_t len, uint8_t *buf) {
  struct i2c_rdwr_ioctl_data packet;
  struct i2c_msg messages[2];

  packet.msgs = messages;
  packet.nmsgs = 2;

  messages[0].addr = addr;
  messages[0].flags = 0;
  messages[0].len = 1;
  messages[0].buf = &reg;

  messages[1].addr = addr;
  messages[1].flags = I2C_M_RD;
  messages[1].len = len;
  messages[1].buf = buf;

  if (ioctl(fd, I2C_RDWR, &packet) < 0) {
    perror("i2c read");
    return false;
  }
  return true;
}

}  // namespace

HW::HW() {
  i2c_file_ = open("/dev/i2c-1", O_RDWR);
  if (i2c_file_ < 0) {
    perror("i2c init");
    exit(1);
  }

  // Init the IMU.
  // Heavily borrowed from
  // https://github.com/a1k0n/cycloid/blob/master/src/hw/imu/invensense.cc
  auto write_imu_or_die = [&](uint8_t reg, uint8_t value) {
    if (!I2CWrite(i2c_file_, kIMUAddr, reg, 1, &value)) {
      fprintf(stderr, "IMU init");
      exit(1);
    }
  };

  write_imu_or_die(107, 0x80);  // reset
  usleep(10000);
  write_imu_or_die(107, 0x00);  // wake
  write_imu_or_die(107, 0x01);  // PLL clock
  write_imu_or_die(108, 0x00);  // enable everything (accel + gyro)

  write_imu_or_die(25, 0x04);  // 200Hz samplerate

  write_imu_or_die(26, 0x03);  // dlpf_cfg = 3, 41Hz gyro b/w, 5.9ms delay
  write_imu_or_die(27, 0x10);  // enable filter, gyro 1000deg/s scale

  write_imu_or_die(28, ACCEL_SHIFT << 3);  // accel sensitivity 16g
  write_imu_or_die(29, 0x03);              // 44.8Hz accel b/w.
}

void HW::SetLedSpeedSteering(uint8_t led, float speed, float steering) {
  speed = std::clamp(speed, -1.0f, 1.0f);
  steering = std::clamp(steering, -1.0f, 1.0f);

  int8_t esc_v = 127.0 * speed;
  int8_t servo_v = 127.0 * steering;
  uint8_t buf[3] = {led, static_cast<uint8_t>(esc_v),
                    static_cast<uint8_t>(servo_v)};

  I2CWrite(i2c_file_, kHatAddr, 0x00, 3, buf);
}

bool HW::ReadSensors(HWSensorReading *values) {
  uint8_t scratch[32];
  if (!I2CRead(i2c_file_, kHatAddr, 0x03, 4, scratch)) {
    return false;
  }
  // Assumes little endian throughout.
  // TODO: convert to meaningful units?
  values->motor_ticks = *(uint16_t *)scratch;
  values->motor_period = *(uint16_t *)(scratch + 2);

  if (!I2CRead(i2c_file_, kIMUAddr, 0x3b, 14, scratch)) {
    return false;
  }
  auto make_int16 = [&scratch](size_t offset) {
    int16_t v = scratch[offset];
    v = (v << 8) | scratch[offset + 1];
    return v;
  };

  // TODO: correct orientation
  // calculations from
  // https://github.com/a1k0n/cycloid/blob/master/src/hw/imu/invensense.cc
  values->accel = Eigen::Vector3f(make_int16(0), make_int16(2), make_int16(4)) /
                  (16384 >> ACCEL_SHIFT);
  values->gyro =
      Eigen::Vector3f(make_int16(8), make_int16(10), make_int16(12)) * 1000.0 *
      M_PI / (180 * 32768.0);
  // correct the axes
  values->accel.x() = -values->accel.x();
  values->accel.y() = -values->accel.y();
  values->gyro.x() = -values->gyro.x();
  values->gyro.y() = -values->gyro.y();

  return true;
}