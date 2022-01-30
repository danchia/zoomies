#pragma once

#include <inttypes.h>

#include <Eigen/Dense>
#include <iostream>
#include <string_view>
#include <vector>

struct Rect {
  int x, y, width, height;

  bool operator==(const Rect& o) const {
    return x == o.x && y == o.y && width == o.width && height == o.height;
  }
  friend std::ostream& operator<<(std::ostream& os, const Rect& rect);
};

// NOTE: modifies img.
std::vector<Rect> FindRect(int width, int height, uint8_t* img,
                           uint8_t pix_thres, int min_area);

// K = [fx fy cx cy]
Eigen::Vector2f FisheyeProject(Eigen::Vector4f K, Eigen::Vector4f D,
                               Eigen::Vector3f point);

// Normalize angle between [-pi, pi]
float normAngle(float x);

class CameraModel {
 public:
  CameraModel(int width, int height, std::string_view lut_file);
  Eigen::Vector2f Lookup(int u, int v) const {
    int idx = u + v * width_;
    return Eigen::Vector2f{camera_lut_[idx * 2], camera_lut_[idx * 2 + 1]};
  }

 private:
  int width_, height_;
  std::vector<float> camera_lut_;
};

class LightFinder {
 public:
  struct Light {
    Eigen::Vector2f pos;
    float pos_variance;
    int u, v;
  };
  LightFinder(const CameraModel& camera_model, int width, int height,
              float ceiling_height, uint8_t pix_thres, int min_area,
              std::string_view ceil_mask_file);

  // NOTE: modifies img.
  std::vector<Light> Find(
      uint8_t* img,
      std::vector<Eigen::Vector2f>* thresholded_positions = nullptr);

 private:
  int width_, height_;
  float ceiling_height_;
  uint8_t pix_thres_;
  int min_area_;
  const CameraModel& camera_model_;
  std::vector<uint8_t> ceil_mask_;
};