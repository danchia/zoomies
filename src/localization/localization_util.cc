#include "localization/localization_util.h"

#include <math.h>

#include <fstream>
#include <limits>

namespace {

constexpr float kLightMinStddev = 0.3f;
constexpr float kLightDistFactor = 0.15f;

struct ImgPoint {
  int u = 0;
  int v = 0;
};
struct FindCenterAlgo {
  void Find(int u, int v) {
    if (img[v * width + u] < thres) return;

    img[v * width + u] = 0;

    ++area;
    avg.u += u;
    avg.v += v;

    for (int du = -1; du <= 1; ++du) {
      for (int dv = -1; dv <= 1; ++dv) {
        int nu = u + du;
        int nv = v + dv;
        if (nu >= 0 && nu < width && nv >= 0 && nv < height) {
          Find(nu, nv);
        }
      }
    }
  };

  int width;
  int height;
  uint8_t* img;
  uint8_t thres;
  ImgPoint avg;

  int area = 0;
};
}  // namespace

std::vector<ImgPoint> FindCenter(int width, int height, uint8_t* img,
                                 uint8_t pix_thres, int min_area) {
  std::vector<ImgPoint> res;

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      uint8_t x = img[v * width + u];
      if (x >= pix_thres) {
        struct FindCenterAlgo finder;
        finder.width = width;
        finder.height = height;
        finder.img = img;
        finder.thres = pix_thres;

        finder.Find(u, v);
        finder.avg.u /= finder.area;
        finder.avg.v /= finder.area;

        if (finder.area > min_area) {
          res.push_back(finder.avg);
        }
      }
    }
  }

  return res;
}

Eigen::Vector2f FisheyeProject(Eigen::Vector4f K, Eigen::Vector4f D,
                               Eigen::Vector3f point) {
  if (fabs(point.z()) < 0.0001) point.z() = 1.0f;
  Eigen::Vector2f x{point.x() / point.z(), point.y() / point.z()};
  double r = std::sqrt(x.dot(x));
  double theta = atan(r);
  double theta3 = theta * theta * theta;
  double theta5 = theta3 * theta * theta;
  double theta7 = theta5 * theta * theta;
  double theta9 = theta7 * theta * theta;
  double theta_d =
      theta + D[0] * theta3 + D[1] * theta5 + D[2] * theta7 + D[3] * theta9;

  Eigen::Vector2f xp = (theta_d / r) * x;
  return Eigen::Vector2f{K[0] * xp[0] + K[2], K[1] * xp[1] + K[3]};
}

float normAngle(float x) {
  while (x > M_PI) {
    x -= 2.0 * M_PI;
  }
  while (x < -M_PI) {
    x += 2.0 * M_PI;
  }
  return x;
}

CameraModel::CameraModel(int width, int height, const std::string& lut_file)
    : width_(width), height_(height) {
  camera_lut_.resize(width_ * height_ * 2);
  std::ifstream f(lut_file, std::ios::in | std::ios::binary);
  if (!f.good()) throw std::runtime_error("error opening lut");
  f.read(reinterpret_cast<char*>(camera_lut_.data()), width_ * height_ * 2 * 4);
  if (!f) throw std::runtime_error("error while reading lut");
  f.close();
}

LightFinder::LightFinder(const CameraModel& camera_model, int width, int height,
                         float ceiling_height, uint8_t pix_thres, int min_area,
                         const std::string& ceil_mask_file)
    : width_(width),
      height_(height),
      ceiling_height_(ceiling_height),
      pix_thres_(pix_thres),
      min_area_(min_area),
      camera_model_(camera_model) {
  ceil_mask_.resize(width_ * height_);

  std::ifstream f(ceil_mask_file, std::ios::in | std::ios::binary);
  if (!f.good()) throw std::runtime_error("error opening ceil_mask");
  f.read(reinterpret_cast<char*>(ceil_mask_.data()), width_ * height_);
  if (!f) throw std::runtime_error("error while reading lut");
  f.close();
}

std::vector<LightFinder::Light> LightFinder::Find(
    uint8_t* img, std::vector<Eigen::Vector2f>* thresholded_positions) {
  int size = width_ * height_;
  for (int i = 0; i < size; ++i) {
    img[i] &= ceil_mask_[i];
  }

  if (thresholded_positions != nullptr) {
    thresholded_positions->clear();

    for (int v = 0; v < height_; ++v) {
      for (int u = 0; u < width_; ++u) {
        if (img[v * width_ + u] > pix_thres_) {
          thresholded_positions->push_back(camera_model_.Lookup(u, v) *
                                           ceiling_height_);
        }
      }
    }
  }

  auto centers = FindCenter(width_, height_, img, pix_thres_, min_area_);

  std::vector<Light> lights;
  lights.reserve(centers.size());
  for (const auto& center : centers) {
    int u = center.u;
    int v = center.v;
    Light& l = lights.emplace_back();
    l.u = u;
    l.v = v;
    l.pos = camera_model_.Lookup(u, v) * ceiling_height_;
    l.pos_variance =
        std::max(kLightMinStddev * kLightMinStddev,
                 l.pos.squaredNorm() * kLightDistFactor * kLightDistFactor);
  }

  return lights;
}
