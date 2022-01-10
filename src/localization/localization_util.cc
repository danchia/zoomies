#include "localization/localization_util.h"

#include <math.h>

#include <fstream>
#include <limits>

namespace {
struct ImgPoint {
  int u = 0;
  int v = 0;
};
struct FindRectAlgo {
  void Find(int u, int v) {
    if (img[v * width + u] < thres) return;

    img[v * width + u] = 0;

    ++area;
    if (u < top_left.u) {
      top_left.u = u;
    } else if (u > bottom_right.u) {
      bottom_right.u = u;
    }
    if (v < top_left.v) {
      top_left.v = v;
    } else if (v > bottom_right.v) {
      bottom_right.v = v;
    }

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

  int area = 0;
  ImgPoint top_left = ImgPoint{
      std::numeric_limits<int>::max(),
      std::numeric_limits<int>::max(),
  };
  ImgPoint bottom_right = ImgPoint{
      std::numeric_limits<int>::min(),
      std::numeric_limits<int>::min(),
  };
};
}  // namespace

std::vector<Rect> FindRect(int width, int height, uint8_t* img,
                           uint8_t pix_thres, int min_area) {
  std::vector<Rect> res;

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      uint8_t x = img[v * width + u];
      if (x >= pix_thres) {
        struct FindRectAlgo finder;
        finder.width = width;
        finder.height = height;
        finder.img = img;
        finder.thres = pix_thres;

        finder.Find(u, v);

        if (finder.area > min_area) {
          res.push_back({finder.top_left.u, finder.top_left.v,
                         finder.bottom_right.u - finder.top_left.u,
                         finder.bottom_right.v - finder.top_left.v});
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

std::ostream& operator<<(std::ostream& os, const Rect& rect) {
  os << "{" << rect.x << "," << rect.y << "," << rect.width << ","
     << rect.height << "}";
  return os;
}

CameraModel::CameraModel(int width, int height, std::string_view lut_file)
    : width_(width), height_(height) {
  camera_lut_.resize(width_ * height_ * 2);
  std::ifstream f("../data/calib/camera_lut.bin",
                  std::ios::in | std::ios::binary);
  if (!f.good()) throw std::runtime_error("error opening lut");
  f.read(reinterpret_cast<char*>(camera_lut_.data()), width_ * height_ * 2 * 4);
  if (!f) throw std::runtime_error("error while reading lut");
  f.close();
}

LightFinder::LightFinder(const CameraModel& camera_model, int width, int height,
                         float ceiling_height, uint8_t pix_thres, int min_area,
                         std::string_view ceil_mask_file)
    : width_(width),
      height_(height),
      ceiling_height_(ceiling_height),
      pix_thres_(pix_thres),
      min_area_(min_area),
      camera_model_(camera_model) {
  ceil_mask_.resize(width_ * height_);

  std::ifstream f("../data/calib/ceil_mask.bin",
                  std::ios::in | std::ios::binary);
  if (!f.good()) throw std::runtime_error("error opening ceil_mask");
  f.read(reinterpret_cast<char*>(ceil_mask_.data()), width_ * height_);
  if (!f) throw std::runtime_error("error while reading lut");
  f.close();
}

std::vector<LightFinder::Light> LightFinder::Find(uint8_t* img) {
  auto rects = FindRect(width_, height_, img, pix_thres_, min_area_);
  return {};
}