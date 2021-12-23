#include "slam/slam_util.h"

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

std::ostream& operator<<(std::ostream& os, const Rect& rect) {
  os << "{" << rect.x << "," << rect.y << "," << rect.width << ","
     << rect.height << "}";
  return os;
}