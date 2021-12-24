#pragma once

#include <inttypes.h>

#include <iostream>
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
