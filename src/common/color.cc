#include "common/color.h"

#include <algorithm>

void YUV420ToRGB(int width, int height, const uint8_t* in, uint8_t* out) {
  const uint8_t* y = in;
  const uint8_t* u = &in[width * height];
  const uint8_t* v = &in[width * height + (width / 2) * (height / 2)];

  for (int j = 0; j < height; j++) {
    for (int i = 0; i < width; i++) {
      int yy = y[(j * width) + i];
      int uu = u[((j / 2) * (width / 2)) + (i / 2)];
      int vv = v[((j / 2) * (width / 2)) + (i / 2)];

      int r = 1.164 * (yy - 16) + 1.596 * (vv - 128);
      int g = 1.164 * (yy - 16) - 0.813 * (vv - 128) - 0.391 * (uu - 128);
      int b = 1.164 * (yy - 16) + 2.018 * (uu - 128);
      *out++ = std::clamp(r, 0, 255);
      *out++ = std::clamp(g, 0, 255);
      *out++ = std::clamp(b, 0, 255);
    }
  }
}