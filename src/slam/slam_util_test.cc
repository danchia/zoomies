#include "slam/slam_util.h"

#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include <fstream>

namespace {

std::vector<uint8_t> TestImage() {
  std::ifstream file("../testdata/frame.bin",
                     std::ios::binary | std::ifstream::in);
  std::vector<uint8_t> img;
  img.resize(640 * 480);
  file.read(reinterpret_cast<char*>(img.data()), 640 * 480);
  file.close();
  return img;
}

TEST(FindRects, Basic) {
  auto img = TestImage();
  auto res = FindRect(640, 480, img.data(), 220, 1000);
  EXPECT_THAT(res, testing::UnorderedElementsAre(Rect{110, 116, 82, 33},
                                                 Rect{287, 143, 62, 43},
                                                 Rect{279, 336, 62, 38}));
}

}  // namespace