#include "localization/localization_util.h"

#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
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

const Eigen::Vector4f K{1.83260687e+02, 1.83044898e+02, 3.18091934e+02,
                        2.50873559e+02};
const Eigen::Vector4f D{0.06782692, -0.03927174, 0.00502321, 0.00063159};

TEST(FisheyeProject, Basic) {
  auto r = FisheyeProject(K, D, {1.06, 2.2, 3.2});
  EXPECT_THAT(r.x(), testing::FloatNear(371.09, 0.01));
  EXPECT_THAT(r.y(), testing::FloatNear(360.75, 0.01));
}

}  // namespace
