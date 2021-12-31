#include "track/track.h"

#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

namespace {

using testing::FloatNear;

TEST(RacingPath, Basic) {
  // clang-format off
  /*
PathPoint(s=0.0, heading=0.0, velocity=0.5, x=1.5308084989341915e-17, y=0.0)
PathPoint(s=0.05, heading=0.10000000000000003, velocity=0.5318309886183791, x=0.04966733269876531, y=0.0049833555396895934)
PathPoint(s=0.1, heading=0.29999999999999993, velocity=0.5636619772367581, x=0.09735458557716262, y=0.019734751499278724)
PathPoint(s=0.15000000000000002, heading=0.5000000000000001, velocity=0.5954929658551372, x=0.14116061834875887, y=0.043666096272580446)
PathPoint(s=0.2, heading=0.7000000000000005, velocity=0.6273239544735163, x=0.1793390227248807, y=0.07582332266320868)
PathPoint(s=0.25, heading=0.8999999999999999, velocity=0.6591549430918953, x=0.21036774620197413, y=0.11492442353296509)
PathPoint(s=0.3, heading=1.0999999999999994, velocity=0.6909859317102744, x=0.2330097714918066, y=0.1594105613808316)
PathPoint(s=0.35, heading=1.3000000000000005, velocity=0.7228169203286534, x=0.24636243249711504, y=0.20750821427493976)
PathPoint(s=0.39, heading=1.4999999999999996, velocity=0.7546479089470325, x=0.2498934007603763, y=0.25729988057532216)
PathPoint(s=0.44, heading=1.7, velocity=0.7864788975654116, x=0.24346190771954881, y=0.3068005236732717)
PathPoint(s=0.49, heading=1.9000000000000004, velocity=0.8183098861837906, x=0.22732435670642043, y=0.35403670913678553)
PathPoint(s=0.54, heading=2.0999999999999996, velocity=0.8501408748021697, x=0.20212410095489758, y=0.39712527931383634)
PathPoint(s=0.6, heading=2.299999999999999, velocity=0.8819718634205488, x=0.16886579513778774, y=0.4343484288853114)
PathPoint(s=0.65, heading=2.5000000000000004, velocity=0.9138028520389279, x=0.128875342955366, y=0.46422218834223683)
PathPoint(s=0.70, heading=2.7, velocity=0.945633840657307, x=0.08374703753897615, y=0.48555558516716457)
PathPoint(s=0.75, heading=2.9, velocity=0.9774648292756861, x=0.03528000201496668, y=0.4974981241501114)
PathPoint(s=0.80, heading=3.100000000000002, velocity=0.9907041821059348, x=-0.014593535856895148, y=0.4995736939486882)
PathPoint(s=0.85, heading=-2.9831853071795855, velocity=0.9588731934875557, x=-0.06388527550670804, y=0.49169954814486516)
PathPoint(s=0.90, heading=-2.783185307179586, velocity=0.9270422048691767, x=-0.11063011082371332, y=0.4741896040835366)
PathPoint(s=0.95, heading=-2.5831853071795865, velocity=0.8952112162507975, x=-0.15296447273568, y=0.447741927978604)
PathPoint(s=1.00, heading=-2.383185307179585, velocity=0.8633802276324185, x=-0.18920062382698222, y=0.4134109052159028)
PathPoint(s=1.05, heading=-2.183185307179586, velocity=0.8315492390140394, x=-0.21789394310339716, y=0.3725652053351747)
PathPoint(s=1.10, heading=-1.9831853071795846, velocity=0.7997182503956604, x=-0.23790051847237909, y=0.3268332174946046)
PathPoint(s=1.15, heading=-1.7831853071795851, velocity=0.7678872617772812, x=-0.24842275090836616, y=0.27803813173376324)
PathPoint(s=1.20, heading=-1.5831853071795852, velocity=0.7360562731589021, x=-0.24904115220896014, y=0.22812525414013796)
PathPoint(s=1.25, heading=-1.3831853071795843, velocity=0.704225284540523, x=-0.23973106866578447, y=0.179084453634193)
PathPoint(s=1.30, heading=-1.183185307179585, velocity=0.6723942959221438, x=-0.2208636639300381, y=0.1328708321749053)
PathPoint(s=1.35, heading=-0.9831853071795839, velocity=0.6405633073037649, x=-0.19319112188899648, y=0.091326781014341)
PathPoint(s=1.40, heading=-0.7831853071795846, velocity=0.6087323186853857, x=-0.15781665946807988, y=0.05610853037243718)
PathPoint(s=1.45, heading=-0.5831853071795838, velocity=0.5769013300670066, x=-0.11615054485343874, y=0.028620120764669965)
PathPoint(s=1.50, heading=-0.3831853071795838, velocity=0.5450703414486275, x=-0.06985387454973081, y=0.009957428337408314)
PathPoint(s=1.55, heading=-0.18318530717958387, velocity=0.5132393528302484, x=-0.020772350704373423, y=0.0008644757441955697)
  */
  // clang-format on

  RacingPath path("../testdata/path_test.bin");
  constexpr float kTol = 0.001f;
  constexpr float kSearchDist = 0.1f;

  {
    auto pinfo = path.GetPathInfo(0.0, 0.00, 0.00, kSearchDist);

    EXPECT_THAT(pinfo.s, FloatNear(0.0f, kTol));
    EXPECT_THAT(pinfo.heading, FloatNear(0.0f, kTol));
    EXPECT_THAT(pinfo.velocity, FloatNear(0.5f, kTol));
    EXPECT_THAT(pinfo.closest_x, FloatNear(0.0f, kTol));
    EXPECT_THAT(pinfo.closest_y, FloatNear(0.0f, kTol));
    EXPECT_THAT(pinfo.dist_to_closest, FloatNear(0.0f, kTol));
  }

  {
    auto pinfo = path.GetPathInfo(0.025, 0.02, 0.02, kSearchDist);

    EXPECT_THAT(pinfo.s, FloatNear(0.022f, kTol));
    EXPECT_THAT(pinfo.heading, FloatNear(0.044f, kTol));
    EXPECT_THAT(pinfo.velocity, FloatNear(0.514f, kTol));
    EXPECT_THAT(pinfo.closest_x, FloatNear(0.022f, kTol));
    EXPECT_THAT(pinfo.closest_y, FloatNear(0.002f, kTol));
    EXPECT_THAT(pinfo.dist_to_closest, FloatNear(-0.018f, kTol));
  }

  {
    auto pinfo = path.GetPathInfo(0.8, -0.04, 0.5, kSearchDist);

    EXPECT_THAT(pinfo.s, FloatNear(0.825f, kTol));
    EXPECT_THAT(pinfo.heading, FloatNear(3.2f, kTol));
    EXPECT_THAT(pinfo.velocity, FloatNear(0.975f, kTol));
    EXPECT_THAT(pinfo.closest_x, FloatNear(-0.039f, kTol));
    EXPECT_THAT(pinfo.closest_y, FloatNear(0.496f, kTol));
    EXPECT_THAT(pinfo.dist_to_closest, FloatNear(0.004f, kTol));
  }
}

}  // namespace