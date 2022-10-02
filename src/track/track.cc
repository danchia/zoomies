#include "track/track.h"

#include <inttypes.h>

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>

#include "spdlog/spdlog.h"

namespace {
// Adapated from GoDot.
float lerp_angle(float p_from, float p_to, float p_weight) {
  float difference = fmod(p_to - p_from, static_cast<float>(2.0f * M_PI));
  float distance =
      fmod(2.0f * difference, static_cast<float>(2.0 * M_PI)) - difference;
  return p_from + distance * p_weight;
}
}  // namespace

RacingPath::PathInfo RacingPath::GetPathInfo(float s_guess, float x, float y,
                                             float search_dist) const {
  while (s_guess > total_length_) {
    s_guess -= total_length_;
  }

  // int idx_guess = static_cast<int>(s_guess / segment_length_);
  int idx_guess = 0;
  while (idx_guess + 1 < path_.size() && path_[idx_guess + 1].s < s_guess) {
    ++idx_guess;
  }
  int search_offset = search_dist / segment_length_;

  PathInfo result;
  result.dist_to_closest = 1e9f;
  bool dist_positive = true;
  for (int i = -search_offset; i <= search_offset; ++i) {
    int idx = idx_guess + i;
    const PathPoint& pt = path_at(idx);
    const PathPoint& next_pt = path_at(idx + 1);

    // https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    Eigen::Vector2f v{pt.x, pt.y}, w{next_pt.x, next_pt.y}, p{x, y};
    const Eigen::Vector2f wv = w - v;
    const float l2 = wv.squaredNorm();
    const float t = std::clamp((p - v).dot(wv) / l2, 0.0f, 0.999f);
    Eigen::Vector2f twv = t * wv;
    Eigen::Vector2f closest = v + twv;

    Eigen::Vector2f pclosest = p - closest;
    float dist = pclosest.squaredNorm();
    if (dist < result.dist_to_closest) {
      result.dist_to_closest = dist;
      result.closest_x = closest.x();
      result.closest_y = closest.y();
      result.s = pt.s + twv.norm();
      result.heading = lerp_angle(pt.heading, next_pt.heading, t);
      result.velocity = ((1.0f - t) * pt.velocity) + t * next_pt.velocity;
      result.curvature = ((1.0f - t) * pt.curvature) + t * next_pt.curvature;

      dist_positive = pclosest.x() * wv.y() - pclosest.y() * wv.x() >= 0.0f;
    }
  }
  result.dist_to_closest = std::sqrt(result.dist_to_closest);
  if (!dist_positive) result.dist_to_closest = -result.dist_to_closest;

  return result;
}

RacingPath::RacingPath(std::string_view fs_path) {
  std::ifstream f;
  f.open(std::string(fs_path), std::ios_base::binary);
  if (f.fail()) {
    spdlog::error("error opening racing path file {}", fs_path);
    throw std::runtime_error("racing path");
  }

  int32_t n_segments;
  float scratch[6];

  f.read(reinterpret_cast<char*>(&n_segments), sizeof(n_segments));
  f.read(reinterpret_cast<char*>(&segment_length_), sizeof(segment_length_));
  f.read(reinterpret_cast<char*>(&max_accel_), sizeof(max_accel_));

  for (int32_t i = 0; i < n_segments; ++i) {
    f.read(reinterpret_cast<char*>(scratch), 6 * sizeof(scratch[0]));
    path_.push_back({scratch[0], scratch[1], scratch[2], scratch[3], scratch[4],
                     scratch[5]});
  }

  if (f.fail()) {
    spdlog::error("error reading racing path file {}", fs_path);
    throw std::runtime_error("racing path");
  }

  total_length_ = path_.back().s;

  f.close();

  spdlog::info("Loaded path, total dist {}m, {} segments, max accel {}",
               total_length_, path_.size(), max_accel_);
}
