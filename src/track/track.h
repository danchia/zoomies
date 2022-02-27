#pragma once

#include <string_view>
#include <vector>

// Racing line represented via (s, heading(s), v(s)).
//  - s is distance along path.
//  - heading(s) is the heading at s.
//  - v(s) is the fwd velocity at s.
//
// The line is stored as a series of discretized (s, K(s)) points,
// with courtesy (x, y) information to aid in calculating closest point on the
// racing line.
class RacingPath {
 public:
  struct PathInfo {
    float s;
    float heading;
    float velocity;
    float closest_x;
    float closest_y;
    float dist_to_closest;
  };
  struct PathPoint {
    float s;
    float heading;
    float velocity;
    float x, y;
  };

  RacingPath(std::string_view fs_path);

  PathInfo GetPathInfo(float s_guess, float x, float y,
                       float search_dist = 0.10f) const;
  float total_length() const { return total_length_; }
  float max_accel() const { return max_accel_; }

  const std::vector<PathPoint>& path_points() { return path_; }

 private:
  const PathPoint& path_at(int idx) const {
    int sz = path_.size();
    while (idx < 0) idx += sz;
    while (idx >= sz) idx -= sz;
    return path_[idx];
  }

  float segment_length_;
  float total_length_;
  float max_accel_;
  std::vector<PathPoint> path_;
};