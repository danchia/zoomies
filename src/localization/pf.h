#pragma once

#include <Eigen/Dense>
#include <stats.hpp>
#include <string>

class ParticleFilter {
 public:
  struct Motion {
    float delta_dist, delta_heading;
    float stddev_dist, stddev_heading;
  };
  struct Landmark {
    Eigen::Vector2f pos;
    float stddev;
  };
  struct UpdateResult {
    Eigen::Vector3f pose;
    Eigen::Vector3f variance;
  };

  ParticleFilter(int num_particles, const std::string& map_filename);

  void SeedLocation(const Eigen::Vector3f& min, const Eigen::Vector3f& max);
  UpdateResult Update(const std::vector<Motion>& motion,
                      const std::vector<Landmark>& landmarks);

  std::pair<const std::vector<Eigen::Vector3f>&, std::vector<float>&>
  pose_particles() {
    return {states_, weights_};
  }
  const std::vector<Eigen::Vector2f>& map() { return map_; }

 private:
  void Resample();

  int num_particles_;
  std::vector<Eigen::Vector3f> states_;
  std::vector<float> weights_;  // log prob
  stats::rand_engine_t rand_gen_;

  std::vector<Eigen::Vector2f> map_;
};