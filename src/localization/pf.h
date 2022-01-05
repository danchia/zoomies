#pragma once

#include <Eigen/Dense>
#include <stats.hpp>

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

  UpdateResult Update(const std::vector<Motion>& motion,
                      const std::vector<Landmark>& landmarks);

 private:
  void Resample();

  int num_particles_;
  std::vector<Eigen::Vector3f> states_;
  std::vector<float> weights_;
  stats::rand_engine_t rand_gen_;

  std::vector<Eigen::Vector2f> map_;
};