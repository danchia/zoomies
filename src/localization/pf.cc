#include "localization/pf.h"

#include <math.h>

#include <Eigen/Geometry>
#include <stats.hpp>

namespace {
const float kNoDetectionProb = logf(0.1f);
}

ParticleFilter::UpdateResult ParticleFilter::Update(
    const std::vector<Motion>& motion, const std::vector<Landmark>& landmarks) {
  for (const auto& m : motion) {
    for (auto& state : states_) {
      float dist_delta = stats::rnorm(m.delta_dist, m.stddev_dist, rand_gen_);
      float heading_delta =
          stats::rnorm(m.delta_heading / 2.0f, m.stddev_heading, rand_gen_);
      state.z() = state.z() + heading_delta;
      state.x() += dist_delta * cos(state.z());
      state.y() += dist_delta * sin(state.z());
    }
  }

  for (int i = 0; i < states_.size(); ++i) {
    const auto& state = states_[i];
    float log_p = 0.0f;
    Eigen::Transform t = Eigen::Translation2f(state.x(), state.y()) *
                         Eigen::Rotation2D(state.z());
    for (const auto& lm : landmarks) {
      auto world_pos = t * lm.pos;
      float dist = 1e10;
      for (const auto& real_light : map_) {
        float c_dist = (real_light - world_pos).squaredNorm();
        if (c_dist < dist) dist = c_dist;
      }
      dist = sqrtf(dist);
      log_p +=
          std::max(kNoDetectionProb, logf(stats::dnorm(dist, 0, lm.stddev)));
    }
    weights_[i] = log_p;
  }

  Resample();

  UpdateResult res;
  Eigen::Vector3f K = states_[0];
  Eigen::Vector3f Ex = Eigen::Vector3f::Zero();
  Eigen::Vector3f Ex2 = Eigen::Vector3f::Zero();

  for (const auto& s : states_) {
    auto diff = s - K;
    Ex += diff;
    Ex2 += Eigen::Vector3f(diff.array().square());
  }

  res.pose = Ex / num_particles_;
  res.variance = Ex2 / num_particles_;
  return res;
}

void ParticleFilter::Resample() {
  std::vector<Eigen::Vector3f> new_states;
  double m_inv = 1.0 / num_particles_;
  double r = stats::runif(0, m_inv, rand_gen_);
  double c = exp(weights_[0]);
  double u = r;
  int i = 0;
  for (int m = 0; m < num_particles_; ++m) {
    while (u > c && i < num_particles_ - 1) {
      ++i;
      c += exp(weights_[i]);
    }
    new_states.push_back(states_[i]);
    u += m_inv;
  }

  states_.swap(new_states);
  for (auto& w : weights_) w = 0.0f;
}