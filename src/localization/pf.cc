#include "localization/pf.h"

#include <math.h>

#include <Eigen/Geometry>
#include <fstream>
#include <stats.hpp>

#include "spdlog/spdlog.h"

namespace {
const float kNoDetectionProb = logf(0.1f);
}

ParticleFilter::UpdateResult ParticleFilter::Update(
    const std::vector<Motion>& motion, const std::vector<Landmark>& landmarks) {
  // TODO: our heading representation isn't canonicalized right now.
  // If the robot rotates enough times around in one direction, we'll loose
  // precision. Fixing is not simple though - we have to fix how we're averaging
  // heading.
  for (const auto& m : motion) {
    for (auto& state : states_) {
      float dist_delta = stats::rnorm(m.delta_dist, m.stddev_dist, rand_gen_);
      float heading_delta =
          stats::rnorm(m.delta_heading / 2.0f, m.stddev_heading, rand_gen_);
      state.z() = state.z() + heading_delta;
      state.x() += dist_delta * cos(state.z());
      state.y() += dist_delta * sin(state.z());
      state.z() = state.z() + heading_delta;
    }
  }

  for (int i = 0; i < states_.size(); ++i) {
    const auto& state = states_[i];
    float log_p = 0.0f;
    // Eigen::Transform t = Eigen::Translation2f(state.x(), state.y()) *
    //                      Eigen::Rotation2D(state.z());
    for (const auto& lm : landmarks) {
      Eigen::Vector2f world_pos = Eigen::Rotation2D(state.z()) * lm.pos +
                                  Eigen::Vector2f{state.x(), state.y()};
      float dist = 1e10;
      for (const auto& real_light : map_) {
        float c_dist = (real_light - world_pos).squaredNorm();
        if (c_dist < dist) dist = c_dist;
      }
      dist = sqrtf(dist);
      log_p +=
          std::max(kNoDetectionProb, logf(stats::dnorm(dist, 0.0f, lm.stddev)));
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

  res.pose = Ex / num_particles_ + states_[0];
  res.variance = Ex2 / num_particles_;
  return res;
}

void ParticleFilter::Resample() {
  double total_w = 0.0f;
  for (float& w : weights_) {
    w = expf(w);
    total_w += w;
  }
  std::vector<Eigen::Vector3f> new_states;
  double m_inv = total_w / num_particles_;
  double r = stats::runif(0.0, m_inv, rand_gen_);
  double c = weights_[0];
  double u = r;
  int i = 0;
  for (int m = 0; m < num_particles_; ++m) {
    while (u > c && i < num_particles_ - 1) {
      ++i;
      c += weights_[i];
    }
    new_states.push_back(states_[i]);
    u += m_inv;
  }

  states_.swap(new_states);
  for (auto& w : weights_) w = 0.0f;
}

ParticleFilter::ParticleFilter(int num_particles,
                               const std::string& map_filename)
    : num_particles_(num_particles) {
  std::ifstream f(map_filename);
  if (!f) throw std::runtime_error("could not open map file");
  int n;
  f >> n;
  float x, y;
  for (int i = 0; i < n; ++i) {
    f >> x;
    f >> y;
    if (!f) throw std::runtime_error("error while reading map location");
    map_.emplace_back(x, y);
  }
}

void ParticleFilter::SeedLocation(const Eigen::Vector3f& min,
                                  const Eigen::Vector3f& max) {
  for (int i = 0; i < num_particles_; ++i) {
    Eigen::Vector3f state =
        Eigen::Vector3f{stats::runif(min.x(), max.x(), rand_gen_),
                        stats::runif(min.y(), max.y(), rand_gen_),
                        stats::runif(min.z(), max.z(), rand_gen_)};
    states_.push_back(state);
    weights_.push_back(0.0);
  }
}