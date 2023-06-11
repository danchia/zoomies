#include "localization/localizer.h"

Localizer::Localizer(const Options& options)
    : camera_model_(options.img_width, options.img_height,
                    options.camera_model_path),
      light_finder_(camera_model_, options.img_width, options.img_height,
                    options.ceil_height, options.pix_thres, options.min_area,
                    options.ceil_mask_path),
      pf_(options.num_particles, options.map_path),
      ceiling_height_(options.ceil_height) {
  pf_.SeedLocation({-1.0f, -0.3f, -0.4f}, {0.3f, 0.3f, 0.4f});

  for (const auto& m : pf_.map()) {
    map_.push_back({m.x(), m.y(), ceiling_height_});
  }
}

Localizer::FrameResult Localizer::OnVideoFrame(int64_t t_us, uint8_t* img) {
  // NOTE: below alters img
  auto lights = light_finder_.Find(img);

  std::vector<ParticleFilter::Landmark> landmarks;
  landmarks.reserve(lights.size());
  for (const auto& light : lights) {
    landmarks.push_back(
        {.pos = light.pos, .stddev = sqrtf(light.pos_variance)});
  }

  Eigen::Vector3f last_pose = Eigen::Vector3f::Zero();
  std::vector<ParticleFilter::Motion> motions;
  float dist = 0.0f;
  motions.reserve(5);
  {
    std::lock_guard l(mu_);
    for (auto u : updates_) {
      dist += u.motion.delta_dist;
      motions.push_back(std::move(u.motion));
      last_pose = u.pos;
    }
    updates_.clear();
  }

  // PF will not generate enough sample variance without motion.
  if (fabsf(dist) > 1e-4) {
    auto pf_result = pf_.Update(motions, landmarks);

    std::lock_guard l(mu_);
    next_result_ = SyncResult{
        .correction = pf_result.pose - last_pose,
        .pose = pf_result.pose,
        .variance = pf_result.variance,
    };
  } else {
    std::lock_guard l(mu_);
    next_result_.reset();
  }

  Localizer::FrameResult result;
  result.landmarks.reserve(landmarks.size());
  for (const auto& lm : landmarks) {
    result.landmarks.push_back({
        lm.pos.x(),
        lm.pos.y(),
        ceiling_height_,
    });
  }

  return result;
}

std::optional<Localizer::SyncResult> Localizer::ControlSync(
    int64_t t_us, const Eigen::Vector3f& pos, float dist_delta,
    float heading_delta, float stddev_dist, float stddev_heading) {
  std::lock_guard l(mu_);
  updates_.push_back({
      .t_us = t_us,
      .pos = pos,
      .motion =
          {
              .delta_dist = dist_delta,
              .delta_heading = heading_delta,
              .stddev_dist = stddev_dist,
              .stddev_heading = stddev_heading,
          },
  });
  std::optional<SyncResult> res;
  using std::swap;
  swap(res, next_result_);
  return res;
}
