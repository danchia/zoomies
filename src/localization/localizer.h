#pragma once

#include <Eigen/Dense>
#include <mutex>
#include <optional>
#include <vector>

#include "localization/localization_util.h"
#include "localization/pf.h"

class Localizer {
 public:
  struct Options {
    int img_width;
    int img_height;
    float ceil_height;
    std::string camera_model_path;
    std::string ceil_mask_path;
    std::string map_path;
    int num_particles = 300;
    uint8_t pix_thres = 220;
    int min_area = 300;
  };

  struct SyncResult {
    // Correction to add to state.
    Eigen::Vector3f correction;

    // Estimated pose.
    Eigen::Vector3f pose;
    // Uncertainty in estimated pose.
    Eigen::Vector3f variance;
  };

  struct FrameResult {
    std::vector<Eigen::Vector3f> landmarks;
  };

  Localizer(const Options& opts);

  FrameResult OnVideoFrame(int64_t t_us, uint8_t* img);

  std::optional<SyncResult> ControlSync(int64_t t_us,
                                        const Eigen::Vector3f& pos,
                                        float dist_delta, float heading_delta,
                                        float stddev_dist,
                                        float stddev_heading);

  const std::vector<Eigen::Vector3f>& map() { return map_; }

 private:
  struct ControlUpdate {
    int64_t t_us;
    Eigen::Vector3f pos;
    ParticleFilter::Motion motion;
  };

  CameraModel camera_model_;
  LightFinder light_finder_;
  ParticleFilter pf_;
  float ceiling_height_;
  std::vector<Eigen::Vector3f> map_;

  std::mutex mu_;
  std::vector<ControlUpdate> updates_;
  std::optional<SyncResult> next_result_;
};
