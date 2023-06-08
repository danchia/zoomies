#include <inttypes.h>

#include <Eigen/Geometry>
#include <fstream>
#include <memory>
#include <optional>
#include <string_view>

#include "common/color.h"
#include "localization/localization_util.h"
#include "localization/pf.h"
#include "mcap/reader.hpp"
#include "mcap/writer.h"
#include "proto/proto_util.h"
#include "ros/geometry_msgs/PoseStamped.pb.h"
#include "ros/sensor_msgs/Image.pb.h"
#include "ros/sensor_msgs/PointCloud2.pb.h"
#include "ros/visualization_msgs/ImageMarker.pb.h"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
#include "zoomies/zoomies.pb.h"

namespace {

constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;
constexpr float kCeilHeight = 2.5f;
constexpr int kNumParticles = 300;
constexpr float kCameraHeight = 0.095f;

class MapMaker {
 public:
  MapMaker(const CameraModel& camera_model, const std::string& floor_mask_file,
           int cam_width, int cam_height, float resolution, float min_x,
           float min_y, float max_x, float max_y)
      : camera_model_(camera_model),
        cam_height_(cam_height),
        cam_width_(cam_width),
        resolution_(resolution),
        min_x_(min_x),
        min_y_(min_y),
        max_x_(max_x),
        max_y_(max_y),
        map_width_((max_x - min_x) / resolution + 1),
        map_height_((max_y - min_y) / resolution + 1),
        map_(map_width_ * map_height_ * 3),
        dist_(map_width_ * map_height_, 100.0f) {
    floor_mask_.resize(cam_width_ * cam_height_);

    std::ifstream f(floor_mask_file, std::ios::in | std::ios::binary);
    if (!f.good()) throw std::runtime_error("error opening floor_mask");
    f.read(reinterpret_cast<char*>(floor_mask_.data()),
           cam_width_ * cam_height_);
    if (!f) throw std::runtime_error("error while reading lut");
    f.close();
  }

  void Update(const Eigen::Vector3f& pose, const std::vector<uint8_t> img) {
    Eigen::Transform t =
        Eigen::Translation2f(pose.x(), pose.y()) * Eigen::Rotation2D(pose.z());

    for (int v = 0; v < cam_height_; ++v) {
      for (int u = 0; u < cam_width_; ++u) {
        if (floor_mask_[v * kImageWidth + u] == 0) continue;

        int img_idx = (v * kImageWidth + u) * 3;
        uint8_t ir = img[img_idx];
        uint8_t ig = img[img_idx + 1];
        uint8_t ib = img[img_idx + 2];

        Eigen::Vector2f floor_pos = camera_model_.Lookup(u, v) * kCameraHeight;
        float nd = floor_pos.norm();
        floor_pos = t * floor_pos;
        int idx = index(floor_pos.x(), floor_pos.y());
        auto& r = map_[idx];
        auto& g = map_[idx + 1];
        auto& b = map_[idx + 2];

        if (r == 0 && g == 0 && b == 0) {
          r = ir;
          g = ig;
          b = ib;
          continue;
        }

        float& dist = dist_[idx / 3];
        float od = dist;
        r = (od * ir + nd * r) / (od + nd);
        g = (od * ig + nd * g) / (od + nd);
        b = (od * ib + nd * b) / (od + nd);
        dist = (2 * od * nd) / (od + nd);
      }
    }
  }

  int height() { return map_height_; }
  int width() { return map_width_; }
  const std::vector<uint8_t>& map() { return map_; }

 private:
  int index(float x, float y) const {
    x = std::clamp(x, min_x_, max_x_ - 1e-6f);
    y = std::clamp(y, min_y_, max_y_ - 1e-6f);
    int ix = (x - min_x_) / resolution_;
    // Flip y since images have y down, but world coord is y up.
    int iy = map_height_ - ((y - min_y_) / resolution_) - 1;
    return (iy * map_width_ + ix) * 3;
  }

  const CameraModel& camera_model_;
  std::vector<uint8_t> floor_mask_;
  int cam_width_;
  int cam_height_;

  float resolution_;
  float min_x_;
  float min_y_;
  float max_x_;
  float max_y_;
  int map_width_;
  int map_height_;

  std::vector<uint8_t> map_;
  std::vector<float> dist_;
};

class Localizer {
 public:
  Localizer();
  void VideoFrame(int64_t t_us, const std::vector<uint8_t>& img);
  void OdoFrame(int64_t t_us, float odo_dist_delta, float imu_rot_z,
                float odo_heading_delta, float dist_stddev,
                float heading_stddev, float x, float y, float z);

 private:
  void InitViz();

  CameraModel camera_model_;
  LightFinder light_finder_;
  ParticleFilter pf_;
  MapMaker map_maker_;

  std::vector<ParticleFilter::Motion> motions_;

  int64_t last_floormap_us_ = 0;

  // Viz
  std::unique_ptr<McapLogWriter> ros_writer_;
  int img_topic_;
  int img_rgb_topic_;
  int floor_map_topic_;
  int landmark_img_overlay_topic_;
  int pf_topic_;
  int map_topic_;
  int landmark_detected_topic_;
  int pf_pose_topic_;
  int log_pose_topic_;
};

Localizer::Localizer()
    : camera_model_(kImageWidth, kImageHeight, "../data/calib/camera_lut.bin"),
      light_finder_(camera_model_, kImageWidth, kImageHeight, kCeilHeight, 220,
                    300, "../data/calib/ceil_mask.bin"),
      pf_(kNumParticles, "../data/map/map.txt"),
      map_maker_(camera_model_, "../data/calib/floor_mask.bin", kImageWidth,
                 kImageHeight, 0.02f, -10.0f, -10.0f, 10.0f, 10.0f) {
  InitViz();

  pf_.SeedLocation({-1.0f, -0.3f, -0.4f}, {0.3f, 0.3f, 0.4f});
}

void Localizer::InitViz() {
  ros_writer_ = McapLogWriter::Make("/tmp/pfviz");
  img_topic_ = ros_writer_->AddChannel("/camera1/image",
                                       ros::sensor_msgs::Image::descriptor());
  img_rgb_topic_ = ros_writer_->AddChannel(
      "/camera1/rgb_image", ros::sensor_msgs::Image::descriptor());
  pf_topic_ = ros_writer_->AddChannel(
      "/localization/pf_cloud", ros::sensor_msgs::PointCloud2::descriptor());
  landmark_img_overlay_topic_ = ros_writer_->AddChannel(
      "/landmarks/img_overlay",
      ros::visualization_msgs::ImageMarker::descriptor());
  map_topic_ = ros_writer_->AddChannel(
      "/landmarks/map", ros::sensor_msgs::PointCloud2::descriptor());
  landmark_detected_topic_ = ros_writer_->AddChannel(
      "/landmarks/detected", ros::sensor_msgs::PointCloud2::descriptor());
  floor_map_topic_ = ros_writer_->AddChannel(
      "/floormap", ros::sensor_msgs::Image::descriptor());
  pf_pose_topic_ = ros_writer_->AddChannel(
      "/localization/pf_pose", ros::geometry_msgs::PoseStamped::descriptor());
  log_pose_topic_ = ros_writer_->AddChannel(
      "/log_pose", ros::geometry_msgs::PoseStamped::descriptor());
}

void Localizer::VideoFrame(int64_t t_us, const std::vector<uint8_t>& img) {
  spdlog::info("vid frame t:{}. motions: {}", t_us, motions_.size());

  // only extract the Y component
  std::vector<uint8_t> img_copy(img.begin(),
                                img.begin() + (kImageWidth * kImageHeight));
  auto lights = light_finder_.Find(img_copy.data());

  std::vector<uint8_t> rgb_img(kImageWidth * kImageHeight * 3);
  YUV420ToRGB(kImageWidth, kImageHeight, img.data(), rgb_img.data());

  std::vector<ParticleFilter::Landmark> landmarks;
  landmarks.reserve(lights.size());
  for (const auto& light : lights) {
    landmarks.push_back(
        {.pos = light.pos, .stddev = sqrtf(light.pos_variance)});
  }

  auto pf_result = pf_.Update(motions_, landmarks);
  // auto pf_result = pf_.Update(motions_, {});
  motions_.clear();

  spdlog::info("PF loc: {} {} {}, var: {} {} {}", pf_result.pose.x(),
               pf_result.pose.y(), pf_result.pose.z(), pf_result.variance.x(),
               pf_result.variance.y(), pf_result.variance.z());

  map_maker_.Update(pf_result.pose, rgb_img);

  // Update visuals
  {
    ros::geometry_msgs::PoseStamped pose;
    *pose.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
    pose.mutable_header()->set_frame_id("/world");
    pose.mutable_pose()->mutable_position()->set_x(pf_result.pose.x());
    pose.mutable_pose()->mutable_position()->set_y(pf_result.pose.y());
    *pose.mutable_pose()->mutable_orientation() =
        HeadingToQuat(pf_result.pose.z());
    ros_writer_->Write(pf_pose_topic_, t_us, pose);
  }

  auto [particles_r, weights] = pf_.pose_particles();
  std::vector<Eigen::Vector4f> particles;
  float max_w = -1e10f;
  for (float w : weights) {
    if (w > max_w) max_w = w;
  }
  for (int i = 0; i < particles_r.size(); ++i) {
    particles.push_back({
        particles_r[i].x(),
        particles_r[i].y(),
        particles_r[i].z(),
        expf(weights[i] - max_w),
    });
  }
  {
    ros::sensor_msgs::Image ros_img;
    *ros_img.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
    ros_img.mutable_header()->set_frame_id("/camera-frame");
    ros_img.set_height(kImageHeight);
    ros_img.set_width(kImageWidth);
    ros_img.set_encoding("mono8");
    ros_img.set_is_bigendian(false);
    ros_img.set_step(kImageWidth);
    *ros_img.mutable_data() = std::string(img_copy.begin(), img_copy.end());
    ros_writer_->Write(img_topic_, t_us, ros_img);
  }
  {
    ros::sensor_msgs::Image ros_img;
    *ros_img.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
    ros_img.mutable_header()->set_frame_id("/camera-frame");
    ros_img.set_height(kImageHeight);
    ros_img.set_width(kImageWidth);
    ros_img.set_encoding("rgb8");
    ros_img.set_is_bigendian(false);
    ros_img.set_step(kImageWidth * 3);
    *ros_img.mutable_data() = std::string(rgb_img.begin(), rgb_img.end());
    ros_writer_->Write(img_rgb_topic_, t_us, ros_img);
  }

  if (t_us > last_floormap_us_ + 500000) {
    last_floormap_us_ = t_us;

    ros::sensor_msgs::Image ros_img;
    *ros_img.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
    ros_img.mutable_header()->set_frame_id("/world");
    ros_img.set_height(map_maker_.height());
    ros_img.set_width(map_maker_.width());
    ros_img.set_encoding("rgb8");
    ros_img.set_is_bigendian(false);
    ros_img.set_step(map_maker_.width() * 3);
    *ros_img.mutable_data() =
        std::string(map_maker_.map().begin(), map_maker_.map().end());
    ros_writer_->Write(floor_map_topic_, t_us, ros_img);
  }

  {
    ros::sensor_msgs::PointCloud2 pt_cloud;
    *pt_cloud.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
    pt_cloud.mutable_header()->set_frame_id("/world");
    pt_cloud.set_height(1);
    pt_cloud.set_width(particles.size());

    auto* field = pt_cloud.add_fields();
    field->set_name("x");
    field->set_offset(0);
    field->set_datatype(7);
    field->set_count(1);

    field = pt_cloud.add_fields();
    field->set_name("y");
    field->set_offset(4);
    field->set_datatype(7);
    field->set_count(1);

    field = pt_cloud.add_fields();
    field->set_name("z");
    field->set_offset(8);
    field->set_datatype(7);
    field->set_count(1);

    field = pt_cloud.add_fields();
    field->set_name("weight");
    field->set_offset(12);
    field->set_datatype(7);
    field->set_count(1);

    pt_cloud.set_is_bigendian(false);
    pt_cloud.set_point_step(16);
    size_t nbytes = 16 * particles.size();
    pt_cloud.set_row_step(nbytes);
    pt_cloud.mutable_data()->resize(nbytes);
    memcpy(pt_cloud.mutable_data()->data(), particles.data(), nbytes);
    pt_cloud.set_is_dense(true);

    // ideally this would have been a pose array
    ros_writer_->Write(pf_topic_, t_us, pt_cloud);
  }

  ros::sensor_msgs::PointCloud2 pt_cloud;
  *pt_cloud.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
  pt_cloud.mutable_header()->set_frame_id("/world");
  pt_cloud.set_height(1);

  auto* field = pt_cloud.add_fields();
  field->set_name("x");
  field->set_offset(0);
  field->set_datatype(7);
  field->set_count(1);

  field = pt_cloud.add_fields();
  field->set_name("y");
  field->set_offset(4);
  field->set_datatype(7);
  field->set_count(1);

  field = pt_cloud.add_fields();
  field->set_name("z");
  field->set_offset(8);
  field->set_datatype(7);
  field->set_count(1);

  pt_cloud.set_is_bigendian(false);
  pt_cloud.set_point_step(12);
  pt_cloud.set_is_dense(true);

  std::vector<Eigen::Vector3f> map;
  for (const auto& lm : pf_.map()) {
    map.push_back({lm.x(), lm.y(), kCeilHeight});
  }
  pt_cloud.set_width(map.size());
  int nbytes = 12 * map.size();
  pt_cloud.set_row_step(nbytes);
  pt_cloud.mutable_data()->resize(nbytes);
  memcpy(pt_cloud.mutable_data()->data(), map.data(), nbytes);
  ros_writer_->Write(map_topic_, t_us, pt_cloud);

  {
    std::vector<Eigen::Vector3f> landmark_pts;
    auto cam_to_world_rot = Eigen::Rotation2Df(pf_result.pose.z());
    for (const auto& lm : landmarks) {
      Eigen::Vector2f pt = {lm.pos.x(), lm.pos.y()};
      pt = cam_to_world_rot * pt +
           Eigen::Vector2f{pf_result.pose.x(), pf_result.pose.y()};

      landmark_pts.push_back({pt.x(), pt.y(), kCeilHeight});
    }
    pt_cloud.set_width(landmark_pts.size());
    nbytes = 12 * landmark_pts.size();
    pt_cloud.set_row_step(nbytes);
    pt_cloud.mutable_data()->resize(nbytes);
    memcpy(pt_cloud.mutable_data()->data(), landmark_pts.data(), nbytes);
    ros_writer_->Write(landmark_detected_topic_, t_us, pt_cloud);
  }

  ros::visualization_msgs::ImageMarker marker;
  *marker.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
  marker.set_ns("landmark_overlay");
  marker.set_id(0);
  marker.set_type(4);  // POINTS
  marker.set_action(0);
  marker.set_scale(5);
  marker.mutable_lifetime()->set_nsec(100000000);
  for (int i = 0; i < lights.size(); ++i) {
    auto& pt = *marker.add_points();
    pt.set_x(lights[i].u);
    pt.set_y(lights[i].v);
    auto& oc = *marker.add_outline_colors();
    oc.set_g(1.0);
    oc.set_a(1.0);
  }
  ros_writer_->Write(landmark_img_overlay_topic_, t_us, marker);
}

void Localizer::OdoFrame(int64_t t_us, float odo_dist_delta,
                         float odo_heading_delta, float imu_rot_z,
                         float dist_stddev, float heading_stddev, float x,
                         float y, float z) {
  spdlog::info("odo frame t:{} dist_d:{} heading_d:{}", t_us, odo_dist_delta,
               odo_heading_delta);

  {
    ros::geometry_msgs::PoseStamped pose;
    *pose.mutable_header()->mutable_stamp() = MicrosToRos(t_us);
    pose.mutable_header()->set_frame_id("/world");
    pose.mutable_pose()->mutable_position()->set_x(x);
    pose.mutable_pose()->mutable_position()->set_y(y);
    *pose.mutable_pose()->mutable_orientation() = HeadingToQuat(z);
    ros_writer_->Write(log_pose_topic_, t_us, pose);
  }

  motions_.push_back({
      .delta_dist = odo_dist_delta,
      .delta_heading = odo_heading_delta,
      .stddev_dist = dist_stddev,
      .stddev_heading = heading_stddev,
  });
}

}  // namespace

int main() {
  spdlog::cfg::load_env_levels();

  std::ifstream input("/home/danchia/log.mcap", std::ios::binary);
  mcap::FileStreamReader dataSource(input);
  mcap::McapReader reader;
  auto status = reader.open(dataSource);
  if (!status.ok()) {
    spdlog::warn("{}", status.message);
    return -1;
  }
  auto onProblem = [](const mcap::Status& problem) {
    spdlog::warn("! {}", problem.message);
  };
  auto messages = reader.readMessages(onProblem);

  Localizer localizer;

  int images = 0;
  int datas = 0;

  for (const auto& msgView : messages) {
    const mcap::Channel& channel = *msgView.channel;
    if (channel.topic == "/camera1/raw") {
      ros::sensor_msgs::Image m;
      if (!m.ParseFromArray(msgView.message.data, msgView.message.dataSize)) {
        spdlog::warn("parse error for {}", channel.topic);
      }
      std::vector<uint8_t> img(m.data().begin(), m.data().end());
      int64_t t_us = msgView.message.publishTime;
      t_us /= int64_t{1000};
      localizer.VideoFrame(t_us, img);
      ++images;
    } else if (channel.topic == "/driver/state") {
      zoomies::DriverLog m;
      if (!m.ParseFromArray(msgView.message.data, msgView.message.dataSize)) {
        spdlog::warn("parse error for {}", channel.topic);
      }
      int64_t t_us = msgView.message.publishTime;
      t_us /= int64_t{1000};
      localizer.OdoFrame(t_us, m.dist_delta(), m.heading_delta(),
                         m.imu_rotation().z(), m.dist_stddev(),
                         m.heading_stddev(), m.x(), m.y(), m.heading());
      ++datas;
    }
  }
  spdlog::info("Processed {} image, {} data frames", images, datas);

  return 0;
}
