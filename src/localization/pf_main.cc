#include <inttypes.h>

#include <Eigen/Geometry>
#include <fstream>
#include <memory>
#include <optional>
#include <string_view>

#include "common/color.h"
#include "localization/localization_util.h"
#include "localization/pf.h"
#include "ros/ros_types.h"
#include "ros/ros_util.h"
#include "ros/ros_writer.h"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
#include "sqlite/sqlite3.h"

namespace {

constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;
constexpr float kCeilHeight = 2.5f;
constexpr int kNumParticles = 300;
constexpr float kCameraHeight = 0.095f;

void sCheck(int rc, std::string_view msg, sqlite3* db) {
  if (rc) {
    const char* err_msg = sqlite3_errmsg(db);
    spdlog::warn("sqlite {} {}: {}", msg, sqlite3_errstr(rc), err_msg);
    throw std::runtime_error("sqlite error");
  }
}

class ImageIter {
 public:
  struct Row {
    int64_t t_us;
    std::vector<uint8_t> image;
  };

  ImageIter(sqlite3* db) : db_(db) {}
  ~ImageIter() { sCheck(sqlite3_finalize(stmt_), "finalize vid sql", db_); }

  void Init() {
    sCheck(
        sqlite3_prepare_v2(db_, "SELECT t_us, data FROM videos ORDER BY t_us",
                           -1, &stmt_, nullptr),
        "video sql", db_);
  }

  bool Next() {
    int rc = sqlite3_step(stmt_);
    if (rc == SQLITE_DONE) return false;
    if (rc != SQLITE_ROW) {
      sCheck(rc, "image iter next", db_);
      return false;
    }

    row_.t_us = sqlite3_column_int64(stmt_, 0);

    row_.image.resize(sqlite3_column_bytes(stmt_, 1));
    memcpy(row_.image.data(), sqlite3_column_blob(stmt_, 1), row_.image.size());
    return true;
  }

  const Row& row() { return row_; }

 private:
  sqlite3* db_;
  sqlite3_stmt* stmt_ = nullptr;

  Row row_;
};

class DataIter {
 public:
  struct Row {
    int64_t t_us;
    float odo_dist_delta;
    float odo_heading_delta;
    float imu_rot_z;
  };

  DataIter(sqlite3* db) : db_(db) {}
  ~DataIter() { sCheck(sqlite3_finalize(stmt_), "finalize data sql", db_); }

  void Init() {
    sCheck(sqlite3_prepare_v2(db_,
                              "SELECT t_us, odo_dist_delta, odo_heading_delta, "
                              "imu_rot_z FROM data ORDER BY t_us",
                              -1, &stmt_, nullptr),
           "data sql", db_);
  }

  bool Next() {
    int rc = sqlite3_step(stmt_);
    if (rc == SQLITE_DONE) return false;
    if (rc != SQLITE_ROW) {
      sCheck(rc, "image iter next", db_);
      return false;
    }

    row_.t_us = sqlite3_column_int64(stmt_, 0);
    row_.odo_dist_delta = sqlite3_column_double(stmt_, 1);
    row_.odo_heading_delta = sqlite3_column_double(stmt_, 2);
    row_.imu_rot_z = sqlite3_column_double(stmt_, 3);

    return true;
  }

  const Row& row() { return row_; }

 private:
  sqlite3* db_;
  sqlite3_stmt* stmt_ = nullptr;

  Row row_;
};

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
        map_(map_width_ * map_height_ * 3) {
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

        r = 0.9 * ir + 0.1 * r;
        g = 0.9 * ig + 0.1 * g;
        b = 0.9 * ib + 0.1 * b;
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
};

class Localizer {
 public:
  Localizer();
  void VideoFrame(int64_t t_us, const std::vector<uint8_t>& img);
  void OdoFrame(int64_t t_us, float odo_dist_delta, float odo_heading_delta,
                float imu_rot_z);

 private:
  void InitViz();

  CameraModel camera_model_;
  LightFinder light_finder_;
  ParticleFilter pf_;
  MapMaker map_maker_;

  std::vector<ParticleFilter::Motion> motions_;

  int64_t last_floormap_us_ = 0;

  // Viz
  std::optional<RosWriter> ros_writer_;
  int img_topic_;
  int img_rgb_topic_;
  int floor_map_topic_;
  int landmark_img_overlay_topic_;
  int pf_topic_;
  int map_topic_;
  int landmark_detected_topic_;
};

Localizer::Localizer()
    : camera_model_(kImageWidth, kImageHeight, "../data/calib/camera_lut.bin"),
      light_finder_(camera_model_, kImageWidth, kImageHeight, kCeilHeight, 220,
                    300, "../data/calib/ceil_mask.bin"),
      pf_(kNumParticles, "../data/map/map.txt"),
      map_maker_(camera_model_, "../data/calib/floor_mask.bin", kImageWidth,
                 kImageHeight, 0.02f, -10.0f, -10.0f, 10.0f, 10.0f) {
  InitViz();

  pf_.SeedLocation({-0.3f, -0.3f, -0.4f}, {0.3f, 0.3f, 0.4f});
}

void Localizer::InitViz() {
  ros_writer_.emplace("/tmp/pfviz");
  img_topic_ =
      ros_writer_->AddConnection("/camera1/image", "sensor_msgs/msg/Image");
  img_rgb_topic_ =
      ros_writer_->AddConnection("/camera1/rgb_image", "sensor_msgs/msg/Image");
  pf_topic_ = ros_writer_->AddConnection("/localization/pf_cloud",
                                         "sensor_msgs/msg/PointCloud2");
  landmark_img_overlay_topic_ = ros_writer_->AddConnection(
      "/landmarks/img_overlay", "visualization_msgs/msg/ImageMarker");
  map_topic_ = ros_writer_->AddConnection("/landmarks/map",
                                          "sensor_msgs/msg/PointCloud2");
  landmark_detected_topic_ = ros_writer_->AddConnection(
      "/landmarks/detected", "sensor_msgs/msg/PointCloud2");
  floor_map_topic_ =
      ros_writer_->AddConnection("/floormap", "sensor_msgs/msg/Image");
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
  motions_.clear();

  spdlog::info("PF loc: {} {} {}, var: {} {} {}", pf_result.pose.x(),
               pf_result.pose.y(), pf_result.pose.z(), pf_result.variance.x(),
               pf_result.variance.y(), pf_result.variance.z());

  map_maker_.Update(pf_result.pose, rgb_img);

  // Update visuals
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
    sensor_msgs__Image ros_img;
    ros_img.header().stamp() = MicrosToRos(t_us);
    ros_img.header().frame_id("/camera-frame");
    ros_img.height(kImageHeight);
    ros_img.width(kImageWidth);
    ros_img.encoding("mono8");
    ros_img.is_bigendian(false);
    ros_img.step(kImageWidth);
    ros_img.data(img);
    ros_writer_->Write(img_topic_, t_us, ros_img);
  }
  {
    sensor_msgs__Image ros_img;
    ros_img.header().stamp() = MicrosToRos(t_us);
    ros_img.header().frame_id("/camera-frame");
    ros_img.height(kImageHeight);
    ros_img.width(kImageWidth);
    ros_img.encoding("rgb8");
    ros_img.is_bigendian(false);
    ros_img.step(kImageWidth * 3);
    ros_img.data(rgb_img);
    ros_writer_->Write(img_rgb_topic_, t_us, ros_img);
  }

  if (t_us > last_floormap_us_ + 100000) {
    last_floormap_us_ = t_us;

    sensor_msgs__Image ros_img;
    ros_img.header().stamp() = MicrosToRos(t_us);
    ros_img.header().frame_id("/world");
    ros_img.height(map_maker_.height());
    ros_img.width(map_maker_.width());
    ros_img.encoding("rgb8");
    ros_img.is_bigendian(false);
    ros_img.step(map_maker_.width() * 3);
    ros_img.data(map_maker_.map());
    ros_writer_->Write(floor_map_topic_, t_us, ros_img);
  }

  {
    sensor_msgs__PointCloud2 pt_cloud;
    pt_cloud.header().stamp() = MicrosToRos(t_us);
    pt_cloud.header().frame_id("/world");
    pt_cloud.height(1);
    pt_cloud.width(particles.size());

    pt_cloud.fields().emplace_back();
    pt_cloud.fields().back().name("x");
    pt_cloud.fields().back().offset(0);
    pt_cloud.fields().back().datatype(7);
    pt_cloud.fields().back().count(1);

    pt_cloud.fields().emplace_back();
    pt_cloud.fields().back().name("y");
    pt_cloud.fields().back().offset(4);
    pt_cloud.fields().back().datatype(7);
    pt_cloud.fields().back().count(1);

    pt_cloud.fields().emplace_back();
    pt_cloud.fields().back().name("z");
    pt_cloud.fields().back().offset(8);
    pt_cloud.fields().back().datatype(7);
    pt_cloud.fields().back().count(1);

    pt_cloud.fields().emplace_back();
    pt_cloud.fields().back().name("weight");
    pt_cloud.fields().back().offset(12);
    pt_cloud.fields().back().datatype(7);
    pt_cloud.fields().back().count(1);

    pt_cloud.is_bigendian(false);
    pt_cloud.point_step(16);
    size_t nbytes = 16 * particles.size();
    pt_cloud.row_step(nbytes);
    pt_cloud.data().resize(nbytes);
    memcpy(pt_cloud.data().data(), particles.data(), nbytes);
    pt_cloud.is_dense(true);

    // ideally this would have been a pose array
    ros_writer_->Write(pf_topic_, t_us, pt_cloud);
  }

  sensor_msgs__PointCloud2 pt_cloud;
  pt_cloud.header().stamp() = MicrosToRos(t_us);
  pt_cloud.header().frame_id("/world");
  pt_cloud.height(1);

  pt_cloud.fields().emplace_back();
  pt_cloud.fields().back().name("x");
  pt_cloud.fields().back().offset(0);
  pt_cloud.fields().back().datatype(7);
  pt_cloud.fields().back().count(1);

  pt_cloud.fields().emplace_back();
  pt_cloud.fields().back().name("y");
  pt_cloud.fields().back().offset(4);
  pt_cloud.fields().back().datatype(7);
  pt_cloud.fields().back().count(1);

  pt_cloud.fields().emplace_back();
  pt_cloud.fields().back().name("z");
  pt_cloud.fields().back().offset(8);
  pt_cloud.fields().back().datatype(7);
  pt_cloud.fields().back().count(1);

  pt_cloud.is_bigendian(false);
  pt_cloud.point_step(12);
  pt_cloud.is_dense(true);

  std::vector<Eigen::Vector3f> map;
  for (const auto& lm : pf_.map()) {
    map.push_back({lm.x(), lm.y(), kCeilHeight});
  }
  pt_cloud.width(map.size());
  int nbytes = 12 * map.size();
  pt_cloud.row_step(nbytes);
  pt_cloud.data().resize(nbytes);
  memcpy(pt_cloud.data().data(), map.data(), nbytes);
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
    pt_cloud.width(landmark_pts.size());
    nbytes = 12 * landmark_pts.size();
    pt_cloud.row_step(nbytes);
    pt_cloud.data().resize(nbytes);
    memcpy(pt_cloud.data().data(), landmark_pts.data(), nbytes);
    ros_writer_->Write(landmark_detected_topic_, t_us, pt_cloud);
  }

  visualization_msgs__ImageMarker marker;
  marker.header().stamp() = MicrosToRos(t_us);
  marker.ns("landmark_overlay");
  marker.id(0);
  marker.type(4);  // POINTS
  marker.action(0);
  marker.scale(5);
  marker.lifetime().nsec(100000000);
  for (int i = 0; i < lights.size(); ++i) {
    auto& pt = marker.points().emplace_back();
    pt.x(lights[i].u);
    pt.y(lights[i].v);
    auto& oc = marker.outline_colors().emplace_back();
    oc.g(1.0);
    oc.a(1.0);
  }
  ros_writer_->Write(landmark_img_overlay_topic_, t_us, marker);
}

void Localizer::OdoFrame(int64_t t_us, float odo_dist_delta,
                         float odo_heading_delta, float imu_rot_z) {
  spdlog::info("odo frame t:{} dist_d:{} heading_d:{}", t_us, odo_dist_delta,
               odo_heading_delta);

  motions_.push_back({
      .delta_dist = odo_dist_delta,
      .delta_heading = odo_heading_delta,
      .stddev_dist = std::max(0.1f * odo_dist_delta, 0.03f * 0.01f),
      .stddev_heading = std::max(fabsf(0.3f * odo_heading_delta), 0.1f * 0.01f),
  });
}

}  // namespace

int main() {
  spdlog::cfg::load_env_levels();

  sqlite3* db = nullptr;

  sCheck(sqlite3_open("/home/danchia/data.db3", &db), "open", db);

  Localizer localizer;

  {
    ImageIter img_iter(db);
    img_iter.Init();
    DataIter data_iter(db);
    data_iter.Init();

    int images = 0;
    int datas = 0;
    bool data_iter_done = false;

    data_iter.Next();  // Should have at least one data frame, right ;)
    while (img_iter.Next()) {
      int64_t target_t = img_iter.row().t_us;
      int64_t last_datas = datas;

      if (target_t > 5500000) break;

      while (true) {
        const auto& row = data_iter.row();
        if (row.t_us > target_t + 5000) break;

        ++datas;
        localizer.OdoFrame(row.t_us, row.odo_dist_delta, row.odo_heading_delta,
                           row.imu_rot_z);

        data_iter_done = !data_iter.Next();
        if (data_iter_done) break;
      }
      if (data_iter_done) break;

      ++images;
      if (datas == last_datas) {
        spdlog::warn("Skipping vid frame {} us due to no data frames",
                     img_iter.row().t_us);
        continue;
      }
      localizer.VideoFrame(img_iter.row().t_us, img_iter.row().image);
    }

    spdlog::info("Processed {} image, {} data frames", images, datas);
  }
  sqlite3_close(db);

  return 0;
}