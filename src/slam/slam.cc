#include <inttypes.h>

#include <Eigen/Geometry>
#include <fstream>
#include <memory>
#include <optional>
#include <string_view>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "ros/ros_types.h"
#include "ros/ros_util.h"
#include "ros/ros_writer.h"
#include "slam_util.h"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
#include "sqlite/sqlite3.h"

constexpr float kCeilHeight = 3.2f;
constexpr float kLightAssocDist = 0.5f;
constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;
constexpr int kImageNoisePixels = 10;

namespace {

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

class SLAM {
 public:
  struct Result {
    std::vector<Eigen::Vector2f> landmarks;
  };

  SLAM();
  void VideoFrame(int64_t t_us, const std::vector<uint8_t>& img);
  void OdoFrame(int64_t t_us, float odo_dist_delta, float odo_heading_delta,
                float imu_rot_z);
  Result Finish();

 private:
  struct Landmark {
    Eigen::Vector2f pos;
    int id = -1;
  };

  Eigen::Vector2f camera_lookup(int u, int v) {
    int idx = u + v * kImageWidth;
    return Eigen::Vector2f{camera_lut_[idx][0], camera_lut_[idx][1]} *
           kCeilHeight;
  }

  void InitViz();

  void AddObservation(int pose_id, int landmark_id, Eigen::Vector2f measure,
                      int u, int v);
  int next_id() { return next_id_++; }

  float x_ = 0.0f;
  float y_ = 0.0f;
  float heading_ = 0.0f;

  float camera_lut_[kImageWidth * kImageHeight][2];
  uint8_t ceil_mask_[kImageWidth * kImageHeight];

  std::vector<Landmark> landmarks_;
  std::vector<Landmark> landmark_candidates_;

  g2o::SparseOptimizer optimizer_;
  int next_id_ = 0;
  int last_pose_id_;

  float dist_since_opt = 0.0f;

  // Viz
  std::optional<RosWriter> ros_writer_;
  int img_topic_;
  int landmark_raw_topic_;
  int landmark_map_topic_;
  int landmark_img_overlay_topic_;
};

void SLAM::VideoFrame(int64_t t_us, const std::vector<uint8_t>& img) {
  spdlog::info("vid frame t:{}. {} {} {}", t_us, x_, y_, heading_);
  // Add current pose
  int pose_id = next_id();
  {
    auto* pose_v = new g2o::VertexSE2();
    pose_v->setId(pose_id);
    pose_v->setEstimate({x_, y_, heading_});
    optimizer_.addVertex(pose_v);

    // Add odom edge
    auto* odom_e = new g2o::EdgeSE2();
    auto* prev_v =
        static_cast<g2o::VertexSE2*>(optimizer_.vertex(last_pose_id_));
    auto prev_pose = prev_v->estimate();
    odom_e->vertices()[0] = prev_v;
    odom_e->vertices()[1] = pose_v;
    double dx = x_ - prev_pose.translation().x();
    double dy = y_ - prev_pose.translation().y();
    double dt = heading_ - prev_pose.rotation().angle();
    odom_e->setMeasurement({dx, dy, dt});
    // terrible, FIXME!
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(0, 0) = std::max(0.2 * 0.2 * dx * dx, 0.05);
    cov(1, 1) = std::max(0.2 * 0.2 * dy * dy, 0.05);
    cov(2, 2) = std::max(0.2 * 0.2 * dt * dt, 0.05);
    Eigen::Matrix3d information = cov.inverse();
    odom_e->setInformation(information);
    optimizer_.addEdge(odom_e);

    last_pose_id_ = pose_id;
  }

  // only extract the Y component
  std::vector<uint8_t> img_copy(img.begin(),
                                img.begin() + (kImageWidth * kImageHeight));
  {
    int size = img_copy.size();
    for (int i = 0; i < size; ++i) {
      img_copy[i] &= ceil_mask_[i];
    }
  }

  auto rects = FindRect(kImageWidth, kImageHeight, img_copy.data(), 220, 300);

  std::vector<Eigen::Vector3f> light_positions;
  std::vector<Eigen::Vector2i> landmark_overlay;

  auto cam_to_world_rot = Eigen::Rotation2Df(heading_);
  for (const auto& rect : rects) {
    int u = rect.x + rect.width / 2;
    int v = rect.y + rect.height / 2;
    Eigen::Vector2f pos_cam = camera_lookup(u, v);
    Eigen::Vector2f pos_world =
        cam_to_world_rot * pos_cam + Eigen::Vector2f{x_, y_};

    landmark_overlay.push_back(Eigen::Vector2i{u, v});

    light_positions.push_back({pos_world.x(), pos_world.y(), kCeilHeight});

    float best_dist = kLightAssocDist;
    int landmark_idx = -1;
    for (int i = 0; i < landmarks_.size(); ++i) {
      auto& landmark = landmarks_[i];

      auto landmark_est =
          static_cast<g2o::VertexPointXY*>(optimizer_.vertex(landmark.id))
              ->estimate();
      Eigen::Vector2f landmark_pos = {landmark_est.x(), landmark_est.y()};
      float dist = (landmark_pos - pos_world).norm();
      if (dist < best_dist) {
        best_dist = dist;
        landmark_idx = i;
      }
    }
    spdlog::trace("best landmark fit {}. my pos {} {} {} {}", best_dist, u, v,
                  pos_world.x(), pos_world.y());
    if (landmark_idx != -1) {
      // spdlog::info("  their pos {} {}", landmarks_[landmark_idx].pos.x(),
      //              landmarks_[landmark_idx].pos.y());
      AddObservation(pose_id, landmarks_[landmark_idx].id, pos_cam, u, v);
      continue;
    }

    best_dist = kLightAssocDist;
    landmark_idx = -1;
    for (int i = 0; i < landmark_candidates_.size(); ++i) {
      auto& lm = landmark_candidates_[i];
      float dist = (lm.pos - pos_world).norm();
      if (dist < best_dist) {
        best_dist = dist;
        landmark_idx = i;
      }
    }
    if (landmark_idx != -1) {
      auto& lm = landmark_candidates_[landmark_idx];

      int id = next_id();
      lm.id = id;
      auto* landmark_v = new g2o::VertexPointXY;
      landmark_v->setId(id);
      landmark_v->setEstimate({lm.pos.x(), lm.pos.y()});
      optimizer_.addVertex(landmark_v);
      AddObservation(pose_id, id, pos_cam, u, v);

      landmarks_.push_back(lm);
      landmark_candidates_.erase(landmark_candidates_.begin() + landmark_idx);
      continue;
    }

    landmark_candidates_.push_back({.pos = pos_world});
  }

  // trigger optimization every so often
  constexpr float kOptDist = 0.3;
  if (dist_since_opt > kOptDist) {
    dist_since_opt -= kOptDist;

    if (!landmarks_.empty()) {
      auto* lm =
          static_cast<g2o::VertexPointXY*>(optimizer_.vertex(landmarks_[0].id));
      spdlog::debug("updating landmark. old: {} {}", lm->estimate().x(),
                    lm->estimate().y());
    }

    auto* last_v = static_cast<g2o::VertexSE2*>(optimizer_.vertex(pose_id));
    spdlog::debug("updating pos. old: {} {} {}",
                  last_v->estimate().translation().x(),
                  last_v->estimate().translation().y(),
                  last_v->estimate().rotation().angle());

    optimizer_.initializeOptimization();
    optimizer_.optimize(10);

    x_ = last_v->estimate().translation().x();
    y_ = last_v->estimate().translation().y();
    heading_ = last_v->estimate().rotation().angle();
    spdlog::debug("updating pos. new: {} {} {}", x_, y_, heading_);

    if (!landmarks_.empty()) {
      auto* lm =
          static_cast<g2o::VertexPointXY*>(optimizer_.vertex(landmarks_[0].id));
      spdlog::debug("updating landmark. new: {} {}", lm->estimate().x(),
                    lm->estimate().y());
    }
  }

  // Update visualizations
  sensor_msgs__Image ros_img;
  ros_img.header().stamp() = MicrosToRos(t_us);
  ros_img.header().frame_id("/camera-frame");
  ros_img.height(kImageHeight);
  ros_img.width(kImageWidth);
  ros_img.encoding("mono8");
  ros_img.is_bigendian(false);
  ros_img.step(kImageWidth);
  ros_img.data(img_copy);
  // ros_img.data(img);
  // ros_img.data().resize(kImageWidth *
  //                       kImageHeight);  // not the most efficient, but it
  //                       works.
  ros_writer_->Write(img_topic_, t_us, ros_img);

  sensor_msgs__PointCloud2 pt_cloud;
  pt_cloud.header().stamp() = MicrosToRos(t_us);
  pt_cloud.header().frame_id("/world");
  pt_cloud.height(1);
  pt_cloud.width(light_positions.size());

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
  size_t nbytes = 12 * light_positions.size();
  pt_cloud.row_step(nbytes);
  pt_cloud.data().resize(nbytes);
  memcpy(pt_cloud.data().data(), light_positions.data(), nbytes);
  pt_cloud.is_dense(true);

  ros_writer_->Write(landmark_raw_topic_, t_us, pt_cloud);

  std::vector<Eigen::Vector3f> landmarks;
  for (const auto& lm : landmarks_) {
    auto* l = static_cast<g2o::VertexPointXY*>(optimizer_.vertex(lm.id));
    landmarks.push_back({l->estimate().x(), l->estimate().y(), kCeilHeight});
  }
  pt_cloud.width(landmarks.size());
  nbytes = 12 * landmarks.size();
  pt_cloud.row_step(nbytes);
  pt_cloud.data().resize(nbytes);
  memcpy(pt_cloud.data().data(), landmarks.data(), nbytes);

  ros_writer_->Write(landmark_map_topic_, t_us, pt_cloud);

  // Would have used image array, but doesn't work in foxglove ros2. Luckily can
  // use POINTS
  visualization_msgs__ImageMarker marker;
  marker.header().stamp() = MicrosToRos(t_us);
  marker.ns("landmark_overlay");
  marker.id(0);
  marker.type(4);  // POINTS
  marker.action(0);
  marker.scale(5);
  marker.lifetime().nsec(100000000);
  for (int i = 0; i < landmark_overlay.size(); ++i) {
    auto& pt = marker.points().emplace_back();
    pt.x(landmark_overlay[i].x());
    pt.y(landmark_overlay[i].y());
    auto& oc = marker.outline_colors().emplace_back();
    oc.g(1.0);
    oc.a(1.0);
  }
  ros_writer_->Write(landmark_img_overlay_topic_, t_us, marker);
}

void SLAM::OdoFrame(int64_t t_us, float odo_dist_delta, float odo_heading_delta,
                    float imu_rot_z) {
  spdlog::debug("odo frame t:{} dist_d:{} heading_d:{}", t_us, odo_dist_delta,
                odo_heading_delta);

  dist_since_opt += odo_dist_delta;

  x_ += odo_dist_delta * cos(heading_ + odo_heading_delta / 2.0);
  y_ += odo_dist_delta * sin(heading_ + odo_heading_delta / 2.0);
  heading_ = normAngle(heading_ + odo_heading_delta);

  // covariance?
}

SLAM::SLAM() {
  InitViz();
  {
    std::ifstream f("../data/calib/camera_lut.bin",
                    std::ios::in | std::ios::binary);
    if (!f.good()) throw std::runtime_error("error opening lut");
    f.read(reinterpret_cast<char*>(camera_lut_),
           kImageWidth * kImageHeight * 2 * 4);
    if (!f) throw std::runtime_error("error while reading lut");
    f.close();
  }

  {
    std::ifstream f("../data/ceil_mask.bin", std::ios::in | std::ios::binary);
    if (!f.good()) throw std::runtime_error("error opening ceil_mask");
    f.read(reinterpret_cast<char*>(ceil_mask_), kImageWidth * kImageHeight);
    if (!f) throw std::runtime_error("error while reading lut");
    f.close();
  }

  using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, 1>>;
  using SlamLinearSolver =
      g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;

  auto linear_solver = std::make_unique<SlamLinearSolver>();
  linear_solver->setBlockOrdering(false);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(
          std::make_unique<SlamBlockSolver>(std::move(linear_solver)));
  optimizer_.setAlgorithm(solver);

  // add initial pose
  auto* pose = new g2o::VertexSE2();
  pose->setFixed(true);
  pose->setEstimate(g2o::SE2());
  pose->setId(next_id());
  optimizer_.addVertex(pose);
  last_pose_id_ = pose->id();

  //  optimizer_.setVerbose(true);
}

void SLAM::InitViz() {
  ros_writer_.emplace("/tmp/slamviz");
  img_topic_ =
      ros_writer_->AddConnection("/camera1/image", "sensor_msgs/msg/Image");
  landmark_raw_topic_ = ros_writer_->AddConnection(
      "/landmarks/raw", "sensor_msgs/msg/PointCloud2");
  landmark_map_topic_ = ros_writer_->AddConnection(
      "/landmarks/map", "sensor_msgs/msg/PointCloud2");
  landmark_img_overlay_topic_ = ros_writer_->AddConnection(
      "/landmarks/img_overlay", "visualization_msgs/msg/ImageMarker");
}

void SLAM::AddObservation(int pose_id, int landmark_id, Eigen::Vector2f measure,
                          int u, int v) {
  auto* e = new g2o::EdgeSE2PointXY();
  e->vertices()[0] = optimizer_.vertex(pose_id);
  e->vertices()[1] = optimizer_.vertex(landmark_id);
  e->setMeasurement({measure.x(), measure.y()});
  float d = 0.1f;
  auto center = camera_lookup(u, v);
  for (int u_p = -kImageNoisePixels; u_p <= kImageNoisePixels;
       u_p += kImageNoisePixels) {
    for (int v_p = -kImageNoisePixels; v_p <= kImageNoisePixels;
         v_p += kImageNoisePixels) {
      if (u_p == 0 && v_p == 0) continue;
      int nu = std::clamp(u + u_p, 0, kImageWidth - 1);
      int nv = std::clamp(v + v_p, 0, kImageHeight - 1);
      auto pt = camera_lookup(nu, nv);
      float d_cand = (pt - center).norm();
      if (d_cand > d) d = d_cand;
    }
  }
  Eigen::Matrix2d cov = (d * d) * Eigen::Matrix2d::Identity();
  e->setInformation(cov.inverse());
  optimizer_.addEdge(e);

  spdlog::trace("added observation u,v {},{} measure: {} {} stddev: {}", u, v,
                measure.x(), measure.y(), d);
}

SLAM::Result SLAM::Finish() {
  optimizer_.initializeOptimization();
  optimizer_.optimize(10);

  std::vector<Eigen::Vector2f> landmarks;
  for (const auto& lm : landmarks_) {
    auto* l = static_cast<g2o::VertexPointXY*>(optimizer_.vertex(lm.id));
    landmarks.push_back({l->estimate().x(), l->estimate().y()});
  }

  return {.landmarks = std::move(landmarks)};
}

}  // namespace

int main() {
  spdlog::cfg::load_env_levels();

  sqlite3* db = nullptr;

  sCheck(sqlite3_open("/home/danchia/data.db3", &db), "open", db);

  SLAM slam;

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

      if (target_t > 4000000) break;

      while (true) {
        const auto& row = data_iter.row();
        if (row.t_us > target_t + 5000) break;

        ++datas;
        slam.OdoFrame(row.t_us, row.odo_dist_delta, row.odo_heading_delta,
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
      slam.VideoFrame(img_iter.row().t_us, img_iter.row().image);
    }

    spdlog::info("Processed {} image, {} data frames", images, datas);
  }
  sqlite3_close(db);

  auto result = slam.Finish();

  spdlog::info("Light locations ({} lights):", result.landmarks.size());
  for (const auto& lm : result.landmarks) {
    spdlog::info("  {},{},{}", lm.x(), lm.y(), kCeilHeight);
  }

  return 0;
}