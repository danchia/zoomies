#include <inttypes.h>

#include <Eigen/Geometry>
#include <fstream>
#include <string_view>

#include "g2o/core/sparse_optimizer.h"
#include "slam_util.h"
#include "spdlog/spdlog.h"
#include "sqlite/sqlite3.h"

constexpr float kCeilHeight = 3.2f;
constexpr float kLightAssocDist = 0.3f;

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
  SLAM();
  void VideoFrame(int64_t t_us, const std::vector<uint8_t>& img);
  void OdoFrame(int64_t t_us, float odo_dist_delta, float odo_heading_delta,
                float imu_rot_z);

 private:
  struct Landmark {
    Eigen::Vector2f pos;
  };

  Eigen::Vector2f camera_lookup(int u, int v) {
    int idx = u + v * 640;
    return {camera_lut_[idx][0], camera_lut_[idx][1]};
  }

  float x_ = 0.0f;
  float y_ = 0.0f;
  float heading_ = 0.0f;

  float camera_lut_[640 * 480][2];
  uint8_t ceil_mask_[640 * 480];

  std::vector<Landmark> landmarks_;
  std::vector<Landmark> landmark_candidates_;

  g2o::SparseOptimizer optimizer_;
};

void SLAM::VideoFrame(int64_t t_us, const std::vector<uint8_t>& img) {
  // only extract the Y component
  std::vector<uint8_t> img_copy(img.begin(), img.begin() + (640 * 480));
  {
    int size = img_copy.size();
    for (int i = 0; i < size; ++i) {
      img_copy[i] &= ceil_mask_[i];
    }
  }

  auto rects = FindRect(640, 480, img_copy.data(), 220, 300);

  spdlog::debug("==== t: {} ====", t_us);
  auto cam_to_world_rot = Eigen::Rotation2Df(heading_);
  for (const auto& rect : rects) {
    Eigen::Vector2f pos_cam =
        camera_lookup(rect.x + rect.width / 2, rect.y + rect.height / 2) *
        kCeilHeight;
    Eigen::Vector2f pos_world =
        cam_to_world_rot * pos_cam + Eigen::Vector2f{x_, y_};

    bool paired = false;
    for (const auto& landmark : landmarks_) {
      float dist = (landmark.pos - pos_world).norm();
      if (dist < kLightAssocDist) {
        spdlog::debug("paired light {} {} with {} {}", pos_world.x(),
                      pos_world.y(), landmark.pos.x(), landmark.pos.y());
        // do something!
        paired = true;
        break;
      }
    }
    if (paired) continue;

    for (auto it = landmark_candidates_.begin();
         it != landmark_candidates_.end(); ++it) {
      float dist = (it->pos - pos_world).norm();
      if (dist < kLightAssocDist) {
        // do something!
        landmarks_.push_back(*it);
        landmark_candidates_.erase(it);

        paired = true;
        break;
      }
    }
    if (paired) continue;

    landmark_candidates_.push_back({.pos = pos_world});
  }
}

void SLAM::OdoFrame(int64_t t_us, float odo_dist_delta, float odo_heading_delta,
                    float imu_rot_z) {
  x_ += odo_dist_delta * cos(heading_ + odo_heading_delta / 2.0);
  y_ += odo_dist_delta * sin(heading_ + odo_heading_delta / 2.0);
  heading_ = normAngle(heading_ + odo_heading_delta);

  // covariance?
}

SLAM::SLAM() {
  {
    std::ifstream f("../data/calib/camera_lut.bin",
                    std::ios::in | std::ios::binary);
    if (!f.good()) throw std::runtime_error("error opening lut");
    f.read(reinterpret_cast<char*>(camera_lut_), 640 * 480 * 2 * 4);
    if (!f) throw std::runtime_error("error while reading lut");
    f.close();
  }

  {
    std::ifstream f("../data/ceil_mask.bin", std::ios::in | std::ios::binary);
    if (!f.good()) throw std::runtime_error("error opening ceil_mask");
    f.read(reinterpret_cast<char*>(ceil_mask_), 640 * 480);
    if (!f) throw std::runtime_error("error while reading lut");
    f.close();
  }
}

int main() {
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
  return 0;
}