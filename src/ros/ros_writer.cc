#include "ros/ros_writer.h"

#include <inttypes.h>

#include <filesystem>
#include <fstream>
#include <string_view>
#include <vector>

#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
#include "sqlite/sqlite3.h"

namespace {

void sCheck(int rc, std::string_view msg, sqlite3* db) {
  if (rc) {
    const char* err_msg = sqlite3_errmsg(db);
    spdlog::warn("sqlite {} {}: {}", msg, sqlite3_errstr(rc), err_msg);
    throw std::runtime_error("sqlite error");
  }
}

constexpr const char* kSchema = R"(
  -- PRAGMA journal_mode = WAL;
  -- PRAGMA synchronous = normal;
  -- PRAGMA wal_autocheckpoint = 0;
  PRAGMA journal_mode = MEMORY;
  PRAGMA synchronous = off;

  CREATE TABLE topics(
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    type TEXT NOT NULL,
    serialization_format TEXT NOT NULL,
    offered_qos_profiles TEXT NOT NULL
  );
  CREATE TABLE messages(
    id INTEGER PRIMARY KEY,
    topic_id INTEGER NOT NULL,
    timestamp INTEGER NOT NULL,
    data BLOB NOT NULL
  );
  CREATE INDEX timestamp_idx ON messages (timestamp ASC);
)";

}  // namespace

RosWriter::RosWriter(std::string_view path) {
  std::filesystem::path fs_path(path);
  auto db_name = fs_path.filename();
  db_name.replace_extension(".db3");
  auto db_path = fs_path / db_name;

  meta_path_ = (fs_path / "metadata.yaml").string();
  db_rel_path_ = db_name.string();

  if (!std::filesystem::create_directories(fs_path)) {
    spdlog::error("could not create directory {}", fs_path);
    throw std::runtime_error("error creating rosbag dir");
  }

  sCheck(sqlite3_open(db_path.string().c_str(), &db_), "open", db_);
  sCheck(sqlite3_exec(db_, kSchema, nullptr, nullptr, nullptr), "schema", db_);

  sCheck(
      sqlite3_prepare_v2(
          db_,
          "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
          -1, &insert_msg_stmt_, nullptr),
      "prepare insert msg", db_);
  sCheck(sqlite3_prepare_v2(
             db_,
             "INSERT INTO topics (id, name, type, serialization_format, "
             "offered_qos_profiles) VALUES (?, ?, ?, 'cdr', '')",
             -1, &insert_topic_stmt_, nullptr),
         "prepare insert topic", db_);
}

RosWriter::~RosWriter() {
  sqlite3_finalize(insert_msg_stmt_);
  sqlite3_finalize(insert_topic_stmt_);
  // sqlite3_wal_checkpoint_v2(db_, nullptr, SQLITE_CHECKPOINT_FULL, nullptr,
  //                           nullptr);
  // sqlite3_exec(db_, "pragma journal_mode=delete;", nullptr, nullptr,
  // nullptr);
  sqlite3_close(db_);

  int64_t msg_count = 0;
  for (const auto& t : topics_) msg_count += t.msg_count;

  std::ofstream f(meta_path_.c_str());
  fmt::print(f, R"(rosbag2_bagfile_information:
  compression_format: ''
  compression_mode: ''
  duration:
    nanoseconds: {}
  message_count: {}
  relative_file_paths:
  - {}
  starting_time:
    nanoseconds_since_epoch: {}
  storage_identifier: sqlite3
  version: 4
  topics_with_message_count:
)",
             (max_time_us_ - min_time_us_) * 1000, msg_count, db_rel_path_,
             min_time_us_ * 1000);

  for (const auto& t : topics_) {
    fmt::print(f, R"(    - message_count: {}
    topic_metadata:
      name: {}
      offered_qos_profiles: ''
      serialization_format: cdr
      type: {}
)",
               t.msg_count, t.name, t.msg_type);
  }

  f.close();
}

int RosWriter::AddConnection(std::string_view topic,
                             std::string_view msg_type) {
  topics_.push_back(Topic{.name = std::string(topic),
                          .msg_type = std::string(msg_type),
                          .msg_count = 0});

  int id = topics_.size();

  sCheck(sqlite3_reset(insert_topic_stmt_), "reset topic", db_);
  sCheck(sqlite3_bind_int(insert_topic_stmt_, 1, id), "bind topic id", db_);
  sCheck(sqlite3_bind_text(insert_topic_stmt_, 2, topic.data(), topic.size(),
                           SQLITE_TRANSIENT),
         "bind topic name", db_);
  sCheck(sqlite3_bind_text(insert_topic_stmt_, 3, msg_type.data(),
                           msg_type.size(), SQLITE_TRANSIENT),
         "bind msg type", db_);
  int rc = sqlite3_step(insert_topic_stmt_);
  if (rc != SQLITE_DONE) {
    sCheck(rc, "insert topic step", db_);
  }

  return id;
}
void RosWriter::InsertMsg(int conn_id, int64_t timestamp_ns,
                          std::string_view data) {
  sCheck(sqlite3_reset(insert_msg_stmt_), "reset msg stmt", db_);
  sCheck(sqlite3_bind_int(insert_msg_stmt_, 1, conn_id), "bind conn id", db_);
  sCheck(sqlite3_bind_int64(insert_msg_stmt_, 2, timestamp_ns), "bind tstamp",
         db_);
  sCheck(sqlite3_bind_blob(insert_msg_stmt_, 3, data.data(), data.size(),
                           SQLITE_TRANSIENT),
         "bind data", db_);
  int rc = sqlite3_step(insert_msg_stmt_);
  if (rc != SQLITE_DONE) {
    sCheck(rc, "insert msg step", db_);
  }
}