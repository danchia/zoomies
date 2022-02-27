#include "ros/ros_reader.h"

#include <string_view>

#include "spdlog/spdlog.h"

namespace {

void sCheck(int rc, std::string_view msg, sqlite3* db) {
  if (rc) {
    const char* err_msg = sqlite3_errmsg(db);
    spdlog::warn("sqlite {} {}: {}", msg, sqlite3_errstr(rc), err_msg);
    throw std::runtime_error("sqlite error");
  }
}

}  // namespace

RosReader::RosReader(const std::string& db_path) {
  sCheck(sqlite3_open(db_path.c_str(), &db_), "open", db_);

  sqlite3_stmt* stmt;
  sCheck(sqlite3_prepare_v2(db_, "SELECT id, name FROM topics", -1, &stmt,
                            nullptr),
         "topics sql", db_);
  while (true) {
    int rc = sqlite3_step(stmt);
    if (rc == SQLITE_DONE) break;
    if (rc != SQLITE_ROW) {
      sCheck(rc, "topics next", db_);
      break;
    }

    int id = sqlite3_column_int(stmt, 0);
    std::string topic(
        reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1)));

    topics_[topic] = id;
  }
  sCheck(sqlite3_finalize(stmt), "finalize topics sql", db_);
}

RosReader::~RosReader() { sqlite3_close(db_); }

void RosReader::ReadMessages(const std::function<bool(MessageView)>& f) {
  sqlite3_stmt* stmt;
  sCheck(sqlite3_prepare_v2(
             db_, "SELECT topic_id, data FROM messages ORDER BY timestamp", -1,
             &stmt, nullptr),
         "read sql", db_);

  while (true) {
    int rc = sqlite3_step(stmt);
    if (rc == SQLITE_DONE) break;
    if (rc != SQLITE_ROW) {
      sCheck(rc, "read next", db_);
      break;
    }

    int topic_id = sqlite3_column_int(stmt, 0);
    int len = sqlite3_column_bytes(stmt, 1);
    const char* buf =
        reinterpret_cast<const char*>(sqlite3_column_blob(stmt, 1));
    if (!f(MessageView{.topic_id = topic_id, .buf = buf, .len = len})) {
      break;
    }
  }

  sCheck(sqlite3_finalize(stmt), "finalize read sql", db_);
}