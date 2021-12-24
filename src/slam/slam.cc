#include <inttypes.h>

#include <string_view>

#include "spdlog/spdlog.h"
#include "sqlite/sqlite3.h"

bool sCheckAndLog(int rc, std::string_view msg, sqlite3* db) {
  if (rc) {
    const char* err_msg = sqlite3_errmsg(db);
    spdlog::warn("sqlite {} {}: {}", msg, sqlite3_errstr(rc), err_msg);
    return false;
  }
  return true;
}

int main() {
  sqlite3* db = nullptr;

  if (!sCheckAndLog(sqlite3_open("/home/danchia/data.db3", &db), "open", db)) {
    return 1;
  }

  sqlite3_stmt* stmt = nullptr;
  if (!sCheckAndLog(
          sqlite3_prepare_v2(db, "SELECT t_us, data FROM videos ORDER BY t_us",
                             -1, &stmt, nullptr),
          "video sql", db)) {
    return 1;
  }

  int rc;
  int images = 0;
  while (true) {
    rc = sqlite3_step(stmt);
    if (rc != SQLITE_ROW) break;

    int64_t t_us = sqlite3_column_int64(stmt, 0);

    std::vector<uint8_t> image;
    image.resize(sqlite3_column_bytes(stmt, 1));
    memcpy(image.data(), sqlite3_column_blob(stmt, 1), image.size());
    ++images;
  }
  if (rc != SQLITE_DONE) {
    sCheckAndLog(rc, "vid step", db);
    return 1;
  }

  spdlog::info("Read {} images", images);

  sCheckAndLog(sqlite3_finalize(stmt), "finalize vid sql", db);

  sqlite3_close(db);
  return 0;
}