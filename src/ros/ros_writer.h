#pragma once

#include <filesystem>
#include <string_view>
#include <vector>

#include "fastcdr/Cdr.h"
#include "sqlite/sqlite3.h"

class RosWriter {
 public:
  RosWriter(std::string_view path);
  ~RosWriter();

  int AddConnection(std::string_view topic, std::string_view msg_type);

  template <typename T>
  void Write(int conn_id, int64_t timestamp_us, const T& data) {
    ++topics_[conn_id - 1].msg_count;
    min_time_us_ = std::min(min_time_us_, timestamp_us);
    max_time_us_ = std::max(max_time_us_, timestamp_us);

    int l = 4 + T::getCdrSerializedSize(data);
    if (l > scratch_.size()) {
      scratch_.resize(l);
    }

    eprosima::fastcdr::FastBuffer fastbuf(scratch_.data(), l);
    eprosima::fastcdr::Cdr cdr_ser(fastbuf,
                                   eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                                   eprosima::fastcdr::Cdr::DDS_CDR);

    cdr_ser.serialize_encapsulation();
    data.serialize(cdr_ser);

    InsertMsg(
        conn_id, timestamp_us * 1000,
        std::string_view(scratch_.data(), cdr_ser.getSerializedDataLength()));
  }

 private:
  struct Topic {
    std::string name;
    std::string msg_type;
    int msg_count = 0;
  };

  void InsertMsg(int conn_id, int64_t timestamp_ns, std::string_view data);

  std::vector<Topic> topics_;
  int64_t min_time_us_ = std::numeric_limits<int64_t>::max();
  int64_t max_time_us_ = 0;

  sqlite3* db_ = nullptr;
  sqlite3_stmt* insert_msg_stmt_ = nullptr;
  sqlite3_stmt* insert_topic_stmt_ = nullptr;
  std::string scratch_;

  std::string meta_path_;
  std::string db_rel_path_;
};
