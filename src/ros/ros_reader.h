#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "fastcdr/Cdr.h"
#include "sqlite/sqlite3.h"

class RosReader {
 public:
  struct MessageView {
    int topic_id;
    const char* buf;
    int len;

    template <typename T>
    void As(T& x) {
      eprosima::fastcdr::FastBuffer fastbuf(const_cast<char*>(buf), len);
      eprosima::fastcdr::Cdr cdr_ser(fastbuf,
                                     eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                                     eprosima::fastcdr::Cdr::DDS_CDR);

      cdr_ser.read_encapsulation();
      x.deserialize(cdr_ser);
    }
  };
  RosReader(const std::string& db_path);
  ~RosReader();

  void ReadMessages(const std::function<bool(MessageView)>& f);

  int topic_id(const std::string& name) {
    auto it = topics_.find(name);
    if (it != topics_.end()) return it->second;
    return -1;
  }

 private:
  sqlite3* db_ = nullptr;

  std::unordered_map<std::string, int> topics_;
};