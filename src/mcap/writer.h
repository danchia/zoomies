#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/message_lite.h"

class McapLogWriter {
 public:
  virtual ~McapLogWriter() {}

  static std::unique_ptr<McapLogWriter> Make(const std::string& path);

  virtual int AddChannel(const std::string& topic,
                         const google::protobuf::Descriptor* d) = 0;

  virtual void WriteMsg(int chan_id, int64_t timestamp_us,
                        const std::string& data) = 0;

  void Write(int chan_id, int64_t timestamp_us,
             const google::protobuf::MessageLite& m) {
    scratch_.clear();
    m.SerializeToString(&scratch_);

    WriteMsg(chan_id, timestamp_us, scratch_);
  }

 private:
  std::string scratch_;
};