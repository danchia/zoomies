#include "writer.h"

#include "google/protobuf/descriptor.pb.h"
#include "mcap/mcap.hpp"

class McapLogWriterImpl final : public McapLogWriter {
 public:
  McapLogWriterImpl(const std::string& path);
  ~McapLogWriterImpl() { mcap_.close(); }

  int AddChannel(const std::string& topic,
                 const google::protobuf::Descriptor* d) final;

  void WriteMsg(int chan_id, int64_t timestamp_us,
                const std::string& data) final;

 private:
  std::ofstream out_;
  mcap::McapWriter mcap_;
  std::unordered_map<std::string, int> schema_ids_;
};

std::unique_ptr<McapLogWriter> McapLogWriter::Make(const std::string& path) {
  return std::make_unique<McapLogWriterImpl>(path);
}

McapLogWriterImpl::McapLogWriterImpl(const std::string& path)
    : out_(path, std::ios::binary) {
  if (out_.bad()) {
    throw std::runtime_error("could not open mcap writer");
  }

  mcap::McapWriterOptions opts("protobuf");
  opts.compressionLevel = mcap::CompressionLevel::Fast;
  mcap_.open(out_, opts);
}

namespace {

void fdSetInternal(google::protobuf::FileDescriptorSet& fd_set,
                   std::unordered_set<std::string>& files,
                   const google::protobuf::FileDescriptor* fd) {
  for (int i = 0; i < fd->dependency_count(); ++i) {
    const auto* dep = fd->dependency(i);
    auto [_, inserted] = files.insert(dep->name());
    if (!inserted) continue;
    fdSetInternal(fd_set, files, fd->dependency(i));
  }
  fd->CopyTo(fd_set.add_file());
}

std::string fdSet(const google::protobuf::Descriptor* d) {
  std::string res;
  std::unordered_set<std::string> files;
  google::protobuf::FileDescriptorSet fd_set;
  fdSetInternal(fd_set, files, d->file());
  return fd_set.SerializeAsString();
}

}  // namespace

int McapLogWriterImpl::AddChannel(const std::string& topic,
                                  const google::protobuf::Descriptor* d) {
  const std::string& msg_name = d->full_name();
  int schema_id = -1;
  auto it = schema_ids_.find(msg_name);
  if (it != schema_ids_.end()) {
    schema_id = it->second;
  } else {
    mcap::Schema schema(msg_name, "protobuf", fdSet(d));
    mcap_.addSchema(schema);
    schema_ids_[msg_name] = schema.id;
    schema_id = schema.id;
  }

  mcap::Channel chan(topic, "protobuf", schema_id);
  mcap_.addChannel(chan);

  return chan.id;
}

void McapLogWriterImpl::WriteMsg(int chan_id, int64_t timestamp_us,
                                 const std::string& data) {
  mcap::Message msg;
  msg.channelId = chan_id;
  msg.logTime = timestamp_us * 1000;  // Required nanosecond timestamp
  msg.publishTime = msg.logTime;
  msg.data = reinterpret_cast<const std::byte*>(data.data());
  msg.dataSize = data.size();
  mcap_.write(msg);
}