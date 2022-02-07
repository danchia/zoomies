#include <chrono>
#include <fstream>

#include "mcap/mcap.hpp"

mcap::Timestamp now() {
  const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  return mcap::Timestamp(timestamp.count());
}

int main() {
  mcap::McapWriter writer;

  auto options = mcap::McapWriterOptions("proto");
  options.compression = mcap::Compression::Zstd;

  std::ofstream out("/tmp/test.mcap", std::ios::binary);
  writer.open(out, options);

  mcap::ChannelInfo topic("/chatter", "protobuf", "std_msgs/String",
                          "string data");
  writer.addChannel(topic);

  mcap::Message msg;
  msg.channelId = topic.channelId;
  msg.sequence = 0;
  msg.publishTime = now();
  msg.recordTime = msg.publishTime;
  msg.dataSize = 0;

  auto s = writer.write(msg);
  if (!s.ok()) {
    writer.terminate();
    out.close();
    std::remove("output.mcap");
    return 1;
  }

  writer.close();
  return 0;
}