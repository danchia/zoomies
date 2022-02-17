#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mcap/mcap.hpp>

// Returns the system time in nanoseconds. std::chrono is used here, but any
// high resolution clock API (such as clock_gettime) can be used.
mcap::Timestamp now() {
  return mcap::Timestamp(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

int main() {
  // Open an output file stream. Other output interfaces can be used as well,
  // including providing your own mcap::IWritable implementation
  std::ofstream out("output.mcap", std::ios::binary);

  // Initialize an MCAP writer with the "ros1" profile and write the file header
  mcap::McapWriter writer;
  writer.open(out, mcap::McapWriterOptions("ros1"));

  // Register a Schema
  mcap::Schema stdMsgsString("std_msgs/String", "ros1msg", "string data");
  writer.addSchema(stdMsgsString);

  // Register a Channel
  mcap::Channel chatterPublisher("/chatter", "ros1", stdMsgsString.id);
  writer.addChannel(chatterPublisher);

  // Create a message payload. This would typically be done by your own
  // serialiation library. In this example, we manually create ROS1 binary data
  std::array<std::byte, 4 + 13> payload;
  const uint32_t length = 13;
  std::memcpy(payload.data(), &length, 4);
  std::memcpy(payload.data() + 4, "Hello, world!", 13);

  // Write our message
  mcap::Message msg;
  msg.channelId = chatterPublisher.id;
  msg.sequence = 1;               // Optional
  msg.logTime = now();            // Required nanosecond timestamp
  msg.publishTime = msg.logTime;  // Set to logTime if not available
  msg.data = payload.data();
  msg.dataSize = payload.size();
  writer.write(msg);

  // Finish writing the file
  writer.close();
}