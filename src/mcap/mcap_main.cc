#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mcap/mcap.hpp>

#include "google/protobuf/descriptor.pb.h"
#include "ros/std_msgs/String.pb.h"

// Returns the system time in nanoseconds. std::chrono is used here, but any
// high resolution clock API (such as clock_gettime) can be used.
mcap::Timestamp now() {
  return mcap::Timestamp(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

void fdSetInternal(google::protobuf::FileDescriptorSet& fd_set,
                   const google::protobuf::FileDescriptor* fd) {
  for (int i = 0; i < fd->dependency_count(); ++i) {
    fdSetInternal(fd_set, fd->dependency(i));
  }
  fd->CopyTo(fd_set.add_file());
}

std::string fdSet(const google::protobuf::Descriptor* d) {
  std::string res;
  google::protobuf::FileDescriptorSet fd_set;
  fdSetInternal(fd_set, d->file());
  return fd_set.SerializeAsString();
}

int main() {
  // Open an output file stream. Other output interfaces can be used as well,
  // including providing your own mcap::IWritable implementation
  std::ofstream out("/tmp/output.mcap", std::ios::binary);

  // Initialize an MCAP writer with the "ros1" profile and write the file header
  mcap::McapWriter writer;
  writer.open(out, mcap::McapWriterOptions("protobuf"));

  // Register a Schema
  mcap::Schema stdMsgsString("ros.std_msgs.String", "proto",
                             fdSet(ros::std_msgs::String::descriptor()));
  writer.addSchema(stdMsgsString);

  // Register a Channel
  mcap::Channel chatterPublisher("/chatter", "protobuf", stdMsgsString.id);
  writer.addChannel(chatterPublisher);

  ros::std_msgs::String s;
  s.set_data("foo");

  std::string payload = s.SerializeAsString();

  // Write our message
  mcap::Message msg;
  msg.channelId = chatterPublisher.id;
  msg.sequence = 1;               // Optional
  msg.logTime = now();            // Required nanosecond timestamp
  msg.publishTime = msg.logTime;  // Set to logTime if not available
  msg.data = reinterpret_cast<std::byte*>(payload.data());
  msg.dataSize = payload.size();
  writer.write(msg);

  // Finish writing the file
  writer.close();
}