#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mcap/mcap.hpp>
#include <unordered_set>

#include "google/protobuf/descriptor.pb.h"
#include "ros/geometry_msgs/PointStamped.pb.h"
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
                   std::unordered_set<std::string> files,
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

int main() {
  // Open an output file stream. Other output interfaces can be used as well,
  // including providing your own mcap::IWritable implementation
  std::ofstream out("/tmp/output.mcap", std::ios::binary);

  // Initialize an MCAP writer with the "ros1" profile and write the file header
  mcap::McapWriter writer;
  mcap::McapWriterOptions opts("protobuf");
  opts.compressionLevel = mcap::CompressionLevel::Fast;
  writer.open(out, opts);

  {
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
  }

  {
    // Register a Schema
    mcap::Schema schema("ros.geometry_msgs.PointStamped", "proto",
                        fdSet(ros::geometry_msgs::PointStamped::descriptor()));
    writer.addSchema(schema);

    // Register a Channel
    mcap::Channel chatterPublisher("/point", "protobuf", schema.id);
    writer.addChannel(chatterPublisher);

    ros::geometry_msgs::PointStamped p;
    p.mutable_header()->set_frame_id("/map");
    p.mutable_point()->set_x(1.0);

    std::string payload = p.SerializeAsString();

    // Write our message
    mcap::Message msg;
    msg.channelId = chatterPublisher.id;
    msg.sequence = 1;               // Optional
    msg.logTime = now();            // Required nanosecond timestamp
    msg.publishTime = msg.logTime;  // Set to logTime if not available
    msg.data = reinterpret_cast<std::byte*>(payload.data());
    msg.dataSize = payload.size();
    writer.write(msg);
  }

  // Finish writing the file
  writer.close();
}