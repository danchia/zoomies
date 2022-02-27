#include <fstream>
#include <iostream>
#include <vector>

#include "fastcdr/Cdr.h"
#include "ros/ros_reader.h"
#include "ros/ros_types.h"
#include "ros/ros_writer.h"

int main() {
  constexpr int kLength = 164817;
  std::vector<uint8_t> img;
  img.resize(164817);
  {
    std::ifstream f("/home/danchia/f.png", std::ios::binary);
    f.read(reinterpret_cast<char*>(img.data()), img.size());
    f.close();
  }

  sensor_msgs__CompressedImage cimg;
  cimg.header().stamp().sec(0);
  cimg.header().stamp().nsec(1000000);
  cimg.header().frame_id("/camera");
  cimg.format("png");
  cimg.data(img);

  {
    RosWriter writer("/tmp/ros_test");
    int conn_id = writer.AddConnection("/camera1/image",
                                       "sensor_msgs/msg/CompressedImage");
    writer.Write(conn_id, 1000, cimg);
    cimg.header().stamp().nsec(2000000);
    writer.Write(conn_id, 2000, cimg);
    cimg.header().stamp().nsec(3000000);
    writer.Write(conn_id, 3000, cimg);
  }

  {
    RosReader reader("/tmp/ros_test/ros_test.db3");
    int cam_topic = reader.topic_id("/camera1/iamge");

    sensor_msgs__CompressedImage cimg;
    reader.ReadMessages([&](RosReader::MessageView v) -> bool {
      v.As(cimg);
      std::cout << cimg.header().stamp().nsec() << "\n";
      return true;
    });
  }

  return 0;
}