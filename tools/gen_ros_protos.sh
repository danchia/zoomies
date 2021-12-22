PROTO_PATH=$HOME/projects/ros-message-schemas/proto
protoc -I=$PROTO_PATH --python_out=ros_msgs  $PROTO_PATH/ros/**/*.proto
protoc -I=$PROTO_PATH $PROTO_PATH/ros/**/*.proto --include_imports --descriptor_set_out=ros_msgs/descriptor.bin
# DESCRIPTOR_PATH=ros_msgs/descriptors
# protoc -I=$PROTO_PATH ros/sensor_msgs/CompressedImage.proto --include_imports --descriptor_set_out=$DESCRIPTOR_PATH/CompressedImage.bin
