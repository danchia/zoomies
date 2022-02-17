file(GLOB_RECURSE PROTO_SRCS generated/*.pb.cc)
add_library(proto ${PROTO_SRCS})

target_link_libraries(proto CONAN_PKG::protobuf)