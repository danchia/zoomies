add_library(ros_compat ros_reader.cc ros_writer.cc ros_types.cxx)
target_link_libraries(ros_compat fastcdr sqlite3 spdlog)

add_executable(ros_test ros_test.cc)
target_link_libraries(ros_test ros_compat)