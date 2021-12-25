add_library(slam_util slam_util.cc)

add_executable(slam slam.cc)
target_link_libraries(slam slam_util sqlite3 core types_slam2d spdlog)

enable_testing()

add_executable(slam_util_test slam_util_test.cc)
target_link_libraries(slam_util_test slam_util gmock_main)

include(GoogleTest)
gtest_discover_tests(slam_util_test)