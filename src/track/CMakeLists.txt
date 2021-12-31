add_library(track track.cc)
target_link_libraries(track spdlog::spdlog)

enable_testing()

add_executable(track_test track_test.cc)
target_link_libraries(track_test track gmock_main)

include(GoogleTest)
gtest_discover_tests(track_test)