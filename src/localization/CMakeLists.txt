add_library(localizer localizer.cc pf.cc)
target_link_libraries(localizer localization_util CONAN_PKG::spdlog)

add_library(localization_util localization_util.cc)

add_executable(slam slam.cc)
target_link_libraries(slam ros_compat localization_util sqlite3 core types_slam2d mcap CONAN_PKG::spdlog)

add_executable(pf_main pf_main.cc)
target_link_libraries(pf_main localizer common ros_compat localization_util sqlite3 mcap CONAN_PKG::spdlog)

enable_testing()

add_executable(localization_util_test localization_util_test.cc)
target_link_libraries(localization_util_test localization_util gmock_main)

include(GoogleTest)
gtest_discover_tests(localization_util_test)