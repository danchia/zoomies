add_library(hw hw.cc)

add_library(camera camera.cc)
target_link_libraries(camera mmal_core mmal_util mmal_components CONAN_PKG::spdlog)

add_executable(camera_test camera_test.cc)
target_link_libraries(camera_test camera CONAN_PKG::spdlog)

add_library(js js.cc)
target_link_libraries(js CONAN_PKG::spdlog)