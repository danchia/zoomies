add_library(hw hw.cc)

add_library(camera camera.cc)
target_link_libraries(camera mmal_core mmal_util mmal_components)

add_executable(camera_test camera_test.cc)
target_link_libraries(camera_test camera)

add_library(js js.cc)