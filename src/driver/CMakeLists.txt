add_library(driver_lib driver.cc datalogger.cc)
target_link_libraries(driver_lib hw common js localizer ros_compat track CONAN_PKG::spdlog)

add_executable(driver driver_main.cc)
target_link_libraries(driver driver_lib hw camera js CONAN_PKG::spdlog)

add_executable(esc_calib esc_calib.cc)
target_link_libraries(esc_calib hw js CONAN_PKG::spdlog)