add_library(driver_lib driver.cc datalogger.cc)
target_link_libraries(driver_lib hw common js ros_compat spdlog::spdlog)

add_executable(driver driver_main.cc)
target_link_libraries(driver driver_lib hw camera js spdlog::spdlog)