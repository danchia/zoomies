add_library(driver_lib controlloop.cc)
target_link_libraries(driver_lib hw common js spdlog::spdlog)

add_executable(driver driver.cc)
target_link_libraries(driver driver_lib hw camera js spdlog::spdlog)