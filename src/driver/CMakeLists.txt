add_executable(driver driver.cc)

target_link_libraries(driver PUBLIC hw -static)