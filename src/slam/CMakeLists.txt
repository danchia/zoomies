add_executable(slam slam.cc)
target_link_libraries(slam sqlite3 core spdlog)