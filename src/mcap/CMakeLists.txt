include_directories(mcap/cpp/mcap/include)

add_executable(mcap_main mcap_main.cc)

target_link_libraries(mcap_main CONAN_PKG::zstd CONAN_PKG::lz4)