include_directories(mcap/cpp/mcap/include)

add_executable(mcap_main mcap_main.cc)

target_link_libraries(mcap_main proto CONAN_PKG::zstd CONAN_PKG::lz4 CONAN_PKG::cryptopp CONAN_PKG::fmt)