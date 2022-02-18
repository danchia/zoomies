add_library(mcap writer.cc)
target_link_libraries(mcap proto CONAN_PKG::zstd CONAN_PKG::lz4 CONAN_PKG::cryptopp CONAN_PKG::fmt)

add_executable(mcap_main mcap_main.cc)

target_link_libraries(mcap_main proto CONAN_PKG::zstd CONAN_PKG::lz4 CONAN_PKG::cryptopp CONAN_PKG::fmt)