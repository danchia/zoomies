mkdir -p build_rpi3
cd build_rpi3
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../rpi-toolchain.cmake ../src
conan install ../src --build=missing -pr=../conan_rpi_release
