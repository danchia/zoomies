mkdir -p build_rpi3
cd build_rpi3
cmake -DCMAKE_TOOLCHAIN_FILE=../rpi-toolchain.cmake ../src
