mkdir -p build
cd build
conan install ../src --build=missing
cmake -DCMAKE_BUILD_TYPE=Release ../src
