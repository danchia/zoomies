cmake_minimum_required(VERSION 3.10)

# set the project name
project(Zoomies)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

OPTION(BUILD_TESTING "" OFF)
OPTION(ABSL_USE_EXTERNAL_GOOGLETEST "" ON)
add_compile_options(-Wno-psabi)

include_directories(.)

add_subdirectory(eigen)
include_directories(eigen)
add_subdirectory(googletest)
add_subdirectory(spdlog)
include_directories(spdlog/include)

set(LIBRARY_TYPE STATIC)
add_subdirectory(userland)
include_directories(userland)

add_subdirectory(common)
add_subdirectory(driver)
add_subdirectory(hw)
