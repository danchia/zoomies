cmake_minimum_required(VERSION 3.10)

# set the project name
project(Zoomies)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add_compile_options(-Wno-psabi)

include_directories(.)

add_subdirectory(eigen)
include_directories(eigen)

set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen)
add_subdirectory(g2o)
include_directories(g2o)
include_directories(${PROJECT_BINARY_DIR}/g2o)

OPTION(INSTALL_GTEST "" OFF)
include_directories(googletest/googletest/include)
include_directories(googletest/googlemock/include)
add_subdirectory(googletest)

add_subdirectory(sqlite)
add_subdirectory(spdlog)
include_directories(spdlog/include)

set(LIBRARY_TYPE STATIC)
add_subdirectory(userland)
include_directories(userland)

add_subdirectory(common)
add_subdirectory(driver)
add_subdirectory(hw)
add_subdirectory(slam)
