cmake_minimum_required(VERSION 3.10)

# set the project name
project(Zoomies)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-Wno-psabi)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include_directories(.)

add_subdirectory(eigen)
include_directories(eigen)

add_subdirectory(Fast-CDR)
include_directories(Fast-CDR/include)

include_directories(stats/include)
include_directories(gcem/include)

set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen)
add_subdirectory(g2o)
include_directories(g2o)
include_directories(${PROJECT_BINARY_DIR}/g2o)

OPTION(INSTALL_GTEST "" OFF)
include_directories(googletest/googletest/include)
include_directories(googletest/googlemock/include)
add_subdirectory(googletest)

add_subdirectory(sqlite)

set(LIBRARY_TYPE STATIC)
add_subdirectory(userland)
include_directories(userland)

include_directories(proto/generated)
add_subdirectory(proto)
add_subdirectory(common)
add_subdirectory(mcap)
add_subdirectory(track)
add_subdirectory(ros)
add_subdirectory(hw)
add_subdirectory(localization)
add_subdirectory(driver)