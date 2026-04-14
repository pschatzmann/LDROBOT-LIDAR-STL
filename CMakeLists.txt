cmake_minimum_required(VERSION 3.16)

project(LD_LIDAR_STL VERSION 1.0.0 LANGUAGES CXX)

add_library(ld_lidar_stl INTERFACE)

target_include_directories(ld_lidar_stl
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(ld_lidar_stl INTERFACE cxx_std_11)

add_executable(linux-example examples/linux-example/linux-example.cpp
)
target_link_libraries(linux-example PRIVATE ld_lidar_stl)