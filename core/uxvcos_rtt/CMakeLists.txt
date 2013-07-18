cmake_minimum_required(VERSION 2.6.0)

file(TO_CMAKE_PATH "$ENV{ROS_ROOT}" ROS_ROOT)
if(ROS_ROOT)
  cmake_minimum_required(VERSION 2.6.0)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  
  # Set the build type.  Options are:
  #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
  #  Debug          : w/ debug symbols, w/o optimization
  #  Release        : w/o debug symbols, w/ optimization
  #  RelWithDebInfo : w/ debug symbols, w/ optimization
  #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
  #set(ROS_BUILD_TYPE RelWithDebInfo)
  
  rosbuild_init()
  
  #set the default path for built executables to the "bin" directory
  #set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #set the default path for built libraries to the "lib" directory
  #set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
  
  #uncomment if you have defined messages
  #rosbuild_genmsg()
  #uncomment if you have defined services
  #rosbuild_gensrv()
  
  #common commands for building c++ executables and libraries
  #rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
  #target_link_libraries(${PROJECT_NAME} another_library)
  #rosbuild_add_boost_directories()
  #rosbuild_link_boost(${PROJECT_NAME} thread)
  #rosbuild_add_executable(example examples/example.cpp)
  #target_link_libraries(example ${PROJECT_NAME})

  rosbuild_find_ros_package(uxvcos)
endif(ROS_ROOT)

find_package(uxvcos_rtt REQUIRED PATHS . NO_DEFAULT_PATH)
include(ToolchainOptions)

include_directories(include include/uxvcos)
add_subdirectory(src)

configure_file(uxvcos_rtt.pc.in ${PROJECT_SOURCE_DIR}/uxvcos_rtt-${OROCOS_TARGET}.pc @ONLY)
