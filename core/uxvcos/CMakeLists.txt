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
endif(ROS_ROOT)

set(UXVCOS_VERSION_MAJOR 0)
set(UXVCOS_VERSION_MINOR 99)
set(UXVCOS_VERSION_PATCH)
set(UXVCOS_VERSION "${UXVCOS_VERSION_MAJOR}.${UXVCOS_VERSION_MINOR}")
if (DEFINED UXVCOS_VERSION_PATCH)
  set(UXVCOS_VERSION "${UXVCOS_VERSION_MAJOR}.${UXVCOS_VERSION_MINOR}.${UXVCOS_VERSION_PATCH}")
else()
  set(UXVCOS_VERSION "${UXVCOS_VERSION_MAJOR}.${UXVCOS_VERSION_MINOR}")
endif()

set(UXVCOS_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_MODULE_PATH  "${PROJECT_SOURCE_DIR}/cmake" "${PROJECT_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})
include(ToolchainOptions)

find_package(uxvcos REQUIRED PATHS "${UXVCOS_PATH}" NO_DEFAULT_PATH)

configure_file(include/uxvcos/version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/uxvcos/version.h @ONLY)

include_directories(include include/uxvcos)
add_subdirectory(src)

configure_file(uxvcos.pc.in ${CMAKE_CURRENT_SOURCE_DIR}/uxvcos.pc @ONLY)

