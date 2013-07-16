if(NOT DEFINED RTT_PLUGINS)
  set(RTT_PLUGINS rtt-scripting rtt-marshalling rtt-typekit)
endif(NOT DEFINED RTT_PLUGINS)

if(DEFINED ENV{OROCOS_TARGET})
  set(OROCOS_TARGET $ENV{OROCOS_TARGET} CACHE STRING "" FORCE)
elseif(NOT DEFINED OROCOS_TARGET)
  if(UNIX)
    set(OROCOS_TARGET "gnulinux" CACHE STRING "")
  elseif(WIN32)
    set(OROCOS_TARGET "win32" CACHE STRING "")
  endif()
endif()
set(OROCOS_SUFFIX "/${OROCOS_TARGET}")

if(ROSBUILD_init_called)
  rosbuild_find_ros_package(rtt)
  set(OROCOS-RTT_HINTS ${rtt_PACKAGE_PATH}/../install/lib/cmake/orocos-rtt ${rtt_PACKAGE_PATH}/install/lib/cmake/orocos-rtt ${rtt_PACKAGE_PATH}/../../lib/cmake/orocos-rtt)
endif(ROSBUILD_init_called)

find_package(OROCOS-RTT REQUIRED ${RTT_PLUGINS} HINTS ${OROCOS-RTT_HINTS})

if(OROCOS-RTT_FOUND)
  include(${OROCOS-RTT_USE_FILE})
  list(APPEND OROCOS-RTT_LIBRARIES ${OROCOS-RTT_RTT-TYPEKIT_LIBRARY})

else(OROCOS-RTT_FOUND)
  set(OROCOS_TARGET)
  message(FATAL_ERROR "Orocos RTT could not be found!")

endif(OROCOS-RTT_FOUND)
