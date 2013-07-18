SET(INTERFACE_SOURCES
  BaseInterface.cpp
  RC.cpp
)

set(INTERFACE_SOURCES_STREAM
  StreamInterface.cpp
)

set(INTERFACE_SOURCES_SERIAL
  SerialInterface.cpp
)

orocos_library(interface-common ${INTERFACE_SOURCES} ${INTERFACE_INCLUDES})
target_link_libraries(interface-common ${INTERFACE_LIBRARIES} uxvcos-rtt)

orocos_generate_package(uxvcos-interface-common-${OROCOS_TARGET} DEPENDS ${UXVCOS_DEPENDS})

option(BUILD_INTERFACE_STREAM "Build an Interface component that uses a Stream object" ON)
if(BUILD_INTERFACE_STREAM)
  orocos_component(interface-stream ${INTERFACE_SOURCES_STREAM})
  target_link_libraries(interface-stream interface-common)

  orocos_generate_package(uxvcos-interface-stream-${OROCOS_TARGET} DEPENDS ${UXVCOS_DEPENDS})
endif(BUILD_INTERFACE_STREAM)

include(CMakeDependentOption)
cmake_dependent_option(BUILD_INTERFACE_SERIAL "Build an Interface component that uses a serial port (requires BUILD_INTERFACE_STREAM)" ON "BUILD_INTERFACE_STREAM" ON)
if(BUILD_INTERFACE_SERIAL)
  orocos_component(interface-serial ${INTERFACE_SOURCES_SERIAL})
  target_link_libraries(interface-serial interface-stream)

  orocos_generate_package(uxvcos-interface-serial-${OROCOS_TARGET} DEPENDS ${UXVCOS_DEPENDS})
endif(BUILD_INTERFACE_SERIAL)
