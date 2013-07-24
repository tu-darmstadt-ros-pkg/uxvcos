# disable optimization for this directory (reduce memory usage)
#string(REPLACE "-O${OPTIMIZATION_LEVEL}" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
#string(REPLACE "-O${OPTIMIZATION_LEVEL}" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

set(INTERFACE_SOURCES_ETHERNET
  EthernetInterface.h
  EthernetInterface.cpp
  EthernetSerialPort.h
  EthernetSerialPort.cpp
  EthernetIMU.h
  EthernetIMU.cpp
  EthernetAHRS.h
  EthernetAHRS.cpp
  EthernetSensorBase.h
  EthernetSensorBase.cpp
  EthernetSensor.h
  EthernetSensors.cpp
  Motor.h
  Servos.h
  DirectFeedback.h
  DirectFeedback.cpp
  arm-interface.h
  ublox.c
  ublox.h
)

orocos_use_package(uxvcos-interface-common)
orocos_use_package(uxvcos-sensors)
orocos_component(interface-ethernet ${INTERFACE_SOURCES_ETHERNET})
