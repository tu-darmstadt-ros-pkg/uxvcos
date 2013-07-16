set(SOURCES
  Sensor.h
  IMU.h
  IMU.cpp
  Ultrasound.h
  Baro.h
  Temperature.h
  Magnetic.h
  Airspeed.h
  Supply.h
)

orocos_library(sensors ${SOURCES})
target_link_libraries(sensors uxvcos)
#target_link_libraries(sensors uxvcos interface-types sensors-types)

#add_subdirectory(gps)

orocos_generate_package(uxvcos-sensors-${OROCOS_TARGET} DEPENDS uxvcos_rtt-${OROCOS_TARGET})

