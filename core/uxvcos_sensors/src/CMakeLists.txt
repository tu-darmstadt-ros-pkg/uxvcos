set(SOURCES
  IMU.cpp
)

orocos_library(sensors ${SOURCES})
target_link_libraries(sensors ${UXVCOS_LIBRARIES})

orocos_generate_package(uxvcos_sensors-${OROCOS_TARGET} DEPENDS ${UXVCOS_DEPENDS})
