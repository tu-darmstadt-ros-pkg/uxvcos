set(SOURCES
  GPS.h
  GPS.cpp
)

include (gpsfunctions/CMakeLists.txt)

source_group("." REGULAR_EXPRESSION "[^/]+$")
source_group("gpsfunctions" REGULAR_EXPRESSION "gpsfunctions/[^/]+$")

orocos_component(gps ${SOURCES})
target_link_libraries(gps uxvcos)
