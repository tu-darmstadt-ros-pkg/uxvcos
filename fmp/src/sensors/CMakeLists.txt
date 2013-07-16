set(SOURCES
  FMP.h
  FMP.cpp
)

orocos_use_package(uxvcos-sensors)
orocos_library(sensors-fmp ${SOURCES})
