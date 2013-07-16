set(SOURCES
  Navigation.h
  Navigation.cpp
  Compass.h
  Barometer.cpp
  Barometer.h
  EarthCalculations.cpp
  EarthCalculations.h
  IMUFilter.cpp
  IMUFilter.h
  Magnetometer3D.cpp
  Magnetometer3D.h
  WorldMagneticModel_2010.cpp
  WorldMagneticModel_2010.h
)

if(ROS_ROOT)
  set(SOURCES ${SOURCES}
    PoseUpdate.cpp
    PoseUpdate.h
  )
  add_definitions(-DHAVE_POSEUPDATE)
endif(ROS_ROOT)

include (models/CMakeLists.txt)

source_group("." REGULAR_EXPRESSION "[^/]+$")
source_group("models" REGULAR_EXPRESSION "models/[^/]+$")

# clear pkg-config configuration from previous runs
set(bfl_COMP_FOUND 0)
set(bfl_eigen_COMP_FOUND 0)
set(bfl_boost_COMP_FOUND 0)
set(orocos-bfl_COMP_FOUND 0)
unset(orocos-bfl_orocos-bfl_LIBRARY CACHE)

# orocos-bfl ist not a ROS package itself.
# Setting USE_FOUND_orocos-bfl_PACKAGE_PATH to 1 prevents orocos_use_package to search for a ROS package named orocos-bfl.
set(USE_FOUND_orocos-bfl_PACKAGE_PATH 1)

#orocos_use_package(bfl)
orocos_use_package(bfl_eigen)
#orocos_use_package(bfl_boost)
orocos_use_package(orocos-bfl)

orocos_component(navigation ${SOURCES})
