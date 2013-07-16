option(BUILD_TESTBED "Whether to build testbed support (reference attitude)" OFF)

if(BUILD_TESTBED)
  set(SOURCES
    Testbed.h
    Testbed.cpp
  )

  include(${UXVCOS_PATH}/contrib/hudaqlib/CMakeLists.txt)
  orocos_component(testbed ${SOURCES})
  target_link_libraries(testbed ${UXVCOS_LIBRARIES} hudaqlib)
endif(BUILD_TESTBED)
