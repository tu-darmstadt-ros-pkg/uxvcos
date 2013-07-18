set(SOURCES
  ControllerTask.cpp
  RC.cpp
  Limit.cpp
)

orocos_library(controller-common ${SOURCES})
target_link_libraries(controller-common ${UXVCOS_LIBRARIES})

orocos_generate_package(uxvcos-controller-common-${OROCOS_TARGET} DEPENDS ${UXVCOS_DEPENDS})
