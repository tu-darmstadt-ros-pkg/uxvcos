set(SOURCES
  ControllerTask.h
  ControllerTask.cpp
  Controller.h
  RC.h
  RC.cpp
  PID.h
  Limit.h
  Limit.cpp
  helper.h
)

orocos_library(controller-common ${SOURCES})
target_link_libraries(controller-common uxvcos)

orocos_generate_package(uxvcos-controller-common-${OROCOS_TARGET} DEPENDS uxvcos_rtt-${OROCOS_TARGET})

