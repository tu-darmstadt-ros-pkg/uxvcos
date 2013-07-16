set(SOURCES
  Autonomy.h
  Autonomy.cpp
)

orocos_component(autonomy ${SOURCES})
target_link_libraries(autonomy ${UXVCOS_LIBRARIES})

option(BUILD_SIMULINK_AUTONOMY "Use a Simulink model compiled with Orocos target as Autonomy Controller" OFF)
if(BUILD_SIMULINK_AUTONOMY AND NOT TARGET simulink)
  message(SEND_ERROR "You have to enable BUILD_SIMULINK_INTERFACE to add support for Simulink TaskContexts.")
  unset(BUILD_SIMULINK_CONTROLLER)
endif()
if(BUILD_SIMULINK_AUTONOMY)
  orocos_component(autonomy-simulink ${LIBRARY_TYPE} Simulink.cpp Simulink.h)
endif()
