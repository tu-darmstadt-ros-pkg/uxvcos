option(BUILD_SIMULINK_SIMULATION "Use a Simulink model compiled with Orocos target as Simulation model" OFF)

set(SOURCES
  Simulink.cpp
  Simulink.h
)

if(BUILD_SIMULINK_SIMULATION)
  include(FindMatlab)
  if(NOT MATLAB_FOUND)
    message(FATAL_ERROR "Matlab not found, try to set MATLAB_ROOT and try again please")
  endif(NOT MATLAB_FOUND)

  set(SIMULINK_SIMULATION_MODEL "" CACHE STRING "Default name of the Simulink model for the simulation")
  set(SIMULINK_SIMULATION_PATH "" CACHE PATH "path to the output of Matlab RTW target compiler")

  orocos_component(simulation-simulink ${SOURCES})
  set_target_properties(simulation-simulink PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/orocos${OROCOS_SUFFIX})
  set_property(TARGET simulation-simulink PROPERTY COMPILE_DEFINITIONS MODELNAME=${SIMULINK_SIMULATION_MODEL} MODELPATH=${SIMULINK_SIMULATION_PATH})
  target_link_libraries(simulation-simulink)
endif()
