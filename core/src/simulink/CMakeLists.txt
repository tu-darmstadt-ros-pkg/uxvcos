option(BUILD_SIMULINK_INTERFACE "Use a Simulink model compiled with Orocos target as Controller" ON)

if(BUILD_SIMULINK_INTERFACE)
  include(FindMatlab)
  if(NOT MATLAB_FOUND)
    message("Matlab not found. Set MATLAB_ROOT variable or SIMULINK_INTERFACE will be disabled.")
    set(BUILD_SIMULINK_INTERFACE OFF)
  endif(NOT MATLAB_FOUND)

  if(MATLAB_FOUND)
    orocos_library(simulink ${LIBRARY_TYPE} SimulinkInterface.cpp SimulinkInterface.h SimulinkInterface.inl)
    #set_target_properties(simulink PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/orocos${OROCOS_SUFFIX})
    target_link_libraries(simulink uxvcos)

    orocos_generate_package(uxvcos-simulink-${OROCOS_TARGET} DEPENDS uxvcos_rtt-${OROCOS_TARGET})
  endif()
endif()
