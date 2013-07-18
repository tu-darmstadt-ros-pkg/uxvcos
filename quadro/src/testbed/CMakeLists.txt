if(BUILD_TESTBED)
  set(SOURCES
    Testbed.h
    Testbed.cpp
  )

  set(HUDAQLIB_LIBRARY ${PROJECT_SOURCE_DIR}/lib/libhudaqlib.so)
  message(STATUS "Building hudaqlib in ${HUDAQLIB_LIBRARY}")

  file(GLOB hudaqlib_DEPENDS "hudaqlib/*.c" "hudaqlib/*.h")
  add_custom_command(
    OUTPUT ${HUDAQLIB_LIBRARY}
    COMMAND ${CMAKE_MAKE_COMMAND}
    COMMAND ${CMAKE_COMMAND} -E copy libhudaqlib.so ${PROJECT_SOURCE_DIR}/lib
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/hudaqlib
    DEPENDS ${hudaqlib_DEPENDS}
  )
  add_custom_target(libhudaqlib DEPENDS ${HUDAQLIB_LIBRARY})

  add_library(hudaqlib SHARED IMPORTED)
  set_target_properties(hudaqlib PROPERTIES
    IMPORTED_LOCATION "${HUDAQLIB_LIBRARY}"
  )
  add_dependencies(hudaqlib libhudaqlib)

  orocos_component(testbed ${SOURCES})
  target_link_libraries(testbed ${UXVCOS_LIBRARIES} hudaqlib)
endif(BUILD_TESTBED)
