find_package( Boost 1.36.0 COMPONENTS program_options serialization)
if(NOT Boost_FOUND)
  message(FATAL_ERROR "boost not found")
endif()

set(uxvcos_SOURCES
  uxvcos.h
  version.h
  version.cpp
)
set(uxvcos_LIBRARIES)

source_group("." FILES ${uxvcos_SOURCES})

include(Subversion)
if(Subversion_FOUND)
  set_source_files_properties(version.cpp PROPERTIES COMPILE_DEFINITIONS "SVN_REVISION=${uxvcos_WC_REVISION}")
endif(Subversion_FOUND)

include(system/CMakeLists.txt)
include(stream/CMakeLists.txt)
include(data/CMakeLists.txt)
include(options/CMakeLists.txt)
include(connector/CMakeLists.txt)

IF (OROCOS_TARGET)
  include(base/CMakeLists.txt)
  orocos_library(uxvcos ${uxvcos_SOURCES})
  target_link_libraries(uxvcos ${OROCOS-RTT_LIBRARIES})
ELSE (OROCOS_TARGET)
  if(WIN32)
    add_library(uxvcos SHARED ${uxvcos_SOURCES})
    set_target_properties(uxvcos PROPERTIES COMPILE_DEFINITIONS "uxvcos_EXPORTS")
  else(WIN32)
    add_library(uxvcos SHARED ${uxvcos_SOURCES})
  endif(WIN32)
ENDIF (OROCOS_TARGET)
target_link_libraries(uxvcos ${Boost_LIBRARIES} ${uxvcos_LIBRARIES})
target_link_libraries(uxvcos_main uxvcos)

#add_subdirectory(types)

if(OROCOS_TARGET)
  add_subdirectory(simulink)
  add_subdirectory(server)
  add_subdirectory(controller)
  add_subdirectory(sensors)
  add_subdirectory(interface)
  add_subdirectory(devices)
  add_subdirectory(logging)
  add_subdirectory(simulation)
  add_subdirectory(watchdog)
endif(OROCOS_TARGET)

add_subdirectory(util EXCLUDE_FROM_ALL)
