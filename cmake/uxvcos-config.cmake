# This script sets the following variables:
#  UXVCOS_FOUND: Boolean that indicates if UXVCOS was found
#  UXVCOS_INCLUDE_DIRS: Paths to the necessary header files
#  UXVCOS_LIBRARIES: Libraries to link against to use UXVCOS
#  UXVCOS_DEFINITIONS: Definitions to use when compiling code that uses UXVCOS
#
#  UXVCOS_PATH: Path of the UxVCoS directory (its CMAKE_INSTALL_PREFIX).
#
#  UXVCOS_VERSION: Package version
#  UXVCOS_VERSION_MAJOR: Package major version
#  UXVCOS_VERSION_MINOR: Package minor version
#  UXVCOS_VERSION_PATCH: Package patch version
#
########################################################################################################################

set(UXVCOS_FOUND true)
get_filename_component(UXVCOS_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(UXVCOS_PATH "${UXVCOS_PATH}" PATH)

set(UXVCOS_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src;${UXVCOS_PATH}/core/src")
set(UXVCOS_LIBRARIES "")
set(UXVCOS_DEFINITIONS "")

foreach(COMPONENT ${uxvcos_FIND_COMPONENTS} ${UXVCOS_FIND_COMPONENTS})
  set(UXVCOS_LIBRARIES ${UXVCOS_LIBRARIES} ${COMPONENT})
endforeach()

IF(WIN32)
  set(UXVCOS_DEFINITIONS ${UXVCOS_DEFINITIONS} "-DSYSTEM_WIN32")
  set(UXVCOS_INCLUDE_DIRS ${UXVCOS_INCLUDE_DIRS} "${UXVCOS_PATH}/src/system/win32/include")
ELSE(UNIX)
  set(UXVCOS_DEFINITIONS ${UXVCOS_DEFINITIONS} "-DSYSTEM_UNIX")
  set(UXVCOS_INCLUDE_DIRS ${UXVCOS_INCLUDE_DIRS} "${UXVCOS_PATH}/src/system/unix/include")
ENDIF ()

IF(APPLE)
  set(UXVCOS_DEFINITIONS ${UXVCOS_DEFINITIONS} "-DSYSTEM_MAC")
  set(UXVCOS_INCLUDE_DIRS ${UXVCOS_INCLUDE_DIRS} "${UXVCOS_PATH}/src/system/mac/include")
ENDIF (APPLE)

#if(NOT TARGET_NAME)
#  set(TARGET_NAME "${PROJECT_NAME}")
#endif()
#string(TOUPPER "${TARGET_NAME}" TARGET_NAME_UPPER)
#set(UXVCOS_DEFINITIONS ${UXVCOS_DEFINITIONS} "-DTARGET_${TARGET_NAME_UPPER}" "-DTARGET_NAME=${TARGET_NAME}")

find_package(Boost)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif()

function(uxvcos_typekit name)
  set(CMAKE_BUILD_TYPE MinSizeRel)
  if(NOT COMMAND orocos_typekit)
    if(WIN32)
      add_library(${name} STATIC ${ARGN})
    else(WIN32)
      add_library(${name} SHARED ${ARGN})
    endif(WIN32)
  else()
    orocos_typekit(${name} ${ARGN})
  endif()
  set_target_properties(${name} PROPERTIES
    COMPILE_FLAGS "-fvisibility=hidden"
    DEFINE_SYMBOL "UXVCOS_TYPEKIT"
  )
  target_link_libraries(${name} uxvcos)
endfunction(uxvcos_typekit)

set(CMAKE_MODULE_PATH "${UXVCOS_PATH}/cmake" ${CMAKE_MODULE_PATH})
include_directories(${UXVCOS_INCLUDE_DIRS})
add_definitions(${UXVCOS_DEFINITIONS})

message(STATUS "Found UxVCoS in ${UXVCOS_PATH}:")
message(STATUS "  Include directories: ${UXVCOS_INCLUDE_DIRS}")
message(STATUS "  Definitions: ${UXVCOS_DEFINITIONS}")
