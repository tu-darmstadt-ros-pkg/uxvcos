set(SOURCES
  Logging.h
  Logging.cpp
  Replay.h
  Replay.cpp
)

add_definitions(-DRTT_STATIC)
orocos_library(logging ${SOURCES} component.cpp)
#set_target_properties(logging PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/orocos${OROCOS_SUFFIX})
target_link_libraries(logging uxvcos)

orocos_generate_package(uxvcos-logging-${OROCOS_TARGET} DEPENDS uxvcos_rtt-${OROCOS_TARGET})

