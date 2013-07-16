set(server_SOURCES
  Server.cpp
  Server.h
)

orocos_component(server ${server_SOURCES})
set_target_properties(server PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/orocos${OROCOS_SUFFIX})
target_link_libraries(server uxvcos)

orocos_generate_package(uxvcos-server-${OROCOS_TARGET} DEPENDS uxvcos_rtt-${OROCOS_TARGET})

