set(uxvcos_rtt_SOURCES
  Application.cpp
  Configuration.cpp
  Module.cpp
  DataPool.cpp
  Mapping.cpp
  data/TypeInfo.cpp
  data/uxvcos-types.cpp
  stream/Stream.cpp
  stream/PropertyStream.cpp
)

orocos_library(uxvcos-rtt ${uxvcos_rtt_SOURCES})
target_link_libraries(uxvcos-rtt uxvcos orocos-ocl-taskbrowser-${OROCOS_TARGET} orocos-ocl-deployment-${OROCOS_TARGET} ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})

source_group("rtt" REGULAR_EXPRESSION "[^/]+$")

orocos_executable(uxvcos_main main.cpp)
set_target_properties(uxvcos_main PROPERTIES OUTPUT_NAME uxvcos)
target_link_libraries(uxvcos_main uxvcos-rtt)

get_property(uxvcos_main_LOCATION TARGET uxvcos_main PROPERTY LOCATION)
add_custom_command(
  TARGET uxvcos_main
  COMMAND sudo setcap cap_net_raw,cap_sys_rawio+ep ${uxvcos_main_LOCATION}
)

add_subdirectory(interface)
add_subdirectory(simulink)
add_subdirectory(watchdog)
