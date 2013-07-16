set(MAIN_SOURCES
  ../../quadro/src/quadro.cpp
)

add_subdirectory(controller)

add_definitions(-DTARGET_AIRPLANE -DTARGET_NAME='"airplane"')
orocos_executable(${TARGET_NAME} ${MAIN_SOURCES})
target_link_libraries(${TARGET_NAME} ${UXVCOS_LIBRARIES} ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} ${OROCOS-RTT_RTT-TYPEKIT_LIBRARY})

rosbuild_link_boost(${TARGET_NAME} program_options)

if(OROCOS_TARGET STREQUAL "xenomai")
  # PluginLoader fails because libpthread_rt.so cannot dlopened when librtt-transport-mqueue-xenomai.so is loaded. Workaroud:
  target_link_libraries(${TARGET_NAME} pthread_rt)
endif()
