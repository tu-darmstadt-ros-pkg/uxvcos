SET(fmp_SOURCES
  fmp.cpp
)

add_subdirectory(interface)
add_subdirectory(sensors)

#add_definitions(-DTARGET_FMP -DTARGET_NAME='"fmp"')
#add_definitions(-DAPPLICATION=FMP)

#orocos_executable(fmp ${UXVCOS_PATH}/core/src/base/main.cpp ${UXVCOS_PATH}/core/src/base/main.inl)
#orocos_executable(fmp ${fmp_SOURCES})
#target_link_libraries(fmp ${uxvcos_LIBRARIES} orocos-ocl-taskbrowser-${OROCOS_TARGET} orocos-ocl-deployment-${OROCOS_TARGET} ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})

add_subdirectory(replay)
