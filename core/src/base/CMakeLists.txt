list(APPEND uxvcos_SOURCES
  base/Application.h
  base/Application.cpp
  base/Configuration.h
  base/Configuration.cpp

  base/Child.h
  base/Module.h
  base/Module.cpp
  base/ModuleContainer.h
  base/PortListener.h

  base/DataPool.h
  base/DataPool.cpp

  base/Mapping.h
  base/Mapping.cpp
  base/MappingFixed.h
  base/MappingTypeInfo.h

  base/Transformation.h
)

list(APPEND uxvcos_LIBRARIES
  orocos-ocl-taskbrowser-${OROCOS_TARGET} orocos-ocl-deployment-${OROCOS_TARGET} ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY}
)

source_group("base" REGULAR_EXPRESSION "base/[^/]+$")

orocos_executable(uxvcos_main base/main.cpp base/main.inl)
set_target_properties(uxvcos_main PROPERTIES OUTPUT_NAME uxvcos)
target_link_libraries(uxvcos_main)
