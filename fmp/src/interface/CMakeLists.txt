orocos_use_package(uxvcos-interface-serial)

option(BUILD_INTERFACE_VISTANAV "Build an Interface for VistaNav box (requires BUILD_INTERFACE_SERIAL)" OFF)
if(BUILD_INTERFACE_VISTANAV)
	set(SOURCES
	  VistaNav.h
	  VistaNav.cpp
	)

	orocos_component(interface-vistanav ${SOURCES})
  target_link_libraries(interface-vistanav sensors-fmp)
endif(BUILD_INTERFACE_VISTANAV)

option(BUILD_INTERFACE_FMPBOX "Build an Interface for the FMPBox (requires BUILD_INTERFACE_SERIAL)" ON)
if(BUILD_INTERFACE_FMPBOX)
	set(SOURCES
	  FMPBox.h
	  FMPBox.cpp
	)

	orocos_component(interface-fmpbox ${SOURCES})
  target_link_libraries(interface-fmpbox sensors-fmp)
endif(BUILD_INTERFACE_FMPBOX)
