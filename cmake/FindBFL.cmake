# Locate BFL install directory

# This module defines
# BLF_INSTALL where to find include, lib, bin, etc.
# BLF_FOUND, is set to true

set(BFL_FOUND false)

if(ROSBUILD_init_called)
  rosbuild_find_ros_package(bfl)
  if(${bfl_PACKAGE_PATH})
    set(BFL_FOUND true)
  endif(${bfl_PACKAGE_PATH})
  return()
endif(ROSBUILD_init_called)

include(FindPkgConfig)

IF ( PKG_CONFIG_FOUND )

    MESSAGE( STATUS "Detecting BFL" )
    MESSAGE( "Looking for BFL in: ${BFL_INSTALL}")
    SET(ENV{PKG_CONFIG_PATH} "${BFL_INSTALL}/lib/pkgconfig/")
    pkg_check_modules(BFL REQUIRED "orocos-bfl >= 0.4.2")
    #PKGCONFIG( "orocos-bfl >= 0.4.2" BFL_FOUND BFL_INCLUDE_DIRS BFL_DEFINES BFL_LINK_DIRS BFL_LIBS )

    IF( BFL_FOUND )
        #MESSAGE("   Includes in: ${BFL_INCLUDE_DIRS} ${BFL_INCLUDE_DIRS}/bfl")
        #MESSAGE("   Libraries in: ${BFL_LIBRARY_DIRS}")
        #MESSAGE("   Libraries: ${BFL_LIBRARIES}")
        #MESSAGE("   CFLAGS: ${BFL_CFLAGS}")

	INCLUDE_DIRECTORIES( ${BFL_INCLUDE_DIRS} ${BFL_INCLUDE_DIRS}/bfl )
	LINK_DIRECTORIES( ${BFL_LIBRARY_DIRS} )

    ENDIF ( BFL_FOUND )

ELSE  ( PKG_CONFIG_FOUND )

    # Can't find pkg-config -- have to search manually
    MESSAGE( FATAL_ERROR "Can't find BFL without pkgconfig !")

ENDIF ( PKG_CONFIG_FOUND )
