OPTION(BUILD_FOR_GEODE "Build code which is optimized for the Geode processor" OFF)
OPTION(WITH_OPENMP "Build with OpenMp support. Just supported for GCC so far." OFF)
OPTION(ENABLE_SSE2 "Enable SSE2 optimized builds. Deactivated for Geode." ON)

IF (WIN32)
  ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE -D_SCL_SECURE_NO_WARNINGS)
  ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
  ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN)
  #ADD_DEFINITIONS(-DNOMINMAX)
ENDIF ()

IF (CMAKE_COMPILER_IS_GNUCXX)
  option(GENERATE_PROFILE "Generate a profile which can be used for profile guided optimization" OFF)
  option(USE_PROFILE "Use generated profile for profile guided optimization" OFF)
  OPTION(BUILD_FOR_GEODE "Build code which is optimized for the Geode processor" OFF)
  include(MacroEnsureVersion)
  include(CheckGCCVersion)
  include(CheckCXXCompilerFlag)
  MESSAGE(STATUS "Found GCC ${_gcc_version}")
  macro_ensure_version("4.2.0" "${_gcc_version}" GCC_IS_AT_LEAST_4_2)
  macro_ensure_version("4.3.0" "${_gcc_version}" GCC_IS_AT_LEAST_4_3)
  check_cxx_compiler_flag(-fvisibility=hidden _HAVE_GCC_VISIBILITY)
  check_cxx_compiler_flag(-fdiagnostics-show-option _HAVE_GCC_DIAGNOSTICS_SHOW_OPTION)
  check_cxx_compiler_flag(-Werror=return-type _HAVE_GCC_WERROR_RETURN_TYPE)
  check_cxx_compiler_flag(-floop-interchange _HAVE_GCC_LOOP-INTERCHANGE)
  check_cxx_compiler_flag(-floop-strip-mine _HAVE_GCC_LOOP-STRIP-MINE)
  check_cxx_compiler_flag(-floop-block _HAVE_GCC_LOOP-BLOCK)

  # -Wno-long-long: Qt code doesn't compile without
  # -Wno-unknown-pragmas:  disable warnings about unknown pragmas used for Msvc
  # -Wno-variadic-macros: we use variadic macros for logging, disable the warning
  # -Wno-reorder: otherwise we get warnings if a contructor initializes members in another order than defined in the header
  # -Wno-unused-parameter: otherwise we don't see the important warnings because of all the unused parameters
  # -Wconversion
  # -Wnon-virtual-dtor: Warn if a class with virtual methods does not have a virtual destructor
  # -Winline: warn if inlining of a inlinable method failed (disabled)
  # -fvisibility=hidden: don't export any symbols. see http://gcc.gnu.org/wiki/Visibility
  # -Werror=return-type: error out if a method ha a non-void return type, but returns nothing
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -fmessage-length=0")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-long-long -Wno-unknown-pragmas -Wno-variadic-macros -Wno-reorder -Wno-unused-parameter")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion")
  #IF (_HAVE_GCC_WERROR_RETURN_TYPE)
  #  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type")
  #ENDIF (_HAVE_GCC_WERROR_RETURN_TYPE)
  #IF (_HAVE_GCC_LOOP-INTERCHANGE)
  #  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -floop-interchange")
  #ENDIF ()
  #IF (_HAVE_GCC_LOOP-STRIP-MINE)
  #  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -floop-strip-mine")
  #ENDIF ()
  #IF (_HAVE_GCC_LOOP-BLOCK)
  #  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -floop-block")
  #ENDIF ()
  #IF (_HAVE_GCC_DIAGNOSTICS_SHOW_OPTION)
  #  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-show-option")
  #ENDIF ()
  #IF (_HAVE_GCC_VISIBILITY AND NOT BUILD_SHARED_LIBRARIES)
  #  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
  #ENDIF ()
  #IF(WITH_OPENMP)
  #  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
  #ENDIF()

  if(ENABLE_SSE2 AND NOT BUILD_FOR_GEODE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
    message(STATUS "Enabling SSE2.")
  endif()

  if(GENERATE_PROFILE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-generate=${CMAKE_BINARY_DIR}/profiling-data")
    SET(CMAKE_SHARED_LINKER_FLAGS "-fprofile-generate=${CMAKE_BINARY_DIR}/profiling-data ${CMAKE_SHARED_LINKER_FLAGS}")
    SET(CMAKE_MODULE_LINKER_FLAGS "-fprofile-generate=${CMAKE_BINARY_DIR}/profiling-data ${CMAKE_MODULE_LINKER_FLAGS}")
    SET(CMAKE_EXE_LINKER_FLAGS    "-fprofile-generate=${CMAKE_BINARY_DIR}/profiling-data ${CMAKE_EXE_LINKER_FLAGS}")
    message(STATUS "Collecting information for PGO enabled.")
  endif()

  if(USE_PROFILE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-use=${CMAKE_BINARY_DIR}/profiling-data -fprofile-correction")
    message(STATUS "Building with collected profiling information.")
  endif()

  # Release build
  SET(OPTIMIZATION_LEVEL 2 CACHE STRING "gcc optimization level")
  SET(CMAKE_CXX_FLAGS_RELEASE "-O${OPTIMIZATION_LEVEL} -DNDEBUG -DQT_NO_DEBUG")
  # Release build with debug symbols
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -O${OPTIMIZATION_LEVEL} -DNDEBUG -DQT_NO_DEBUG")
  # Debug Build
  SET(CMAKE_CXX_FLAGS_DEBUG "-g")

  IF (BUILD_FOR_GEODE)
    # I trust http://wiki.laptop.org/go/Geode more than http://gentoo-wiki.com/Safe_Cflags
    # http://wiki.laptop.org/go/User:Bluefoxicy/gcc_optimizations recommends "-O2 -fno-tree-pre". we should do some benchmarks
    SET(CMAKE_CXX_FLAGS_RELEASE "-O${OPTIMIZATION_LEVEL} -DNDEBUG")
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -O${OPTIMIZATION_LEVEL} -DNDEBUG")
    IF (NOT GCC_IS_AT_LEAST_4_3)
      # we enable 3dnow support, mmx is already enabled with -march=pentiumpro
      SET(CMAKE_CXX_FLAGS "-march=pentiumpro -m3dnow ${CMAKE_CXX_FLAGS}")
    ELSE (NOT GCC_IS_AT_LEAST_4_3)
      # -march=geode: Code is optimized for the geode processor (can be used with GCC 4.3)
      SET(CMAKE_CXX_FLAGS "-m32 -march=geode ${CMAKE_CXX_FLAGS}")
    ENDIF (NOT GCC_IS_AT_LEAST_4_3)

  ELSE (BUILD_FOR_GEODE)
    IF (GCC_IS_AT_LEAST_4_2)
      IF(APPLE)
        SET(CMAKE_CXX_FLAGS "-march=i686 ${CMAKE_CXX_FLAGS}")
      ENDIF(APPLE)
    ENDIF (GCC_IS_AT_LEAST_4_2)
  ENDIF (BUILD_FOR_GEODE)

  if (UNIX)
#    SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined -Wl,--as-needed ${CMAKE_SHARED_LINKER_FLAGS}")
#    SET(CMAKE_MODULE_LINKER_FLAGS "-Wl,--no-undefined -Wl,--as-needed ${CMAKE_MODULE_LINKER_FLAGS}")
#    SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--no-undefined -Wl,--as-needed ${CMAKE_EXE_LINKER_FLAGS}")
  endif (UNIX)

  # tune garbage collection to reduce memory consumption during compilation
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-inline --param ggc-min-expand=0 --param ggc-min-heapsize=131072")

  # use the same options for C
  SET(CMAKE_C_FLAGS ${CMAKE_CXX_FLAGS})
  SET(CMAKE_C_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
  SET(CMAKE_C_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG})
  SET(CMAKE_C_FLAGS_RELWITHDEBINFO ${CMAKE_CXX_FLAGS_RELWITHDEBINFO})
ENDIF (CMAKE_COMPILER_IS_GNUCXX)

IF(MSVC)
  # disable warnings
  # when using /Wall and/or /W4 disabling these is a must. otherwise visual will complain about its own standard headers, qt headers, simply everything...
  foreach(warning 4820 4548 4284 4668 4619 4365 4191 4266 4264 4263 4710 4061 4571 4100 4686 4242 4625 4127 4626 4512 4826 4711 4640 4510 4610 4302 4251 4355)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd${warning}")
  endforeach(warning)

  # try to behave a bit like gcc. does not completely work, but it is better than nothing.
  foreach(warning 4018 4062 4146 4189 4244 4245 4265 4267 4389 4701 4706 4946 4996) # 4702 4505
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w3${warning}")
  endforeach(warning)

  # treat warnings as error
  foreach(warning 4715)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /we${warning}")
  endforeach(warning)


  # /Wp64: warn if there are 64bit compatibility problem. visual studio 2008 complains that this option is deprecated,
  #    but just because one should use their 64bit compiler. as we still use the 32bit one it is completely fine for us.
  # Disabled for the moment just for the reason, that there are more important warning. Enable this one day!
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wp64")

  IF(ENABLE_SSE2)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2")
    MESSAGE(STATUS "Enabling SSE2.")
  ENDIF()

  # Visual Studio complains:
  # fatal error C1128: Die Anzahl von Abschnitten hat das Formatierungslimit der Objektdatei überschritten: Kompilieren mit /bigobj.
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zm400")
ENDIF(MSVC)

# set default build type
IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF (NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE_UPPER MATCHES "RELWITHDEBINFO")
  SET (ALL_CXX_FLAGS ${CMAKE_CXX_FLAGS_RELWITHDEBINFO})
endif(CMAKE_BUILD_TYPE_UPPER MATCHES "RELWITHDEBINFO")
if(CMAKE_BUILD_TYPE_UPPER MATCHES "DEBUG")
  SET (ALL_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})
endif(CMAKE_BUILD_TYPE_UPPER MATCHES "DEBUG")
if(CMAKE_BUILD_TYPE_UPPER MATCHES "RELEASE")
  SET (ALL_CXX_FLAGS ${CMAKE_CXX_FLAGS_RELEASE})
endif(CMAKE_BUILD_TYPE_UPPER MATCHES "RELEASE")

# Boost options
if (WIN32)
  set(Boost_USE_STATIC_LIBS ON)
else (WIN32)
  set(Boost_USE_STATIC_LIBS OFF)
endif (WIN32)
set(Boost_USE_MULTITHREADED ON)

# Configure output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
if(MSVC)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG           ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL      ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif(MSVC)
message(STATUS "Setting runtime output directory to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#unset(CMAKE_LIBRARY_OUTPUT_DIRECTORY)
if(MSVC)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG           ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE         ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL      ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
endif(MSVC)
#message(STATUS "Setting library output directory to ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")

#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
unset(CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
if(MSVC)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG           ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE         ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO  ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL      ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
endif(MSVC)
#message(STATUS "Setting archive output directory to ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")

MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Compiler Flags: ${ALL_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
