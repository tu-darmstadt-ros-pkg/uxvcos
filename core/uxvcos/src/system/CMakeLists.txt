if(UNIX)
  set(uxvcos_SOURCES ${uxvcos_SOURCES}
    system/unix/Error.cpp
    system/unix/SerialPort.cpp
    system/unix/Socket.cpp
    system/unix/FileStream.cpp
    system/unix/Pipe.cpp
    system/unix/backtrace.cpp
  )
endif(UNIX)

if(OROCOS_TARGET STREQUAL "xenomai")
  set(XENOMAI 1)
endif()

if(XENOMAI)
  add_definitions(-DUSE_XENOMAI)

  set(USE_RTNET ON CACHE BOOL "Use RTnet for real-time networking")

  if(USE_RTNET)
    include_directories(/usr/rtnet/include /usr/rtnet/include)
    list(APPEND uxvcos_LIBRARIES rtdm)
    add_definitions(-DUSE_RTNET)
  endif(USE_RTNET)

  set(uxvcos_SOURCES ${uxvcos_SOURCES}
    system/xenomai/Pipe.cpp
  )
endif(XENOMAI)

if(WIN32)
  set(uxvcos_SOURCES ${uxvcos_SOURCES}
    system/win32/Error.cpp
    system/win32/SerialPort.cpp
    system/win32/Socket.cpp
    system/win32/FileStream.cpp
  )
  
  set(uxvcos_LIBRARIES ${uxvcos_LIBRARIES} Ws2_32)
endif(WIN32)

set(uxvcos_SOURCES ${uxvcos_SOURCES}
  system/SerialPort.cpp
  system/SystemCall.cpp
  ${SYSTEM_SOURCES}
)

source_group("system" REGULAR_EXPRESSION "system/[^/]+$")
source_group("system\\xenomai" REGULAR_EXPRESSION "system/xenomai/[^/]+$")
source_group("system\\gnulinux" REGULAR_EXPRESSION "system/gnulinux/[^/]+$")
source_group("system\\win32" REGULAR_EXPRESSION "system/win32/[^/]+$")

