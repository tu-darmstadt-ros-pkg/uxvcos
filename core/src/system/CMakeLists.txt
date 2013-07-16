if(UNIX)
  set(uxvcos_SOURCES ${uxvcos_SOURCES}
    system/unix/SystemCall.h
    system/unix/Error.h
    system/unix/Error.cpp
    system/unix/SerialPort.h
    system/unix/SerialPort.cpp
    system/unix/Socket.h
    system/unix/Socket.cpp
    system/unix/FileStream.h
    system/unix/FileStream.cpp
    system/unix/Pipe.h
    system/unix/Pipe.cpp
    system/unix/backtrace.h
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
    system/xenomai/Xenomai.h
    system/xenomai/SystemCall.h
    system/xenomai/Pipe.h
    system/xenomai/Pipe.cpp
  )
endif(XENOMAI)

if(WIN32)
  set(uxvcos_SOURCES ${uxvcos_SOURCES}
    system/win32/Error.h
    system/win32/Error.cpp
    system/win32/SerialPort.h
    system/win32/SerialPort.cpp
    system/win32/Socket.h
    system/win32/Socket.cpp
    system/win32/FileStream.h
    system/win32/FileStream.cpp
  )
  
  set(uxvcos_LIBRARIES ${uxvcos_LIBRARIES} Ws2_32)
endif(WIN32)

set(uxvcos_SOURCES ${uxvcos_SOURCES}
  system/Error.h
  system/Exception.h
  system/BaseSerialPort.h
  system/SerialPort.h
  system/SerialPort.cpp
  system/BaseSocket.h
  system/Socket.h
  system/NetworkAddress.h
  system/EthernetAddress.h
  system/BaseFileStream.h
  system/FileStream.h
  system/Pipe.h
  system/Priorities.h
  system/Priorities.cpp
  system/SystemCall.h
  system/SystemCall.cpp
  system/systemcalls.h
  ${SYSTEM_SOURCES}
)

source_group("system" REGULAR_EXPRESSION "system/[^/]+$")
source_group("system\\xenomai" REGULAR_EXPRESSION "system/xenomai/[^/]+$")
source_group("system\\gnulinux" REGULAR_EXPRESSION "system/gnulinux/[^/]+$")
source_group("system\\win32" REGULAR_EXPRESSION "system/win32/[^/]+$")

