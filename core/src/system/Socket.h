#ifndef SYSTEM_SOCKET_H
#define SYSTEM_SOCKET_H

#if defined(SYSTEM_UNIX)
  #include "unix/Socket.h"
  namespace System { using Unix::Socket; }
#endif

#if defined(SYSTEM_WIN32)
  #include "win32/Socket.h"
  namespace System { using Win32::Socket; }
#endif

#endif // SYSTEM_SOCKET_H
