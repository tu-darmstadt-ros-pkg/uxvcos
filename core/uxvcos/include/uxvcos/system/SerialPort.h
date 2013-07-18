#ifndef SYSTEM_SERIALPORT_H
#define SYSTEM_SERIALPORT_H

#ifdef SYSTEM_UNIX
  #include "unix/SerialPort.h"
  namespace System {
    using Unix::SerialPort;
  };
#endif

#ifdef SYSTEM_WIN32
  #include "win32/SerialPort.h"
  namespace System {
    using Win32::SerialPort;
  };
#endif

#endif
