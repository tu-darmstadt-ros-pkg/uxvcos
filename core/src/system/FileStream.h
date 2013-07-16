#ifndef SYSTEM_FILESTREAM_H
#define SYSTEM_FILESTREAM_H

#ifdef SYSTEM_UNIX
  #include "unix/FileStream.h"
  namespace System {
    using Unix::InFileStream;
    using Unix::OutFileStream;
  };

#elif SYSTEM_WIN32
  #include "win32/FileStream.h"
  namespace System {
    using Win32::InFileStream;
    using Win32::OutFileStream;
  };
#endif

#endif
