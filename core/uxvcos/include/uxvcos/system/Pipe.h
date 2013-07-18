#ifndef SYSTEM_PIPE_H
#define SYSTEM_PIPE_H

#ifdef HAVE_SYSTEM_PIPE
#undef HAVE_SYSTEM_PIPE
#endif // HAVE_SYSTEM_PIPE

#ifdef USE_XENOMAI
  #include "xenomai/Pipe.h"
  namespace System {
    using Xenomai::Pipe;
  };
  #define HAVE_SYSTEM_PIPE
#elif SYSTEM_UNIX
  #include "unix/Pipe.h"
  namespace System {
    using Unix::Pipe;
  };
  #define HAVE_SYSTEM_PIPE
#elif SYSTEM_WIN32
#endif

#endif // SYSTEM_PIPE_H
