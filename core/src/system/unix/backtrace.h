#ifndef SYSTEM_BACKTRACE_H

#include <uxvcos.h>

namespace System {
  UXVCOS_API void backtrace(const char *text = 0);
};

#endif // SYSTEM_BACKTRACE_H
