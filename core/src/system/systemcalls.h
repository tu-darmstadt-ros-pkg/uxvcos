#ifndef SYSTEM_SYSTEMCALLS_H
#define SYSTEM_SYSTEMCALLS_H

#include <stdio.h>
#include <string.h>
#include <time.h>

#if WIN32
  #define snprintf sprintf_s
#endif

#endif // SYSTEM_SYSTEMCALLS_H
