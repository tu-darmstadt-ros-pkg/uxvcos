#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>

#include "system/backtrace.h"
#define SIZE 100

namespace System {

void backtrace(const char *text) {
  int j, nptrs;
  void *buffer[100];
  char **strings;

  if (text) printf("%s\n", text);

  nptrs = ::backtrace(buffer, SIZE);
  printf("Backtrace:");

   /* The call backtrace_symbols_fd(buffer, nptrs, STDOUT_FILENO)
       would produce similar output to the following: */

  strings = backtrace_symbols(buffer, nptrs);
  if (strings == NULL) {
    perror("backtrace_symbols");
    exit(EXIT_FAILURE);
  }

  for (j = 0; j < nptrs; j++)
    printf("%s\n", strings[j]);

  free(strings);
}

} // namespace System
