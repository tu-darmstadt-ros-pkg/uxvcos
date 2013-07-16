#include <stdlib.h>
#include <string.h>

#include "version.h"

#ifndef SVN_REVISION
  #define SVN_REVISION 0
#endif

#ifndef STRINGIFY
  #define STRINGIFY(x) #x
  #define XSTRINGIFY(x) STRINGIFY(x)
#endif

const char *SVN_REVISION_STRING = XSTRINGIFY(SVN_REVISION);
const unsigned int SVN_REVISION_UINT = static_cast<unsigned int>(atol(XSTRINGIFY(SVN_REVISION)));

const char *getProjectName() {
  return XSTRINGIFY(PROJECT_NAME);
}

unsigned int getRevision() {
  return SVN_REVISION_UINT;
}

char VERSION_STRING[32] = "";
const char *getVersionString() {
  if (*VERSION_STRING == 0) {
    strcpy(VERSION_STRING, XSTRINGIFY(VERSION));
    #ifdef SVN_REVISION
      strcat(VERSION_STRING, ".r");
      strcat(VERSION_STRING, SVN_REVISION_STRING);
    #endif
  }
  
  return VERSION_STRING;
}
