#ifndef VERSION_H
#define VERSION_H

#include <string>
#include <uxvcos.h>

#define PROJECT_NAME   UxVCoS
#define VERSION_MAJOR  0
#define VERSION_MINOR  99
#define VERSION        VERSION_MAJOR.VERSION_MINOR

extern const unsigned int SVN_REVISION_UINT;
extern const char *SVN_REVISION_STRING;

UXVCOS_API const char *getProjectName();
UXVCOS_API unsigned int getRevision();
UXVCOS_API const char *getVersionString();

#endif // VERSION_H
