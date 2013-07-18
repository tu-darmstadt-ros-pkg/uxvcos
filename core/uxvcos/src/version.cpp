//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

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
