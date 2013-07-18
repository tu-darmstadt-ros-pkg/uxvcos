//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#ifndef DATA_STREAMABLECONTAINER_H
#define DATA_STREAMABLECONTAINER_H

#include "Streamable.h"
#include "TypeInfo.h"

#include <map>

namespace Data {

class StreamableContainer {
public:
  StreamableContainer() {}
  virtual ~StreamableContainer() { clear(); }

  Data::Streamable *operator[](const Data::TypeInfo *typeInfo) {
    Map::const_iterator it;
    if (map.count(typeInfo) == 0) {
      it = map.insert(Map::value_type(typeInfo, typeInfo->create()));
    } else {
      it = map.find(typeInfo);
    }

    if (it == map.end()) return 0;
    return it->second;
  }

  void free(Data::Streamable *data) {
    for(Map::iterator it = map.begin(); it != map.end(); ++it) {
      if (it->second == data) {
        delete it->second;
        map.erase(it);
      }
    }
  }

  void clear() {
    for(Map::iterator it = map.begin(); it != map.end(); ++it) delete it->second;
    map.clear();
  }

protected:
  typedef std::multimap<const Data::TypeInfo *,Data::Streamable *> Map;
  Map map;
};

} // namespace Data

#endif // DATA_STREAMABLECONTAINER_H
