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

#ifndef UXVCOS_ECHELON_H
#define UXVCOS_ECHELON_H

#include <iostream>

namespace uxvcos {

  namespace Echelon {
    enum Enum { None = 0, Raw, Sensors, Navigation, Controller, Logfile, Default };
  }
  typedef Echelon::Enum EchelonType;

} // namespace uxvcos

static inline std::ostream& operator<<(std::ostream& os, uxvcos::EchelonType echelon) {
  using namespace uxvcos::Echelon;

  switch(echelon) {
    case None:       os << "None"; break;
    case Raw:        os << "Raw"; break;
    case Sensors:    os << "Sensors"; break;
    case Navigation: os << "Navigation"; break;
    case Controller: os << "Controller"; break;
    case Logfile:    os << "Logfile"; break;
    default:         os << "(unknown)"; break;
  }
  return os;
}

#include <rtt/types/TypeStreamSelector.hpp>
namespace RTT
{
    namespace types {

    template<>
        struct TypeStreamSelector<uxvcos::EchelonType,false>
        {
            static std::ostream& write(std::ostream& os, uxvcos::EchelonType e)
            {
                return os << e;
            }
            static std::istream& read(std::istream& is, uxvcos::EchelonType& e)
            {
                return is;
            }
        };
    }
}

#endif // UXVCOS_ECHELON_H
