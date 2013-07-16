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
