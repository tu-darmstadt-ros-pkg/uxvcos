#include "Mapping.h"

#include <boost/lexical_cast.hpp>
#include <sstream>

namespace uxvcos {

std::string MappingBase::toString() const {
  if (this->size() == 0) return std::string();
  std::ostringstream ss;
  ss << (*this)[0];
  for(size_t i = 1; i < this->size(); ++i)
    ss << std::string::value_type(',') << (*this)[i];
  return ss.str();
}

bool MappingBase::fromString(const std::string& s) {
  std::istringstream ss(s);
  size_t i = 0;
  std::string text;

  while(std::getline(ss, text, std::string::value_type(','))) {
    size_t temp = boost::lexical_cast<size_t>(text);
    (*this)[i++] = temp;
  }
  for(; i < this->size(); ++i) (*this)[i] = 0;
  this->set_size(i);
  return true;
}

std::ostream& operator<<(std::ostream& os, const MappingBase& map) {
  return os << '[' << map.toString() << ']';
}

std::istream& operator>>(std::istream& is, MappingBase& map) {
  std::string s;
  if (!(is >> s) || !map.fromString(s)) is.setstate(std::ios_base::failbit);
  return is;
}

} // namespace uxvcos
