#ifndef SYSTEM_EXCEPTION_H
#define SYSTEM_EXCEPTION_H

#include <rtt/Logger.hpp>

#include <exception>
#include <cstring>
#include <uxvcos.h>

namespace System {

class UXVCOS_API Exception : public std::exception {
public:
    Exception(const int code, std::string command = "") throw()
      : code(code)
      , error(std::strerror(-code))
    {
        RTT::log( RTT::Error ) << "Exception by " << command << ": " << error << "(" << code << ")" << RTT::endlog();
    }
    virtual ~Exception() throw() { }

    Exception(const char *description) : code(0), error(description) {
        RTT::log( RTT::Error ) << error << RTT::endlog();
    }

    const char *what() const throw() {
        return error.c_str();
    }

    int code;
    std::string error;
};

} // namespace System

#endif // SYSTEM_EXCEPTION_H
