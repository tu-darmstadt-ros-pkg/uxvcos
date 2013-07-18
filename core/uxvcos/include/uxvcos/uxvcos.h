#ifndef UXVCOS_MAIN_H
#define UXVCOS_MAIN_H

#ifdef OROCOS_TARGET
#include <rtt/typekit/Types.hpp>
#endif // OROCOS_TARGET

//
// See: <http://gcc.gnu.org/wiki/Visibility>
//
#if defined(__GNUG__) && (defined(__unix__) || defined(__APPLE__))

# if defined(uxvcos_EXPORTS)
   // Use UXVCOS_API for normal function exporting
#  define UXVCOS_API    __attribute__((visibility("default")))

   // Use UXVCOS_EXPORT for static template class member variables
   // They must always be 'globally' visible.
#  define UXVCOS_EXPORT __attribute__((visibility("default")))

   // Use UXVCOS_HIDE to explicitly hide a symbol
#  define UXVCOS_HIDE   __attribute__((visibility("hidden")))

# else
#  define UXVCOS_API
#  define UXVCOS_EXPORT __attribute__((visibility("default")))
#  define UXVCOS_HIDE   __attribute__((visibility("hidden")))
# endif
#else
   // Win32 and NOT GNU
# if defined( WIN32 ) && !defined ( __MINGW32__ )
#  if defined(uxvcos_EXPORTS)
#   define UXVCOS_API    __declspec(dllexport)
#   define UXVCOS_EXPORT __declspec(dllexport)
#   define UXVCOS_HIDE   
#  else
#   define UXVCOS_API	 __declspec(dllimport)
#   define UXVCOS_EXPORT __declspec(dllexport)
#   define UXVCOS_HIDE 
#  endif
# else
#  define UXVCOS_API
#  define UXVCOS_EXPORT
#  define UXVCOS_HIDE
# endif
#endif

// forward declarations
class UXVCOS_API BaseStream;
class UXVCOS_API InStream;
class UXVCOS_API OutStream;
class UXVCOS_API Stream;

namespace Data {
  class UXVCOS_API Streamable;
  class UXVCOS_API TypeInfo;
  class UXVCOS_API Timestamp;
  class UXVCOS_API Key;
}

#endif // UXVCOS_MAIN_H
