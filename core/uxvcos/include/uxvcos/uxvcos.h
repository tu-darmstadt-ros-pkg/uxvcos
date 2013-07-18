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
