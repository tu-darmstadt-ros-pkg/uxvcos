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

#ifndef SYSTEM_XENOMAI_H
#define SYSTEM_XENOMAI_H

#include <native/types.h>
#include <native/timer.h>
#include <rtdk.h>

#define XENOMAI_DEFAULT_TIMEOUT_MS 1000
#define XENOMAI_DEFAULT_TIMEOUT System::Xenomai::milliseconds(XENOMAI_DEFAULT_TIMEOUT_MS)

// erzwinge Verwendung von Realtime-Ausgabe
//#define XENO_FORCE_RTDK
#ifdef XENO_FORCE_RTDK
  #define printf(...)    rt_printf(__VA_ARGS__)
  #define fprintf(...)   rt_fprintf(__VA_ARGS__)
  #define vprintf(...)   rt_vprintf(__VA_ARGS__)
  #define vfprintf(...)  rt_vfprintf(__VA_ARGS__)
#endif

namespace System {
namespace Xenomai {

static inline RTIME milliseconds(const long ms) {
  if (ms == -1) return TM_INFINITE;
  if (ms == 0) return TM_NONBLOCK;
  return ((RTIME) ms) * ((RTIME) 1000000);
}

static const RTIME INFINITE = TM_INFINITE;
static const RTIME NONBLOCK = TM_NONBLOCK;

} // namespace Xenomai
} // namespace System

#endif // SYSTEM_XENOMAI_H
