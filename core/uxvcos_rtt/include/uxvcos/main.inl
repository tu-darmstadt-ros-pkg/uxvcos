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

#include <rtt/os/startstop.h>
#include <uxvcos/options/options.h>

#include "Application.h"

#ifndef APPLICATION
  #error APPLICATION needs to be defined when including this file!
#endif

using namespace uxvcos;

int main(int argc, char* argv[])
{
  int exit_code = 0;
  Application *application = new APPLICATION();
  application->setProgramOptions(argc, argv);

  __os_init(argc, argv);

  // Set log level more verbose than default,
  // such that we can see output :
  if ( RTT::log().getLogLevel() == RTT::Logger::Debug ) {
    RTT::log().setLogLevel( RTT::Logger::RealTime );
    RTT::log(RTT::Info) << argv[0] << " manually raises LogLevel to 'RealTime' (7). See also file 'orocos.log'." << RTT::endlog();
  }
  RTT::log().allowRealTime();

  RTT::Logger::In in("main");
  if (!application->setup()) { exit_code = -1; goto exit_1; }
  if (!application->configure()) { exit_code = -2; goto exit_2; }
  if (!application->start()) { exit_code = -3; goto exit_3; }
  application->wait();
  application->stop();
exit_3:
  application->cleanup();

exit_2:
exit_1:
  delete application;
  RTT::log(RTT::Debug) << "Application finished." << RTT::endlog();

  __os_exit();
  return Application::exit(exit_code);
}
