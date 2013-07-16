#include <rtt/os/startstop.h>
#include <base/Application.h>
#include <options/options.h>

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
