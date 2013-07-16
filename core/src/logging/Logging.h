#ifndef UXVCOS_LOGGING_H
#define UXVCOS_LOGGING_H

#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>

#include <boost/shared_ptr.hpp>
#include <string>

#include <stream/Stream.h>
#include <stream/ubx/Stream.h>
#include <base/DataPool.h>
#include <base/PortListener.h>

namespace uxvcos {
namespace Logging {

class RTT_EXPORT Logging : public RTT::TaskContext {
  public:
    Logging(const std::string& name = "Logging", DataPool *dataPool = 0);
    Logging(boost::shared_ptr<OutStream> stream, const std::string& name = "Logging", DataPool *dataPool = 0);
    virtual ~Logging();

    static int setup();

    void addPorts(const RTT::DataFlowInterface::Ports& ports) {
      portListener.addPorts(ports);
    }

  protected:
    bool configureHook();
    bool startHook();
    void updateHook();
    bool breakUpdateHook();
    void stopHook();
    void cleanupHook();

    bool log(RTT::base::DataSourceBase::shared_ptr dsb);
    bool log(const Data::Streamable& message);

  protected:
    /**
     * File name to write reports to.
     */
    boost::shared_ptr<OutStream> stream;
    UBX::Encoder* ubx;
    RTT::Property<std::string> directory;
    RTT::Property<std::string> file;
    RTT::Property<std::string> extension;
    RTT::Property<bool> appendTimestamp;
    RTT::Property<std::string> sync;

    std::string fullname;

  private:
    bool construct();

    PortListener portListener;
};

} // namespace Logging
} // namespace uxvcos

#endif // UXVCOS_LOGGING_H
