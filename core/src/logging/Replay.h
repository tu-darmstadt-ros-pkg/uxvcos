#ifndef UXVCOS_REPLAY_H
#define UXVCOS_REPLAY_H

#include <rtt/TaskContext.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>

#include <boost/shared_ptr.hpp>
#include <string>
#include <map>

#include <stream/Stream.h>
#include <stream/ubx/Stream.h>
#include <base/DataPool.h>
#include <base/Echelon.h>
#include <interface/BaseInterface.h>

#include <uxvcos/Time.h>

namespace uxvcos {
namespace Logging {

class RTT_EXPORT Replay : public RTT::TaskContext {
  public:
    typedef EchelonType Mode;
    typedef std::multimap<Mode, RTT::TaskContext *> Tasks;
    typedef std::pair<Mode, RTT::TaskContext *> TaskPair;

    Replay(const std::string& name = "Replay");
    Replay(boost::shared_ptr<InStream> stream, const std::string& name = "Replay");
    virtual ~Replay();

    static int setup();

    Mode mode() const {
      return _mode;
    }

    void setDataPool(DataPool *dataPool) {
      this->dataPool = dataPool;
    }

    virtual uxvcos::Time getTimestamp() const { return timestamp; }

  protected:
    bool configureHook();
    bool startHook();
    void updateHook();
    bool breakUpdateHook();
    void stopHook();
    void cleanupHook();

  protected:
    /**
     * File name to write reports to.
     */
    boost::shared_ptr<InStream> stream;
    UBX::Decoder* decoder;
    RTT::Property<std::string> directory;
    RTT::Property<std::string> file;

    RTT::Attribute<double> rate;
    RTT::Attribute<double> startTime;
    RTT::Attribute<double> stopTime;
    uxvcos::Time realTimestamp;
    
  private:
    bool construct();
    RTT::TaskContext* target;
    DataPool *dataPool;
    Mode _mode;
    Tasks tasks;
    
    bool breakUpdate;

    uxvcos::Time timestamp;
    uxvcos::Time progress;
  };

} // namespace Logging
} // namespace uxvcos

#endif // UXVCOS_REPLAY_H
