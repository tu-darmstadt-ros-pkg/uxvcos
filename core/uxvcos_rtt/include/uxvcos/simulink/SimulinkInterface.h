#ifndef SIMULINK_SIMULINKINTERFACE_H
#define SIMULINK_SIMULINKINTERFACE_H

#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>
#include <rtt/Operation.hpp>

#include <uxvcos/Time.h>
#include <rosgraph_msgs/typekit/Clock.h>

#include <vector>
#include <map>
#include <boost/tuple/tuple.hpp>

namespace uxvcos {
namespace Simulink {

typedef std::vector<double> DoubleVector;
typedef RTT::OutputPort<DoubleVector> VectorDoubleOutputPort;
typedef RTT::OutputPort<double> SingleDoubleOutputPort;
typedef RTT::InputPort<DoubleVector> VectorDoubleInputPort;
typedef RTT::InputPort<double> SingleDoubleInputPort;

typedef std::vector<int> IntegerVector;
typedef RTT::OutputPort<IntegerVector> VectorIntegerOutputPort;
typedef RTT::OutputPort<int> SingleIntegerOutputPort;
typedef RTT::InputPort<IntegerVector> VectorIntegerInputPort;
typedef RTT::InputPort<int> SingleIntegerInputPort;

class UXVCOS_API SimulinkInterface
{
  public:
    SimulinkInterface(RTT::TaskContext *owner, const std::string& modelName);
    virtual ~SimulinkInterface();

    static int setup();

    template <typename T> RTT::InputPort<T>& addInputPort(const std::string& name, RTT::InputPort<T>& port);
    template <typename T> RTT::OutputPort<T>& addOutputPort(const std::string& name, RTT::OutputPort<T>& port);

    bool hasInputPort(const std::string &name);
    template <typename T> RTT::OutputPort<T> *getInputPort(const std::string &name);
    bool hasOutputPort(const std::string &name);
    template <typename T> RTT::InputPort<T> *getOutputPort(const std::string &name);

    bool connectPorts();
    bool unconnectedPorts();

  protected:
    virtual bool reset();
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    uxvcos::Time getTimestamp() const { return timestamp; }
    void setTimestamp(const uxvcos::Time& timestamp) { this->timestamp = timestamp; }

  protected:
    template <typename T> bool connectToSimulinkPort(RTT::OutputPort<T> &external, const std::string& name);
    template <typename T> bool connectToSimulinkPort(RTT::OutputPort<T> &external);
    template <typename T> bool connectToSimulinkPort(RTT::InputPort<T> &external, const std::string& name);
    template <typename T> bool connectToSimulinkPort(RTT::InputPort<T> &external);

  private:
    RTT::TaskContext *owner;
    RTT::TaskContext *model;

    uxvcos::Time timestamp;

    void *dlhandle;
    time_t model_mtime;
    std::string model_name;
    std::string model_filename;

    bool loadLibrary(const std::string& modelname = "");
    void unloadLibrary();
    
    RTT::TaskContext *loadModel(const std::string& instance = "Model");
    void unloadModel();

    typedef std::map<std::string, RTT::base::OutputPortInterface *> InputPorts;
    InputPorts inputPorts;
    typedef std::map<std::string, RTT::base::InputPortInterface *> OutputPorts;
    OutputPorts outputPorts;

    typedef std::map<RTT::base::InputPortInterface *, RTT::DataFlowInterface::SlotFunction> Connections;
    Connections inputConnections;
    Connections outputConnections;
    template <typename T> friend class SimulinkConnector;

  protected:
    RTT::Property<std::string> modelName;
    RTT::InputPort<rosgraph_msgs::Clock> portClock;

    typedef std::map<std::string, boost::tuple<RTT::base::InputPortInterface *, boost::function<bool(RTT::base::OutputPortInterface *simulink)> > > ExternalInputPorts;
    ExternalInputPorts externalInputPorts;
    typedef std::map<std::string, boost::tuple<RTT::base::OutputPortInterface *, boost::function<bool(RTT::base::InputPortInterface *simulink)> > > ExternalOutputPorts;
    ExternalOutputPorts externalOutputPorts;
};

template <typename T>
struct UXVCOS_API SimulinkConnector {
  static bool connectInput(SimulinkInterface *obj, RTT::InputPort<T> *external, RTT::base::OutputPortInterface *simulinkPort);
  static bool connectOutput(SimulinkInterface *obj, RTT::OutputPort<T> *external, RTT::base::InputPortInterface *simulinkPort);
};

template <typename T, typename SimulinkType>
struct UXVCOS_API SimulinkConverter {
  static bool toSimulink(const T& data, SimulinkType& simulink) { return false; }
  static bool fromSimulink(const SimulinkType& simulink, T& data) { return false; }
};

template <typename T, typename SimulinkType>
class UXVCOS_API SimulinkInput {
public:
  SimulinkInput(RTT::InputPort<T> *input_port, RTT::OutputPort<SimulinkType> *output_port);
  void operator()(RTT::base::PortInterface *);

private:
  T data;
  SimulinkType simulink;
  RTT::InputPort<T> *input_port;
  RTT::OutputPort<SimulinkType> *output_port;
};

template <typename T, typename SimulinkType>
struct UXVCOS_API SimulinkOutput {
  SimulinkOutput(RTT::OutputPort<T> *output_port, RTT::InputPort<SimulinkType> *input_port);
  void operator()(RTT::base::PortInterface *);
private:
  T data;
  SimulinkType simulink;
  RTT::OutputPort<T> *output_port;
  RTT::InputPort<SimulinkType> *input_port;
};

} // namespace Simulink
} // namespace uxvcos

#include "SimulinkInterface.inl"

#endif // SIMULINK_SIMULINKINTERFACE_H
