#ifndef SIMULINK_SIMULINKINTERFACE_INL
#define SIMULINK_SIMULINKINTERFACE_INL

#include "SimulinkInterface.h"
#include <rtt/Handle.hpp>

#include <boost/algorithm/string/replace.hpp>

namespace uxvcos {
namespace Simulink {

template <typename T>
RTT::InputPort<T>& SimulinkInterface::addInputPort(const std::string& name, RTT::InputPort<T>& port)
{
  port.setName(name);
  externalInputPorts[name] = boost::tuple<RTT::base::InputPortInterface *, boost::function<bool(RTT::base::OutputPortInterface *simulink)> >(&port, boost::bind(&SimulinkConnector<T>::connectInput, this, &port, _1));
  return port;
}

template <typename T>
RTT::OutputPort<T>& SimulinkInterface::addOutputPort(const std::string& name, RTT::OutputPort<T>& port)
{
  port.setName(name);
  externalOutputPorts[name] = boost::tuple<RTT::base::OutputPortInterface *, boost::function<bool(RTT::base::InputPortInterface *simulink)> >(&port, boost::bind(&SimulinkConnector<T>::connectOutput, this, &port, _1));
  return port;
}

template <typename T>
bool SimulinkInterface::connectToSimulinkPort(RTT::InputPort<T> &external, const std::string& name) {
  if (!hasInputPort(name)) {
    RTT::log(RTT::Debug) << "Did not find a corresponding input port for " << external.getName() << " in the model" << RTT::endlog();
    return false;
  }

  // if (!owner->ports()->getPort(external.getName())) owner->addPort(external);
  RTT::base::OutputPortInterface *simulink = inputPorts[name];
  return SimulinkConnector<T>::connectInput(this, &external, simulink);
}

template <typename T>
bool SimulinkInterface::connectToSimulinkPort(RTT::InputPort<T> &external) {
  std::string simulink_name(external.getName());
  boost::algorithm::replace_all(simulink_name, "/", "//");
  return connectToSimulinkPort(external, simulink_name);
}

template <typename T>
bool SimulinkInterface::connectToSimulinkPort(RTT::OutputPort<T> &external, const std::string& name) {
  if (!hasOutputPort(name)) {
    RTT::log(RTT::Debug) << "Did not find a corresponding output port for " << external.getName() << " in the model" << RTT::endlog();
    return false;
  }

  // if (!owner->ports()->getPort(external.getName())) owner->addPort(external);
  RTT::base::InputPortInterface *simulink = outputPorts[name];
  return SimulinkConnector<T>::connectOutput(this, &external, simulink);
}

template <typename T>
bool SimulinkInterface::connectToSimulinkPort(RTT::OutputPort<T> &external) {
  std::string simulink_name(external.getName());
  boost::algorithm::replace_all(simulink_name, "/", "//");
  return connectToSimulinkPort(external, simulink_name);
}

bool SimulinkInterface::hasInputPort(const std::string &name) {
  return inputPorts.count(name) > 0;
}

template <typename T>
RTT::OutputPort<T> *SimulinkInterface::getInputPort(const std::string &name) {
  if (!inputPorts.count(name)) return 0;
  return dynamic_cast<RTT::OutputPort<T> *>(inputPorts[name]);
}

bool SimulinkInterface::hasOutputPort(const std::string &name) {
  return outputPorts.count(name) > 0;
}

template <typename T>
RTT::InputPort<T> *SimulinkInterface::getOutputPort(const std::string &name) {
  if (!outputPorts.count(name)) return 0;
  return dynamic_cast<RTT::InputPort<T> *>(outputPorts[name]);
}

template <typename T>
bool SimulinkConnector<T>::connectInput(SimulinkInterface *obj, RTT::InputPort<T> *external, RTT::base::OutputPortInterface *simulink) {
  VectorDoubleOutputPort *vector_port = dynamic_cast<VectorDoubleOutputPort *>(simulink);
  if (vector_port) {
    obj->inputConnections[external] = SimulinkInput<T,DoubleVector>(external, vector_port);
    RTT::log(RTT::Info) << "Connected input port " << external->getName() << " to double vector port " << simulink->getName() << RTT::endlog();
    return true;
  }

  SingleDoubleOutputPort *single_port = dynamic_cast<SingleDoubleOutputPort *>(simulink);
  if (single_port) {
    obj->inputConnections[external] = SimulinkInput<T,double>(external, single_port);
    RTT::log(RTT::Info) << "Connected input port " << external->getName() << " to double port " << simulink->getName() << RTT::endlog();
    return true;
  }

  VectorIntegerOutputPort *vector_int_port = dynamic_cast<VectorIntegerOutputPort *>(simulink);
  if (vector_int_port) {
    obj->inputConnections[external] = SimulinkInput<T,IntegerVector>(external, vector_int_port);
    RTT::log(RTT::Info) << "Connected input port " << external->getName() << " to integer vector port " << simulink->getName() << RTT::endlog();
    return true;
  }

  SingleIntegerOutputPort *single_int_port = dynamic_cast<SingleIntegerOutputPort *>(simulink);
  if (single_int_port) {
    obj->inputConnections[external] = SimulinkInput<T,int>(external, single_int_port);
    RTT::log(RTT::Info) << "Connected input port " << external->getName() << " to integer port " << simulink->getName() << RTT::endlog();
    return true;
  }

  RTT::log(RTT::Error) << "Input port " << external->getName() << " is not compatible with model input port " << simulink->getName() << RTT::endlog();
  return false;
}

template <typename T>
bool SimulinkConnector<T>::connectOutput(SimulinkInterface *obj, RTT::OutputPort<T> *external, RTT::base::InputPortInterface *simulink) {
  VectorDoubleInputPort *vector_port = dynamic_cast<VectorDoubleInputPort *>(simulink);
  if (vector_port) {
    obj->outputConnections[vector_port] = SimulinkOutput<T,DoubleVector>(external, vector_port);
    RTT::log(RTT::Info) << "Connected output port " << external->getName() << " to double vector port " << simulink->getName() << RTT::endlog();
    return true;
  }

  SingleDoubleInputPort *single_port = dynamic_cast<SingleDoubleInputPort *>(simulink);
  if (single_port) {
    obj->outputConnections[single_port] = SimulinkOutput<T,double>(external, single_port);
    RTT::log(RTT::Info) << "Connected output port " << external->getName() << " to double port " << simulink->getName() << RTT::endlog();
    return true;
  }

  VectorIntegerInputPort *vector_int_port = dynamic_cast<VectorIntegerInputPort *>(simulink);
  if (vector_int_port) {
    obj->outputConnections[vector_int_port] = SimulinkOutput<T,IntegerVector>(external, vector_int_port);
    RTT::log(RTT::Info) << "Connected output port " << external->getName() << " to integer vector port " << simulink->getName() << RTT::endlog();
    return true;
  }

  SingleIntegerInputPort *single_int_port = dynamic_cast<SingleIntegerInputPort *>(simulink);
  if (single_int_port) {
    obj->outputConnections[single_int_port] = SimulinkOutput<T,int>(external, single_int_port);
    RTT::log(RTT::Info) << "Connected output port " << external->getName() << " to integer port " << simulink->getName() << RTT::endlog();
    return true;
  }

  RTT::log(RTT::Error) << "Output port " << external->getName() << " is not compatible with model output port " << simulink->getName() << RTT::endlog();
  return false;
}

template <typename T, typename SimulinkType>
SimulinkInput<T,SimulinkType>::SimulinkInput(RTT::InputPort<T> *input_port, RTT::OutputPort<SimulinkType> *output_port)
  : input_port(input_port)
  , output_port(output_port)
{
}

template <typename T, typename SimulinkType>
SimulinkOutput<T,SimulinkType>::SimulinkOutput(RTT::OutputPort<T> *output_port, RTT::InputPort<SimulinkType> *input_port)
  : output_port(output_port)
  , input_port(input_port)
{
}

template <typename T, typename SimulinkType>
void SimulinkInput<T,SimulinkType>::operator()(RTT::base::PortInterface *) {
  if (input_port->readNewest(data, false) != RTT::NewData) return;
  if (!SimulinkConverter<T,SimulinkType>::toSimulink(data, simulink)) {
    return;
  }
  output_port->write(simulink);
  RTT::log(RTT::Debug) << "Updated input port " << output_port->getName() << RTT::endlog();
}

template <typename T, typename SimulinkType>
void SimulinkOutput<T,SimulinkType>::operator()(RTT::base::PortInterface *) {
  if (input_port->readNewest(simulink, false) != RTT::NewData) return;
  if (!SimulinkConverter<T,SimulinkType>::fromSimulink(simulink, data)) {
    return;
  }
  output_port->write(data);
  RTT::log(RTT::Debug) << "Updated output port " << input_port->getName() << RTT::endlog();
}

} // namespace Simulink
} // namespace uxvcos

#endif // SIMULINK_SIMULINKINTERFACE_INL
