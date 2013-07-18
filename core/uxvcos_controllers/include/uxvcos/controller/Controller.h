//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#ifndef UXVCOS_CONTROLLER_SUBCONTROLLER_H
#define UXVCOS_CONTROLLER_SUBCONTROLLER_H

#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <uxvcos/Module.h>
#include <string>

#include <boost/type_traits.hpp>
#include <ros/message_traits.h>

namespace uxvcos {
namespace Controller {

static inline bool connectTo(RTT::base::InputPortInterface *input, RTT::base::OutputPortInterface *output) {
  if (!input || !output) return false;
  // if (input->connected()) input->disconnect();
  return output->connectTo(input);
}

class ControllerTask;

class BaseController : public Module {
public:
  friend class ControllerTask;
  typedef boost::shared_ptr<BaseController> shared_ptr;

  BaseController(ControllerTask* owner, const std::string& name, const std::string& description);
  virtual ~BaseController() {}

  ControllerTask *getOwner() const;

  virtual bool isActive() { return _is_active; }
  bool autostart() const { return _autostart; }
  BaseController& autostart(bool flag) { _autostart = flag; return *this; }

  bool start();
  void stop();

  virtual void disconnect() = 0;
  virtual void reset() = 0;

protected:
  virtual bool update(RTT::FlowStatus, double dt) = 0;
  virtual bool update(double dt) = 0;
  virtual bool update() = 0;
  virtual bool beforeStart() = 0;
  virtual void afterStop() = 0;

private:
  friend class ModuleContainer;
  virtual bool init() = 0;
  virtual void execute() = 0;
  virtual void cleanup() = 0;

protected:
  std::string topic;

private:
  bool _is_active;
  bool _autostart;
};

struct NoInput {};
template <class Parent, class Output = double, class Input = NoInput, class Rate = NoInput>
class Controller : public BaseController {
  public:
    typedef Output OutputType;
    typedef Input  InputType;
    typedef Rate   RateType;

    Controller(Parent* controller, const std::string& name, const std::string& description)
      : BaseController(controller, name, description)
      , portCommand(topic + "_command")
      , portInput(topic + "_input")
      , portRateCommand(topic + "_rate_command")
      , portRateInput(topic + "_rate_input")
      , portRateOutput(topic + "_rate_output")
      , portOutput(topic + "_output")
      , controller(controller)
    {
      if (hasInput()) {
        this->addPort(portCommand).doc(name + " command input");
        this->addPort(portInput).doc(name + " input feedback");
        this->addOperation("getInput",  &Controller<Parent,Output,Input,Rate>::getInput,  this).doc("Returns the current input value of the " + name + " controller");
        this->addOperation("setInput",  &Controller<Parent,Output,Input,Rate>::setInput,  this).doc("Sets the input value of the " + name + " controller");
      }

      if (hasRate()) {
        this->addPort(portRateCommand).doc(name + " rate command input");
        this->addPort(portRateInput).doc(name + " rate input feedback");
        this->addPort(portRateOutput).doc(name + " rate output");
        this->addOperation("getRateInput",  &Controller<Parent,Output,Input,Rate>::getRateInput,  this).doc("Returns the current input rate value of the " + name + " controller");
        this->addOperation("setRateInput",  &Controller<Parent,Output,Input,Rate>::setRateInput,  this).doc("Sets the input rate value of the " + name + " controller");
        this->addOperation("getRateOutput",  &Controller<Parent,Output,Input,Rate>::getRateOutput,  this).doc("Returns the current output rate value of the " + name + " controller");
        this->addOperation("setRateOutput",  &Controller<Parent,Output,Input,Rate>::setRateOutput,  this).doc("Sets the output rate value of the " + name + " controller");
      }

      this->addPort(portOutput).doc(name + " output");
      this->addOperation("getOutput", &Controller<Parent,Output,Input,Rate>::getOutput, this).doc("Returns the current output value of " + name + " controller");
      this->addOperation("setOutput", &Controller<Parent,Output,Input,Rate>::setOutput, this).doc("Sets the output value of " + name + " controller");
    }

    virtual ~Controller()
    {}

    Parent *getOwner() const
    {
      return static_cast<Parent *>(BaseController::getOwner());
    }

    static bool hasInput() {
      return boost::is_same<Input,NoInput>::value == false;
    }

    static bool hasRate() {
      return boost::is_same<Rate,NoInput>::value == false;
    }

    virtual void setInput(const InputType& new_value) {
      input = new_value;
    }

    virtual const InputType& getInput() const {
      return input;
    }

    virtual void setRateInput(const RateType& new_value) {
      rate_input = new_value;
    }

    virtual const RateType& getRateInput() const {
      return rate_input;
    }

    RTT::InputPort<InputType> *getCommandPort() {
      return &portCommand;
    }

    RTT::OutputPort<InputType> *getInputPort() {
      return &portInput;
    }

    RTT::InputPort<RateType> *getRateCommandPort() {
      return &portRateCommand;
    }

    RTT::OutputPort<RateType> *getRateInputPort() {
      return &portRateInput;
    }

    virtual void setOutput(const OutputType& new_value) {
      if (&output != &new_value) output = new_value;
      if (ros::message_traits::HasHeader<OutputType>::value) ros::message_traits::Header<OutputType>::pointer(output)->stamp = controller->getTimestamp();
      portOutput.write(getOutput());
    }

    virtual const OutputType& getOutput() const {
      return output;
    }

    RTT::OutputPort<OutputType> *getOutputPort() {
      return &portOutput;
    }

    virtual void setRateOutput(const RateType& new_value) {
      if (&rate_output != &new_value) rate_output = new_value;
      if (ros::message_traits::HasHeader<RateType>::value) ros::message_traits::Header<RateType>::pointer(rate_output)->stamp = controller->getTimestamp();
      portRateOutput.write(getRateOutput());
    }

    virtual const RateType& getRateOutput() const {
      return rate_output;
    }

    RTT::OutputPort<RateType> *getRateOutputPort() {
      return &portRateOutput;
    }

    bool connect(RTT::base::InputPortInterface *input) {
      return connectTo(input, &portOutput);
    }

    bool connect(RTT::base::OutputPortInterface *output) {
      return connectTo(&portCommand, output);
    }

    template <class OtherOutput, class OtherRate>
    bool connect(Controller<Parent,OtherOutput,Output,OtherRate>& other) {
      return connectTo(other.portCommandPort(), &portOutput);
    }

    template <class OtherInput, class OtherRate>
    bool connect(Controller<Parent,Input,OtherInput,OtherRate>& other) {
      return connectTo(&portCommand, other.getOutputPort());
    }

    void disconnect() {
      portOutput.disconnect();
    }

    virtual void reset() {
    }

  protected:
    virtual bool update(RTT::FlowStatus input_status, RTT::FlowStatus rate_status, double dt) { return update(input_status, dt); }
    virtual bool update(RTT::FlowStatus input_status, double dt) { return update(dt); }
    virtual bool update(double dt)                   { return update(); }
    virtual bool update()                            { return false; }

    virtual bool beforeStart() {
      reset();
      return true;
    }

    virtual void afterStop() {
      setOutput(OutputType());
    }

  private:
    virtual bool init() {
      return true;
    }

    virtual void cleanup() {
      stop();
    }

    void execute() {
      RTT::FlowStatus input_status = RTT::NoData, rate_status = RTT::NoData;

      if (hasRate()) rate_status  = portRateCommand.readNewest(rate_input, false);
      if (hasInput()) input_status = portCommand.readNewest(input, false);

      if (!isActive()) return;
      if (!update(input_status, rate_status, controller->getDt())) return;

      if (hasInput())     portInput.write(getInput());
      if (hasRate()) portRateInput.write(getRateInput());

      setOutput(output);
      if (hasRate()) setRateOutput(rate_output);
    }

  protected:
    RTT::InputPort<InputType> portCommand;
    RTT::OutputPort<InputType> portInput;
    InputType input;

    RTT::InputPort<RateType> portRateCommand;
    RTT::OutputPort<RateType> portRateInput;
    RateType rate_input;

    RTT::OutputPort<RateType> portRateOutput;
    RateType rate_output;

    RTT::OutputPort<OutputType> portOutput;
    OutputType output;

    Parent *controller;
    bool is_active;
};

} // namespace Controller
} // namespace uxvcos

#endif // UXVCOS_CONTROLLER_SUBCONTROLLER_H
