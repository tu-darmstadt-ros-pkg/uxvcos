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

#ifndef QUADRO_AUTONOMY_SIMULINK_H
#define QUADRO_AUTONOMY_SIMULINK_H

#include "Autonomy.h"
#include <simulink/Simulink.h>

#include <types/interface.h>
#include <types/sensors.h>
#include <types/navigation.h>
#include <types/controller.h>
#include <controller/types/UsedHeight.h>
#include <controller/types/State.h>

namespace Autonomy {

class Simulink : public Autonomy, public ::Simulink::Interface
{
  public:
    Simulink(const std::string &name = "Autonomy", const std::string& model = "Autonomy");
    virtual ~Simulink();

    static int setup();

  protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void reset();
    virtual void stopHook();
    virtual void cleanupHook();

    virtual void sendCommand(unsigned int command);

  protected:
    RTT::InputPort<Data::Navigation::BiasedIMU>      portIMU;
    RTT::InputPort<Data::Navigation::Solution>       portNavigation;
    RTT::InputPort<Data::Navigation::LocalSolution>  portLocalNavigation;

    RTT::InputPort<Data::Sensor::Supply>             portSupply;
    RTT::InputPort<Data::Sensor::Ultrasound>         portUltrasound;
    RTT::InputPort<Data::Sensor::Baro>               portBaro;
    RTT::InputPort<Data::Sensor::Temperature>        portTemperature;
    RTT::InputPort<Data::Sensor::Airspeed>           portAirspeed;
    RTT::InputPort<Data::Navigation::GPS>            portGPS;
    RTT::InputPort<Data::Navigation::Compass>        portCompass;
    RTT::InputPort<Data::Interface::MotorStatus>     portMotorStatus;

    RTT::InputPort<Data::Controller::RC>             portRC;
    RTT::InputPort<Data::Controller::UsedHeight>     portUsedHeight;
    RTT::InputPort<Data::Controller::State>          portControllerState;

    RTT::InputPort<Data::Controller::Position>       portPositionInput;
    RTT::InputPort<Data::Controller::Velocity>       portVelocityInput;
    RTT::InputPort<Data::Controller::Attitude>       portAttitudeInput;
    RTT::InputPort<Data::Controller::Heading>        portHeadingInput;
    RTT::InputPort<Data::Controller::Turnrate>       portTurnrateInput;
    RTT::InputPort<Data::Controller::Height>         portHeightInput;
    RTT::InputPort<Data::Controller::VelocityZ>      portClimbrateInput;
    RTT::InputPort<Data::Controller::Throttle>       portThrustInput;
    RTT::InputPort<Data::Interface::MotorCommand>    portMotorCommandInput;

    RTT::OutputPort<Data::Controller::Position>      portPositionOutput;
    RTT::OutputPort<Data::Controller::Velocity>      portVelocityOutput;
    RTT::OutputPort<Data::Controller::Attitude>      portAttitudeOutput;
    RTT::OutputPort<Data::Controller::Heading>       portHeadingOutput;
    RTT::OutputPort<Data::Controller::Turnrate>      portTurnrateOutput;
    RTT::OutputPort<Data::Controller::Height>        portHeightOutput;
    RTT::OutputPort<Data::Controller::VelocityZ>     portClimbrateOutput;
    RTT::OutputPort<Data::Controller::Throttle>      portThrustOutput;
    RTT::OutputPort<Data::Interface::MotorCommand>   portMotorCommandOutput;

    RTT::OutputPort<unsigned int>                    portControlMode;

    RTT::Property<Data::Navigation::ReferencePosition> *propertyReference;
};
} // namespace Autonomy

#endif // QUADRO_AUTONOMY_SIMULINK_H
