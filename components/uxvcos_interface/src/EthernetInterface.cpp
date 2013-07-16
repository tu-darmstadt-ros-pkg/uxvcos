#include <string.h>

#include <options/options.h>
#include <base/SetupFunction.h>

#include <system/Socket.h>
#include <system/SerialPort.h>
#include <system/Error.h>

#include "EthernetInterface.h"
#include <interface/RateControl.h>

#include "ublox.h"

#include <rtt/Component.hpp>

using namespace RTT;

namespace uxvcos {
namespace Interface {

namespace { SetupFunction setup(&EthernetInterface::setup, "EthernetInterface"); }

EthernetInterface::EthernetInterface(const std::string &name)
  : BaseInterface(name)

  , motor(new Motor(this))
  , servos(new Servos(this))
  , uart0(new EthernetSerialPort(this, 0, "Ethernet0"))
  , uart1(new EthernetSerialPort(this, 1, "Ethernet1"))

  , connectionType("ConnectionType", "Type of connection (0 = Ethernet, 1 = Serial)", MODE_ETHERNET)
  , device("Device", "Network interface/serial port to use", "eth0")
  , destination("Destination", "Destination MAC address (default ff:ff:ff:ff:ff:ff)", "ff:ff:ff:ff:ff:ff")
  , protocol("Protocol", "Ethernet Protocol", 0x1234)
  , baudrate("Baudrate", "Baudrate for serial connection mode", 115200)
  , recv_timeout("ReceiveTimeout", "Timeout for packet reception in milliseconds", 20)

  , timeoutCounter(0)

  , debugProperties("Debug", "Properties for debugging purposes")
  , rateDebug(&(debugProperties.value()), &rateControl)

  , requestMethod("request", &EthernetInterface::request, this)

  , interfaceDelay("InterfaceDelay")
  , overallDelay("OverallDelay")
  
  , checkVersionMethod("checkVersion", &EthernetInterface::checkVersion, this)

  , socket(0), serialPort(0), stream(0)
  , sendTimestamp(0)
{
  this->addProperty(connectionType);
  this->addProperty(device);
  this->addProperty(destination);
  this->addProperty(protocol);
  this->addProperty(baudrate);
  this->addProperty(recv_timeout);

  this->addAttribute(interfaceDelay);
  this->addAttribute(overallDelay);
  this->addProperty(debugProperties);

  this->addOperation( requestMethod ).doc("Requests a specific piece of information from the hardware interface.").arg("request", "ID of the request.");
  this->addOperation( checkVersionMethod ).doc("Retrieves the version and hardware revision from the Interface Board");

  setupSensors();

  addModule(motor);
  addModule(servos);
  addModule(uart0);
  addModule(uart1);

  theRequest[0] = 0;
  theRequest[1] = 0;
}

EthernetInterface::~EthernetInterface() {
  stop();
  cleanup();
}

int EthernetInterface::setup() {
  Options::Description options("Interface options");
  options.add_options()
    ("interface,i", Options::value<std::string>(), "Network interface/serial port to use");
  Options::options().add(options);
  return 0;
}

bool EthernetInterface::configureHook()
{
  Logger::In in(getName());

  // reconfigure?
  if (getTaskState() == RTT::TaskContext::Stopped) cleanup();

  if (stream) return false;

  if (!Options::variables("interface").empty()) {
    device = Options::variables<std::string>("interface");
    if (device.get()[0] == '/') {
      connectionType = MODE_SERIAL;
    } else {
      connectionType = MODE_ETHERNET;
    }
  }

  if (connectionType.get() == MODE_ETHERNET) {
    unsigned short protocol = static_cast<unsigned short>(this->protocol);

    socket = new System::Socket(System::Socket::SOCKET_PACKET);
    stream = socket;

    remoteAddress.set(destination, protocol);
    if (!remoteAddress) {
      log( Fatal ) << "invalid destination MAC address " << destination.get() << endlog();
      cleanupHook();
      return false;
    }

    // create rt-socket
    if (!socket->create(protocol))
    {
      log( Fatal ) << "socket(): " << System::lastError().string() << endlog();
      cleanupHook();
      return false;
    }

     // enable broadcasts
     socket->setBroadcast(true);
  //   if (!socket->setBroadcast(true))
  //   {
  //     log( Fatal ) << "setsockopt(socketId, SOL_SOCKET, SO_BROADCAST, ...): " << System::lastError().string() << endlog();
  //     cleanupHook();
  //     return false;
  //   }
     socket->setNonBlocking(true);

    std::string temp(device);
  #ifdef USE_RTNET
    if (temp.compare(0, 2, "rt") != 0) {
      device = std::string("rt") + temp;
    }
  #else
    if (temp.compare(0, 2, "rt") == 0) {
      device = temp.substr(2);
    }
  #endif

    if (!socket->bind(device, protocol, &localAddress)) {
      log( Fatal ) << "Could not bind socket to local interface " << device.get() << ": " << System::lastError().string() << endlog();
      cleanupHook();
      return false;
    }

    remoteAddress.setInterface(localAddress);
    if (!socket->connect(remoteAddress)) {
      log( Fatal ) << "Could not connect to " << remoteAddress.toString() << ": " << System::lastError().string() << endlog();
      cleanupHook();
      return false;
    }

    log( Info ) << "Successfully configured EthernetInterface:" << nlog();
    log( Info ) << "  using interface " << device.get() << " with address " << localAddress.toString() << nlog();
    log( Info ) << "  using destination MAC address "  << remoteAddress.toString() << endlog();

  } else if (connectionType.get() == MODE_SERIAL) {
    serialPort = new System::SerialPort(device.get(), baudrate.get());
    stream = serialPort;

    if (!serialPort->open()) {
      log( Fatal ) << "Could not open serial port " << device.get() << " with baudrate " << baudrate.get() << ": " << System::lastError().string() << endlog();
      cleanupHook();
      return false;
    }
  }

  initializeModules();

  return true;
}

void EthernetInterface::cleanupHook()
{
  Logger::In in(getName());

  cleanupModules();

  if (socket) {
    socket->close();
    delete socket;
    socket = 0;
  }

  if (serialPort) {
    serialPort->close();
    delete serialPort;
    serialPort = 0;
  }

  stream = 0;
}

bool EthernetInterface::startHook()
{
  Logger::In in(getName());

  rateControl.counter() = 0;
  timeoutCounter = 0;
  buffer_in.clear();

  checkVersion();

  // RTT::OperationCaller<bool(double, unsigned int)>(imu->getOperation("setZero")).send(5.0, 63);
  return true;
}

void EthernetInterface::stopHook()
{
  Logger::In in(getName());
}
bool EthernetInterface::send(const void *data, size_t size, unsigned char classId, unsigned char messageId)
{
  return send(buffer_out, ublox_encode(data, size, classId, messageId, buffer_out, sizeof(buffer_out)));
}

bool EthernetInterface::send(const char *buffer, int length)
{
  RTT::os::MutexLock lock(sendMutex);

  if (!stream || !stream->good()) return false;
  if (length < 0)  return false;
  if (length == 0) return true;

  stream->flush();
  if (!stream->write(buffer, length))
  {
    log( Error ) << "send(): " << System::lastError().string() << endlog();
    return false;
  }
  // log( Debug ) << "send(): " << ret << "/" << length << " bytes transmitted" << endlog();

  return true;
}

bool EthernetInterface::newMotorCommand(const hector_uav_msgs::MotorPWM& data) {
  struct ArmMotorCommand_t ArmMotorCommand;
  Logger::In in(getName());

  if (!this->isRunning() || !stream || !stream->good()) return false;

  for(std::size_t i = 0; i < ARMMOTOR_COUNT; ++i) {
    if (i < data.pwm.size()) ArmMotorCommand.CommandMotorPWM[i] = data.pwm[i]; else ArmMotorCommand.CommandMotorPWM[i] = 0;
  }

  overallDelay.set(RTT::os::TimeService::Instance()->secondsSince(sendTimestamp));
  if (debug)
  {
    snprintf(debugBuffer, sizeof(debugBuffer) - 1, "PWM[0]:%03d PWM[1]:%03d PWM[2]:%03d PWM[3]:%03d",
    data.pwm[0], data.pwm[1], data.pwm[2], data.pwm[3]);
    log(RealTime) << "MotorCommand: " << std::string(debugBuffer) << endlog();

    log(RealTime) << "Overall Latency: " << (overallDelay.get() * 1e3) << " ms" << endlog();
  }

  return send(&ArmMotorCommand, sizeof(ArmMotorCommand), ARM_INTERFACE_CLASS, ARM_MOTOR_ID);
}

bool EthernetInterface::newServoCommand(const hector_uav_msgs::ServoCommand& data) {
  struct ArmServo_t ArmServo;
  Logger::In in(getName());

  if (!this->isRunning() || !stream || !stream->good()) return false;

  for(std::size_t i = 0; i < ARMSERVO_CHANNELS; ++i) {
    if (i < data.value.size()) ArmServo.value[i] = data.value[i]; else ArmServo.value[i] = 0;
  }

  if (debug)
  {
    snprintf(debugBuffer, sizeof(debugBuffer) - 1, "Servo[0]:%04d Servo[1]:%04d  Servo[2]:%04d Servo[3]:%04d Servo[4]:%04d Servo[5]:%04d",
    data.value[0], data.value[1], data.value[2], data.value[3], data.value[4], data.value[5]);
    log(RealTime) << "ServoCommand: " << std::string(debugBuffer) << endlog();

    overallDelay.set(RTT::os::TimeService::Instance()->secondsSince(sendTimestamp));
    log(RealTime) << "Overall Latency: " << (overallDelay.get() * 1e3) << " ms" << endlog();
  }

  return send(&ArmServo, sizeof(ArmServo), ARM_INTERFACE_CLASS, ARM_SERVO_ID);
}

bool EthernetInterface::setBaudrate(EthernetSerialPort* port, unsigned long baudrate) {
  unsigned char messageId;
  struct ArmUARTConfig_t config;
  Logger::In in(getName());

  // if (!this->isRunning() || !stream || !stream->good()) return false;

  config.baudrate = baudrate;
  config.mode  = 0x03; // UART_8N1
  // config.fmode = 0x81; // UART_FIFO_8
  config.fmode = 0x01; // UART_FIFO_1

  switch(port->getIndex()) {
    case 0:  messageId = ARM_CONFIG_UART0_ID; break;
    case 1:  messageId = ARM_CONFIG_UART1_ID; break;
    default: return false;
  }

  log(Info) << "setting baudrate of Ethernet Serial Port " << port->getIndex() << " to " << baudrate << endlog();
  return send(&config, sizeof(config), ARM_INTERFACE_CLASS, messageId);
}

int EthernetInterface::uartSend(EthernetSerialPort* port, const void* source, unsigned int length) {
  unsigned char messageId;

  switch(port->getIndex()) {
    case 0:  messageId = ARM_UART0_ID; break;
    case 1:  messageId = ARM_UART1_ID; break;
    default: return false;
  }

  return send(source, length, ARM_INTERFACE_CLASS, messageId) ? length : 0;
}

bool EthernetInterface::uartPoll(EthernetSerialPort* port) {
  unsigned int request;

  if (this->isRunning()) return false;

  switch(port->getIndex()) {
    case 0:  request = ARM_UART0_ID; break;
    case 1:  request = ARM_UART1_ID; break;
    default: return false;
  }

  // wait 20ms
  TIME_SPEC wait;
  wait.tv_sec  = 0;
  wait.tv_nsec = 20000000;
  rtos_nanosleep(&wait, 0);

//   log(Debug) << "polling Ethernet Serial Port " << port->getIndex() << "..." << endlog();
  return this->request(request);
}

bool EthernetInterface::checkVersion()
{
  return request(ARM_BOARD_VERSION);  
}

bool EthernetInterface::request(unsigned int request) {
  theRequest[0] = 0;
  theRequest[1] = 0;

  SET_REQUEST(request, theRequest);

  struct ArmRequest_t ArmRequest;
  ArmRequest.word[0] = theRequest[0];
  ArmRequest.word[1] = theRequest[1];

  RTT::os::MutexLock lock(requestMutex);
  debug = true;
  // sendTimestamp = RTT::os::TimeService::Instance()->getTicks();
  return send(&ArmRequest, sizeof(ArmRequest), ARM_INTERFACE_CLASS, ARM_REQUEST_ID)
           && receiveAnswer();
}

bool EthernetInterface::sendPeriodicRequest()
{
  theRequest[0] = 0;
  theRequest[1] = 0;

  for(EthernetSensorContainer::set_iterator it = sensors.begin(); it != sensors.end(); ++it) {
    EthernetSensorPtr sensor = *it;
    if (sensor->isTime()) sensor->request(theRequest);
  }

  if (uart0->isOpen()) SET_REQUEST(ARM_UART0_ID, theRequest);
  if (uart1->isOpen()) SET_REQUEST(ARM_UART1_ID, theRequest);

  if (theRequest[0] == 0 && theRequest[1] == 0) return false;

  struct ArmRequest_t ArmRequest;
  ArmRequest.word[0] = theRequest[0];
  ArmRequest.word[1] = theRequest[1];

  // sendTimestamp = RTT::os::TimeService::Instance()->getTicks();
  return send(&ArmRequest, sizeof(ArmRequest), ARM_INTERFACE_CLASS, ARM_REQUEST_ID);
}

bool EthernetInterface::receiveAnswer(bool retry)
{
  bool receivedSomeData = false;

  if (!stream || !stream->good()) return false;

  // System::EthernetAddress receiveAddress;

  struct UBloxHeader uBloxHeader;
  void *UBloxPayload;
  size_t UBloxPayloadLength;
  size_t length = 0;

  if (!retry) {
    sendTimestamp = RTT::os::TimeService::Instance()->getTicks();
    buffer_in.clear();
  }

  while(theRequest[0] != 0 || theRequest[1] != 0)
  {
    // check for timeout
    if (RTT::os::TimeService::Instance()->secondsSince(sendTimestamp) > recv_timeout.get() / 1000.0) {
      log(Warning) << "incomplete answer from interface board within " << recv_timeout.get() << " ms" << endlog();
      ++timeoutCounter;
      buffer_in.clear();
      if (timeoutCounter >= 20)
      {
	      log(Error) << "too many failures, interface board is not responding!" << endlog();
	      stop();
      }
      return false;
    }

    // receive data
    if (stream->wait(recv_timeout.get())) {
      buffer_in.add(stream->readsome(buffer_in.tail(), buffer_in.tailfree()));
    } else
      continue;

    if (stream->eof()) {
      System::Error error = System::lastError();
      if (error == ETIMEDOUT || error == EAGAIN) continue;
      log(Error) << "Error while reading: " << error << endlog();
      continue;
    }

    // log(Info) << "read " << length << " bytes, buffer has " << buffer_in.size() << " bytes now" << endlog();

    // search ublox packets
    char *pos = reinterpret_cast<char *>(buffer_in.data());
    length = buffer_in.size();

    while (ublox_find(&pos, &length, NULL) > 0)
    {
      if (ublox_decode(pos, length, &UBloxPayload, &UBloxPayloadLength, &uBloxHeader) <= 0 || uBloxHeader.classId != ARM_INTERFACE_CLASS)
      {
        ublox_findNext(&pos, &length);
        buffer_in.del(pos - reinterpret_cast<char *>(buffer_in.data()));
        continue;
      }

      if (uBloxHeader.messageId == ARM_REQUEST_ID) {
        // if a request is received, ignore packet and wait for another answer!
        // log(Debug) << "received a copy of a my own request, ignoring..." << endlog();

        ublox_findNext(&pos, &length);
        buffer_in.del(pos - reinterpret_cast<char *>(buffer_in.data()));
        return receiveAnswer(true);
      }

      // some data bytes have been received successfully
      if (!receivedSomeData) {
        receivedSomeData = true;
        timeoutCounter = 0;

        // don't care for further UART data if at least some data have been received
        CLEAR_REQUEST(ARM_UART0_ID, theRequest);
        CLEAR_REQUEST(ARM_UART1_ID, theRequest);

        // determine and show interface latency
        interfaceDelay.set(RTT::os::TimeService::Instance()->secondsSince(sendTimestamp));
        if (debug) {
          log(RealTime) << "Interface Latency: " << (interfaceDelay.get() * 1e3) << " ms" << endlog();
        }

        // update timestamp
        setTimestamp(uxvcos::Time::now());
      }

      for (EthernetSensorContainer::map_iterator it = sensors.lower_bound(uBloxHeader.messageId); it != sensors.upper_bound(uBloxHeader.messageId); ++it) {
        EthernetSensorPtr sensor = it->second;
        if (!sensor->decode(UBloxPayload, UBloxPayloadLength, uBloxHeader.messageId)) continue;

        if (debug) {
          log(RealTime) << sensor->getName() << ": ";
          sensor->log();
          log() << endlog();
        }
      }

      switch (uBloxHeader.messageId)
      {
        case ARM_UART0_ID:
          uart0->receiveHandler(UBloxPayload, UBloxPayloadLength, getTimestamp());
          break;

        case ARM_UART1_ID:
          uart1->receiveHandler(UBloxPayload, UBloxPayloadLength, getTimestamp());
          break;

        case ARM_BOARD_VERSION:
          if (UBloxPayloadLength >= sizeof(ArmBoardVersion_t))
          {
            struct ArmBoardVersion_t *ArmBoardVersion = reinterpret_cast<struct ArmBoardVersion_t *>(UBloxPayload);
            log( Info ) << __FILE__ " compiled at " __DATE__ " " __TIME__ << nlog();
            log() << "ArmBoardVersion:   " << ArmBoardVersion->versionID << nlog();
            log() << "ENC28J60 Revision: " << (unsigned int) ArmBoardVersion->enc28j60_revision << nlog();
            log() << endlog();
          }
          break;
    
        default:
          break;
      }

      CLEAR_REQUEST(uBloxHeader.messageId, theRequest);

      ublox_findNext(&pos, &length);
      buffer_in.del(pos - reinterpret_cast<char *>(buffer_in.data()));
    }

    if (connectionType.get() == MODE_ETHERNET) break;
    if (buffer_in.size() > 0 && buffer_in[0] == 0) break;
  }

  if (theRequest[0] != 0 || theRequest[1] != 0) {
    for(int i = 0; i < (int) sizeof(theRequest) * 8; ++i) {
      std::string fail;
      if (IS_REQUESTED(i, theRequest)) {
        switch(i) {
          case ARM_RAW_IMU_ID:          fail = "RawIMU"; break;
          case ARM_IMU_ID:              fail = "IMU"; break;
          case ARM_ATTITUDE_ID:         fail = "Attitude"; break;
          case ARM_BIAS_ID:             fail = "Bias"; break;
          case ARM_VELOCITY_ID:         fail = "Velocity"; break;
          case ARM_MOTOR_ID:            fail = "MotorStatus"; break;
          case ARM_US_SENSOR_ID:        fail = "Ultrasound"; break;
          case ARM_RC_ID:               fail = "RC"; break;
          case ARM_RAW_MAG3D_ID:        fail = "Magnetic"; break;
          case ARM_RAW_BARO_ID:         fail = "Baro"; break;
          case ARM_RAW_BARO_TEMP_ID:    fail = "Temperature"; break;
          case ARM_RAW_AIRSPEED_ID:     fail = "Airspeed"; break;
          case ARM_RAW_AD0_ID:          fail = "RawAD0"; break;
          case ARM_RAW_AD1_ID:          fail = "RawAD1"; break;
          case ARM_INTERNAL_AD_ID:      fail = "RawInternalAD"; break;
          case ARM_BOARD_VERSION:       fail = "BoardVersion"; break;
          default: continue;
        }
        log( Warning ) << "did not receive an answer for request: " << fail << endlog();
      }
    }
  }

  return true;
}

void EthernetInterface::updateHook()
{
  RTT::os::MutexLock lock(requestMutex);

  rateControl.period() = this->getPeriod();
  rateControl.step();
  debug = rateDebug.isTime();

  sendPeriodicRequest() && receiveAnswer();
  executeModules();
}

RateControl *EthernetInterface::getRateControl() {
  return &rateControl;
}

RateControl::counter_t& EthernetInterface::getCounter() {
  return getRateControl()->counter();
}

bool EthernetInterface::debugEnabled() const {
  return debug;
}

ORO_CREATE_COMPONENT(EthernetInterface)

} // namespace Interface
} // namespace uxvcos
