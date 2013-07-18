#include "interface/RC.h"

#include <uxvcos/system/systemcalls.h>
#include <uxvcos/Configuration.h>
#include <rtt/typekit/RTTTypes.hpp>

#include <hector_uav_msgs/RC/functions.h>

namespace uxvcos {
namespace Interface {

class RC::Axis : public Child {
public:
  Axis(RTT::PropertyBag *bag, const std::string& name, const std::string& description)
    : Child(bag, name, description)
    , function("Function", "Function ID", 0)
    , channel("Channel", "Channel number for this axis", 0)
    , transformation(bag)
  {
    this->declareProperty(function);
    this->declareProperty(channel);

    RTT::log(RTT::Info) << "Configuring axis \"" << name << "\" with function " << std::string(hector_uav_msgs::getFunctionString(function.get())) << " on channel " << channel << RTT::endlog();
  }

  typedef boost::shared_ptr<Axis> shared_ptr;
  static shared_ptr Create(RTT::PropertyBag *bag, const std::string& name, const std::string& description = "") {
    return boost::make_shared<Axis>(bag, name, description);
  }

  float convert(unsigned short raw) const {
    if (raw == 0) return 0.0;
    float val = transformation((static_cast<double>(raw) - 1500.0) / 400.0);
    if (val < -1.0f) val = -1.0f;
    if (val > 1.0f)  val =  1.0f;
    return val;
  }

public:
  RTT::Property<unsigned int> function;
  RTT::Property<int> channel;

private:
  LinearTransformation<double,double> transformation;
};

class RC::Switch : public Child {
public:
  Switch(RTT::PropertyBag *bag, const std::string& name, const std::string& description)
    : Child(bag, name, description)
    , function("Function", "Function ID", 0)
    , channel("Channel", "Channel number for this axis", 0)
  {
    this->declareProperty(function);
    this->declareProperty(channel);

    RTT::log(RTT::Info) << "Configuring switch \"" << name << "\" on channel " << channel << RTT::endlog();
  }

  typedef boost::shared_ptr<Switch> shared_ptr;
  static shared_ptr Create(RTT::PropertyBag *bag, const std::string& name, const std::string& description = "") {
    return boost::make_shared<Switch>(bag, name, description);
  }

  unsigned char convert(unsigned short raw) const {
    if (raw > 1650) return 2;
    if (raw < 1350) return 0;
    return 1;
  }

public:
  RTT::Property<unsigned int> function;
  RTT::Property<int> channel;
};


RC::RC(RTT::TaskContext* parent, const std::string& name, const std::string& port_name, const std::string& description)
  : Sensors::RawSensor<hector_uav_msgs::RawRC, hector_uav_msgs::RC>(parent, name, port_name, description)
  , configuration(this)
{
  this->output.doc("Current RC commands");
  this->rawInput.doc("Current raw RC commands");

  // this->properties()->ownProperty(new RTT::Property<RTT::PropertyBag>("Channels", "List of all channels and functions"));
}

bool RC::initialize() {
  configuration.readAll("rc");

  cleanup();

  for(RTT::PropertyBag::iterator it = properties()->begin(); it != properties()->end(); ++it) {
    RTT::Property<RTT::PropertyBag> *property = RTT::Property<RTT::PropertyBag>::narrow(*it);
    if (!property) continue;

    RTT::Property<std::string> *type = property->value().getPropertyType<std::string>("Type");
    if (!type) continue;

    if (type->value() == "Axis") {
      axes.push_back(Axis::Create(&(property->value()), property->getName(), property->getDescription()));
    }

    if (type->value() == "Switch") {
      switches.push_back(Switch::Create(&(property->value()), property->getName(), property->getDescription()));
    }
  }

  return true;
}

void RC::cleanup() {
  axes.clear();
  switches.clear();
}

bool RC::convert(const hector_uav_msgs::RawRC& update)
{
  data.header = update.header;
  data.status   = update.status;
  data.valid    = data.status >= 180;
  data.axis.clear();
  data.axis_function.clear();
  data.swit.clear();
  data.swit_function.clear();

  for(std::vector<Axis::shared_ptr>::iterator it = axes.begin(); it != axes.end(); ++it) {
    Axis::shared_ptr axis = *it;
    if (axis->channel == 0 || axis->channel > (int) update.channel.size()) continue;
    hector_uav_msgs::setAxis(data, axis->function, axis->convert(update.channel[axis->channel-1]));
  }

  for(std::vector<Switch::shared_ptr>::iterator it = switches.begin(); it != switches.end(); ++it) {
    Switch::shared_ptr swit = *it;
    if (swit->channel == 0 || swit->channel > (int) update.channel.size()) continue;
    hector_uav_msgs::setSwitch(data, swit->function, swit->convert(update.channel[swit->channel-1]));
  }

  return true;
}

std::ostream& operator<<(std::ostream& os, const RC& rc) {
  char debugBuffer[256];
  snprintf(debugBuffer, sizeof(debugBuffer) - 1, "status:% 3d 1:%4u 2:%4u 3:%4u 4:%4u 5:%4u 6:%4u 7:%4u 8:%4u 9:%4u 10:%4u",
           rc.data.status, rc.raw.channel[0], rc.raw.channel[1], rc.raw.channel[2], rc.raw.channel[3], rc.raw.channel[4],
                           rc.raw.channel[5], rc.raw.channel[6], rc.raw.channel[7], rc.raw.channel[8], rc.raw.channel[9]);
  return os << std::string(debugBuffer);
}

} // namespace Interface
} // namespace uxvcos
