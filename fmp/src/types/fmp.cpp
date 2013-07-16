#include <data/Registry.h>
#include <data/StreamableTypeInfo.h>
#include <types/classes.h>
#include <data/Typekit.h>

#include "fmp.h"

namespace Data {
namespace FMP {
  bool Typekit::loadTypes() {
    types()->addType<FMPRawData>    ("FMP::RawData", (0x7B << 8) | 0x6F).setShortName("FMPRaw");
    types()->addType<FMPSensorData> ("FMP::SensorData", (0x7B << 8) | 0x01).setShortName("FMPSensor");
    return true;
  }

  std::string Typekit::getName() {
    return "FMP";
  }
}}

UXVCOS_TYPEKIT_PLUGIN(FMP)
