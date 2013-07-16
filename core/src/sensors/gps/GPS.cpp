#include "GPS.h"
#include "gpsfunctions/Tubloxgps.h"

#include <types/navigation.h>
#include <rtt/Component.hpp>

namespace uxvcos {
namespace Sensors {

GPS::GPS(const std::string& name, const std::string& description)
  : RTT::TaskContext(name, RTT::TaskContext::PreOperational)
  // , rawIn("RawGPS")
//  , rawOut("RawGPS")
//  , gps("GPS")
//  , svinfo("SVInfo")
//  , sbasdgps("SBASDGPS")
  , port(0)
  , UbloxGPS(0)
{
  // this->addEventPort( rawIn ).doc("Raw GPS solution from module");
//  this->addPort( rawOut ).doc("Raw GPS solution from module");
//  this->addPort( gps ).doc("GPS solution");
//  this->addPort( svinfo ).doc("Satellite View packet");
//  this->addPort( sbasdgps ).doc("SBAS/DGPS information");
  this->addPort("fix", portFix);
  this->addPort("fix_velocity", portVelocity);
  this->addProperty("device", device).doc("Name of the SerialPort device to use for GPS communication");

  this->addOperation("configureDynModel", &GPS::configureDynModel, this, RTT::OwnThread);
  this->addOperation("configureFixMode", &GPS::configureFixMode, this, RTT::OwnThread);
}

void GPS::setSerialPort(System::BaseSerialPort *port) {
  this->port = port;
}

std::string GPS::getDevice() const {
  return device;
}

bool GPS::configureHook() {
  RTT::Logger::In in(getName());

  port = System::getSerialPort(device);
  if (!port) return false;
  RTT::log( RTT::Info ) << "Using port " << device << " for the GPS" << RTT::endlog();
  
  //--> Initialisiere TUBLOXGPS-Objekt
  UbloxGPS = new TUBLOXGPS(port);
  if (!UbloxGPS->Initialize()) {
    RTT::log ( RTT::Error ) << "Failed to initialize uBlox GPS module!" << RTT::endlog();
    cleanupHook();
    return false;
  }
  return true;  
}

bool GPS::startHook() {
  RTT::Logger::In in(getName());
  return true;
}

void GPS::updateHook()
{
  RTT::Logger::In in(getName());

  UBLOX_DATA_TYPE GPSPacketType;

  if (!UbloxGPS) return;
  
//  Data::Navigation::RawGPS rawData;
//  if (rawIn.read( rawData ) == RTT::NewData) {
//    Data::Navigation::GPS gpsData;

//    UbloxGPS->ProcessNavSol(&rawData);
//    UbloxGPS->GetNavSol(&gpsData);
//    gpsData.setTimestamp(rawData);
//    gps.write( gpsData );
//  }
  
  while(((GPSPacketType = UbloxGPS->ProcessGPSQueue()) != uBloxNoDataAvailable))
  {
    if (GPSPacketType == uBloxNavSolData) {
      Data::Navigation::GPS gpsData;
      
      UbloxGPS->GetNavSol(&gpsData);

      fix.header.stamp = port->getTimestamp();
      fix.latitude = gpsData.lat * 180.0/M_PI;
      fix.longitude = gpsData.lon * 180.0/M_PI;
      fix.altitude = gpsData.altitude;
      if (gpsData.signalQuality >= 3 && gpsData.numberOfSatellites >= 4)
        fix.status.status = fix.status.STATUS_FIX;
      else
        fix.status.status = fix.status.STATUS_NO_FIX;
      portFix.write(fix);

      velocity.header.stamp = port->getTimestamp();
      velocity.vector.x = gpsData.v_n;
      velocity.vector.y = -gpsData.v_e;
      velocity.vector.z = -gpsData.v_d;
      portVelocity.write(velocity);

//      Data::Navigation::RawGPS rawData;

//      UbloxGPS->GetGPSRaw(&rawData);
//      rawData.setTimestamp(port->getTimestamp());
//      rawOut.write( rawData );
    }
    
//    if (GPSPacketType == uBloxSvInfoData) {
//      Data::Navigation::SVInfo svinfoData;
      
//      UbloxGPS->GetSvInfo(&svinfoData);
//      svinfoData.setTimestamp(port->getTimestamp());
//      svinfo.write( svinfoData );
//    }
    
//    if ((GPSPacketType == uBloxDGPSData) || (GPSPacketType == uBloxSBASData)) {
//      Data::Navigation::SBASDGPS sbasdgpsData;
      
//      UbloxGPS->GetNavDGPSSBAS(&sbasdgpsData);
//      sbasdgpsData.setTimestamp(port->getTimestamp());
//      sbasdgps.write( sbasdgpsData );
//    }
  }
}

void GPS::stopHook() {
  RTT::Logger::In in(getName());
}

void GPS::cleanupHook() {
  RTT::Logger::In in(getName());

  if (port) port->close();

  delete UbloxGPS;
  UbloxGPS = 0;
}

bool GPS::configureDynModel(unsigned int dynModel) {
  return UbloxGPS->configureNavigationEngine(static_cast<TUBLOXGPS::DynModel>(dynModel));
}

bool GPS::configureFixMode(unsigned int fixMode) {
  return UbloxGPS->configureNavigationEngine(static_cast<TUBLOXGPS::FixMode>(fixMode));
}

ORO_CREATE_COMPONENT(GPS)

} // namespace Sensors
} // namespace uxvcos
