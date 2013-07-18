//
// -*- c++ -*-
//
// $Id: nav_gps.cpp, v 1.0 2006/02/28 08:00:00 Schmidt-Bruecken Exp $
//

//!
//! @file gpscalculations.cpp
//!
//! @author Oliver Mittenzwei, Frank Schmidt-Bruecken
//!
//! @version 1.0
//!
//! @date 28 February 2006
//!


#include "gpscalculations.h"


TGpsCalculations::TGpsCalculations()
{

}

TGpsCalculations::~TGpsCalculations()
{

}

/*
Matrix TGpsCalculations::ECEF2LLA(double pX, double pY, double pZ)
{
  const double a = GPS_SEMI_MAJOR_RADIUS;         // Semi major axis radius
  const double b = GPS_SEMI_MINOR_RADIUS_KURZ;    // Semi minor axis radius

  const double e = sqrt(pow(a,2.0)  - pow(b,2.0)) / a;  // Earth eccentricy
  const double p = sqrt(pow(pX,2.0) + pow(pY,2.0));     // Helper
  double N;                                       // Helper
  double lat;                                     // Latitude [rad]
  double lon;                                     // Longitude [rad]
  double alt;                                     // Altitude [m]
  double l0;                                      // Initial latitude estimate [rad]
  Matrix result(3,1);

//  e = sqrt(pow(a,2.0)  - pow(b,2.0)) / a;         // Needs conversion to float!
//  p = sqrt(pow(pX,2.0) + pow(pY,2.0));
  lon = atan2(pY, pX);
  lat = atan2(pZ, (p * (1.0 - pow(e,2.0))));
  do
  {
    l0 = lat;
    N  = pow(a,2.0) / sqrt(pow(a * cos(l0),2.0) + pow(b * sin(l0),2.0));
    alt = p / cos(l0) - N;
    lat = atan2(pZ, (p * (1.0 - pow(e,2.0) * N / (N + alt))));
  } while  (fabs(l0-lat) >= 1E9);    //  until abs(l0-lat)<1E9;
  result.setup_vector_3_1(lat, lon, alt);

  return result;
}

Matrix TGpsCalculations::ECEF2NED(double pLat, double pLon, double pX, double pY, double pZ)
{
  double slat;                                    // Sine of the latitude
  double clat;                                    // Cosine of the latitude
  double slon;                                    // Sine of the longitude
  double clon;                                    // Cosine of the longitude
  Matrix result(3,1);

  slat = sin(pLat);
  slon = sin(pLon);
  clat = cos(pLat);
  clon = cos(pLon);
  result.setup_vector_3_1(-pX * clon * slat - pY * slon * slat + pZ * clat,
    -pX * slon + pY * clon,
    -pX * clon * clat - pY * slon * clat - pZ * slat);

  return result;
}
*/

void TGpsCalculations::ECEF2LLA(double pX, double pY, double pZ,
                                double *pLat, double *pLon, double *pAlt)

{
    const double a = GPS_SEMI_MAJOR_RADIUS;         // Semi major axis radius
    const double b = GPS_SEMI_MINOR_RADIUS_KURZ;    // Semi minor axis radius

    const double e = sqrt(pow(a,2.0)  - pow(b,2.0)) / a;  // Earth eccentricy
    const double p = sqrt(pow(pX,2.0) + pow(pY,2.0));     // Helper
    double N;                                       // Helper
//  double lat;                                     // Latitude [rad]
//  double lon;                                     // Longitude [rad]
//  double alt;                                     // Altitude [m]
    double l0;                                      // Initial latitude estimate [rad]

//  e = sqrt(pow(a,2.0)  - pow(b,2.0)) / a;         // Needs conversion to float!
//  p = sqrt(pow(pX,2.0) + pow(pY,2.0));
    *pLon = atan2(pY, pX);
    *pLat = atan2(pZ, (p * (1.0 - pow(e,2.0))));
    do
    {
        l0 = *pLat;
        N  = pow(a,2.0) / sqrt(pow(a * cos(l0),2.0) + pow(b * sin(l0),2.0));
        *pAlt = p / cos(l0) - N;
        *pLat = atan2(pZ, (p * (1.0 - pow(e,2.0) * N / (N + *pAlt))));
    } while  (fabs(l0- *pLat) >= 1E9);    //  until abs(l0-lat)<1E9;
}

void TGpsCalculations::ECEF2NED(double pLat, double pLon, double pX, double pY, double pZ,
                                double *pvel_north, double *pvel_east, double *pvel_down)
{
    double slat;                                    // Sine of the latitude
    double clat;                                    // Cosine of the latitude
    double slon;                                    // Sine of the longitude
    double clon;                                    // Cosine of the longitude

    slat = sin(pLat);
    slon = sin(pLon);
    clat = cos(pLat);
    clon = cos(pLon);
    *pvel_north = -pX * clon * slat - pY * slon * slat + pZ * clat;
    *pvel_east  = -pX * slon + pY * clon;
    *pvel_down  = -pX * clon * clat - pY * slon * clat - pZ * slat;
}

/*
TScalar GpsCalculations::GPSTime2FracYear(
                                          unsigned long int week,
                                          unsigned long int itow)
{
  QDate               date(1980,1,6);   // january, 6th 1980

  date = date.addDays(week * 7 + (unsigned int)(itow / 1000.0 / 3600.0 / 24.0));

  return date.year() + ( (TScalar)date.dayOfYear() / (TScalar)date.daysInYear() );
}
*/

