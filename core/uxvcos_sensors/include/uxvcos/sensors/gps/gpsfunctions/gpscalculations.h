//
// -*- c++ -*-
//
// $Id: gpscalculations.h, v 1.0 2006/02/28 08:00:00 Schmidt-Bruecken Exp $
//

//!
//! @file gpscalculations.h
//!
//! @brief Class with functions for coordinate-system-manipulations
//!
//! @author Oliver Mittenzwei, Frank Schmidt-Bruecken
//!
//! @version 1.0
//!
//! @date 28 February 2006
//!


#ifndef _GPS_CALCULATIONS_H
#define _GPS_CALCULATIONS_H

#include <math.h>



// ---------------------------------------------------------
// GPS
// ---------------------------------------------------------
//! @{
//! @name GPS
#define GPS_SEMI_MAJOR_RADIUS   6378137.0000            //!< Earth semi-major axis
#define GPS_SEMI_MINOR_RADIUS   6356752.3142            //!< Earth semi-minor axis
#define GPS_SEMI_MINOR_RADIUS_KURZ  6356752.314         //!< Earth semi-minor axis

//! @}



//! @brief Class with functions for coordinate-system-manipulations
class TGpsCalculations
{
public:

    //! @brief Create and initialize the object
    TGpsCalculations(void);

    //! @brief Destroy the object
    virtual ~TGpsCalculations();

    //! @brief Convert ECEF data to LLA position
    //! @param x ECEF x-coordinate [m]
    //! @param y ECEF y-coordinate [m]
    //! @param z ECEF z-coordinate [m]
    //! @param lat [rad] return
    //! @param lon [rad] return
    //! @param alt [m] return
    static void ECEF2LLA(double pX, double pY, double pZ,
                         double *pLat, double *pLon, double *pAlt);

    //! @brief Convert velocity data from the ECEF to the
    //! North/East/Down coordinate system.
    //! @param lat Latitude [rad]
    //! @param lon Longitude [rad]
    //! @param x ECEF x-coordinate [m]
    //! @param y ECEF y-coordinate [m]
    //! @param z ECEF z-coordinate [m]
    //! @param vel_north [m/s] return
    //! @param vel_east  [m/s] return
    //! @param vel_down [m/s] return
    static void ECEF2NED(double pLat, double pLon, double pX, double pY, double pZ,
                         double *pvel_north, double *pvel_east, double *pvel_down);

    /*
      //! @brief Convert ECEF data to LLA position
      //! @param x ECEF x-coordinate [m]
      //! @param y ECEF y-coordinate [m]
      //! @param z ECEF z-coordinate [m]
      //! @return Vector containing lat/lon/alt [rad, rad, m]
      static Matrix ECEF2LLA(double pX, double pY, double pZ);

      //! @brief Convert velocity data from the ECEF to the
      //! North/East/Down coordinate system.
      //! @param lat Latitude [rad]
      //! @param lon Longitude [rad]
      //! @param x ECEF x-coordinate [m]
      //! @param y ECEF y-coordinate [m]
      //! @param z ECEF z-coordinate [m]
      //! @return Vector containing NED velocity
      static Matrix ECEF2NED(double pLat, double pLon, double pX, double pY, double pZ);
    */

    /*
      //! @brief Convert GPS utc date and time to the fractional year format
      //!
      //! This function converts the current GPS utc date and time
      //! to the fractional year format needed by the WMM object.
      //! The integer part of the result represents the current year
      //! (e.g. 2005), while the fractional part represents the
      //! percentage of the year that has passed already.
      //! @param week GPS number of weeks since 80/01/06
      //! @param itow Time passed since beginning of the current week
      //! @return Fractional year
      static TScalar GPSTime2FracYear(
        unsigned long int week,
        unsigned long int itow);
    */
};

#endif
