//!
//! @file EarthCalculations.h
//!
//! @author Frank Schmidt-Bruecken
//!
//! @version 1.1
//!
//! @date 23 July 2008
//! @update 16 June 2010 by Martin Nowara
//!

#ifndef _EARTHCALCULATIONS_H
#define _EARTHCALCULATIONS_H

#include <math.h>

//! @brief Class with functions for earth calculations
class TEARTHCALCULATIONS
{
public:
  #define EARTH_R_ERDE  6.378137e+6      // Aequatorradius [m]
  #define EARTH_W_ERDE  7.2921151467e-5  // Erddrehrate [rad/sec]
  #define EARTH_E_ERDE  3.3528106647e-3  // Elliptizitaet Erde
  #define EARTH_M_ERDE -6.694379990e-3   // m_erde = (e-2)e
  #define EARTH_G0      9.7803267714     // Aequator [m/sec^2]
  #define EARTH_G1      0.00193185138639 // Parameter (WGS 84)
  #define EARTH_G2      0.00669437999013 // Parameter (WGS 84)

  //! @brief Create and initialize the object
  TEARTHCALCULATIONS(void);

  //! @brief Destroy the object
  virtual ~TEARTHCALCULATIONS();

  void CalcLocalGravity(double Lat, double pHeight, double* pLocalGravity);
  void CalcLocalRadii(double Lat, double pHeight, double* pRmH, double* pRnH);

private:
  double R_geoc;
  double Rm_erde, Rn_erde;

  double sin_Lat2;
  double hlp;
};

#endif
