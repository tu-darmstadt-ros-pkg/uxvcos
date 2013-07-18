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
