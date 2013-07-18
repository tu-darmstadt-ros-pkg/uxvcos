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
//! @file earthcalculations.cpp
//!
//! @author Frank Schmidt-Bruecken
//!
//! @version 1.1
//!
//! @date 23 July 2008
//! @update 16 June 2010 by Martin Nowara
//!

#include "EarthCalculations.h"

TEARTHCALCULATIONS::TEARTHCALCULATIONS()
{
}

TEARTHCALCULATIONS::~TEARTHCALCULATIONS()
{
}

void TEARTHCALCULATIONS::CalcLocalGravity(double pLat, double pHeight, double* pg_model)
{
    sin_Lat2 = pow(sin(pLat), 2);
    R_geoc = EARTH_R_ERDE * pow(1.0 - EARTH_E_ERDE * sin_Lat2,2);
    *pg_model = EARTH_G0 * (1.0 + EARTH_G1 * sin_Lat2) / sqrt(1.0 - EARTH_G2 * sin_Lat2);
    *pg_model = *pg_model / (1.0 + 2*pHeight / R_geoc);
}

void TEARTHCALCULATIONS::CalcLocalRadii(double pLat, double pHeight, double* pRmH, double* pRnH)
{
    sin_Lat2 = pow(sin(pLat), 2);
    hlp = sqrt(1.0 + EARTH_M_ERDE * sin_Lat2);
    Rm_erde = EARTH_R_ERDE * (1.0 + EARTH_M_ERDE) / pow(hlp, 3);
    Rn_erde = EARTH_R_ERDE / hlp;
    *pRmH  =  Rm_erde + pHeight;              // Nav_Radius Nord
    *pRnH  = (Rn_erde + pHeight) * cos(pLat); // Nav_Radius Ost
}
