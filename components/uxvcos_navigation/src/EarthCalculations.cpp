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
