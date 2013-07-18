/* Humusoft data acquisition library.
 *
 * Example that shows using multiphase PWM output channel.
 * This example works on MF625 only!
 */

/* Copyright 2002-2007 Humusoft s.r.o. */

#include <stdio.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;
  unsigned phases[] =  {  0,  1,  2,  3,  4,  5};
  double duties[] =    {0.2,0.4,0.5,0.6,0.7,0.8};

double d[]= {1,1.05,1.1,1,1,1};

  /* open a handle to the first MF625 device in the system */
  h = HudaqOpenDevice("MF625", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device MF625 not found.\n");
    return(-1);
  }

  /* set the first PWM channel to frequency 2.5kHz with duty cycles 0.1 0.5 0.9 */
  HudaqPWMWriteMultiphase(h, 0, 6, phases, 2500, duties);

//HudaqSetParameter(h, 0, HudaqPwmCLOCKSOURCE, HudaqCtrCLOCKINFALLING);
//HudaqSetParameter(h, 0, HudaqPwmFILTER, 0);

HudaqPWMWriteMultiphase(h, 0, 6, phases, 12000000 /*0.005*/, d);

  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

