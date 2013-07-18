/* Humusoft data acquisition library.
 *
 * Example that shows using of 3 phase PWM output channels.
 * This example works on MF625 only!
 */

/* Copyright 2002-2007 Humusoft s.r.o. */

#include <stdio.h>
#include <windows.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;
  double value;

  /* open a handle to the first MF625 device in the system */
  h = HudaqOpenDevice("MF625", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device MF625 not found.\n");
    return(-1);
  }

  /* set the first PWM channel to frequency 1.5kHz with duty cycles 0.1 0.5 0.9 */
  HudaqPWM3Write(h, 0, 1500, 0.5, 0.5, 0.9);

value=0;
while(1)
{
  HudaqSetParameter(h, 0, HudaqPwmDEADBAND, 1e-5);
  HudaqPWM3Write(h, 0, 1500, value, 1-value, value);
  value+=5e-4;
  if(value>1) value=0;
  Sleep(1);
}


  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

