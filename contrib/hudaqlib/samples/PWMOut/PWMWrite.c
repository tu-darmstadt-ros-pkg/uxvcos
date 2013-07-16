/* Humusoft data acquisition library.
 *
 * Example that shows using of PWM output channels.
 */

/* Copyright 2002-2007 Humusoft s.r.o. */

#include <stdio.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;
  double value;

  /* open a handle to the first MF624 device in the system */
  h = HudaqOpenDevice("MF624", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device not found.\n");
    return(-1);
  }

  /* set first PWM channel to frequency 1.5kHz with duty cycle 0.5 */
  HudaqPWMWrite(h,0,1500,0.5);

  /* set second PWM channel to frequency 2.5kHz with duty cycle 0.75 */
  HudaqPWMWrite(h,1,2500,0.75);

  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

