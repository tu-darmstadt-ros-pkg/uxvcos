/* Humusoft data acquisition library.
 * Example that shows writing to analog output channels
 * using the function to write a single channel.
 */

/* Copyright 2002-2007 Humusoft s.r.o. */

#include <stdio.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;
  unsigned i;
  double value;

  /* open a handle to the first MF624 device in the system */
  h = HudaqOpenDevice("MF624", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device not found.\n");
    return(-1);
  }

  /* write all the 8 analog outputs in a loop */
  /* the voltage written to the output is computed as (channel number - 5) */
  for (i=0; i<8; i++)
  {
    value = i-5.0;
    HudaqAOWrite(h, i, value);
    printf("Analog output channel %d, value written %fV.\n", i, value);
  }

  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

