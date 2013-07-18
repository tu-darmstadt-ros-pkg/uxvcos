/* Humusoft data acquisition library.
 * Example that shows reading of analog input channels
 * using the function to read a single channel.
 */

/* Copyright 2002-2006 Humusoft s.r.o. */

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

  /* read all the 8 analog inputs in a loop, print their values */
  for (i=0; i<8; i++)
  {
    value = HudaqAIRead(h,i);
    printf("Analog channel %d, value read %fV.\n", i, value);
  }

  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

