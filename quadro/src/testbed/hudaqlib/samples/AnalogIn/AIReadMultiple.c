/* Humusoft data acquisition library.
 *
 * Example that shows reading of analog input channels
 * using the function to read multiple channels together.
 */

/* Copyright 2002-2006 Humusoft s.r.o. */

#include <stdio.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;
  unsigned i;
        /* Buffer for channel numbers. Order of channels is not signifficant.
           Duplicated channels are also supported. */
  unsigned channels[8] = {4,5,6,7,0,1,2,3};
	/* Buffer for receiving values read. Is size must correspond to
	   buffer of channels. */
  double values[8];  	

  /* open a handle to the first MF624 device in the system */
  h = HudaqOpenDevice("MF624", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device not found.\n");
    return(-1);
  }

  /* read all the 8 analog inputs together */
  HudaqAIReadMultiple(h,8,channels,values);

  /* print values read */
  for (i=0; i<8; i++)
  {
    printf("Analog channel %d, value read %fV.\n", channels[i], values[i]);
  }

  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

