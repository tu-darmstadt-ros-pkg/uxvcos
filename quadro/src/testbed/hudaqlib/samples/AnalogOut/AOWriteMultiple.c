/* Humusoft data acquisition library.
 *
 * Example that shows writing to analog output channels
 * using the function to write multiple channels together.
 */

/* Copyright 2002-2006 Humusoft s.r.o. */

#include <stdio.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;
        /* Buffer for channel numbers. Order of channels is not signifficant.
           Duplicated channels are also supported. */
  unsigned channels[8] = {4,5,6,7,0,1,2,3};
        /* Buffer that contains values to be written.
	   Is size must correspond to buffer of channels. */
  double values[8] = {5.0, 6.0, 7.0, 8.0, 1.0, 2.0, 3.0, 4.0};

  /* Open a handle to the first MF624 device in the system. */
  h = HudaqOpenDevice("MF624", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device not found.\n");
    return(-1);
  }

  /* Write all the 8 analog outputs in one call. */
  if(HudaqAOWriteMultiple(h, 8, channels, values)==HUDAQSUCCESS)
  {
    printf("\nData has been written.\n");
  }

  /* Close the device handle. */
  HudaqCloseDevice(h);

  return(0);
}

