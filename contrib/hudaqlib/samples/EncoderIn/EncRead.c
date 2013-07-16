/* Humusoft data acquisition library.
 *
 * Example that shows decoding IRC position 
 * using the function to read a single encoder.
 */

/* Copyright 2002-2006 Humusoft s.r.o. */

#include <stdio.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;
  unsigned i;
  int value;

  /* open a handle to the first MF624 device in the system */
  h = HudaqOpenDevice("MF624", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device not found.\n");
    return(-1);
  }

  printf("Counting external IRC pulses by encoders, press Enter to continue.\n");
  getchar();

  /* Read all the 4 encoder values in a loop, print them. */
  for (i=0; i<4; i++)
  {
    value = HudaqEncRead(h,i);
    printf("Encoder channel %d, value read %d.\n", i, value);
  }

  /* Close the device handle. */
  HudaqCloseDevice(h);

  return(0);
}

