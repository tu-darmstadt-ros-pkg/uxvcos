/* Humusoft data acquisition library.
 *
 * Example that shows writing separate bits to a digital output channel
 * using the function to write a single bit.
 */

/* Copyright 2002-2006 Humusoft s.r.o. */

#include <stdio.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;
  unsigned i;  

  /* open a handle to the first MF624 device in the system */
  h = HudaqOpenDevice("MF624", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device not found.\n");
    return(-1);
  }

  /* HudaqOpenDevice initialises all digital output bits to 0 */
  for(i=0; i<8; i++)
  {
    printf("\nPress any key to set a bit %d to '1'", i);
    getchar();
    HudaqDOWriteBit(h, 0, i, 1);
  }

  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

