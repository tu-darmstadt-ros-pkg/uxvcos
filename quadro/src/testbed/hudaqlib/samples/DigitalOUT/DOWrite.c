/* Humusoft data acquisition library.
 * Example that shows writing all bits to a digital output channels
 * using the function to write a single channel.
 */

/* Copyright 2002-2007 Humusoft s.r.o. */

#include <stdio.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;

  /* open a handle to the first MF624 device in the system */
  h = HudaqOpenDevice("MF624", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device not found.\n");
    return(-1);
  }

  /* write 0xFF to whole digital channel at once */
  HudaqDOWrite(h,0,0xFF);

  printf("\n0xFF was writen to digital output. Press any key to continue.");
  getchar();

  /* write 0x00 to whole digital channel at once */
  HudaqDOWrite(h,0,0x0);
  printf("\n0x00 has been writen to digital output.");

  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

