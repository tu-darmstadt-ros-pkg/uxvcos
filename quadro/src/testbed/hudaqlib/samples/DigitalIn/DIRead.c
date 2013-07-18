/* Humusoft data acquisition library.
 * Example that shows reading of digital input channels
 * using the function to read a single channel.
 */

/* Copyright 2002-2007 Humusoft s.r.o. */

#include <stdio.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;
  unsigned value;

  /* open a handle to the first MF624 device in the system */
  h = HudaqOpenDevice("PCT7303B", 1, 0);
//  h = HudaqOpenDevice("PCD7004", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device not found.\n");
    return(-1);
  }

  /* read whole digital channel at once */
  value = HudaqDIRead(h,0);
  printf("\nValue read from digital channel 0: %Xh ", value);

  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

