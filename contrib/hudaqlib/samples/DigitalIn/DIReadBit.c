/* Humusoft data acquisition library.
 * Example that shows reading of digital input channels using 
 * the function to read separate bits from a single channel.
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

  /* read all 8 bits from digital inputs in a loop, print their values */
  for(i=0; i<8; i++)        
  {
    value = HudaqDIReadBit(h,0,i);  /* Read one bit from digital input */
    printf("bit:%d, %d ", i, value);
  }                   
  printf("\n");

  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

