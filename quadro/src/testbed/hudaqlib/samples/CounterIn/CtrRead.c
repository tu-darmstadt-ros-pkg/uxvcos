/* Humusoft data acquisition library.
 *
 * Example that shows reading of counters and counting pulses.
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

  /* Do reset of counters. Each counter is switched to counting mode
     after its first usage. */
  for (i=0; i<4; i++)
  {
    HudaqCtrReset(h,i);
  }

  printf("Counting external pulses by counters, press Enter to continue.\n", i, value);
  getchar();

  /* read all the 4 counters in a loop, print their values */
  for (i=0; i<4; i++)
  {
    value = HudaqCtrRead(h,i);
    printf("Counter channel %d, value read %d.\n", i, value);
  }

  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

