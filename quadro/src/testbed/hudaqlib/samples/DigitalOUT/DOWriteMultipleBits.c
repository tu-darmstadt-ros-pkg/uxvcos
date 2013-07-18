/* Humusoft data acquisition library.
 *
 * Example that demonstrates using HudaqDOWriteMultipleBits.
 * This function allows to influence only selected bits from
 * digital outputs.
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

  /* HudaqOpenDevice initializes all digital output bits to 0 */

  /* Set first and fifth bits to value '1'. */
  HudaqDOWriteMultipleBits(h, 0, 0x11, 0x11);
  printf("\nBits 1 and 5 are set. Press any key to continue.");
  getchar();

  /* Reset bit 1 and set bit 6. */
  HudaqDOWriteMultipleBits(h, 0, 0x21, 0x20);
  printf("\nBit 1 is reset and bit 6 is set. Press any key to continue.");
  getchar();


  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

