/* Humusoft data acquisition library.
 *
 * Example that shows configuration of
 * encoder.
 */

/* Copyright 2002-2007 Humusoft s.r.o. */

#include <stdio.h>

#include "hudaqlib.h"


int main(int argc, char* argv[])
{
  HUDAQHANDLE h;
  int value;

  /* open a handle to the first MF624 device in the system */
  h = HudaqOpenDevice("MF624", 1, 0);
  if (h==0)
  {
    printf("\nData acquisition device not found.\n");
    return(-1);
  }

  /* Configure encoder to count input pulses. */  
  if(HudaqSetParameter(h, 0, HudaqEncMODE, HudaqEncMODERISING) != HUDAQSUCCESS)
  {
    printf("\nCannot switch encoder to counting mode.\n");
    HudaqCloseDevice(h);
    return(-2);
  }

  /* Turn on hardware filter for input signal. */  
  if(HudaqSetParameter(h, 0, HudaqEncFILTER, 1) != HUDAQSUCCESS)
  {
    printf("\nCannot filter input signal.\n");
  }      

  printf("Counting external pulses on input A by encoder, press Enter to continue.\n");
  getchar();

  /* Read encoder 0 value, print it. */
  value = HudaqEncRead(h,0);
  printf("Encoder channel 0, value read %d.\n", 0, value);
  
  /* close the device handle */
  HudaqCloseDevice(h);

  return(0);
}

