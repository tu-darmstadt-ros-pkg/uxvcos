/* DaWrite.c:
 *   This demo demonstrates extract information from all available devices.  
 */
#include <stdio.h>
#include "hudaqlib.h"


int main(void)
{
HUDAQHANDLE h;
const HudaqResourceInfo *HRI;
int i,j;
double value;
int NoAnalogIn,NoDigitalIn,NoEncoders;
int dev=1;

	/* Open first device found of any name. */
  h = HudaqOpenDevice("",1,0);
  if(h==0)
    {printf("No HUDAQ device found\n"); return -1;}

  while(h!=0)
  {
    HRI = HudaqGetDeviceResources(h);
    printf("\nBus number %d, Slot number %d.",HRI->BusNumber, HRI->SlotNumber);
    printf("\nVendorID %Xh, DeviceID %Xh.",HRI->VendorID,HRI->DeviceID);

    for(i=0;i<HRI->NumMemResources;i++)
    {
      printf("\n Memory resource %d: Base:%Xh, Length:%Xh",
          i, HRI->MemResources[i].Base, HRI->MemResources[i].Length);
    }

    for(i=0;i<HRI->NumIOResources;i++)
    {
      printf("\n IO resource %d: Base:%Xh, Length:%Xh",
        i, HRI->IOResources[i].Base, HRI->IOResources[i].Length);
    }

    NoAnalogIn = HudaqGetParameter(h,0,HudaqAINUMCHANNELS);
    printf("\nAnalog channels %d", NoAnalogIn);
    for (i=0; i<NoAnalogIn; i++)
    {
      value = HudaqAIRead(h,i);
      printf("\n  Analog channel %d, value read %fV.", i, value);
    }

    NoDigitalIn = HudaqGetParameter(h,0,HudaqDINUMCHANNELS);
    printf("\nDigital channels %d", NoDigitalIn);
    for (i=0; i<NoDigitalIn; i++)
    {
      printf("\n  Digital input %d: %d",i,HudaqDIRead(h,i));
    }

    NoEncoders = HudaqGetParameter(h,0,HudaqEncNUMCHANNELS);
    printf("\nEncoder channels %d", NoEncoders);
    for (i=0; i<NoEncoders; i++)
    {
      printf("\n  Encoder value %d: %d",i,HudaqEncRead(h,i));
    }


    printf("\nIRQ counter: %g (%g)  press any key to continue...",
	     HudaqGetParameter(h,0,HudaqIRQ), HudaqGetParameter(h, 0, HudaqIRQ+1));

    HudaqCloseDevice(h);
    h = HudaqOpenDevice("",++dev,0);
  }

  //  HudaqSetParameter(h, 0, HudaqIRQ, 1);
  i = HudaqGetParameter(h, 0, HudaqIRQ);
  printf("\n");
return 0;
}
