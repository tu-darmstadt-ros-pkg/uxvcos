/* DaWrite.c:
 *   This demo demonstrates how to list devices.  
 */
#include <malloc.h>
#include <string.h>
#include <stdio.h>

#include "hudaqlib.h"


typedef struct
{
  char *DevName;
  int DevOrder;
} DeviceStruct;


int ListDevices(DeviceStruct *D)
{
int index=0;
int i;
HUDAQHANDLE h;

  do
  {
    h = HudaqOpenDevice("", index+1, HudaqOpenNOINIT);  //Open a device
    if(h==0) break;

    D[index].DevName = strdup(HudaqGetBoardName(h));
    D[index].DevOrder=1;

    for(i=0;i<index;i++) 
      if(!strcmp(D[index].DevName,D[i].DevName)) D[index].DevOrder++;
    HudaqCloseDevice(h);

    index++;   
  } while(h!=0);

return index;
}


int main(int argc, char* argv[])
{
int i, count;
char ch;
HUDAQHANDLE h;
DeviceStruct ds[10];

  count = ListDevices(ds);
  if(count<=0) {printf("\nNo Hudaqdevice found");return(-1);}

  if(count>1) 
    {
    for(i=0;i<count;i++)
      printf("\nPlease hit %c to choose %s card [%d]:",i+'A',ds[i].DevName,ds[i].DevOrder);
    printf("\n");
    i=toupper(getchar());
    i-='A';
    if (i<0 || i>=10) return;
    }
  else
    i=0;

  h = HudaqOpenDevice(ds[i].DevName,ds[i].DevOrder, HudaqOpenNOINIT);
  if(h==0) 
    return -3;			       /* Device cannot be openned. */    


  printf("\nDevice [%s(%d)] has been succesfully opened.", ds[i].DevName, ds[i].DevOrder);
  
  HudaqCloseDevice(h);                 /* Close handle */
    
return 0;
}

