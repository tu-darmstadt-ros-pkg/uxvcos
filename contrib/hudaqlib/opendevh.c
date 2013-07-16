/******************************************************************/
/**@file opendev.c:
 * Desctiption: Humusoft data acquisition Linux library
                implementation.                                   *
 * Dependency: Linux                                              *
 *                Copyright 2006-2007 Jaroslav Fojtik             *
 ******************************************************************/
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <dirent.h>

#include "hudaqlib.h"
#include "hudaq_internal.h"


static int sysfs_get_value(const char *FileName)
{
FILE *F;
int RetVal;

  F=fopen(FileName,"rb");
  if(F==NULL) return -1;

  fscanf(F,"%X",&RetVal);
  fclose(F);
return RetVal;
}

static int ScanSys(DWORD DeviceID, int deviceorder, DriverRecord *Rec)
{
DIR *dir;
FILE *F;
char ch;
struct dirent *entry;
unsigned vendor_id;
unsigned device_id;
char FileName[1024];
const char *SysDir="/sys/bus/pci/devices";
unsigned long long start, end, size;
DWORD Resources[14];
int i;
int RetVal=0;

  if(deviceorder <= 0) return -1;
  if(Rec==NULL) return -2;
  
  Rec->NumPhysMemResources = Rec->Resources.NumIOResources = 0;

  sprintf(FileName,"%s",SysDir);
  dir = opendir(FileName);
  if(!dir)
    {
    //fprintf(stderr,"Cannot open /sys/bus/pci/devices");
    return -3;
    }
    
  while ((entry = readdir(dir)))
    {
    if(entry->d_name[0]=='.') continue;

    sprintf(FileName,"%s/%s/vendor",SysDir,entry->d_name);
    vendor_id = sysfs_get_value(FileName);
    sprintf(FileName,"%s/%s/device",SysDir,entry->d_name);
    device_id = sysfs_get_value(FileName);

    memset(Resources, 0, sizeof(Resources));    
    sprintf(FileName,"%s/%s/resource",SysDir,entry->d_name);
    F=fopen(FileName,"rb");
    if(F)
      {
      for(i=0;i<7;i++)
        {
        if(fscanf(F,"%llx %llx", &start, &end) != 2)
	  {
	  fprintf(stderr,"Syntax error in %s", FileName);
	  break;
	  }
	do
	  {  
	  ch=fgetc(F);
          if(feof(F)) break;
          } while(ch!='\r' && ch!='\n' && ch!=0);  
	  
        if(start)
	  size = end - start + 1;
        else
	  size = 0;
	  
	Resources[i]=start;
	Resources[i+7]=size;
        }
      fclose(F);
      }

    if(DeviceID==(device_id+(vendor_id<<16)))
      {
      //printf("DEVICE HAS BEEN FOUND!");
      if(--deviceorder<1)
        {
        for(i=0;i<7;i++)
          {
          if(Resources[i]!=0)
            {
            if(Resources[i] > 0xFFFF)
               {
               Rec->PhysMemResources[Rec->NumPhysMemResources].Base=Resources[i];
               Rec->PhysMemResources[Rec->NumPhysMemResources].Length=Resources[i+7];
               Rec->NumPhysMemResources++;
               }
            else
               {
               Rec->Resources.IOResources[Rec->Resources.NumIOResources].Base=Resources[i];
               Rec->Resources.IOResources[Rec->Resources.NumIOResources].Length=Resources[i+7];
               Rec->Resources.NumIOResources++;
               }
            }
          }
        RetVal=1;    //device has been found!!
        break;
        }
      }    
    
//  printf("%X %X %s\n", vendor_id, device_id, entry->d_name);
    }
    
  closedir(dir);    
  return RetVal;
}


static int ScanProc(DWORD DeviceID, int deviceorder, DriverRecord *Rec)
{
FILE *devices;
DWORD DevTblID;
DWORD DevPos, DevInt;
DWORD Resources[14];
int i;
char ch;
int RetVal=0;

  if(deviceorder <= 0) return -1;
  if(Rec==NULL) return -2;
  
  Rec->NumPhysMemResources = Rec->Resources.NumIOResources = 0;

  //devices=fopen("devices","rb");
  devices=fopen("/proc/bus/pci/devices","rb");
  if(devices==NULL) return -4;

    /* loop over devices */
  while (!feof(devices))
    {
    if(fscanf(devices,"%X %X %X", &DevPos, &DevTblID, & DevInt)!=3)
      return -5;
      
    memset(Resources, 0, sizeof(Resources));
    for(i=0; i<14; i++)
      {
      if(fscanf(devices,"%X", &Resources[i])!=1)
        {
	if(i==6 || i==7) break;  //correct
	break; //strange
	}
      }
    do {    
       ch=fgetc(devices);
       if(feof(devices)) break;
       } while(ch!='\r' && ch!='\n' && ch!=0);

    if(DeviceID==DevTblID)
      {
      if(--deviceorder<1)
        {
        for(i=0;i<7;i++)
          {
          if(Resources[i]!=0)
            {
            if(Resources[i] & 1)
               {
	       Rec->Resources.IOResources[Rec->Resources.NumIOResources].Base = Resources[i] & ~1;
               Rec->Resources.IOResources[Rec->Resources.NumIOResources].Length=Resources[i+7];
               Rec->Resources.NumIOResources++;
               }
            else
               {
	       Rec->PhysMemResources[Rec->NumPhysMemResources].Base=Resources[i];
               Rec->PhysMemResources[Rec->NumPhysMemResources].Length=Resources[i+7];
               Rec->NumPhysMemResources++;
               }
            }
          }
        RetVal=1;    //device has been found!!
        break;
        }
      }
    }

  fclose(devices);
return(RetVal);
}





/*****************************************************************************
;*
;*              OpenDeviceHandle
;*              opens device handle by device name and order
;*
;*              Input:  device name
;*                      device order number
;*              Output: positive number when success; 0 or negative when fails.
;*
;****************************************************************************/
int OpenDeviceHandle(const char *devicename, int deviceorder, DriverRecord *Rec)
{
DWORD DeviceID=0;
int RetVal;

  if(deviceorder <= 0) return -1;
  if(Rec==NULL) return -2;

    /* Humusoft */
#ifdef MF61X
  if(!strcmp(devicename,"AD612")) DeviceID=0x186C0612;
  if(!strcmp(devicename,"MF614")) DeviceID=0x186C0614;
#endif
#ifdef MF62X
  if(!strcmp(devicename,"AD622")) DeviceID=0x186C0622;
  if(!strcmp(devicename,"MF624")) DeviceID=0x186C0624;
  if(!strcmp(devicename,"MF625")) DeviceID=0x186C0625;
#endif  
      
    /* Advantech */
#ifdef PCI1753
  if(!strcmp(devicename,"PCI1753")) DeviceID=0x13FE1753;
#endif

    /* Tedia */  
#ifdef PCD7004
  if(!strcmp(devicename,"PCD7004")) DeviceID=0x17600101;
#endif
#ifdef PCT7303B
  if(!strcmp(devicename,"PCT7303B")) DeviceID=0x17600201;
#endif

  //printf("\n%s deviceID = %X", devicename, DeviceID);
    
  if(DeviceID==0) return -3;

  RetVal = ScanSys(DeviceID, deviceorder, Rec);
  if(RetVal>0) return RetVal;
  RetVal = ScanProc(DeviceID, deviceorder, Rec);
  if(RetVal>0) return RetVal;  

return(RetVal);
}


