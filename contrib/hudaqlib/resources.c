/******************************************************************/
/**@file resources.c:
 * Desctiption: Humusoft data acquisition Linux library
                implementation.                                   *
 * Dependency: Linux                                              *
 *                Copyright 2006-2008 Jaroslav Fojtik             *
 ******************************************************************/
#include <stdio.h>
#include <malloc.h>

#include <sys/mman.h>
#include <sys/io.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <byteswap.h>
#include <inttypes.h>
#include <time.h>

#include "hudaqlib.h"
#include "hudaq_internal.h"


#ifdef _MSC_VER
 #pragma comment(lib, "setupapi")
#endif //_MSC_VER


int IOPLsUsed = 0;

/** Calltables for all available devices. */
const CallTable * const AllCTs[] = 
{
#ifdef MF61X
  &CT612, &CT614,
#endif
#ifdef MF62X
  &CT622, &CT624, &CT625,
#endif
#ifdef PCI1753
  &CTPCI1753,
#endif
#ifdef PCD7004
  &CTPCD7004,
#endif
#ifdef PCT7303B
  &CTPCT7303,
#endif
};

int OpenDeviceHandle(const char* devicename, int deviceorder, DriverRecord *Rec);


static uintptr_t device_mmap_range(int fd, uintptr_t mem_start, size_t mem_length)
{
  size_t pagesize;
  unsigned char *mm;
  unsigned char *ptr;

  pagesize=getpagesize();
  mem_length+=(mem_start & (pagesize-1))+pagesize-1;
  mem_length&=~(pagesize-1);

  mm = mmap(NULL, mem_length, PROT_WRITE|PROT_READ,
              MAP_SHARED, fd, mem_start & ~(pagesize-1));

  if ((mm == NULL) || (mm == (unsigned char*)-1)) {
    fprintf(stderr,"\nHudaqlib: mmap FAILED for range 0x%08lx length 0x%08lx\n",
            (long)mem_start,(long)mem_length);
    return 0;
  }
  ptr=mm + (mem_start & (pagesize-1));

  //fprintf(stderr,"mmap OK for range 0x%08lx length 0x%08lx -> 0x%08lx\n",
  //            (long)mem_start,(long)mem_length,(long)ptr);

  return  (uintptr_t)ptr;
}

static void device_unmmap_range(uintptr_t mem_start, size_t mem_length)
{
  size_t pagesize;
  unsigned char *mm;
  
  pagesize=getpagesize();
  mem_length+=(mem_start & (pagesize-1))+pagesize-1;
  mem_length&=~(pagesize-1);
  
  mm = (mem_start & ~(pagesize-1));

  if(munmap(mm, mem_length)!=0)
    printf("\nHudaqlib: unmmap failed.");
}


/*********************************
 *********************************/
int DeviceIoControl(int Operation, DriverRecord *Rec)
{
int i,ret;
int fd;
const char *memdev="/dev/mem";

  if(Rec==NULL) return 0;
  
  if(Operation) //Allocate resources
    {
    fd = open(memdev, O_RDWR|O_SYNC);
    if (fd >= 0)
      {    
      for(i=0;i<Rec->NumPhysMemResources;i++)
        {
         Rec->Resources.MemResources[i].Base=
	    device_mmap_range(fd, Rec->PhysMemResources[i].Base, Rec->PhysMemResources[i].Length);
        /*	    
         Rec->Resources.MemResources[i].Base = //Rec->PhysMemResources[i].Base;
	    (size_t)mmap(0,
	         Rec->PhysMemResources[i].Length,
	         PROT_WRITE|PROT_READ,
                 MAP_SHARED, 
		 fd,
		 Rec->PhysMemResources[i].Base); */        		 
        Rec->Resources.MemResources[i].Length =
          Rec->PhysMemResources[i].Length;
        Rec->Resources.NumMemResources++;
        }	    
      close(fd);
      }

    for(i=0;i<Rec->Resources.NumIOResources;i++)
      {       /*baze, delka useku, 1=povoleni 0=zakazani*/
      //printf("IOPERM %X %X",Rec->Resources.IOResources[i].Base,Rec->Resources.IOResources[i].Length);
      if(Rec->Resources.IOResources[i].Base+Rec->Resources.IOResources[i].Length<=0x3FF)
        {
        ret = ioperm(Rec->Resources.IOResources[i].Base,
                     Rec->Resources.IOResources[i].Length, 1);
        }
      else        
        {
        iopl(3);
        IOPLsUsed++;
        }
      }
    return 1;
    }
  else          //free resources
    {
    for(i=0;i<Rec->Resources.NumMemResources;i++)
      {
      device_unmmap_range((void *)Rec->Resources.MemResources[i].Base,
                          Rec->Resources.MemResources[i].Length);
      }
    Rec->Resources.NumMemResources=0;
    
    for(i=0;i<Rec->Resources.NumIOResources;i++)
      {
      /* !!!!!!!!!!! BUG, scan all allocated ranges prevent disabling already used space. !!!!!!!!!!! */
      if(Rec->Resources.IOResources[i].Base+Rec->Resources.IOResources[i].Length<=0x3FF)
        {      /*base, IO length, 1=enable 0=disable*/
        ret = ioperm(Rec->Resources.IOResources[i].Base,
                     Rec->Resources.IOResources[i].Length, 0);
        }
      else 
        {
        if(IOPLsUsed>0) IOPLsUsed--;
        if(IOPLsUsed<=0) iopl(0);
        }
      }
    return 1;
    }
}



/*********************************
 *********************************/

HUDAQSTATUS InternalResetDevice(DeviceRecord *DevRecord, int ShareFlag)
{
int Iter;

  if(DevRecord==NULL) return HUDAQFAILURE;

    /* Traverse a list with supported devices. A PCI ID would be much more better. */
  for(Iter=0; Iter<(sizeof(AllCTs)/sizeof(CallTable*));Iter++)
    {
    //printf("\nTry to match: %s %s",AllCTs[Iter]->DeviceString,DevRecord->Name);
    if(!strcmp(AllCTs[Iter]->DeviceString,DevRecord->Name))
      {
      if(AllCTs[Iter]->HudaqInit(DevRecord, ShareFlag) > 0)
        return HUDAQSUCCESS;
      else
        return HUDAQFAILURE;
      }
    }

     // unsupported device
  DevRecord->pCT = &CtDummy;
  return HUDAQPARTIAL;
}


/** Opens a given device. */
HUDAQHANDLE HUDAQAPI HudaqOpenDevice(const char* devicename, int deviceorder, int shareflag)
{
  DeviceRecord* devrecord;

  if (devicename==NULL) return(0);  /* Invalid device name. */

  /* allocate memory for the device record */
  devrecord = calloc(1, sizeof(DeviceRecord));
  if (devrecord==0)
    return(0);

  /* copy device name and order */
  devrecord->DrvRes.Size = sizeof(DriverRecord);
  strncpy(devrecord->Name, devicename, sizeof(devrecord->Name)-1);
  devrecord->Order = deviceorder;

  /* open the system device handle */
  if(OpenDeviceHandle(devrecord->Name, devrecord->Order, &devrecord->DrvRes)<0)
    {
    free(devrecord);
    return(0);
    }

  /* get the resources */
  if(!DeviceIoControl(1, &(devrecord->DrvRes)))
    {
    free(devrecord);    
    return 0; /* DeviceIoControl returns (bool) 'false' if fails.  */
    }

  devrecord->pCT = &CtDummy;
  devrecord->DrvRes.DriverData = calloc(1, 1024); //!!!! THIS SHOULD BE SHARED !!!!
  devrecord->DrvRes.DriverDataSize = 1024;

  switch(InternalResetDevice(devrecord,shareflag))
    {
    case HUDAQSUCCESS: break;  // everything is OK
    case HUDAQPARTIAL: break;  // unknown device, return its resources only
    case HUDAQFAILURE:         // initialisation failed - go on
    default:                   // unexpected return code
                       HudaqCloseDevice((HUDAQHANDLE)devrecord);
                       return(0);
    }
    
  return((HUDAQHANDLE) devrecord);
}


HUDAQSTATUS HUDAQAPI HudaqResetDevice(HUDAQHANDLE handle)
{
  return InternalResetDevice(HudaqGetDeviceRecord(handle), 0);
}



/** Closes a valid handle to a device. */
void HUDAQAPI HudaqCloseDevice(HUDAQHANDLE handle)
{

  /* typecast handle to device record, check for validity */
  DeviceRecord* devrecord = (DeviceRecord*) handle;
  if (devrecord==0) return;

  devrecord->pCT->HudaqDone(devrecord);

  /* free the resources */
  DeviceIoControl(0, &(devrecord->DrvRes));

  if(devrecord->DrvRes.DriverData)
    {free(devrecord->DrvRes.DriverData); devrecord->DrvRes.DriverData=NULL;}

  /* close the system device handle */
//  CloseHandle(devrecord->SysHandle);

  /* free the device record memory */
  free(devrecord);
}



const HudaqResourceInfo* HUDAQAPI HudaqGetDeviceResources(HUDAQHANDLE handle)
{
  /* typecast handle to device record, check for validity */
  DeviceRecord* devrecord = (DeviceRecord*) handle;
  if (devrecord==0) return(0);

  /* return the pointer to resource structure */
  return(&(devrecord->DrvRes.Resources));
}


/** This procedure returns full device name. */
const char* HUDAQAPI HudaqGetBoardName(HUDAQHANDLE handle)
{
	/* typecast handle to device record, check for validity */
  DeviceRecord* devrecord = (DeviceRecord*) handle;
  if (devrecord==0) return(NULL);

	/* Return a pointer to the device name. */
  return(devrecord->Name);
}


/** Entry / Exit code of whole dll. */
/*
int APIENTRY DllMain(HINSTANCE hInstance, DWORD dwReason, LPVOID lpReserved)
{
  switch(dwReason)
    {
    case DLL_PROCESS_ATTACH:
       break;

    case DLL_PROCESS_DETACH:
       if(SaveHandle)   //Is there any handle opened by "GetBoardIOAddress"?
         {
         HudaqCloseDevice(SaveHandle);
         SaveHandle = 0;
         }
       break;

    default: ;
       //MessageBox(NULL, "DllMain.", "HUDAQ", MB_OK | MB_APPLMODAL | MB_ICONHAND);
    }


return 1;   // Ok
}
*/


/*************************************************/
/* Return time in ms. 				 */
int timeGetTime(void)
{
struct timespec current_time;
  clock_gettime(CLOCK_REALTIME,&current_time);

  return current_time.tv_nsec/1000 + current_time.tv_sec*1000;
}