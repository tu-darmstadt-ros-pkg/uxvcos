/****************************************************************/
/**@file hudaq_internal.h:
 * Description: Internal hudaq header with private information. *
 * Dependency: Only for Linux                                   *
 *                Copyright 2006-2010 Jaroslav Fojtik           *
 ****************************************************************/

#ifndef HUDAQ_INTERNAL_H__
#define HUDAQ_INTERNAL_H__

#include "types.h"

#define __int8 char
#define __int16 short int
#define __int32 int

#ifndef max
 #define max(x,y) (((x)>(y))?(x):(y))
#endif
#ifndef min
 #define min(x,y) (((x)>(y))?(x):(y))
#endif


/**
 * Resource information structure obtained from low level kernel driver.
*/
typedef struct
{
  size_t Size;                        ///< Size of this structure.
  int NumPhysMemResources;            ///< Amount of memory mapped ranges in particular device.
  HudaqResourceRange PhysMemResources[8];///< Memory resources available for user access.
  HudaqResourceInfo Resources;        ///< External structure for users
  size_t DriverDataSize;              ///< Size of shared memory among all applications.
  void* DriverData;                   ///< Internal data for one device shared across all applications.

        // TODO: these two items are planned to be filled from kernel driver
} DriverRecord;


typedef struct CallTable;

/** Device record that contains common information about general device resources. */
typedef struct
{
//HANDLE SysHandle;             ///< System handle assigned from kernel driver
  char Name[256];               ///< Device name in ASCII
  int Order;                    ///< Order of device
  DriverRecord DrvRes;          ///< Kernel resource info

  const struct CallTable *pCT;  ///< Interface functions
} DeviceRecord;


#include "hudaq_int_common.h"


#if __GNUC__ == 3 && __GNUC_MINOR__ >= 3
 #ifdef HUDAQAPI
  #undef HUDAQAPI
  #define HUDAQAPI __attribute__ ((visibility("default")))
  #pragma visibility(hidden)
 #endif
#endif

int timeGetTime(void);

#endif
