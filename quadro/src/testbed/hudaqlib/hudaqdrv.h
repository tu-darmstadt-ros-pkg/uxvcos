/****************************************************************/
/**@file hudaqdrv.h:
 * Description: Windows driver constants.		        *
 * Dependency: ---                                              *
 *              Copyright 2006-2007 Humusoft s.r.o.             *
 *              Copyright 2009-2010 Jaroslav Fojtik             *
 ****************************************************************/

#define GETRESOURCES	 0x100
#define FREERESOURCES	 0x104
#define WAITFORIRQ	 0x108
#define CANCELWAITFORIRQ 0x10A
#define GETDVRVERSION	 0x10C


#if defined(_WIN32) || defined(_WIN64)
static const GUID HUDAQ_DEVINTERFACE_GUID =
{
  /* 90c3d703-9505-4887-8924-69f69cb8d5b9 */
  0x90c3d703,
  0x9505,
  0x4887,
  {0x89, 0x24, 0x69, 0xf6, 0x9c, 0xb8, 0xd5, 0xb9}
};
#endif


