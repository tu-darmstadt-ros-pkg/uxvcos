#ifndef __TYPES_INCLUDED
#define __TYPES_INCLUDED

/******** Definition of items with uniquely size **********
(c) 1997-2008 Jaroslav Fojtik
   if you don't use below mentionted compiler, please correct this items
   for your compiler and send me your correction to:
			fojtik@penguin.cz or fojtik@humusoft.cz

List of supported types:

   Type  Alternate  Size & Description
   -----+---------+------+---------------------
   __u8   BYTE     1 byte  =  8 bit
   __s8   SBYTE    1 byte  =  8 bit signed
   __u16  WORD     2 bytes = 16 bit
   __s16  SWORD    2 bytes = 16 bit signed
   __u32  DWORD    4 bytes = 32 bit
   __s32  SDWORD   4 bytes = 32 bit signed

***************************************************************/


#ifdef __BORLANDC__
 #ifndef LO_ENDIAN
  #define LO_ENDIAN
 #endif 
 #define __u8    unsigned char
 #define __s8    signed char
 #if defined(__OS2__) || defined(__WIN32__)
   #define __u16   unsigned short int
   #define __s16   signed short int
 #else
   #define __u16   unsigned int
   #define __s16   signed int
 #endif
 #define __u32   unsigned long int
 #define __s32   signed long int
#else  /* __BORLANDC__ */


#if defined(__EGC__) || defined(__GNUC__)
 #ifdef __DJGPP__
   #ifndef LO_ENDIAN
    #define LO_ENDIAN
   #endif
 #endif
 typedef int __s8 __attribute__((mode(QI)));
 typedef unsigned int __u8 __attribute__((mode(QI)));
 typedef int __s16 __attribute__((mode(HI)));
 typedef unsigned int __u16 __attribute__((mode(HI)));
 typedef int __s32 __attribute__((mode(SI)));
 typedef unsigned int __u32 __attribute__((mode(SI))); 
 typedef int __s64 __attribute__((mode(DI)));
 typedef unsigned int __u64 __attribute__((mode(DI))); 
 #define __u8 __u8
 #define __u16 __u16
 #define __u32 __u32
 #define __u64 __u64
#else  /* __GNUC__ */


#ifdef __WATCOMC__
 #ifdef __386__
  #define __u8    unsigned char
  #define __s8    signed char
  #define __u16   unsigned short int
  #define __s16   signed short int
  #define __u32   unsigned int
  #define __s32   signed int
 #else
  #define __u8    unsigned char
  #define __s8    signed char
  #define __u16   unsigned int
  #define __s16   signed int
  #define __u32   unsigned long int
  #define __s32   signed long int
 #endif
#else /*__WATCOMC__*/

#ifdef __HPUXC__
 #define __u8	unsigned char
 #define __s8	signed char
 #define __u16	unsigned short int
 #define __s16	signed short int
 #define __u32	unsigned int
 #define __s32	signed int
#else

/* Here you may include your definition for other C */
#endif
#endif
#endif
#endif


#ifndef __u8
 #define __u8    unsigned char
 #define __s8    signed char
#endif 
#ifndef __u16
 #define __u16   unsigned short
 #define __s16   signed short
#endif  
#ifndef __u32
 #define __u32   unsigned long int
 #define __s32   signed long int
#endif

#ifndef BYTE
 #define BYTE    __u8
 #define SBYTE   __s8
 #define WORD    __u16
 #define SWORD   __s16
 #define DWORD   __u32
 #define SDWORD  __s32
#endif

#ifdef __u64
 #define QWORD   __u64
 #define SQWORD  __s64
#endif


#define Read_u16(v) ((WORD *)v)
#define Read_u32(v) ((DWORD *)v)

#define CF_LE_u16(v) (v)
#define CT_LE_u16(v) (v)
#define C_ST_u16(p,v) {*(((__u16*)p)++)=v;}
#define C_LD_u16(p,v) {v=*(((__u16*)p)++);}



#endif  		/* End of Header Types.h */
