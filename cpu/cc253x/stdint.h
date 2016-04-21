/*-------------------------------------------------------------------------
   stdint.h - ISO C99 7.18 Integer types <stdint.h>
   data type definition for Contiki port to CC253X (IAR EW8051)
-------------------------------------------------------------------------*/

#ifndef __STDINT_H__
#define __STDINT_H__

/*--------------------------------------------
*  Default datatype defined in IAR EW8051 is:
*  type         size    aligment
*  bool         8bits       1
*  char         8bits       1
*  short        16bits      1
*  int          16bits      1
*  long         32bits      1
*  long long    32bits      1
----------------------------------------------*/

/* Exact integral types.  */

/* Signed.  */
typedef signed char             int8_t;
typedef signed int              int16_t;
typedef signed long             int32_t;

/* Unsigned.  */
typedef unsigned char           uint8_t;
typedef unsigned int            uint16_t;
typedef unsigned long           uint32_t;

/* Types for `void *' pointers.  */
//  typedef long int              intptr_t;
//  typedef unsigned long int     uintptr_t;

#endif /* stdint.h */
