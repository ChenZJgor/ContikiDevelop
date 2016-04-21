/*
 * \file
 *	This file contains a set of configuration for using IAR EW8051 as a compiler.
 *	
 *
 * \author
 *	 YoCiHou - < www.iotdev.net >
 *	   (updates for the wsn2530dk ports)
 */

#ifndef __8051_DEF_H__
#define __8051_DEF_H__

#include <stdint.h>

/* In watchdog mode, our WDT can't be stopped once started
 * Include watchdog_stop()'s declaration and then trash it */
#include "dev/watchdog.h"
#define watchdog_stop() watchdog_periodic()

/* This port no longer implements the legacy clock_delay. Hack its usages
 * outta the way till it gets phased out completely
 * NB: This also overwrites the prototype so delay_usec() is declared twice */
#define clock_delay(t) clock_delay_usec(t)

#define CC_CONF_FUNCTION_POINTER_ARGS	1
#define CC_CONF_FASTCALL
#define CC_CONF_VA_ARGS		1
#define CC_CONF_UNSIGNED_CHAR_BUGS	0
#define CC_CONF_REGISTER_ARGS		0
#define CC_CONF_FUNCTION_POINTER_KEYWORD __reentrant
#define CC_CONF_NON_BANKED_OPTIMIZATION 1


/*
 * Max Stack Depth manipulation. It is possible to get up to 247 bytes
 * allocated for the stack if:
 * - You set this to 1 and
 * - You have a patched toolchain and
 * - You don't use __bit variables
 * - You do not link libraries using BIT registers (e.g. printf_large)
 * Enabling this will mean ISRs will NOT push bits (#pragma exclude bits) so
 * make sure you know what you're doing
 *
 * More information on the wiki
 */
#define CC_CONF_OPTIMIZE_STACK_SIZE 0

#if CC_CONF_OPTIMIZE_STACK_SIZE
#define CC_AT_DATA
#else
#define CC_AT_DATA __data
#endif

/* Generic types. */
typedef unsigned short uip_stats_t;

/* Time type. */
typedef unsigned short clock_time_t;
#define MAX_TICKS (~((clock_time_t)0) / 2)
/* Defines tick counts for a second. */
#define CLOCK_CONF_SECOND   128

/* Compiler configurations */
#define CCIF
#define CLIF

/* Critical section management */
#define DISABLE_INTERRUPTS()  do {EA = 0;} while(0)
#define ENABLE_INTERRUPTS()   do {EA = 1;} while(0)

/* Macro for a soft reset. */
#define SOFT_RESET() do {((void (__code *) (void)) 0x0000) ();} while(0)

/* We don't provide architecture-specific checksum calculations */
#define UIP_ARCH_ADD32    0
#define UIP_ARCH_CHKSUM	  0

#define CC_CONF_ASSIGN_AGGREGATE(dest, src)	\
    memcpy(dest, src, sizeof(*dest))

#define uip_ipaddr_copy(dest, src)		\
    memcpy(dest, src, sizeof(*dest))

#endif /* __8051_DEF_H__ */
