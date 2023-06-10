/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is a PUBLIC Aerotech U600 include, and is used by
   EVERYBODY (Win apps, device drivers, library, firmware).
   and must also be included by all customer apps (library, SDK, VB).

   Only #defines and typedefs should be placed in this file. (no structs, due to possible alignment problems)

+++*/

#ifndef __AERTYPE_H__
#define __AERTYPE_H__

/* Basic types */

#define DWORD_NULL      ((DWORD)-1)
#define WORD_NULL       ((WORD)-1)
#define BYTE_NULL       ((BYTE)-1)
#define BYTE_NULL2      ((BYTE)-2)

typedef double DOUBLE;
typedef DOUBLE *PDOUBLE;


typedef DWORD        AERERR_CODE;       /* THE Error Code type */
typedef AERERR_CODE  *PAERERR_CODE;

typedef PBYTE	PAIBYTE;	   // designates a ptr to an array
typedef PSHORT  PAISHORT;      // designates a ptr to an array
typedef PWORD   PAIWORD;       // designates a ptr to an array
typedef PULONG  PAIULONG;      // designates a ptr to an array
typedef PDWORD  PAIDWORD;      // designates a ptr to an array
typedef PLONG   PAILONG;       // designates a ptr to an array
typedef PFLOAT  PAIFLOAT;      // designates a ptr to an array
typedef PDOUBLE PAIDOUBLE;     // designates a ptr to an array

typedef PBYTE	PAOBYTE;	   // designates a ptr to an array
typedef PSHORT  PAOSHORT;      // designates a ptr to an array
typedef PWORD   PAOWORD;       // designates a ptr to an array
typedef PULONG  PAOULONG;      // designates a ptr to an array
typedef PDWORD  PAODWORD;      // designates a ptr to an array
typedef PLONG   PAOLONG;       // designates a ptr to an array
typedef PFLOAT  PAOFLOAT;      // designates a ptr to an array
typedef PDOUBLE PAODOUBLE;     // designates a ptr to an array


#include "AerTDef.H"

// typedefs related to "AerTDef.h"
typedef DWORD            AXISINDEX;
typedef AXISINDEX       *PAXISINDEX;

typedef DWORD           AXISMASK;          // don't kid yourself, AXISMASK HAS TO BE A 32 bit WORD!
typedef AXISMASK       *PAXISMASK;

typedef DWORD           PHYSAXISINDEX;
typedef PHYSAXISINDEX   *PPHYSAXISINDEX;
typedef DWORD           PHYSAXISMASK;
typedef PHYSAXISMASK    *PPHYSAXISMASK;

typedef DWORD           TASKAXISINDEX;
typedef TASKAXISINDEX   *PTASKAXISINDEX;
typedef DWORD           TASKAXISMASK;
typedef TASKAXISMASK    *PTASKAXISMASK;

typedef DWORD     TASKINDEX;
typedef TASKINDEX *PTASKINDEX;
typedef DWORD     TASKMASK;
typedef TASKMASK  *PTASKMASK;

typedef DWORD        SPINDLEINDEX;
typedef SPINDLEINDEX *PSPINDLEINDEX;
typedef DWORD        SPINDLEMASK;
typedef SPINDLEMASK  *PSPINDLEMASK;

typedef DWORD           CSPARMINDEX;
typedef CSPARMINDEX     *PCSPARMINDEX;
typedef DWORD           CSPARMMASK;
typedef CSPARMMASK      *PCSPARMMASK;

//typedef union tagAERVIRT_BINARY_DATA
//{
//   WORD  tWord[MAX_VIRT_BINARY_WORDS];
//} AERVIRT_BINARY_DATA;
//typedef AERVIRT_BINARY_DATA   *PAERVIRT_BINARY_DATA;

//typedef union tagAERVIRT_REGISTER_DATA
//{
//   WORD  tWord[MAX_REGISTERS];
//} AERVIRT_REGISTER_DATA;
//typedef AERVIRT_REGISTER_DATA *PAERVIRT_REGISTER_DATA;

/* Analog IO Index definititions */
//typedef DWORD        ANALOGINDEX;
//typedef ANALOGINDEX  *PANALOGINDEX;
//typedef DWORD        ANALOGMASK;
//typedef ANALOGMASK   *PANALOGMASK;

typedef WORD DRV_COMM_LENG;

#endif   /* __AERTYPE_H__  */
