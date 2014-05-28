/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used for the Aerotech Class libraries.

   The class libraries are part of the Software Development Kit (SDK).
+++*/

#ifndef __AERCLASS_H__
#define __AERCLASS_H__

/* AERCLASS is different libraries for debug and release */
#ifdef _DEBUG
   #pragma comment(lib, "A3DClass.lib")
#else
   #pragma comment(lib, "A32Class.lib")
#endif

//
//
// Aerotech class library header file
#include <acl.h>

#endif
// __AERCLASS_H__
