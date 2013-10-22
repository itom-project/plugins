////////////////////////////////////////////////////////////////////////////////
// Header file for FireGrab Log utility.
// intek, c. kuehnel, 18.01.2008
////////////////////////////////////////////////////////////////////////////////

#ifndef LOGUTIL_H
#define LOGUTIL_H

#include <windows.h>

typedef void __stdcall LOGFCT(char *pFormat,...);

#ifdef __cplusplus
 extern "C" {
#endif
 
void __stdcall LogMsg(char* pFormat,...);

#ifdef __cplusplus
 }
#endif

#endif
