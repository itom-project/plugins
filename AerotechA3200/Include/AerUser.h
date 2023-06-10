/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_USER_H__
#define __AER_USER_H__

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerUserSetSecurity( HAERCTRL hAerCtrl, HSECURE hSecure );
AERERR_CODE AER_DLLENTRY AerUserGetSecurity( HAERCTRL hAerCtrl, HSECURE *phSecure );

#ifdef __cplusplus
}
#endif

#endif
// __AER_USER_H__
