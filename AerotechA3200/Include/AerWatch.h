/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AERWATCH_H__
#define __AERWATCH_H__

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerWatchdogEnable( HAERCTRL hAerCtrl, DWORD dwTime );
AERERR_CODE AER_DLLENTRY AerWatchdogDisable( HAERCTRL hAerCtrl );
AERERR_CODE AER_DLLENTRY AerWatchdogHitTimer( HAERCTRL hAerCtrl );

//AERERR_CODE AER_DLLENTRY AerENetClientWatchdogThreadStart(HAERCTRL hAerCtrl, WORD wWathcdogTimerMsec, PHANDLE phWatchdogEvent);
//AERERR_CODE AER_DLLENTRY   AerENetClientWatchdogThreadStop( HAERCTRL hAerCtrl, HANDLE phWatchdogEvent);

#ifdef __cplusplus
}
#endif

#endif
// __AERWATCH_H__
