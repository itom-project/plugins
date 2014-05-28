/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_PHASE_H__
#define __AER_PHASE_H__

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerPhaseSetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis, WORD wMode );
AERERR_CODE AER_DLLENTRY AerPhaseGetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis, PWORD pwMode );

#ifdef __cplusplus
}
#endif

#endif
// __AER_PHASE_H__
