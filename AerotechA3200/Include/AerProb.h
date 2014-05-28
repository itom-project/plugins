/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_PROB_H__
#define __AER_PROB_H__

typedef struct tagAER_PROBE_INPUT_PACKET
{
   WORD  wInput;
   WORD  wLevel;
} AER_PROBE_INPUT_PACKET;
typedef AER_PROBE_INPUT_PACKET   *PAER_PROBE_INPUT_PACKET;

typedef struct tagAER_PROBE_STATUS_PACKET
{
   WORD  wStatus;
   WORD  wInput;
   WORD  wLevel;
} AER_PROBE_STATUS_PACKET;
typedef AER_PROBE_STATUS_PACKET  *PAER_PROBE_STATUS_PACKET;

typedef struct tagAER_PROBE_POS_PACKET
{
	 WORD    wStatus;
	 DWORD   dwPos;
} AER_PROBE_POS_PACKET;
typedef AER_PROBE_POS_PACKET  *PAER_PROBE_POS_PACKET;

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerProbeSetInput( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                           WORD wInput, WORD wLevel );
AERERR_CODE AER_DLLENTRY AerProbeGetStatus( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                            PAER_PROBE_STATUS_PACKET pStatus );
AERERR_CODE AER_DLLENTRY AerProbeGetStatusEx( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                              PWORD pwStatus, PWORD pwInput, PWORD pwLevel );
AERERR_CODE AER_DLLENTRY AerProbeGetPosition( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                              PAER_PROBE_POS_PACKET pPos );
AERERR_CODE AER_DLLENTRY AerProbeGetPositionEx( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                                PWORD pwStatus, PDWORD pdwPos );
AERERR_CODE AER_DLLENTRY AerProbeEnable( HAERCTRL hAerCtrl, AXISINDEX iAxis );
AERERR_CODE AER_DLLENTRY AerProbeDisable( HAERCTRL hAerCtrl, AXISINDEX iAxis );

#ifdef __cplusplus
}
#endif

#endif
// __AER_PROB_H__
