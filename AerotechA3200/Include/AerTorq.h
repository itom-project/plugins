/*++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_TORQ_H__
#define __AER_TORQ_H__

typedef struct tagAER_TORQ_PACKET
{
   WORD  wMode;
   WORD  wChannel;
   FLOAT dAmpsVel;
   FLOAT dAmpsTemp;
   WORD  wNomTemp;
} AER_TORQ_PACKET;
typedef AER_TORQ_PACKET *PAER_TORQ_PACKET;

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY  AerTorqueSetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis,
				     WORD wMode, FLOAT dAmpsVel);
AERERR_CODE AER_DLLENTRY  AerTorqueGetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis,
				     PWORD pwMode, PFLOAT pdAmpsVel);

#ifdef __cplusplus
}
#endif

#endif
// __AER_TORQ_H__
