/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_AUX_H__
#define __AER_AUX_H__

#define  MAX_AUX_POINTS  50

#define  AUX_TABLE_OFF   0x00
#define  AUX_TABLE_ON    0x01

typedef struct tagAER_AUX_ALLOC_PACKET
{
   WORD  wTable;
   DWORD dwSize;
} AER_AUX_ALLOC_PACKET;
typedef AER_AUX_ALLOC_PACKET *PAER_AUX_ALLOC_PACKET;

typedef struct tagAER_AUX_STATUS_PACKET
{
   DWORD  dwSize;
   WORD   wStatus;
} AER_AUX_STATUS_PACKET;
typedef AER_AUX_STATUS_PACKET *PAER_AUX_STATUS_PACKET;

typedef struct tagAER_AUX_SETPOINT_PACKET
{
   WORD  wTable;
   DWORD dwPoint;
   LONG  lMaster;
   WORD  wLevel;
} AER_AUX_SETPOINT_PACKET;
typedef AER_AUX_SETPOINT_PACKET *PAER_AUX_SETPOINT_PACKET;

typedef struct tagAER_AUX_GETPOINT_SEND
{
   WORD  wTable;
   DWORD dwPoint;
} AER_AUX_GETPOINT_SEND;
typedef AER_AUX_GETPOINT_SEND *PAER_AUX_GETPOINT_SEND;

typedef struct tagAER_AUX_POINT
{
   LONG  lMaster;
   WORD  wLevel;
} AER_AUX_POINT;
typedef AER_AUX_POINT *PAER_AUX_POINT;

#define BASE_AUX_MULTI_PACKET \
   WORD  wTable;              \
   DWORD dwStart;             \
   DWORD dwNumPoints

typedef struct tagAER_BASE_AUX_MULTI_PACKET
{
   BASE_AUX_MULTI_PACKET;
} AER_BASE_AUX_MULTI_PACKET;

typedef struct tagAER_AUX_SETMULTI_PACKET
{
   BASE_AUX_MULTI_PACKET;
   AER_AUX_POINT           tPoint[MAX_AUX_POINTS];
} AER_AUX_SETMULTI_PACKET;
typedef AER_AUX_SETMULTI_PACKET   *PAER_AUX_SETMULTI_PACKET;

typedef struct tagAER_AUX_GETMULTI_PACKET
{
   BASE_AUX_MULTI_PACKET;
} AER_AUX_GETMULTI_PACKET;
typedef AER_AUX_GETMULTI_PACKET   *PAER_AUX_GETMULTI_PACKET;

typedef struct tagAER_AUX_MODE_PACKET
{
   WORD  wTable;
   WORD  wMode;
} AER_AUX_MODE_PACKET;
typedef AER_AUX_MODE_PACKET   *PAER_AUX_MODE_PACKET;

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerAuxTableGetTables( HAERCTRL hAerCtrl, PWORD pwNumTables );
AERERR_CODE AER_DLLENTRY AerAuxTableAllocate( HAERCTRL hAerCtrl, WORD wTable,
                                        DWORD dwSize );
AERERR_CODE AER_DLLENTRY AerAuxTableFree( HAERCTRL hAerCtrl, WORD wTable );
AERERR_CODE AER_DLLENTRY AerAuxTableSetPoint( HAERCTRL hAerCtrl, WORD wTable,
                                        DWORD dwPoint, LONG lMaster, WORD wLevel );
AERERR_CODE AER_DLLENTRY AerAuxTableGetPoint( HAERCTRL hAerCtrl, WORD wTable,
                                        DWORD dwPoint, PLONG plMaster,
                                        PWORD pwLevel );
AERERR_CODE AER_DLLENTRY AerAuxTableSetMultPoints( HAERCTRL hAerCtrl, WORD wTable,
                                             DWORD dwStart, DWORD dwCount,
                                             PAER_AUX_POINT pPoint );
AERERR_CODE AER_DLLENTRY AerAuxTableGetMultPoints( HAERCTRL hAerCtrl, WORD wTable,
                                             DWORD dwStart, DWORD dwNumPoints,
                                             PAER_AUX_POINT pPoint );
AERERR_CODE AER_DLLENTRY AerAuxTableGetStatus( HAERCTRL hAerCtrl, WORD wTable,
                                         PDWORD pdwSize, PWORD pwStatus );
AERERR_CODE AER_DLLENTRY AerAuxTableSetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                       WORD wTable, WORD wMode );
AERERR_CODE AER_DLLENTRY AerAuxTableGetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                       PWORD pwTable, PWORD pwMode );

#ifdef __cplusplus
}
#endif

#endif
// __AER_AUX_H__
