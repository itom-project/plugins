/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_SYNC_H__
#define __AER_SYNC_H__

#define NUM_AXISCAM_UNITS 4
#define NUM_AXISCAM_KEYWORDS 3

typedef struct tagAER_CAM_ALLOC_PACKET
{
   DWORD dwTable;
   DWORD dwSize;
} AER_CAM_ALLOC_PACKET;

typedef struct tagAER_CAM_SETTORQUE_PACKET
{
   DWORD    dwTable;
   DWORD    dwPoint;
   DWORD    dwTorque;
} AER_CAM_SETTORQUE_PACKET;

typedef struct tagAER_CAM_GETTORQUE_PACKET
{
   DWORD    dwTable;
   DWORD    dwPoint;
} AER_CAM_GETTORQUE_PACKET;

typedef struct tagAER_CAM_GEAR_PACKET
{
   DWORD  dwMStart;
   DWORD  dwMAccel;
   DWORD  dwSAccel;
} AER_CAM_GEAR_PACKET;
typedef AER_CAM_GEAR_PACKET   *PAER_CAM_GEAR_PACKET;


#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY  AerCamTableAllocate (HAERCTRL hAerCtrl, DWORD dwTable,
                                               DWORD dwSize);
AERERR_CODE AER_DLLENTRY  AerCamTableFree (HAERCTRL hAerCtrl, DWORD dwTable);

AERERR_CODE AER_DLLENTRY  AerCamTableCalcCoeff (HAERCTRL hAerCtrl, DWORD dwTable);

AERERR_CODE AER_DLLENTRY  AerCamTableSetUnits (HAERCTRL hAerCtrl, DWORD dwTable,
                                               DWORD  dwMasterPosUnitType, DWORD dwSlavePosUnitType);
AERERR_CODE AER_DLLENTRY  AerCamTableGetUnits (HAERCTRL hAerCtrl, DWORD dwTable,
                                               PDWORD pdwMasterPosUnitType, PDWORD pdwSlavePosUnitType);

AERERR_CODE AER_DLLENTRY  AerCamTableSetPoint (HAERCTRL hAerCtrl, DWORD dwTable, LONG lPoint,
                                               DOUBLE dMasterPos, DOUBLE dSlavePos, DWORD dwType );
AERERR_CODE AER_DLLENTRY  AerCamTableGetPoint (HAERCTRL hAerCtrl, DWORD wTable, LONG lPoint,
                                               PDOUBLE pdMasterPos, PDOUBLE pdSlavePos,
                                               PDOUBLE pdCoeffA, PDOUBLE pdCoeffB,
                                               PDOUBLE pdCoeffC, PDOUBLE pdCoeffD,
                                               PDWORD pdwType );

AERERR_CODE AER_DLLENTRY  AerCamTableSetMultPoints( HAERCTRL hAerCtrl, DWORD dwTable,
                                                    DWORD dwStart, DWORD dwCount,
                                                    PAER_CAM_SETPOINT pPoint );
AERERR_CODE AER_DLLENTRY  AerCamTableGetMultPoints( HAERCTRL hAerCtrl, DWORD dwTable,
                                                    DWORD dwStart, DWORD dwCount,
                                                    PAER_CAM_GETPOINT pPoint );

AERERR_CODE AER_DLLENTRY  aerCamTableGetStatusPacket (HAERCTRL hAerCtrl, DWORD dwTable,
                                                      PAER_CAM_STATUS_PACKET pStatus);
AERERR_CODE AER_DLLENTRY  AerCamTableGetStatus (HAERCTRL hAerCtrl, DWORD dwTable,
                                                PDWORD pdwSize, PDWORD pdwStatus);

AERERR_CODE AER_DLLENTRY  AerCamTableSetMode (HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                              DWORD dwTable, DWORD dwMode);
AERERR_CODE AER_DLLENTRY  AerCamTableGetMode (HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                              PDWORD pdwTable, PDWORD pdwMode);
AERERR_CODE AER_DLLENTRY  AerCamTableMSetMode(HAERCTRL hAerCtrl, AXISMASK mAxis,
                                              PDWORD pdwTable, PDWORD pdwMode);

AERERR_CODE AER_DLLENTRY AerCamTableFileDownload (HAERCTRL hAerCtrl, DWORD dwTable,
                                                  AXISINDEX iMasterAxis, AXISINDEX iSlaveAxis,
                                                  DWORD dwInterpolationType, DWORD dwCommandType,
                                                  LPCTSTR pszFile);

AERERR_CODE AER_DLLENTRY  AerCamTableCheck (HAERCTRL hAerCtrl, DWORD dwTable, AXISINDEX iMaster, AXISINDEX iSlave);

AERERR_CODE AER_DLLENTRY  AerCamTableSetMasterConfig (HAERCTRL hAerCtrl, AXISINDEX iSlaveAxis,
                                                      AXISINDEX iMasterAxis, DWORD dwCommandType);
AERERR_CODE AER_DLLENTRY  AerCamTableGetMasterConfig (HAERCTRL hAerCtrl, AXISINDEX iSlaveAxis,
                                                      PAXISINDEX piMasterAxis, PDWORD pdwCommandType);
AERERR_CODE AER_DLLENTRY  AerCamSetFirstPointLineNumber (HAERCTRL hAerCtrl, DWORD dwTable, ULONG ulLineNumberInFile);

// Specialized function for 3M
//AERERR_CODE AER_DLLENTRY  AerCamTableSetTorque( HAERCTRL hAerCtrl, DWORD dwTable,
//                                                DWORD dwPoint, DWORD dwTorque );

// Specialized function for 3M
//AERERR_CODE AER_DLLENTRY  AerCamTableGetTorque( HAERCTRL hAerCtrl, DWORD dwTable,
//                                                DWORD dwPoint, PDWORD pdwTorque );

// What is this?  Ron Rekowski doesn't even know
//AERERR_CODE AER_DLLENTRY  AerCamTableSetTrack( HAERCTRL hAerCtrl, AXISINDEX iAxis,
//                                              DWORD dwMStart, DWORD dwMAccel,
//                                              DWORD dwSAccel );

// What is this?  Ron Rekowski doesn't even know
//AERERR_CODE AER_DLLENTRY  AerCamTableGetTrack( HAERCTRL hAerCtrl, AXISINDEX iAxis,
//                                              PDWORD pdwMStart, PDWORD pdwMAccel,
//                                              PDWORD pdwSAccel );

#ifdef __cplusplus
}
#endif

#endif
// __AER_SYNC_H__
