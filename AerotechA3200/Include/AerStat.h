/*+++
   Copyright (c) Aerotech Inc., 1996 - 2001

   This header file is used with the Aerotech System libraries.

+++*/
#ifndef __AER_STAT_H__
#define __AER_STAT_H__

#include "aer960.h"
#include "aertdef.h"

// prevents "name decoration" ("name mangling") of functions by C++
#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerStatusGetStatusWord( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwStatusWord, PDWORD pdwValue );
AERERR_CODE AER_DLLENTRY AerStatusGetStatusWordIsMaskSet( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwStatusWord, DWORD dwMask, PDWORD pdwSet );

AERERR_CODE AER_DLLENTRY AerStatusGetAliveMask( HAERCTRL hAerCtrl, PAXISMASK pMask);
AERERR_CODE AER_DLLENTRY AerStatusGetNumAliveNodes( HAERCTRL hAerCtrl, PDWORD pdwNum);

AERERR_CODE AER_DLLENTRY AerStatusGetDriveInfoWord( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwInfoWord, PDWORD pdwValue );
AERERR_CODE AER_DLLENTRY AerStatusGetAmpPeakRating( HAERCTRL hAerCtrl, AXISINDEX iAxis, PDWORD pdwPeakRating);
AERERR_CODE AER_DLLENTRY AerStatusGetDriveInfo2( HAERCTRL hAerCtrl, DWORD dwDriveInfoBlockNum, AXISINDEX iAxis,
                                                 PDWORD pdwOne, PDWORD pdwTwo, PDWORD pdwThree, PDWORD pdwFour);
AERERR_CODE AER_DLLENTRY AerStatusConvertDriveInfoBlock0( DWORD dwOne, DWORD dwTwo, DWORD dwThree, DWORD dwFour,
														  PDWORD pdwStickyBits, PDWORD pdwDriveSize, PDWORD pdwMajor,
														  PDWORD pdwMinor, PDWORD pdwBuild, PDWORD pdwFPGA,
														  PDWORD pdwMXH, PDWORD pdwDriveHardware, PDWORD pdwSpare );
AERERR_CODE AER_DLLENTRY AerStatusGetAnalogScaleFactor( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwAnaNum, PDOUBLE pd16BitToVolts);

AERERR_CODE AER_DLLENTRY AerStatusGetDriveInfo2All(HAERCTRL hAerCtrl, AXISINDEX iAxis, PAODWORD pdwData);
AERERR_CODE AER_DLLENTRY AerStatusGetDriveInfo2AllMode(HAERCTRL hAerCtrl, DWORD dwOnOff);
//
// Generic status
AERERR_CODE AER_DLLENTRY AerStatusGetItems( HAERCTRL hAerCtrl, DWORD nItems, PAIWORD wItemAxes, PAIWORD wItemCodes,
                                            PAIDWORD dwItemExtras, PAODOUBLE dItemValues);
AERERR_CODE AER_DLLENTRY AerStatusGetItemName( HAERCTRL hAerCtl, WORD wItem, LPTSTR pszShortName, LPTSTR pszLongName);
AERERR_CODE AER_DLLENTRY AerStatusHasAddress( HAERCTRL hAerCtl, WORD wItem, PDWORD pdwHasAddress,
                                              PDWORD pdwMinAddress, PDWORD pdwMaxAddress, LPTSTR pszAddressName, PDWORD pdwIsHex);
//
//  "Info" functions, returning structures of data
//
AERERR_CODE AER_DLLENTRY AerStatusGetTaskInfo ( HAERCTRL hAerCtrl, TASKMASK mTask, PAER_TASK_DATA pData );
AERERR_CODE AER_DLLENTRY AerStatusGetAxisInfoD( HAERCTRL hAerCtrl, AXISMASK mAxis, PAER_AXIS_DATAD pData );

//
// VB compatibile functions to the above "Info" functions, that return data in individual arguments
// rather in a structures. There are so many, cause we dont know what they want, and if we give it all the
// function would have 9 million arguments.
//
AERERR_CODE AER_DLLENTRY AerStatusGetAxisInfoEx( HAERCTRL hAerCtrl, AXISMASK mAxis, DWORD dwUnits,
                                                 PAODWORD pdwDriveStatus,
                                                 PAODWORD pdwAxisStatus,
                                                 PAODWORD pdwFault,
                                                 PAODOUBLE pdPosition,
                                                 PAODOUBLE pdPositionCmd,
                                                 PAODOUBLE pdVelocityAvg );

AERERR_CODE AER_DLLENTRY AerStatusGetAxisInfoConvert( HAERCTRL hAerCtrl,
                                                      AXISMASK mAxis,
                                                      PAODOUBLE pdRollOver,
                                                      PAODOUBLE pdPosFactor,
                                                      PAODWORD pdwType,
                                                      PAOWORD pwNumDecimals,
                                                      PAOWORD pwUnits,
                                                      PAOWORD pwMinutes );

AERERR_CODE AER_DLLENTRY AerStatusGetAxisInfoCurrent( HAERCTRL hAerCtrl,
                                                      AXISMASK mAxis,
                                                      PAOWORD pwCurrent,
                                                      PAOWORD pwCurrentCommand );

AERERR_CODE AER_DLLENTRY AerStatusGetAxisInfoIO( HAERCTRL hAerCtrl,
                                                 AXISMASK mAxis,
                                                 PAODOUBLE pdAnalogIn0,
                                                 PAODOUBLE pdAnalogIn1,
                                                 PAOWORD pwDigitalIn,
                                                 PAOWORD pwDigitalOut );

AERERR_CODE AER_DLLENTRY AerStatusGetAxisInfoIOEx( HAERCTRL hAerCtrl,
                                                   AXISMASK mAxis,
                                                   PAODOUBLE pdAnalogIn0,
                                                   PAODOUBLE pdAnalogIn1,
                                                   PAODWORD pdwDigitalIn,
                                                   PAODWORD pdwDigitalOut );

AERERR_CODE AER_DLLENTRY AerStatusGetAxisInfoPosition( HAERCTRL hAerCtrl,
                                                       AXISMASK mAxis,
                                                       DWORD dwUnits,
                                                       PAODOUBLE pdPosition,
                                                       PAODOUBLE pdPositionCmd,
                                                       PAODOUBLE pdPositionCal,
                                                       PAODOUBLE pdAuxEncoderPos,
                                                       PAODOUBLE pdAvgVelocity,
                                                       PAODOUBLE pdPosProgCmdUserUnits,
                                                       PAODOUBLE pdFixtureOffsetUserUnits,
                                                       PAODOUBLE pdTargetPosUserUnits,
                                                       PAODOUBLE pdAvgVelocityUserUnits );

AERERR_CODE AER_DLLENTRY AerStatusGetAxisInfoStatus( HAERCTRL hAerCtrl,
                                                     AXISMASK mAxis,
                                                     PAODWORD pdwDriveStatus,
                                                     PAODWORD pdwAxisStatus,
                                                     PAODWORD pdwFault );

/////////////////////////////////////////////////////////////////////////////////////

AERERR_CODE AER_DLLENTRY AerStatusGetTaskInfoStatus( HAERCTRL hAerCtrl, TASKMASK mTask,
                                                     PAODWORD pdwStatus,
                                                     PAODWORD pdwStatus2,
                                                     PAODWORD pdwStatus3,
                                                     PAODWORD pdwMode ,
                                                     PAODWORD pdwFault,
                                                     PAODWORD pdwWarning );

AERERR_CODE AER_DLLENTRY AerStatusGetTaskInfoProgram( HAERCTRL hAerCtrl, TASKMASK mTask,
                                                      PAODWORD pdwCallStackDepth,
                                                      PAODWORD pdwCurrentProgNumber,
                                                      PAODWORD pdwCurrentLine960,
                                                      PAODWORD pdwCurrentLineUser,
                                                      PAODWORD pdwCurrentPriorityLevel );

AERERR_CODE AER_DLLENTRY AerStatusGetProgRunning( HAERCTRL hAerCtrl, TASKINDEX iTask, LPTSTR pszName);

AERERR_CODE AER_DLLENTRY AerStatusGetTaskInfoEx( HAERCTRL hAerCtrl, TASKMASK mTask,
                                                 PAODWORD pdwStatus,
                                                 PAODWORD pdwStatus2,
                                                 PAODWORD pdwStatus3,
                                                 PAODWORD pdwMode ,
                                                 PAODWORD pdwFault,
                                                 PAODWORD pdwWarning );


AERERR_CODE AER_DLLENTRY AerStatusGetStopWatchData(   HAERCTRL hAerCtrl,
                                                      PAODWORD pdwLast,
                                                      PAODWORD pdwMax,
                                                      PAODWORD pdwMin,
                                                      PAODWORD pdwAvg);
DWORD AER_DLLENTRY AerStatusGetDriveCmdPend(HAERCTRL hAerCtrl, AXISINDEX iAxis);

#ifdef __cplusplus
}
#endif

#endif
