/*+++
   Copyright (c) Aerotech Inc., 1996 - 1997

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_MOTN_H__
#define __AER_MOTN_H__

//#include "cmdcodes.h"

typedef struct tagAER_MOTN_PACKET
{
   DOUBLE  fdMove;         // +/- machine steps to move (to)      (or amplitude for oscillate command)
   DOUBLE  fdSpeed;        // machine steps per second to move at (or frequency for oscillate command)
   DWORD   dwCycles;       // (used only in oscillate command)
   DWORD   dwFrequencies;  // (used only in oscillate command)
} AER_MOTN_PACKET;
typedef AER_MOTN_PACKET  *PAER_MOTN_PACKET;
typedef struct tagAER_MOTN_PACKETM
{
   DOUBLE  fdMove[MAX_AXES];         // +/- machine steps to move (to)      (or amplitude for oscillate command)
   DOUBLE  fdSpeed[MAX_AXES];        // machine steps per second to move at (or frequency for oscillate command)
   DWORD   dwCycles[MAX_AXES];       // (used only in oscillate command)
   DWORD   dwFrequencies[MAX_AXES];  // (used only in oscillate command)
} AER_MOTN_PACKETM;
typedef AER_MOTN_PACKETM  *PAER_MOTN_PACKETM;


typedef struct tagAER_MOTN_JOG_PACKET
{
   WORD  wMode;
   DWORD dwSpeed;
   WORD  wEnableBit;
   WORD  wDirBit;
} AER_MOTN_JOG_PACKET;
typedef AER_MOTN_JOG_PACKET   *PAER_MOTN_JOG_PACKET;

// prevents "name decoration" ("name mangling") of functions by C++
#ifdef __cplusplus
extern "C" {
#endif
//
// These are only for compatibility to versions 1.04 or lower
#define AerMoveMulti(a,b,c,d,e) AerMoveMAxis(a,b,c,d,e)
#define AerMoveWaitDoneMulti(a,b,c,d) AerMoveMWaitDone(a,b,c,d)

AERERR_CODE AER_DLLENTRY AerMoveAxis( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                      DWORD dwMove, DOUBLE dDistance, DOUBLE dSpeed,
                                      DWORD dwCycles, DWORD dwFrequencies );
AERERR_CODE AER_DLLENTRY AerMoveMAxis( HAERCTRL hAerCtrl, AXISMASK mAxis,
                                       DWORD dwMove, PAIDOUBLE pdDistanceArray, PAIDOUBLE pdSpeedArray,
                                       PDWORD pdwCycles, PDWORD pdwFrequencies );
AERERR_CODE AER_DLLENTRY AerMoveWaitDone( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwTimeOutMsec, LONG lflags);
AERERR_CODE AER_DLLENTRY AerMoveMWaitDone( HAERCTRL hAerCtrl, AXISMASK mAxis, DWORD dwTimeOutMsec, LONG lflags);
AERERR_CODE AER_DLLENTRY AerMoveLinear( HAERCTRL hAerCtrl, AXISMASK mAxes,
                                        PAIDOUBLE pdTargetArray, DOUBLE dSpeed, DWORD dwTaskMode, TASKINDEX iTask );
//AERERR_CODE AER_DLLENTRY AerMoveIndexArray(HAERCTRL hAerCtrl, AXISMASK mAxis,
//                                           PLONG dist, PLONG speed);

AERERR_CODE AER_DLLENTRY AerMoveJogSetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                            WORD wMode, DWORD dwSpeed,
                                            WORD wEnableBit, WORD wDirBit );
AERERR_CODE AER_DLLENTRY AerMoveJogGetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                            PWORD pwMode, PDWORD pdwSpeed,
                                            PWORD pwEnableBit, PWORD pwDirBit );

AERERR_CODE AER_DLLENTRY AerMoveAbsolute( HAERCTRL hAerCtrl, AXISINDEX iAxis, LONG lTarg, DWORD dwSpeed )      ;
AERERR_CODE AER_DLLENTRY AerMoveMAbsolute( HAERCTRL hAerCtrl, AXISMASK mAxes, PAILONG plMoveAry, PAIDWORD pdwSpeedAry);
AERERR_CODE AER_DLLENTRY AerMoveHome( HAERCTRL hAerCtrl, AXISINDEX iAxis)             ;
AERERR_CODE AER_DLLENTRY AerMoveMHome( HAERCTRL hAerCtrl, AXISMASK mAxes ) ;
AERERR_CODE AER_DLLENTRY AerMoveIncremental( HAERCTRL hAerCtrl, AXISINDEX iAxis, LONG lLen, DWORD dwSpeed )      ;
AERERR_CODE AER_DLLENTRY AerMoveMIncremental( HAERCTRL hAerCtrl, AXISMASK mAxes, PAILONG plMoveAry, PAIDWORD pdwSpeedAry ) ;
AERERR_CODE AER_DLLENTRY AerMoveFreerun( HAERCTRL hAerCtrl, AXISINDEX iAxis, LONG lDir, DWORD dwSpeed )          ;
AERERR_CODE AER_DLLENTRY AerMoveMFreerun( HAERCTRL hAerCtrl, AXISMASK mAxes, PAILONG plMoveAry, PAIDWORD pdwSpeedAry ) ;
//AERERR_CODE AER_DLLENTRY AerMoveInfeedSlave( HAERCTRL hAerCtrl, AXISINDEX iAxis, LONG lDist, DWORD dwSpeed )     ;
//AERERR_CODE AER_DLLENTRY AerMoveMInfeedSlave( HAERCTRL hAerCtrl, AXISMASK mAxes, PLONG plMoveAry, PDWORD pdwSpeedAry ) ;
AERERR_CODE AER_DLLENTRY AerMoveOscillate( HAERCTRL hAerCtrl, AXISINDEX iAxis, LONG lDir, DOUBLE dSpeed,
                                           DWORD dwCycles, DWORD dwFrequencies );
AERERR_CODE AER_DLLENTRY AerMoveMOscillate( HAERCTRL hAerCtrl, AXISMASK mAxes, PAILONG plMoveAry, PAIDOUBLE pdSpeedAry,
                                            PDWORD pdwCycles, PDWORD pdwFrequencies );
AERERR_CODE AER_DLLENTRY AerMoveAbsoluteD( HAERCTRL hAerCtrl, AXISINDEX iAxis, DOUBLE dTarg, DOUBLE dSpeed );
AERERR_CODE AER_DLLENTRY AerMoveMAbsoluteD( HAERCTRL hAerCtrl, AXISMASK mAxes, PAIDOUBLE pdMoveAry, PAIDOUBLE pdSpeedAry);


AERERR_CODE AER_DLLENTRY AerMoveHalt( HAERCTRL hAerCtrl, AXISINDEX iAxis );
AERERR_CODE AER_DLLENTRY AerMoveMHalt( HAERCTRL hAerCtrl, AXISMASK mAxes );
AERERR_CODE AER_DLLENTRY AerMoveAbort( HAERCTRL hAerCtrl, AXISINDEX iAxis );
AERERR_CODE AER_DLLENTRY AerMoveMAbort( HAERCTRL hAerCtrl, AXISMASK mAxes );
AERERR_CODE AER_DLLENTRY AerMoveFeedhold( HAERCTRL hAerCtrl, TASKINDEX iTask);
AERERR_CODE AER_DLLENTRY AerMoveRelease( HAERCTRL hAerCtrl, TASKINDEX iTask);
AERERR_CODE AER_DLLENTRY AerMoveEnable( HAERCTRL hAerCtrl, AXISINDEX iAxis );
AERERR_CODE AER_DLLENTRY AerMoveMEnable( HAERCTRL hAerCtrl, AXISMASK mAxes );
AERERR_CODE AER_DLLENTRY AerMoveDisable( HAERCTRL hAerCtrl, AXISINDEX iAxis );
AERERR_CODE AER_DLLENTRY AerMoveMDisable( HAERCTRL hAerCtrl, AXISMASK mAxes );
AERERR_CODE AER_DLLENTRY AerMoveBrakeEnable( HAERCTRL hAerCtrl, AXISINDEX iAxis );
AERERR_CODE AER_DLLENTRY AerMoveBrakeMEnable( HAERCTRL hAerCtrl, AXISMASK mAxes );
AERERR_CODE AER_DLLENTRY AerMoveBrakeDisable( HAERCTRL hAerCtrl, AXISINDEX iAxis );
AERERR_CODE AER_DLLENTRY AerMoveBrakeMDisable( HAERCTRL hAerCtrl, AXISMASK mAxes );
AERERR_CODE AER_DLLENTRY AerMoveSetPositionCommand( HAERCTRL hAerCtrl, AXISINDEX iAxis, LONG lPosCmd );
AERERR_CODE AER_DLLENTRY AerMoveMSetPositionCommand( HAERCTRL hAerCtrl, AXISMASK mAxes, PAILONG plPosCmdAry );

AERERR_CODE AER_DLLENTRY AerMoveRealignGantries(HAERCTRL hAerCtrl, TASKINDEX iTask, PAXISMASK pmMoved);
AERERR_CODE AER_DLLENTRY AerMoveSimulation( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwEnable );
//AERERR_CODE AER_DLLENTRY AerMoveG0Index(HAERCTRL hAerCtrl, TASKINDEX iTask, AXISMASK mAxes, PDOUBLE plMoveAry, PDOUBLE pdwSpeedAry );
//AERERR_CODE AER_DLLENTRY AerMoveG0Free(HAERCTRL hAerCtrl, TASKINDEX iTask, AXISMASK mAxes, PDOUBLE pdwSpeedAry );
AERERR_CODE AER_DLLENTRY AerMoveMSet(HAERCTRL hAerCtrl, AXISINDEX iAxis, DOUBLE dTorque, DWORD dwAngle );
AERERR_CODE AER_DLLENTRY AerMoveMBlockMotion(HAERCTRL hAerCtrl, AXISMASK mAxis, DWORD dwBlocked);
//
//
//#define  AerMoveQueueAbsolute( hAerCtrl, iAxis, lTarg, dwSpeed )   \
//   AerMoveAxis( hAerCtrl, iAxis, AERMOVE_QABSOLUTE, lTarg, dwSpeed )
//#define  AerMoveMQueueAbsolute( hAerCtrl, mAxes, plMoveAry, pdwSpeedAry ) \
//   AerMoveMAxis( hAerCtrl, mAxes, AERMOVE_QABSOLUTE, plMoveAry, pdwSpeedAry )
//
//#define  AerMoveHomeQuick( hAerCtrl, iAxis, lDir, dwSpeed )        \
//   AerMoveAxis( hAerCtrl, iAxis, AERMOVE_HOMEQUICK, lDir, dwSpeed )
//#define  AerMoveMHomeQuick ( hAerCtrl, mAxes, plMoveAry, pdwSpeedAry ) \
//   AerMoveMAxis( hAerCtrl, mAxes, AERMOVE_HOMEQUICK, plMoveAry, pdwSpeedAry )
//
//#define  AerMoveHomeAlt( hAerCtrl, iAxis, lDir, dwSpeed )          \
//   AerMoveAxis( hAerCtrl, iAxis, AERMOVE_HOMEALT_REV, lDir, dwSpeed )
//#define  AerMoveMHomeAlt( hAerCtrl, mAxes, plMoveAry, pdwSpeedAry ) \
//   AerMoveMAxis( hAerCtrl, mAxes, AERMOVE_HOMEALT_REV, plMoveAry, pdwSpeedAry )
//
//#define  AerMoveHomeVirtual( hAerCtrl, iAxis, lDir, dwSpeed )          \
//   AerMoveAxis( hAerCtrl, iAxis, AERMOVE_HOME_VIRTUAL, lDir, dwSpeed )
//#define  AerMoveMHomeVirtual( hAerCtrl, mAxes, plMoveAry, pdwSpeedAry ) \
//   AerMoveMAxis( hAerCtrl, mAxes, AERMOVE_HOME_VIRTUAL, plMoveAry, pdwSpeedAry )
//
//#define  AerMoveHomeRev( hAerCtrl, iAxis, lDir, dwSpeed )          \
//   AerMoveAxis( hAerCtrl, iAxis, AERMOVE_HOMEALT_REV, lDir, dwSpeed )
//#define  AerMoveMHomeRev( hAerCtrl, mAxes, plMoveAry, pdwSpeedAry ) \
//   AerMoveMAxis( hAerCtrl, mAxes, AERMOVE_HOMEALT_REV, plMoveAry, pdwSpeedAry )
//
//#define  AerMoveHomeNoLimit( hAerCtrl, iAxis, lDir, dwSpeed )      \
//   AerMoveAxis( hAerCtrl, iAxis, AERMOVE_HOMENOLIMIT, lDir, dwSpeed )
//#define  AerMoveMHomeNoLimit( hAerCtrl, mAxes, plMoveAry, pdwSpeedAry ) \
//   AerMoveMAxis( hAerCtrl, mAxes, AERMOVE_HOMENOLIMIT, plMoveAry, pdwSpeedAry )

//
//**document
//#define  AerMoveQueueFlush( hAerCtrl, iAxis )  \
//   AerMoveAxis( hAerCtrl, iAxis, AERMOVE_QFLUSH, 0, 0 )
//#define  AerMoveMQueueFlush( hAerCtrl, mAxes ) \
//   AerMoveMAxis( hAerCtrl, mAxes, AERMOVE_QFLUSH, NULL, NULL )
//
//**document
//#define  AerMoveQueueHold( hAerCtrl, iAxis ) \
//   AerMoveAxis( hAerCtrl, iAxis, AERMOVE_QHOLD, 0, 0 )
//#define  AerMoveMQueueHold( hAerCtrl, mAxes ) \
//   AerMoveMAxis( hAerCtrl, mAxes, AERMOVE_QHOLD, NULL, NULL )
//
////**document
//#define  AerMoveQueueRelease( hAerCtrl, iAxis ) \
//   AerMoveAxis( hAerCtrl, iAxis, AERMOVE_QRELEASE, 0, 0 )
//#define  AerMoveMQueueRelease( hAerCtrl, mAxes ) \
//   AerMoveMAxis( hAerCtrl, mAxes, AERMOVE_QRELEASE, NULL, NULL )

#ifdef __cplusplus
}
#endif

#endif
// __AER_MOTN_H__
