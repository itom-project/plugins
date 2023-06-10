/*+++

	Copyright (c) Aerotech Inc., 1996 - 2006

	This header file is an  PUBLIC Aerotech A3200 include file, and is used by
	all (Win apps, device drivers, library, firmware).
	and must also be included by all customer apps (library, SDK, VB).

	NOTE:	A change will usually mean a change in two or three places at once, as data is stored in
			parallel here. For example adding a task parameter means a change in both the TASKPARM_
			constants, and a change in the TaskParmInfo data structure. Especially true when changing
			status constants, when you may need to change 4 places or so.

	NOTE:	This file is auto generated.  Changes made to this file will be lost the next time the
			file is generated.

+++*/


#ifndef __AERPINFO_H__
#define __AERPINFO_H__

#include "aerbld.rc"  /* for version numbers */
#include "AerPDef.h"

#include <float.h>
#include <limits.h>

#define  MAX_PARM_NAME_LEN    40

/*** Status word definitions ***************************************************************************/
//
// TO MAKE A CHANGE IN THESE YOU MUST CHANGE ALL THE BELOW PLACES IN PARALLEL ! :
// See FIRST set (bit names) for comments on bit meanings
//    1. (AERPINFO.H) Structure members (to allow easy bit access to the bits)
//         (change both the bit numbers and the masks)
//    2. (AERPINFO.H) Names for the bits (used by NStat, etc.)
//    3. (AERPDEF.H) Bit numbers
//    4. (INI/AERPARAM.PGM) CNC bit numbers
//         (change the bit numbers, there are no masks)
//    5. (A32SYS/A32SYS.ODL) CNC bit numbers
//         (change the masks, there are no bit numbers)
//
#ifndef __INIT_AER_P_INFO_H__
extern CHAR                szTaskStatusName[MAX_TASKSTATUS_DWORD][MAX_PARM_NAME_LEN];
extern CHAR                szTaskModeName[MAX_TASKSTATUS_DWORD][MAX_PARM_NAME_LEN];
#else
CHAR  szTaskStatusName[MAX_TASKSTATUS_DWORD*32][MAX_PARM_NAME_LEN]=
{
   /* TASKSTATUS1_ */
   "ProgramAssociated",                /*  0 Program is loaded into this task */
   "ProgramActive",                    /*  1 Not running a program line now, but in the middle of a program */
   "ProgramExecuting",                 /*  2 Running a program line now */
   "ImmediateCodeExecuting",           /*  3 Running an immediate line now */
   "ReturnMotionExecuting",            /*  4 Running return motion (from a jog+return) */
   "ProgramAborted",                   /*  5 Program was aborted (abort key hit or AerTaskProgramAbort() called). This stays on until another program execution starts */
   "StepIntoSubroutine",               /*  6 If in step mode, then step over subroutines */
   "StepOverSubroutine",               /*  7 If in step mode, then step into subroutines */
   "ProgramReset",                     /*  8 Turns on when a Program Reset hit (AerTaskProgramReset() called) This stays on until another program execution starts */
   "PendingAxesStop",                  /*  9 In "program feedhold", and waiting for axes stop.  Turns off after axes come to a halt This is used in conjunction with the other "pending bits" */
   "EStopActive",                      /* 10 Task software estop currently on */
   "FeedHoldActive",                   /* 11 Feedhold is currently on (motion deceling or stopped) */
   "CallBackHoldActive",               /* 12 Firmware saw a callback, halted the program, and signaled the front end to handle it */
   "CallBackResponding",               /* 13 Front end saw signal from firmware, and is doing the callback (it called Aer960TaskCallBackGetData()) */
   "SpindleActive1",                   /* 14 running this spindle (M3,M4) */
   "SpindleActive2",                   /* 15 running this spindle (M23,M24) */
   "SpindleActive3",                   /* 16 running this spindle (M33,M34) */
   "SpindleActive4",                   /* 17 running this spindle (M43,M44) */
   "ProbeCycle",                       /* 18 */
   "Retrace",                          /* 19 in retrace mode */
   "SoftHomeActive",                   /* 20 G92 active */
   "InterruptMotionActive",            /* 21 Motion Interrupt happened via the TASKPARM_InterruptMotion parameter (comes on after decel done, stays on till return motion done) */
   "SlewActive",                       /* 22 */
   "CornerRounding",                   /* 23 */
   "ROReq1Active",                     /* 24 */
   "SlewLowFeedModeActive",            /* 25 */
   "CannedFunctionActive",             /* 26 Canned function execution has been requested */
   "CannedFunctionExecuting",          /* 27 Canned function is actually executing */
   "Spare",                            /* 28 */
   "Spare",                            /* 29 */
   "Spare",                            /* 30 */
   "Spare",                            /* 31 */

   /* TASKSTATUS2_ */
   "MotionModeAbsOffsets",             /*  0 IJK offsets are absolute (for this move only) */
   "ASyncSMCMotionAbortPending",       /*  1 now deceling ASYNCRONOUS motion axes */
   "ProfileQueRunning",                /*  2 debug only (for starvation checks) */
   "RetraceRequested",                 /*  3 retrace has been changed, but not processed yet */
   "MSOChange",                        /*  4 MSO has been changed, but not processed yet */
   "SpindleFeedHeld",                  /*  5 In feedhold mode, and spindle motion had to be deceled On during decel, and until */
   "FeedHeldAxesStopped",              /*  6 in feedhold AND axes actually stopped motion */
   "CutterEnabling",                   /*  7 */
   "CutterDisabling",                  /*  8 */
   "CutterOffsetsEnablingPos",         /*  9 */
   "CutterOffsetsEnablingNeg",         /* 10 */
   "CutterOffsetsDisabling",           /* 11 */
   "MFOChange",                        /* 12 MFO has been changed, but not processed yet */
   "InterruptFaultPending",            /* 13 frontend interrupt failed first time, now retrying it */
   "RetraceNegativeHold",              /* 14 In retrace, but program execution held on this line until retrace taken off */
   "OnGosubPending",                   /* 15 Doing a "program feedhold", immediatly prior to a Ongosub activation. Turns off when Ongosub starts. */
   "ProgramAbortPending",              /* 16 Doing a "program feedhold", immediatly prior to a Program Abort. Turns off when Program Abort starts */
   "CannedFunctionPending",            /* 17 Doing a "program feedhold", immediatly prior to a Canned function activation. Turns off when canned function activated */
   "NoMFOFloor",                       /* 18 MFOMin has been set to less then 0 */
   "Interrupted",                      /* 19 Motion Interrupt has occurred (ONGOSUB, TASKPARM_InterruptMotion, etc), turned on before decel, turned off when line changes */
   "Spare",                            /* 20 */
   "Spare",                            /* 21 */
   "Spare",                            /* 22 */
   "Spare",                            /* 23 */
   "Spare",                            /* 24 */
   "Spare",                            /* 25 */
   "Spare",                            /* 26 */
   "Spare",                            /* 27 */
   "Spare",                            /* 28 */
   "Spare",                            /* 29 */
   "Spare",                            /* 30 */
   "Spare",                            /* 31 */

   /* TASKSTATUS3_ */
   "RotationActive",                   /*  0 G84 (parts rotation) active */
   "RThetaPolarActive",                /*  1 G46 active */
   "RThetaCylindricalActive",          /*  2 G47 active */
   "ScalingActive",                    /*  3 G151 mode active */
   "OffsetFixtureActive",              /*  4 G52->G59 active */
   "ProfileActive",                    /*  5 now doing a profiled move (G0,G1,G2,G3) */
   "MotionTypePtToPt",                 /*  6 G0 mode */
   "MotionTypeInterp",                 /*  7 G1/G2/G3 mode */
   "MotionTypeProfilePoint",           /*  8 PVT mode */
   "MotionContinuous",                 /*  9 not deceling at the end of this G1/G2/G3 move */
   "MotionNoAccel",                    /* 10 G8 */
   "VppActive",                        /* 11 */
   "CutterOffsetsActivePos",           /* 12 */
   "CutterActiveLeft",                 /* 13 */
   "CutterActiveRight",                /* 14 */
   "CutterOffsetsActiveNeg",           /* 15 */
   "NormalcyActiveLeft",               /* 16 */
   "NormalcyActiveRight",              /* 17 */
   "NormalcyAlignment",                /* 18 performing a normalcy alignment move */
   "MotionTypeCW",                     /* 19 G2 mode */
   "MotionTypeCCW",                    /* 20 G3 mode */
   "LimitFeedRateActive",              /* 21 MaxFeedRate speed limit being applied */
   "LimitMFOActive",                   /* 22 MFO being limited, due to MaxFeedRate/Blend* speed limit */
   "Coord1Plane1",                     /* 23 G17 (I,J) */
   "Coord1Plane2",                     /* 24 G18 (K,I) */
   "Coord1Plane3",                     /* 25 G19 (J,K) */
   "Coord2Plane1",                     /* 26 G27 (I,J) */
   "Coord2Plane2",                     /* 27 G28 (K,I) */
   "Coord2Plane3",                     /* 28 G29 (J,K) */
   "NewG1",                            /* 29 */
   "MirrorActive",                     /* 30 G83 (mirroring) active */
   "Spare2"                            /* 31 */          // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
};
CHAR  szTaskModeName[MAX_TASKMODE_DWORD*32][MAX_PARM_NAME_LEN]=
{
   /* TASKMODE1_ */
   "English",                          /*  0 - !Metric (G70/G71) */
   "Absolute",                         /*  1 - !Incremental (G90/G91) */
   "AccelModeLinear",                  /*  2 - !1-cosine (G63/G64) */
   "AccelModeRate",                    /*  3 - !Time based (G67/G68) */
   "RotaryDominant",                   /*  4 - !Linear (G98/G99) */
   "MotionContinuous",                 /*  5 - !DecelToZero (G9/G109/G108) */
   "InverseCircular",                  /*  6 - (G110/G111) */
   "SpindleStopOnProgHalt",            /*  7 - (G100/G101) */
   "BlockDelete",                      /*  8 - (G112/G113) */
   "OptionalStop",                     /*  9 - (G114/G115) */
   "AccelModeScurve",                  /* 10 */
   "MFOLock",                          /* 11 */
   "MSOLock",                          /* 12 */
   "DryRunFeedRate",                   /* 13 - (G116/G117) */
   "Spare",                            /* 14 */
   "AutoMode",                         /* 15 !StepMode */
   "ProgramFeedRateMPU",               /* 16 */
   "ProgramFeedRateUPR",               /* 17 */
   "ProgramSFeedRateSurf1",            /* 18 */
   "ProgramSFeedRateSurf2",            /* 19 */
   "ProgramSFeedRateSurf3",            /* 20 */
   "ProgramSFeedRateSurf4",            /* 21 */
   "BlockDelete2",                     /* 22 - (G212/G213) */
   "RunOverMode",                      /* 23 !Into */
   "MultiBlockLookAhead",              /* 24 - (G300/G301) */
   "HighSpeedLookAhead",               /* 25 - (M41/M42)   */
   "MFOActiveOnJog",                   /* 26 - (G120/G121) */
   "WaitForInPos",                     /* 27 - (G360/G361) */
   "Minutes",                          /* 28 - (G75/G76)   */
   "Counts",                           /* 29 */
   "WaitNone",                         /* 30 */
   "Spare",                            /* 31 */   // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
};
#endif

/* TASKPARM_Status structure */
typedef struct tagAER_TASK_STATUS
{
   union
   {
      DWORD    dwMask;
      struct
      {
         unsigned ProgramAssociated:1;                /*  0 */
         unsigned ProgramActive:1;                    /*  1 */
         unsigned ProgramExecuting:1;                 /*  2 */
         unsigned ImmediateCodeExecuting:1;           /*  3 */
         unsigned ReturnMotionExecuting:1;            /*  4 */
         unsigned ProgramAborted:1;                   /*  5 */
         unsigned SingleStepInto:1;                   /*  6 */
         unsigned SingleStepOver:1;                   /*  7 */
         unsigned ProgramReset:1;                     /*  8 */
         unsigned PendingAxesStop:1;                  /*  9 */
         unsigned EStopActive:1;                      /* 10 */
         unsigned FeedHoldActive:1;                   /* 11 */
         unsigned CallBackHoldActive:1;               /* 12 */
         unsigned CallBackResponding:1;               /* 13 */
         unsigned SpindleActive:MAX_SPINDLES;         /* 14->17 */
         unsigned ProbeCycle:1;                       /* 18 */
         unsigned Retrace:1;                          /* 19 */
         unsigned SoftHomeActive:1;                   /* 20 */
         unsigned InterruptMotionActive:1;            /* 21 */
         unsigned SlewActive:1;                       /* 22 */
         unsigned CornerRounding:1;                   /* 23 */
         unsigned ROReq1Active:1;                     /* 24 */
         unsigned SlewLowFeedModeActive:1;            /* 25 */
         unsigned CannedFunctionActive:1;             /* 26 */
         unsigned CannedFunctionExecuting:1;          /* 27 */
         unsigned Spare3:1;                           /* 28 */
         unsigned Spare4:1;                           /* 29 */
         unsigned Spare5:1;                           /* 30 */
         unsigned Spare6:1;                           /* 31 */     // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
      } Bit;
   } DW1;

   union
   {
      DWORD    dwMask;
      struct
      {
         unsigned MotionModeAbsOffsets:1;             /*  0 */
         unsigned ASyncSMCMotionAbortPending:1;       /*  1 */
         unsigned ProfileQueRunning:1;                /*  2 */
         unsigned RetraceRequested:1;                 /*  3 */
         unsigned MSOChange:1;                        /*  4 */
         unsigned SpindleFeedHeld:1;                  /*  5 */
         unsigned FeedHeldAxesStopped:1;              /*  6 */
         unsigned CutterEnabling:1;                   /*  7 */
         unsigned CutterDisabling:1;                  /*  8 */
         unsigned CutterOffsetsEnablingPos:1;         /*  9 */
         unsigned CutterOffsetsEnablingNeg:1;         /* 10 */
         unsigned CutterOffsetsDisabling:1;           /* 11 */
         unsigned MFOChange:1;                        /* 12 */
         unsigned InterruptFaultPending:1;            /* 13 */
         unsigned RetraceNegativeHold:1;              /* 14 */
         unsigned OnGosubPending:1;                   /* 15 */
         unsigned ProgramAbortPending:1;              /* 16 */
         unsigned CannedFunctionPending:1;            /* 17 */
         unsigned NoMFOFloor:1;                       /* 18 */
         unsigned Interrupted:1;                      /* 19 */
         unsigned ProgramStepOnce:1;                  /* 20 */
         unsigned Spare9:1;                           /* 21 */
         unsigned Spare10:1;                          /* 22 */
         unsigned Spare11:1;                          /* 23 */
         unsigned Spare12:1;                          /* 24 */
         unsigned Spare13:1;                          /* 25 */
         unsigned Spare14:1;                          /* 26 */
         unsigned Spare15:1;                          /* 27 */
         unsigned Spare16:1;                          /* 28 */
         unsigned Spare17:1;                          /* 29 */
         unsigned Spare18:1;                          /* 30 */
         unsigned Spare19:1;                          /* 31 */     // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
      } Bit;
   } DW2;

   union
   {
      DWORD    dwMask;
      struct
      {
         unsigned RotationActive:1;                   /*  0 */
         unsigned RThetaPolarActive:1;                /*  1 */
         unsigned RThetaCylindricalActive:1;          /*  2 */
         unsigned ScalingActive:1;                    /*  3 */
         unsigned OffsetFixtureActive:1;              /*  4 */
         unsigned ProfileActive:1;                    /*  5 */
         unsigned MotionTypePtToPt:1;                 /*  6 */
         unsigned MotionTypeInterp:1;                 /*  7 */
         unsigned MotionTypeProfilePoint:1;           /*  8 */
         unsigned MotionContinuous:1;                 /*  9 */
         unsigned MotionNoAccel:1;                    /* 10 */
         unsigned VppActive:1;                           /* 11 */
         unsigned CutterOffsetsActivePos:1;           /* 12 */
         unsigned CutterActiveLeft:1;                 /* 13 */
         unsigned CutterActiveRight:1;                /* 14 */
         unsigned CutterOffsetsActiveNeg:1;           /* 15 */
         unsigned NormalcyActiveLeft:1;               /* 16 */
         unsigned NormalcyActiveRight:1;              /* 17 */
         unsigned NormalcyAlignment:1;                /* 18 */
         unsigned MotionTypeCW:1;                     /* 19 */
         unsigned MotionTypeCCW:1;                    /* 20 */
         unsigned LimitFeedRateActive:1;              /* 21 */
         unsigned LimitMFOActive:1;                   /* 22 */
         unsigned Coord1Plane1:1;                     /* 23 */
         unsigned Coord1Plane2:1;                     /* 24 */
         unsigned Coord1Plane3:1;                     /* 25 */
         unsigned Coord2Plane1:1;                     /* 26 */
         unsigned Coord2Plane2:1;                     /* 27 */
         unsigned Coord2Plane3:1;                     /* 28 */
         unsigned NewG1:1;                            /* 29 */
         unsigned MirrorActive:1;                     /* 30 */
         unsigned Spare3:1;                           /* 31 */    // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
      } Bit;
   } DW3;
} AER_TASK_STATUS;
typedef AER_TASK_STATUS *PAER_TASK_STATUS;
typedef const AER_TASK_STATUS *const PCAER_TASK_STATUS;

/* TASKPARM_Mode1 structure */
typedef struct tagAER_TASK_MODE
{
   union
   {
      DWORD    dwMask;
      struct
      {
         unsigned English:1;                          /*  0 */
         unsigned Absolute:1;                         /*  1 */
         unsigned AccelModeLinear:1;                  /*  2 */
         unsigned AccelModeRate:1;                    /*  3 */
         unsigned RotaryDominant:1;                   /*  4 */
         unsigned MotionContinuous:1;                 /*  5 */
         unsigned InverseCircular:1;                  /*  6 */
         unsigned SpindleStopOnProgHalt:1;            /*  7 */
         unsigned BlockDelete:1;                      /*  8 */
         unsigned OptionalStop:1;                     /*  9 */
         unsigned AccelModeScurve:1;                  /* 10 */
         unsigned MFOLock:1;                          /* 11 */
         unsigned MSOLock:1;                          /* 12 */
         unsigned DryRunFeedRate:1;                   /* 13 */
         unsigned Spare2:1;                            /* 14 */
         unsigned AutoMode:1;                         /* 15 */
         unsigned ProgramFeedRateMPU:1;               /* 16 */
         unsigned ProgramFeedRateUPR:1;               /* 17 */
         unsigned ProgramSFeedRateSurf1:1;            /* 18 */
         unsigned ProgramSFeedRateSurf2:1;            /* 19 */
         unsigned ProgramSFeedRateSurf3:1;            /* 20 */
         unsigned ProgramSFeedRateSurf4:1;            /* 21 */
         unsigned BlockDelete2:1;                     /* 22 */
         unsigned RunOverMode:1;                      /* 23 */
         unsigned MultiBlockLookAhead:1;              /* 24 */
         unsigned HighSpeedLookAhead:1;               /* 25 */
         unsigned MFOActiveOnJog:1;                   /* 26 */
         unsigned WaitForInPos:1;                     /* 27 */
         unsigned Minutes:1;                          /* 28 */
         unsigned Counts:1;                           /* 29 */
         unsigned WaitNone:1;                         /* 30 */
         unsigned Spare:1;                            /* 31 */    // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
      } Bit;
   } DW1;
} AER_TASK_MODE;
typedef AER_TASK_MODE *PAER_TASK_MODE;
typedef const AER_TASK_MODE *const PCAER_TASK_MODE;

/*** Parameter types ***************************************************************************/
#define MAX_NUMBER_PARM_TYPES      7
typedef struct tagAER_PARM_TYPES_INFO
{
   LONG     lNumParmTypes;
   LONG     lParmTypes[MAX_NUMBER_PARM_TYPES];
} AER_PARM_TYPES_INFO;
typedef AER_PARM_TYPES_INFO *PAER_PARM_TYPES_INFO;
typedef const AER_PARM_TYPES_INFO *const PCAER_PARM_TYPES_INFO;

/*** Parameters ***************************************************************************/
/* min/max/default values stored here for dist/vel/accel parms are always in "internal inits" (in|deg/sec) */

typedef struct tagAER_PARM_INFO
{
   CHAR     szName[MAX_PARM_NAME_LEN];
   DOUBLE   fdMin;
   DOUBLE   fdMax;
   DWORD    dwAttr;
   DWORD    dwDisplayAttr;
   DOUBLE   fdDefault;
   DWORD    dwDisplaySubGroup;
} AER_PARM_INFO;
typedef AER_PARM_INFO *PAER_PARM_INFO;
typedef const AER_PARM_INFO *const PCAER_PARM_INFO;

#define IsParmRead(pInfo)           ( (pInfo)->dwAttr & PARM_ATTR_READ )
#define IsParmWrite(pInfo)          ( (pInfo)->dwAttr & PARM_ATTR_WRITE )
#define IsParmUnsigned(pInfo)       ( (pInfo)->dwAttr & PARM_ATTR_UNSIGNED )
#define IsParmUpdate(pInfo)         ( (pInfo)->dwAttr & PARM_ATTR_UPDATE )
#define IsParmInteger(pInfo)        ( (pInfo)->dwAttr & PARM_ATTR_INTEGER )
#define IsParmDrive(pInfo)          ( (pInfo)->dwAttr & PARM_ATTR_DRIVE )
#define IsParm32Bit(pInfo)          ( (pInfo)->dwAttr & PARM_ATTR_32_BIT )
#define IsParm64Bit(pInfo)          ( (pInfo)->dwAttr & PARM_ATTR_64_BIT )
#define IsParmNoLimit(pInfo)        ( (pInfo)->dwAttr & PARM_ATTR_NOLIMIT )
#define IsParmDistance(pInfo)       ( (pInfo)->dwAttr & PARM_ATTR_DISTANCE )
#define IsParmVelocity(pInfo)       ( (pInfo)->dwAttr & PARM_ATTR_VELOCITY )
#define IsParmAcceleration(pInfo)   ( (pInfo)->dwAttr & PARM_ATTR_ACCELERATION )
#define IsParmDistVelAcc(pInfo)     ( (pInfo)->dwAttr & (PARM_ATTR_DISTANCE | PARM_ATTR_VELOCITY | PARM_ATTR_ACCELERATION) )
#define IsParmDistAcc(pInfo)        ( (pInfo)->dwAttr & (PARM_ATTR_DISTANCE | PARM_ATTR_ACCELERATION) )
#define IsParmTaskRotary(pInfo)     ( (pInfo)->dwAttr & PARM_ATTR_TASK_ROTARY )

/* Axis Parameter Min, Max, default, attributes */
/*  (min/max/default values shown here for dist/vel/accel parms are always in "internal inits" (in|deg/sec)) */
#ifndef __INIT_AER_P_INFO_H__
	extern AER_PARM_INFO       AxisParmInfo[MAX_AXISPARMS];
#else
AER_PARM_INFO     AxisParmInfo[MAX_AXISPARMS]=
{// Parameter Name,                          Minimum,                 Maximum,                 Type Attribute,          Display Attribute,       DefaultValue,            DisplaySubGroup
{"Type",                                     0,                       AXISTYPE_MAX-1,          PARM_ATTR_RWI,           PD_ATTR_C,               AXISTYPE_LINEAR,         PD_ASG_UNITS        },
{"CntsPerEnglishUnit",                       0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V_FDBK,          25400.0,                 PD_ASG_UNITS        },
{"CntsPerMetricUnit",                        0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V_FDBK,          1000.0,                  PD_ASG_UNITS        },
{"CntsPerRotaryUnit",                        0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V_FDBK,          600,                     PD_ASG_UNITS        },
{"RotaryFeedRateScaleFactor",                0,                       0,                       PARM_ATTR_RWN_US,        PD_ATTR_V,               1,                       PD_ASG_UNITS        },
{"RolloverDistanceCnts",                     ROLLOVER_360_NO_FASTEST, LONG_MAX,                PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_ASG_UNITS        },
{"HomeType",                                 0,                       HOMETYPE_MAX-1,          PARM_ATTR_RWI,           PD_ATTR_C,               HOMETYPE_REVERSE,        PD_ASG_MOTION       },
{"HomeDirection",                            0,                       HOMEDIRECTION_MAX,       PARM_ATTR_RWI,           PD_ATTR_B2X,             0,                       PD_ASG_MOTION       },	// def was 1
{"HomeFeedRate",                             0,                       0,                       PARM_ATTR_RWNV_US,       PD_ATTR_V,               1.0,                     PD_ASG_MOTION       },
{"HomeOffset",                               0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.0,                     PD_ASG_MOTION       },
{"NumDecimalsEnglish",                       0,                       14,                      PARM_ATTR_RWI,           PD_ATTR_V_FDBK,          4,                       PD_ASG_UNITS        },
{"NumDecimalsMetric",                        0,                       14,                      PARM_ATTR_RWI,           PD_ATTR_V_FDBK,          3,                       PD_ASG_UNITS        },
{"NumDecimalsRotary",                        0,                       14,                      PARM_ATTR_RWI,           PD_ATTR_V,               3,                       PD_ASG_UNITS        },
{"AxisState",                                0,                       PHYSAXIS_STATE_MAX-1,    PARM_ATTR_RIU,           PD_ATTR_V,               PHYSAXIS_STATE_FREE,     PD_SG_NONE          },
{"ControllingTask",                          TASKINDEX_NONE,          MAX_TASKS-1,             PARM_ATTR_RIU,           PD_ATTR_V,               TASKINDEX_NONE,          PD_SG_NONE          },
{"PositionUnits",                            0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               0.0,                     PD_SG_NONE          },
{"PositionCmdUnits",                         0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               0.0,                     PD_SG_NONE          },
{"PositionProgCmdUnits",                     0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               0.0,                     PD_SG_NONE          },
{"AvgVelTimeMsec",                           0,                       1000,                    PARM_ATTR_RW,            PD_ATTR_V,               100,                     PD_SG_NONE          },	// def was 50
{"ScaleFactor",                              0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               0.0,                     PD_SG_NONE          },
{"PositionProgUnits",                        0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               0.0,                     PD_SG_NONE          },
{"FixtureOffset1",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.0,                     PD_ASG_MISC         },
{"FixtureOffset2",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.0,                     PD_ASG_MISC         },
{"FixtureOffset3",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.0,                     PD_ASG_MISC         },
{"FixtureOffset4",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.0,                     PD_ASG_MISC         },
{"FixtureOffset5",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.0,                     PD_ASG_MISC         },
{"FixtureOffset6",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.0,                     PD_ASG_MISC         },
{"JogDistance",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.5,                     PD_ASG_JOG          },
{"JogVelocity",                              0,                       0,                       PARM_ATTR_RWNV,          PD_ATTR_V,               1.0,                     PD_ASG_JOG          },
{"UnusedAxis",                               0,                       1,                       PARM_ATTR_RWIU,          PD_ATTR_V,               1,                       PD_SG_NONE          },
{"ReverseSlewDir",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_ASG_MOTION       },
{"VelTimeConst",                             0,                       ULONG_MAX,               PARM_ATTR_RWI_US,        PD_ATTR_V,               0,                       PD_ASG_FILTER       },
{"Clock",                                    0,                       0,                       PARM_ATTR_RWNIU,         PD_ATTR_V,               0,                       PD_SG_NONE          },
{"MaxFeedRate",                              0,                       0,                       PARM_ATTR_RWNV_US,       PD_ATTR_V,               500.0,                   PD_ASG_MOTION       },
{"RapidFeedRate",                            0,                       0,                       PARM_ATTR_RWNV_US,       PD_ATTR_V,               250.0,                   PD_ASG_MOTION       },
{"AccelModeAxis",                            0,                       ACCELMODEAXIS_MAX,       PARM_ATTR_RWI,           PD_ATTR_C,               2,                       PD_ASG_MOTION       },
{"DecelModeAxis",                            0,                       ACCELMODEAXIS_MAX,       PARM_ATTR_RWI,           PD_ATTR_C,               2,                       PD_ASG_MOTION       },
{"AccelRateAxis",                            0,                       0,                       PARM_ATTR_RWNA_US,       PD_ATTR_V,               39.37,                   PD_ASG_MOTION       },
{"DecelRateAxis",                            0,                       0,                       PARM_ATTR_RWNA_US,       PD_ATTR_V,               39.37,                   PD_ASG_MOTION       },
{"AccelTimeSecAxis",                         0,                       0,                       PARM_ATTR_RWN_US,        PD_ATTR_V,               0.1,                     PD_ASG_MOTION       },
{"DecelTimeSecAxis",                         0,                       0,                       PARM_ATTR_RWN_US,        PD_ATTR_V,               0.1,                     PD_ASG_MOTION       },
{"PositionCnts",                             0,                       0,                       PARM_ATTR_RNIU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"PositionCmdCnts",                          0,                       0,                       PARM_ATTR_RNIU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Fault",                                    0,                       0,                       PARM_ATTR_RWNIU,         PD_ATTR_BX,              0,                       PD_SG_NONE          },
{"DriveStatus",                              0,                       0,                       PARM_ATTR_RNIU,          PD_ATTR_BX,              0,                       PD_SG_NONE          },
{"AxisStatus",                               0,                       0,                       PARM_ATTR_RNIU,          PD_ATTR_BX,              0,                       PD_SG_NONE          },
{"CalDisable1D",                             0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_C,               0,                       PD_ASG_CONFIG       },
{"SlaveAdvance",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_ASG_MISC         },
{"SlaveOffset",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_ASG_MISC         },
{"GearMaster",                               0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               1.0,                     PD_ASG_MISC         },
{"GearSlave",                                0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               1.0,                     PD_ASG_MISC         },
{"GearMode",                                 0,                       GEARMODE_MAX-1,          PARM_ATTR_RWI,           PD_ATTR_H,               GEARMODE_OFF,            PD_ASG_MISC         },
{"VelocityCntsSecAvg",                       0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               0.0,                     PD_SG_NONE          },
{"ATAmplitude",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               1.0,                     PD_ASG_SERVO        },	// should be 1 inch
{"ATStartFrequency",                         0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               1.5,                     PD_ASG_SERVO        },
{"ATBandWidth",                              0,                       2000,                    PARM_ATTR_RW,            PD_ATTR_V,               30.0,                    PD_ASG_SERVO        },
{"ATDamping",                                0.1,                     0.9,                     PARM_ATTR_RW,            PD_ATTR_V,               0.7,                     PD_ASG_SERVO        },
{"ATPhaseMargin",                            0,                       180,                     PARM_ATTR_RW,            PD_ATTR_V,               60.0,                    PD_ASG_SERVO        },	// per ARW, was 45
{"JoyStickLowFeedRate",                      0,                       0,                       PARM_ATTR_RWNV,          PD_ATTR_V,               1,                       PD_ASG_JOG          },
{"JoyStickHighFeedRate",                     0,                       0,                       PARM_ATTR_RWNV,          PD_ATTR_V,               5,                       PD_ASG_JOG          },
{"HomeAccelDecelRateAxis",                   0,                       0,                       PARM_ATTR_RWNA_US,       PD_ATTR_V,               39.37,                   PD_ASG_MOTION       },
{"AffDecelScale",                            0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               1.0,                     PD_ASG_MOTION       },
{"SafeLimitMode",                            SAFE_MODE_OFF,           SAFE_MODE_NO_ENTER,      PARM_ATTR_RWI_US,        PD_ATTR_V,               SAFE_MODE_OFF,           PD_ASG_FAULT        },
{"SafeZoneCWCnts",                           0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_ASG_FAULT        },
{"SafeZoneCCWCnts",                          0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_ASG_FAULT        },
{"PositionExtUnits",                         0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               0.0,                     PD_SG_NONE          },
{"PositionExtCnts",                          0,                       0,                       PARM_ATTR_RNIU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"SoftLimitMode",                            0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_BX,              SOFTLIMIT_MODE_NOT_ACTIVE_UNTIL_HOMED, PD_ASG_FAULT},
{"PositionErrUnits",                         0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               0.0,                     PD_SG_NONE          },
{"PositionErrCnts",                          0,                       0,                       PARM_ATTR_RNIU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"EnableGainCalibration",                    0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_C,               1,                       PD_ASG_CONFIG       },
{"InPos2PeakToPeakError",                    0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_H,               0,                       PD_SG_NONE          },
{"InPos2WindowLengthMsec",                   0,                       50,                      PARM_ATTR_RWI_US,        PD_ATTR_H,               0,                       PD_SG_NONE          },
{"HomePositionSet",                          0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.0,                     PD_ASG_MOTION       },
{"CalBackLashTimeConst",                     0,                       ULONG_MAX,               PARM_ATTR_RWI_US,        PD_ATTR_V,               0,                       PD_ASG_FILTER       },
{"Reserved5",                                0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved6",                                0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved7",                                0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved8",                                0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved9",                                0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved10",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved11",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved12",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved13",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved14",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved15",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved16",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved17",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved18",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved19",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved20",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved21",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved22",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved23",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved24",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved25",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved26",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved27",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved28",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Reserved29",                               0,                       0,                       PARM_ATTR_NU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
/* Drive parameters start here */
{"ServoUpdateRate",                          0,                       8,                       PARM_ATTR_RWI64_USD,     PD_ATTR_V,               1,                       PD_ASG_SERVO        },
{"ServoMask",                                0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              5,                       PD_ASG_SERVO        },
{"GainKpos",                                 0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V_PID,           150.0,                   PD_ASG_SERVO        },
{"GainKi",                                   0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V_PID,           6000.0,                  PD_ASG_SERVO        },
{"GainKp",                                   0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V_PID,           44000.0,                 PD_ASG_SERVO        },
{"GainVff",                                  0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0,                       PD_ASG_SERVO        },
{"GainAff",                                  0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0,                       PD_ASG_SERVO        },
{"GainKv",                                   0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               1.0,                     PD_ASG_SERVO        },
{"GainKpi",                                  0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"FiltCurrent1N0",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.090205,                PD_ASG_FILTER       },	// def. was 0
{"FiltCurrent1N1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.180411,                PD_ASG_FILTER       },	// def. was 0
{"FiltCurrent1N2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.090205,                PD_ASG_FILTER       },	// def. was 0
{"FiltCurrent1D1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           -0.989472,               PD_ASG_FILTER       },	// def. was 0
{"FiltCurrent1D2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.350293,                PD_ASG_FILTER       },	// def. was 0
{"FiltCurrent2N0",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           1.0,                     PD_ASG_FILTER       },	// def. was 0
{"FiltCurrent2N1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent2N2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent2D1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent2D2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"IGainDTimeUsec",                           0,                       12.775,                  PARM_ATTR_RW32_USD,      PD_ATTR_V,               3,                       PD_ASG_SERVO        },
{"IGainDTimeCorrectUSec",                    0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0,                       PD_ASG_SERVO        },
{"IGainK",                                   0,                       ULONG_MAX,               PARM_ATTR_RWI64_USD,     PD_ATTR_V_PI,            256,                     PD_ASG_SERVO        },	// def. was 1
{"GainReserved",                             0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               0,                       PD_ASG_SERVO        },
{"IGainKi",                                  0,                       ULONG_MAX,               PARM_ATTR_RWI64_USD,     PD_ATTR_V_PI,            8600,                    PD_ASG_SERVO        },	// def was 8388
{"IGainKp",                                  0,                       ULONG_MAX,               PARM_ATTR_RWI64_USD,     PD_ATTR_V_PI,            51000,                   PD_ASG_SERVO        },	// def. was 5872025
{"IGainIaOffset",                            0,                       0,                       PARM_ATTR_RWN32D,        PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"IGainIbOffset",                            0,                       0,                       PARM_ATTR_RWN32D,        PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"IGainVaOffset",                            0,                       0,                       PARM_ATTR_RWN32D,        PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"IGainVbOffset",                            0,                       0,                       PARM_ATTR_RWN32D,        PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"IGainVcOffset",                            0,                       0,                       PARM_ATTR_RWN32D,        PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"FaultMask",                                0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0x1407CF,                PD_ASG_FAULT        },
{"FaultMaskDisable",                         0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0x160BC3,                PD_ASG_FAULT        },
{"FaultMaskDecel",                           0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0x160C0E,                PD_ASG_FAULT        },
{"FaultMaskStop",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0,                       PD_ASG_FAULT        },
{"FaultMaskInterrupt",                       0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0,                       PD_ASG_FAULT        },
{"BrakeOnDriveDisable",                      0,                       1,                       PARM_ATTR_RWI64_USD,     PD_ATTR_C,               0,                       PD_ASG_MISC         },
{"FaultMaskAux",                             0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0,                       PD_ASG_FAULT        },
{"FaultEStopInput",                          -2,                      21,                      PARM_ATTR_RWI64D,        PD_ATTR_V,               FAULT_ESTOP_NONE,        PD_ASG_FAULT        },
{"ThresholdPosErr",                          0,                       0,                       PARM_ATTR_RWNS64_USD,    PD_ATTR_V,               0.15748031,              PD_ASG_FAULT        },
{"ThresholdAvgIAmp",                         0,                       100,                     PARM_ATTR_RW32_USD,      PD_ATTR_V_MTR,           2.6,                     PD_ASG_FAULT        },
{"ThresholdAvgITimeMsec",                    0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_V,               4000,                    PD_ASG_FAULT        },
{"ThresholdVelCmd",                          0,                       0,                       PARM_ATTR_RWNV64_USD,    PD_ATTR_V,               501,                     PD_ASG_FAULT        },	// def was 12725.4
{"ThresholdVelError",                        0,                       0,                       PARM_ATTR_RWNV64_USD,    PD_ATTR_V,               2.36,                    PD_ASG_FAULT        },	// def. was 60
{"ThresholdSoftCCW",                         0,                       0,                       PARM_ATTR_RWNS64D,       PD_ATTR_V,               0,                       PD_ASG_FAULT        },
{"ThresholdSoftCW",                          0,                       0,                       PARM_ATTR_RWNS64D,       PD_ATTR_V,               0,                       PD_ASG_FAULT        },
{"ThresholdClampIAmp",                       0,                       150,                     PARM_ATTR_RW32_USD,      PD_ATTR_V_MTR,           10.6,                    PD_ASG_FAULT        },
{"ThresholdInPosDist",                       0,                       0,                       PARM_ATTR_RWNS64D,       PD_ATTR_V,               0.00019685,              PD_ASG_FAULT        },
{"CfgMotType",                               0,                       CFG_MOTOR_MAX-1,         PARM_ATTR_RWI64_USD,     PD_ATTR_C_MTR,           CFG_MOTOR_AC_BRUSHLESS,  PD_ASG_CONFIG       },
{"CfgMotCyclesRev",                          0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_V_MTR,           4,                       PD_ASG_CONFIG       },
{"CfgMotCntsRev",                            0,                       0,                       PARM_ATTR_RWN64_USD,     PD_ATTR_V_MTR,           4000.0,                  PD_ASG_CONFIG       },
{"CfgMotCntsInternal",                       0,                       0,                       PARM_ATTR_RWN64_USD,     PD_ATTR_H,               0,                       PD_ASG_CONFIG       },
{"CfgMotOffsetAng",                          0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgMotMSetTimeMsec",                       0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_V,               1000,                    PD_ASG_CONFIG       },
{"CfgMotMSetIAmps",                          0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               1.0,                     PD_ASG_CONFIG       },
{"CfgFbkPosType",                            0,                       CFG_FDBCK_TYPE_MAX-1,    PARM_ATTR_RWI64_USD,     PD_ATTR_C_FDBK,          CFG_FDBCK_TYPE_ENCODER,  PD_ASG_CONFIG       },
{"CfgFbkPosChan",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_C,               0,                       PD_ASG_CONFIG       },
{"CfgFbkPosMultiplier",                      0,                       0,                       PARM_ATTR_RWNI64D,       PD_ATTR_V,               1,                       PD_ASG_CONFIG       },
{"CfgFbkVelType",                            0,                       CFG_FDBCK_TYPE_MAX-1,    PARM_ATTR_RWI64_USD,     PD_ATTR_C_FDBK,          CFG_FDBCK_TYPE_NULL,     PD_ASG_CONFIG       },
{"CfgFbkVelChan",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_C,               0,                       PD_ASG_CONFIG       },
{"CfgFbkVelMultiplier",                      0,                       0,                       PARM_ATTR_RWNI64D,       PD_ATTR_V,               1.0,                     PD_ASG_CONFIG       },
{"CfgFbkEncMxhSetup",                        0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              1,                       PD_ASG_CONFIG       },
{"CfgFbkEncMultFactorMXH",                   1.0,                     16384.0,                 PARM_ATTR_RW64_USD,      PD_ATTR_V_FDBK,          1.0,                     PD_ASG_CONFIG       },
{"CfgFbkEncMultFactorMXU",                   1,                       4096,                    PARM_ATTR_RWI64_USD,     PD_ATTR_V_FDBK,          1.0,                     PD_ASG_CONFIG       },
{"CfgFbkEncSineGain",                        0,                       0xFF,                    PARM_ATTR_RWI64_USD,     PD_ATTR_V,               150,                     PD_ASG_CONFIG       },	// def. was 50
{"CfgFbkEncSineOffset",                      0,                       0xFF,                    PARM_ATTR_RWI64_USD,     PD_ATTR_V,               128,                     PD_ASG_CONFIG       },
{"CfgFbkEncCosGain",                         0,                       0xFF,                    PARM_ATTR_RWI64_USD,     PD_ATTR_V,               150,                     PD_ASG_CONFIG       },	// def. was 50
{"CfgFbkEncCosOffset",                       0,                       0xFF,                    PARM_ATTR_RWI64_USD,     PD_ATTR_V,               128,                     PD_ASG_CONFIG       },
{"CfgFbkEncPhase",                           0,                       0xFF,                    PARM_ATTR_RWI64_USD,     PD_ATTR_V,               128,                     PD_ASG_CONFIG       },
{"CfgGantryMasterAxis",                      0,                       MAX_AXES,                PARM_ATTR_RWI64_USD,     PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgGantrySlaveAxis",                       0,                       MAX_AXES,                PARM_ATTR_RWI64_USD,     PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"InetDriveIPAddress",                       0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_E,               174391727,               PD_ASG_ETHERNET     },
{"InetDriveSubnetMask",                      0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_E,               4294901760,              PD_ASG_ETHERNET     },
{"InetDriveGateway",                         0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_E,               174391297,               PD_ASG_ETHERNET     },
{"InetIOIPAddress",                          0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_E,               174391728,               PD_ASG_ETHERNET     },
{"InetIOSubnetMask",                         0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_E,               4294901760,              PD_ASG_ETHERNET     },
{"InetIOGateway",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_E,               174391297,               PD_ASG_ETHERNET     },
{"InetDefNumInputWords",                     0,                       MAX_INET_DEF,            PARM_ATTR_RWI64_USD,     PD_ATTR_V,               2,                       PD_ASG_ETHERNET     },
{"InetDefNumOutputWords",                    0,                       MAX_INET_DEF,            PARM_ATTR_RWI64_USD,     PD_ATTR_V,               2,                       PD_ASG_ETHERNET     },
{"InetDefNumInputBits",                      0,                       MAX_INET_DEF,            PARM_ATTR_RWI64_USD,     PD_ATTR_V,               4,                       PD_ASG_ETHERNET     },
{"InetDefNumOutputBits",                     0,                       MAX_INET_DEF,            PARM_ATTR_RWI64_USD,     PD_ATTR_V,               4,                       PD_ASG_ETHERNET     },
{"InetDefNumInputProcess",                   0,                       MAX_INET_DEF,            PARM_ATTR_RWI64_USD,     PD_ATTR_V,               0,                       PD_ASG_ETHERNET     },
{"InetDefNumOutputProcess",                  0,                       MAX_INET_DEF,            PARM_ATTR_RWI64_USD,     PD_ATTR_V,               0,                       PD_ASG_ETHERNET     },
{"InetConfigFlags",                          0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0,                       PD_ASG_ETHERNET     },
{"DecelRateMoveAbort",                       0,                       0,                       PARM_ATTR_RWNA32_USD,    PD_ATTR_V,               39.37,                   PD_ASG_MOTION       },
{"LimitDecelDistCnts",                       0,                       281474976710656,         PARM_ATTR_RWI64_USD,     PD_ATTR_V,               2000,                    PD_ASG_MOTION       },
{"LimitDebounceTimeMsec",                    0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_V,               100,                     PD_ASG_MOTION       },
{"LimitLevelMask",                           0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              7,                       PD_ASG_CONFIG       },
{"BacklashDistCnts",                         0,                       0,                       PARM_ATTR_RWNI64D,       PD_ATTR_V,               0,                       PD_ASG_MISC         },
{"AuxOutBitMask",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0,                       PD_ASG_FAULT        },
{"AuxOutBitLevel",                           0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0,                       PD_ASG_FAULT        },
{"DriveIOConfig",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0x42002207,              PD_ASG_CONFIG       },
{"BrakeOutput",                              -1,                      64,                      PARM_ATTR_RWI64D,        PD_ATTR_V,               -1,                      PD_ASG_MISC         },
{"PsoSsi1Config",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_C,               1,                       PD_ASG_PSO          },
{"PsoSsi2Config",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_C,               1,                       PD_ASG_PSO          },
{"PsoMrk1Config",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_C,               1,                       PD_ASG_PSO          },
{"PsoMrk2Config",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_C,               1,                       PD_ASG_PSO          },
{"PsoMrk3Config",                            0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_C,               1,                       PD_ASG_PSO          },
{"EncoderDivider",                           0,                       1023,                    PARM_ATTR_RWI64_USD,     PD_ATTR_V,               0,                       PD_ASG_MISC         },
{"FaultAuxInput",                            0,                       0,                       PARM_ATTR_RWNI64D,       PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_ASG_FAULT        },
{"FaultDisableDelay",                        0,                       1000,                    PARM_ATTR_RWI64_USD,     PD_ATTR_V,               0,                       PD_ASG_FAULT        },
{"CfgGantryMode",                            0,                       GANTRY_MODE_MAX-1,       PARM_ATTR_RWI64_USD,     PD_ATTR_V,               GANTRY_MODE_NONE,        PD_ASG_CONFIG       },
{"CfgMotStepperRes",                         0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgMotStepperHighCur",                     0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgMotStepperLowCur",                      0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgMotStepperDGain",                       0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgPhsAdvMaxAngle",                        0,                       0,                       PARM_ATTR_RWN32U_USD,    PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgPhsAdvMaxVel",                          0,                       0,                       PARM_ATTR_RWNI64U_USD,   PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgPhsAdvSegAngle",                        0,                       0,                       PARM_ATTR_RWN32U_USD,    PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgPhsAdvSegVel",                          0,                       0,                       PARM_ATTR_RWNI64U_USD,   PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgFbkEncQuadDivider",                     0,                       16384.0,                 PARM_ATTR_RW32_USD,      PD_ATTR_V,               1,                       PD_ASG_CONFIG       },
{"CfgFbkEncQuadChan",                        0,                       3,                       PARM_ATTR_RWI64_USD,     PD_ATTR_C,               0,                       PD_ASG_CONFIG       },
{"GainDff",                                  0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0,                       PD_ASG_SERVO        },
{"GainSFComp",                               -150,                    150,                     PARM_ATTR_RW32D,         PD_ATTR_V,               0,                       PD_ASG_SERVO        },
{"FiltCurrent3N0",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           1.0,                     PD_ASG_FILTER       },
{"FiltCurrent3N1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent3N2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent3D1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent3D2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent4N0",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           1.0,                     PD_ASG_FILTER       },
{"FiltCurrent4N1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent4N2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent4D1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent4D2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"CfgFbkRDGain",                             0,                       127,                     PARM_ATTR_RWI64_USD,     PD_ATTR_V,               0.0,                     PD_ASG_CONFIG       },
{"CfgFbkRDCosPhase",                         0.0,                     180.0,                   PARM_ATTR_RW32_USD,      PD_ATTR_V,               90.0,                    PD_ASG_CONFIG       },
{"CfgFbkRDConfig",                           0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0.0,                     PD_ASG_CONFIG       },
{"ExtAmpMaxCurrent",                         0.0001,                  ULONG_MAX,               PARM_ATTR_RW32_USD,      PD_ATTR_V,               20.0,                    PD_ASG_CONFIG       },
{"LimitDebounceDistCnts",                    0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_V,               0,                       PD_ASG_MOTION       },
{"CfgEnableDelay",                           0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"CfgMotStepperVerSpeed",                    0,                       100,                     PARM_ATTR_RW32_USD,      PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"ThresholdInPosTimeMSec",                   0,                       LONG_MAX,                PARM_ATTR_RWI64_USD,     PD_ATTR_V,               0.0,                     PD_ASG_FAULT        },
{"CfgFbkEncMxMrkLoc",                        0,                       359,                     PARM_ATTR_RWI64_USD,     PD_ATTR_V,               0,                       PD_ASG_CONFIG       },
{"FiltCurrent5N0",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           1.0,                     PD_ASG_FILTER       },
{"FiltCurrent5N1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent5N2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent5D1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent5D2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent6N0",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           1.0,                     PD_ASG_FILTER       },
{"FiltCurrent6N1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent6N2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent6D1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent6D2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent7N0",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           1.0,                     PD_ASG_FILTER       },
{"FiltCurrent7N1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent7N2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent7D1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent7D2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent8N0",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           1.0,                     PD_ASG_FILTER       },
{"FiltCurrent8N1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent8N2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent8D1",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"FiltCurrent8D2",                           -4,                      FILT_MAX,                PARM_ATTR_RW32D,         PD_ATTR_V_FLT,           0.0,                     PD_ASG_FILTER       },
{"EnableGainScaling",                        0,                       4,                       PARM_ATTR_RWI64_USD,     PD_ATTR_C,               0,                       PD_ASG_SERVO        },
{"GainScaleThreshLow",                       0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"GainScaleThreshHigh",                      0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"GainKpos2",                                0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               150.0,                   PD_ASG_SERVO        },
{"GainKi2",                                  0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               6000.0,                  PD_ASG_SERVO        },
{"GainKp2",                                  0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               44000.0,                 PD_ASG_SERVO        },
{"GainKpi2",                                 0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"FaultMaskDisableDelay",                    0,                       0,                       PARM_ATTR_RWNI64_USD,    PD_ATTR_BX,              0x00000,                 PD_ASG_FAULT        },
{"InPosDisableTimeoutMsec",                  0,                       60000,                   PARM_ATTR_RWI64_USD,     PD_ATTR_H,               0,                       PD_SG_NONE          },
{"BrakeDisableDelay",                        0,                       1000,                    PARM_ATTR_RWI64_USD,     PD_ATTR_V,               0,                       PD_ASG_MISC         },
{"GainScaleThreshLow2",                      0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"GainScaleThreshHigh2",                     0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"GainKpos3",                                0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               150.0,                   PD_ASG_SERVO        },
{"GainKi3",                                  0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               6000.0,                  PD_ASG_SERVO        },
{"GainKp3",                                  0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               44000.0,                 PD_ASG_SERVO        },
{"GainKpi3",                                 0,                       0,                       PARM_ATTR_RWN32_USD,     PD_ATTR_V,               0.0,                     PD_ASG_SERVO        },
{"CfgGantryCurrLimit",                       0,                       100,                     PARM_ATTR_RW32_USD,      PD_ATTR_H,               0,                       PD_ASG_CONFIG       },
{"CfgGantryDiffPosLimit",                    0,                       0,                       PARM_ATTR_RWNS64_USD,    PD_ATTR_V,               0.0,                     PD_ASG_CONFIG       },
};
#endif

/* Task Parameter Min, Max, default, attributes */
/*  (min/max/default values shown here for dist/vel/accel parms are always in "internal inits" (in|deg/sec)) */
#ifndef __INIT_AER_P_INFO_H__
	extern AER_PARM_INFO       TaskParmInfo[MAX_TASKPARMS];
#else
AER_PARM_INFO     TaskParmInfo[MAX_TASKPARMS]=
{// Parameter Name,                          Minimum,                 Maximum,                 Type Attribute,          Display Attribute,       DefaultValue,            DisplaySubGroup
{"Number",                                   0,                       MAX_TASKS-1,             PARM_ATTR_RI,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"TaskFault",                                0,                       0,                       PARM_ATTR_RWNIU,         PD_ATTR_X,               0,                       PD_SG_NONE          },
{"TaskWarning",                              0,                       0,                       PARM_ATTR_RWNIU,         PD_ATTR_X,               0,                       PD_SG_NONE          },
{"BoundAxesMask",                            0,                       AXISMASK_ALL,            PARM_ATTR_RWI,           PD_ATTR_BX,              0xFFFF,                  PD_TSG_AXES         },	//def. was 0
{"DisplayAxesMask",                          0,                       AXISMASK_ALL,            PARM_ATTR_RWI,           PD_ATTR_BX,              0xFFFF,                  PD_TSG_AXES         },	//def. was 0
{"RotateX",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWIU,          PD_ATTR_AS,              AXISINDEX_1,             PD_TSG_MOTION       },
{"RotateY",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWIU,          PD_ATTR_AS,              AXISINDEX_2,             PD_TSG_MOTION       },
{"RotateAngleDeg",                           0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               TASKPARM_ROTATEANGLEDEG_VALUE_OFF, PD_SG_NONE},
{"RThetaX",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWIU,          PD_ATTR_AS,              0,                       PD_TSG_MOTION       },
{"RThetaY",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWIU,          PD_ATTR_AS,              0,                       PD_TSG_MOTION       },
{"RThetaR",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWIU,          PD_ATTR_AS,              0,                       PD_TSG_MOTION       },
{"RThetaT",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWIU,          PD_ATTR_AS,              0,                       PD_TSG_MOTION       },
{"RThetaRadius",                             0,                       0,                       PARM_ATTR_RWNSU,         PD_ATTR_V,               0,                       PD_TSG_MOTION       },
{"RThetaEnabled",                            0,                       2,                       PARM_ATTR_RWIU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"AccelRateLinear",                          0,                       0,                       PARM_ATTR_RWNA_US,       PD_ATTR_V,               39.37,                   PD_TSG_MOTION       },
{"DecelRateLinear",                          0,                       0,                       PARM_ATTR_RWNA_US,       PD_ATTR_V,               39.37,                   PD_TSG_MOTION       },
{"AccelRateRotary",                          0,                       0,                       PARM_ATTR_RWNAT_US,      PD_ATTR_V,               360.0,                   PD_TSG_MOTION       },
{"DecelRateRotary",                          0,                       0,                       PARM_ATTR_RWNAT_US,      PD_ATTR_V,               360.0,                   PD_TSG_MOTION       },
{"AccelTimeSec",                             0,                       0,                       PARM_ATTR_RWN_US,        PD_ATTR_V,               .1,                      PD_TSG_MOTION       },
{"DecelTimeSec",                             0,                       0,                       PARM_ATTR_RWN_US,        PD_ATTR_V,               .1,                      PD_TSG_MOTION       },
{"DecelOnProgramAbortMask",                  0,                       AXISMASK_ALL,            PARM_ATTR_RWI,           PD_ATTR_BX,              AXISMASK_ALL,            PD_TSG_PROGRAM      },
{"FeedRateLinear",                           0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               0,                       PD_TSG_MOTION       },
{"FeedRateRotary",                           0,                       0,                       PARM_ATTR_RWNUT,         PD_ATTR_V,               0,                       PD_TSG_MOTION       },
{"FeedRateLinearDefault",                    0,                       0,                       PARM_ATTR_RWNV,          PD_ATTR_V,               125,                     PD_TSG_MOTION       },
{"FeedRateRotaryDefault",                    0,                       0,                       PARM_ATTR_RWNVT,         PD_ATTR_V,               360,                     PD_TSG_MOTION       },	//def. was 0
{"DryRunFeedRateLinear",                     0,                       0,                       PARM_ATTR_RWNV,          PD_ATTR_V,               5,                       PD_TSG_MOTION       },	//def. was 0
{"DryRunFeedRateRotary",                     0,                       0,                       PARM_ATTR_RWNVT,         PD_ATTR_V,               360,                     PD_TSG_MOTION       },	//def. was 0
{"FeedRateLinearActual",                     0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               0,                       PD_SG_NONE          },
{"FeedRateRotaryActual",                     0,                       0,                       PARM_ATTR_RNUT,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"BlendMaxAccelLinearIPS2",                  0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_MOTION       },
{"BlendMaxAccelRotaryDPS2",                  0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_MOTION       },
{"BlendMaxAccelCircleIPS2",                  0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_MOTION       },
{"MaxLookAheadMoves",                        1,                       1000000,                 PARM_ATTR_RWI,           PD_ATTR_V,               100,                     PD_TSG_MOTION       },
{"MFO",                                      0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               1,                       PD_SG_NONE          },
{"UserFeedRateMode",                         0,                       2+MAX_SPINDLES-1,        PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_MOTION       },
{"FeedHold",                                 0,                       1,                       PARM_ATTR_RWIU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Coord1I",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_1,             PD_TSG_CIRCULAR     },
{"Coord1J",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_2,             PD_TSG_CIRCULAR     },
{"Coord1K",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_3,             PD_TSG_CIRCULAR     },
{"Coord1Plane",                              PLANE_XY,                PLANE_YZ,                PARM_ATTR_RWI,           PD_ATTR_V,               PLANE_XY,                PD_TSG_CIRCULAR     },
{"Coord2I",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_9,             PD_TSG_CIRCULAR     },
{"Coord2J",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_10,            PD_TSG_CIRCULAR     },
{"Coord2K",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_11,            PD_TSG_CIRCULAR     },
{"Coord2Plane",                              PLANE_XY,                PLANE_YZ,                PARM_ATTR_RWI,           PD_ATTR_V,               PLANE_XY,                PD_TSG_CIRCULAR     },
{"MaxRadiusAdjustDeg",                       0,                       360.0,                   PARM_ATTR_RW,            PD_ATTR_V,               0.05,                    PD_TSG_CIRCULAR     },
{"MaxRadiusError",                           0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               5,                       PD_TSG_CIRCULAR     },
{"CutterX",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_1,             PD_TSG_AXES         },
{"CutterY",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_2,             PD_TSG_AXES         },
{"CutterZ",                                  -1,                      MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_V,               -1,                      PD_SG_NONE          },
{"CutterLength",                             0,                       0,                       PARM_ATTR_RWNSU,         PD_ATTR_V,               0,                       PD_SG_NONE          },
{"CutterWear",                               0,                       0,                       PARM_ATTR_RWNSU,         PD_ATTR_V,               0,                       PD_SG_NONE          },
{"CutterRadius",                             0,                       0,                       PARM_ATTR_RWNSU,         PD_ATTR_AS,              0,                       PD_SG_NONE          },
{"CutterOffsetX",                            0,                       0,                       PARM_ATTR_RWNSU,         PD_ATTR_V,               0,                       PD_SG_NONE          },
{"CutterOffsetY",                            0,                       0,                       PARM_ATTR_RWNSU,         PD_ATTR_V,               0,                       PD_SG_NONE          },
{"CutterActive",                             0,                       0,                       PARM_ATTR_RWNIU,         PD_ATTR_V,               0,                       PD_SG_NONE          },
{"CutterToleranceDeg",                       0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0.5,                     PD_SG_NONE          },
{"NormalcyX",                                0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_1,             PD_TSG_AXES         },
{"NormalcyY",                                0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_2,             PD_TSG_AXES         },
{"NormalcyAxis",                             0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_8,             PD_TSG_AXES         },
{"NormalcyToleranceDeg",                     0,                       20,                      PARM_ATTR_RW,            PD_ATTR_V,               0.01,                    PD_TSG_MOTION       },
{"Status1",                                  0,                       0xFFFFFFFF,              PARM_ATTR_RI,            PD_ATTR_BX,              0,                       PD_SG_NONE          },
{"Status2",                                  0,                       0xFFFFFFFF,              PARM_ATTR_RI,            PD_ATTR_BX,              0,                       PD_SG_NONE          },
{"Status3",                                  0,                       0xFFFFFFFF,              PARM_ATTR_RI,            PD_ATTR_BX,              0,                       PD_SG_NONE          },
{"Mode1",                                    0,                       0xFFFFFFFF,              PARM_ATTR_RWIU,          PD_ATTR_BX,              10,                      PD_SG_NONE          },
{"ErrCode",                                  0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_X,               0,                       PD_TSG_MISC         },
{"HaltTaskOnAxisFault",                      0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               1,                       PD_TSG_PROGRAM      },
{"InterruptMotion",                          0,                       1,                       PARM_ATTR_RWIU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"InterruptMotionReturnType",                0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               1,                       PD_TSG_PROGRAM      },
{"SoftEStopInput",                           0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_MISC         },
{"FeedHoldInput",                            0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_MISC         },
{"FeedHoldEdgeInput",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_MISC         },
{"AnalogMFOInput",                           0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_IA,              NO_SUCH_IO_BIT,          PD_TSG_MISC         },
{"S1_AnalogMSOInput",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_IA,              NO_SUCH_IO_BIT,          PD_TSG_SPINDLES     },
{"S2_AnalogMSOInput",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_IA,              NO_SUCH_IO_BIT,          PD_TSG_SPINDLES     },
{"S3_AnalogMSOInput",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_IA,              NO_SUCH_IO_BIT,          PD_TSG_SPINDLES     },
{"S4_AnalogMSOInput",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_IA,              NO_SUCH_IO_BIT,          PD_TSG_SPINDLES     },
{"S1_Index",                                 -1,                      MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              -1,                      PD_TSG_SPINDLES     },
{"S1_FeedRate",                              0,                       0,                       PARM_ATTR_RWNVUT,        PD_ATTR_V,               0,                       PD_TSG_SPINDLES     },
{"S2_Index",                                 -1,                      MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              -1,                      PD_TSG_SPINDLES     },
{"S2_FeedRate",                              0,                       0,                       PARM_ATTR_RWNVUT,        PD_ATTR_V,               0,                       PD_TSG_SPINDLES     },
{"S3_Index",                                 -1,                      MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              -1,                      PD_TSG_SPINDLES     },
{"S3_FeedRate",                              0,                       0,                       PARM_ATTR_RWNVUT,        PD_ATTR_V,               0,                       PD_TSG_SPINDLES     },
{"S4_Index",                                 -1,                      MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              -1,                      PD_TSG_SPINDLES     },
{"S4_FeedRate",                              0,                       0,                       PARM_ATTR_RWNVUT,        PD_ATTR_V,               0,                       PD_TSG_SPINDLES     },
{"S1_MSO",                                   0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               1,                       PD_SG_NONE          },
{"S2_MSO",                                   0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               1,                       PD_SG_NONE          },
{"S3_MSO",                                   0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               1,                       PD_SG_NONE          },
{"S4_MSO",                                   0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               1,                       PD_SG_NONE          },
{"S1_SpindleRadialAxis",                     0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_SPINDLES     },
{"S2_SpindleRadialAxis",                     0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_SPINDLES     },
{"S3_SpindleRadialAxis",                     0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_SPINDLES     },
{"S4_SpindleRadialAxis",                     0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_SPINDLES     },
{"ROReq1",                                   -1,                      MAX_VIRT_REGISTERS-1,    PARM_ATTR_RWI,           PD_ATTR_V,               -1,                      PD_TSG_PROGRAM      },
{"RIAction1",                                -1,                      MAX_VIRT_REGISTERS-1,    PARM_ATTR_RWI,           PD_ATTR_V,               -1,                      PD_TSG_PROGRAM      },
{"ROAction1",                                -1,                      MAX_VIRT_REGISTERS-1,    PARM_ATTR_RWI,           PD_ATTR_V,               -1,                      PD_TSG_PROGRAM      },
{"ROReq1Mask",                               -1,                      0xFFFF,                  PARM_ATTR_RWI,           PD_ATTR_V,               -1,                      PD_TSG_PROGRAM      },
{"RIActionOpCode",                           0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"RIActionAxis",                             0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"RIActionParm1",                            0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"RIActionParm2",                            0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"ActiveFixtureOffset",                      0,                       6,                       PARM_ATTR_RU,            PD_ATTR_V,               0,                       PD_SG_NONE          },
{"MaxCallStack",                             1,                       100,                     PARM_ATTR_RWI,           PD_ATTR_V,               10,                      PD_TSG_PROGRAM      },
{"MaxModeStack",                             0,                       100,                     PARM_ATTR_RWI,           PD_ATTR_V,               10,                      PD_TSG_PROGRAM      },
{"NumTaskDoubles",                           0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               10,                      PD_TSG_PROGRAM      },
{"NumTaskStrings",                           0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               10,                      PD_TSG_PROGRAM      },
{"NumTaskAxisPts",                           0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               10,                      PD_TSG_PROGRAM      },
{"MaxMonitorData",                           0,                       1000,                    PARM_ATTR_RWI,           PD_ATTR_V,               10,                      PD_TSG_PROGRAM      },
{"MaxOnGosubData",                           0,                       1000,                    PARM_ATTR_RWI,           PD_ATTR_V,               10,                      PD_TSG_PROGRAM      },
{"UpdateTimeSec",                            0.001,                   0.050,                   PARM_ATTR_RW,            PD_ATTR_V,               0.002,                   PD_TSG_MOTION       },
{"UpdateNumEntries",                         1,                       32,                      PARM_ATTR_RWI,           PD_ATTR_V,               4,                       PD_TSG_MOTION       },
{"ExecuteNumLines",                          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               1,                       PD_TSG_PROGRAM      },
{"ExecuteNumMonitors",                       0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               -1,                      PD_TSG_PROGRAM      },
{"ExecuteNumSpindles",                       0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               1,                       PD_TSG_PROGRAM      },
{"JogPair1EnableIn",                         0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JogPair1Mode",                             0,                       3,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"JogPair1Axis1",                            0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_1,             PD_TSG_JOGGING      },
{"JogPair1Axis1PlusIn",                      0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JogPair1Axis1MinusIn",                     0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JogPair1Axis2",                            0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_2,             PD_TSG_JOGGING      },
{"JogPair1Axis2PlusIn",                      0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JogPair1Axis2MinusIn",                     0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JogPair2EnableIn",                         0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JogPair2Mode",                             0,                       3,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"JogPair2Axis1",                            0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_3,             PD_TSG_JOGGING      },
{"JogPair2Axis1PlusIn",                      0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JogPair2Axis1MinusIn",                     0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JogPair2Axis2",                            0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              AXISINDEX_4,             PD_TSG_JOGGING      },
{"JogPair2Axis2PlusIn",                      0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JogPair2Axis2MinusIn",                     0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JoyStickDeadband",                         0,                       0xFFFFFFFF,              PARM_ATTR_RW,            PD_ATTR_V,               0.1,                     PD_TSG_JOGGING      },
{"SlewPair1",                                0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_SPX,             3,                       PD_TSG_JOGGING      },
{"SlewPair2",                                0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_SPX,             12,                      PD_TSG_JOGGING      },
{"SlewPair3",                                0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_SPX,             0,                       PD_TSG_JOGGING      },
{"SlewPair4",                                0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_SPX,             0,                       PD_TSG_JOGGING      },
{"SlewPair5",                                0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_SPX,             0,                       PD_TSG_JOGGING      },
{"SlewPair6",                                0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_SPX,             0,                       PD_TSG_JOGGING      },
{"SlewPair7",                                0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_SPX,             0,                       PD_TSG_JOGGING      },
{"SlewPair8",                                0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_SPX,             0,                       PD_TSG_JOGGING      },
{"LineNumberUser",                           0,                       0,                       PARM_ATTR_RNIU,          PD_ATTR_V,               -1,                      PD_SG_NONE          },
{"LineNumberSMC",                            0,                       0,                       PARM_ATTR_RNIU,          PD_ATTR_V,               -1,                      PD_SG_NONE          },	// name was LineNumber960
{"CannedFunctionID",                         0,                       0,                       PARM_ATTR_RWNIU,         PD_ATTR_V,               0,                       PD_SG_NONE          },
{"IgnoreAxesMask",                           0,                       AXISMASK_ALL,            PARM_ATTR_RWIU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"ChordicalToleranceInch",                   0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0.0001,                  PD_TSG_CIRCULAR     },	//def. was 0
{"ChordicalSlowdownMsec",                    -20,                     20,                      PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_MOTION       },
{"CommandVelocityVariance",                  0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0.1,                     PD_TSG_MOTION       },
{"Group1GCodeMode",                          0,                       0,                       PARM_ATTR_RWNU,          PD_ATTR_V,               0,                       PD_SG_NONE          },
{"SlewPair1Invert",                          0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"SlewPair2Invert",                          0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"SlewPair3Invert",                          0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"SlewPair4Invert",                          0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"SlewPair5Invert",                          0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"SlewPair6Invert",                          0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"SlewPair7Invert",                          0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"SlewPair8Invert",                          0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"JoyStickAnalogHorizInput",                 0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_IA,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JoyStickAnalogVertInput",                  0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_IA,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JoyStickDigitalAxisPairSelInput",          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JoyStickDigitalFeedRateSelInput",          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JoyStickDigitalInterlockInput",            0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_ID,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"JoyStickMinVoltage",                       0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"JoyStickMaxVoltage",                       0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               5,                       PD_TSG_JOGGING      },
{"Slew3DAxisMask1",                          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_S3DX,            7,                       PD_TSG_JOGGING      },
{"Slew3DAxisMask2",                          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_S3DX,            56,                      PD_TSG_JOGGING      },
{"Slew3DAxisMask3",                          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_S3DX,            0,                       PD_TSG_JOGGING      },
{"Slew3DAxisMask4",                          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_S3DX,            0,                       PD_TSG_JOGGING      },
{"Slew3DAxisMask5",                          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_S3DX,            0,                       PD_TSG_JOGGING      },
{"Slew3DAxisMask6",                          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_S3DX,            0,                       PD_TSG_JOGGING      },
{"Slew3DAxisMask7",                          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_S3DX,            0,                       PD_TSG_JOGGING      },
{"Slew3DAxisMask8",                          0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_S3DX,            0,                       PD_TSG_JOGGING      },
{"Slew3DConfig1",                            0,                       SLEW3DCONFIG_MAX,        PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"Slew3DConfig2",                            0,                       SLEW3DCONFIG_MAX,        PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"Slew3DConfig3",                            0,                       SLEW3DCONFIG_MAX,        PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"Slew3DConfig4",                            0,                       SLEW3DCONFIG_MAX,        PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"Slew3DConfig5",                            0,                       SLEW3DCONFIG_MAX,        PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"Slew3DConfig6",                            0,                       SLEW3DCONFIG_MAX,        PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"Slew3DConfig7",                            0,                       SLEW3DCONFIG_MAX,        PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"Slew3DConfig8",                            0,                       SLEW3DCONFIG_MAX,        PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"JoyStickAnalog3DInput",                    0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_IA,              NO_SUCH_IO_BIT,          PD_TSG_JOGGING      },
{"PciIODelayMode",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_TSG_MISC         },
{"AnalogMFOMinInputVoltage",                 0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               -10,                     PD_TSG_MISC         },
{"AnalogMFOMaxInputVoltage",                 0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               10,                      PD_TSG_MISC         },
{"MFOMax",                                   0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               2.0,                     PD_TSG_MISC         },
{"JoyStickVertMinVoltage",                   0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"JoyStickVertMaxVoltage",                   0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"JoyStickVertDeadband",                     0,                       0xFFFFFFFF,              PARM_ATTR_RW,            PD_ATTR_V,               0.1,                     PD_TSG_JOGGING      },
{"JoyStick3DMinVoltage",                     0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"JoyStick3DMaxVoltage",                     0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_TSG_JOGGING      },
{"JoyStick3DDeadband",                       0,                       0xFFFFFFFF,              PARM_ATTR_RW,            PD_ATTR_V,               0.1,                     PD_TSG_JOGGING      },
{"MFOMin",                                   0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0.0,                     PD_TSG_MISC         },
{"DwellPercentComplete",                     0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               100.0,                   PD_SG_NONE          },
{"WaitTimeMsec",                             0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               0,                       PD_SG_NONE          },
{"MFOStep",                                  0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               5,                       PD_TSG_MISC         },
{"DfltSCurve",                               0,                       100.0,                   PARM_ATTR_RW_US,         PD_ATTR_H,               50.0,                    PD_TSG_MOTION       },
};
#endif

/* Global Parameter Min, Max, default, attributes */
#ifndef __INIT_AER_P_INFO_H__
	extern AER_PARM_INFO       GlobalParmInfo[MAX_GLOBPARMS];
#else
AER_PARM_INFO     GlobalParmInfo[MAX_GLOBPARMS]=
{// Parameter Name,                          Minimum,                 Maximum,                 Type Attribute,          Display Attribute,       DefaultValue,            DisplaySubGroup
{"AvgPollTimeSec",                           0,                       0,                       PARM_ATTR_RNU,           PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Version",                                  0,                       0,                       PARM_ATTR_RN,            PD_ATTR_V,               A3200_VERSION_MAJOR+(A3200_VERSION_MINOR/100.0), PD_SG_NONE},
{"NumGlobalDoubles",                         0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               10,                      PD_SG_NONE          },
{"NumGlobalStrings",                         0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               10,                      PD_SG_NONE          },
{"NumGlobalAxisPts",                         0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               10,                      PD_SG_NONE          },
{"CallBackTimeoutSec",                       0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               1,                       PD_SG_NONE          },
{"BuildNumber",                              0,                       0,                       PARM_ATTR_RN,            PD_ATTR_V,               A3200_VERSION_BUILD,     PD_SG_NONE          },
{"UserMode",                                 0,                       0,                       PARM_ATTR_RWNIU,         PD_ATTR_V,               0,                       PD_SG_NONE          },
{"ThrowTaskWarningsAsFaults",                -1,                      1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_SG_NONE          },
{"Enable2DCalibration",                      0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               1,                       PD_SG_NONE          },
{"NumCannedFunctions",                       0,                       1000,                    PARM_ATTR_RWI,           PD_ATTR_V,               10,                      PD_SG_NONE          },
{"CompatibilityMode",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_BX,              0xDE40632,               PD_SG_NONE          },
{"NumDecimalsCompare",                       0,                       14,                      PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_SG_NONE          },
{"CNCMeasurementMode",                       0,                       0,                       PARM_ATTR_RWNIU,         PD_ATTR_V,               0,                       PD_SG_NONE          },
{"MaxNPlotAxes",                             0,                       MAX_AXES,                PARM_ATTR_RWI,           PD_ATTR_V,               6,                       PD_SG_NONE          },
{"PositionsDisplayMask",                     0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_BX,              0,                       PD_SG_NONE          },
{"InetGlobalIOIPAddress",                    0,                       0,                       PARM_ATTR_RWNI_US,       PD_ATTR_E,               174391728,               PD_GSG_ETHERNET     },
{"InetGlobalIOSubnetMask",                   0,                       0,                       PARM_ATTR_RWNI_US,       PD_ATTR_E,               4294901760,              PD_GSG_ETHERNET     },
{"InetGlobalIOGateway",                      0,                       0,                       PARM_ATTR_RWNI_US,       PD_ATTR_E,               174391297,               PD_GSG_ETHERNET     },
{"InetGlobalDefNumInputWords",               0,                       MAX_INET_DEF,            PARM_ATTR_RWI_US,        PD_ATTR_V,               2,                       PD_GSG_ETHERNET     },
{"InetGlobalDefNumOutputWords",              0,                       MAX_INET_DEF,            PARM_ATTR_RWI_US,        PD_ATTR_V,               2,                       PD_GSG_ETHERNET     },
{"InetGlobalDefNumInputBits",                0,                       MAX_INET_DEF,            PARM_ATTR_RWI_US,        PD_ATTR_V,               4,                       PD_GSG_ETHERNET     },
{"InetGlobalDefNumOutputBits",               0,                       MAX_INET_DEF,            PARM_ATTR_RWI_US,        PD_ATTR_V,               4,                       PD_GSG_ETHERNET     },
{"InetGlobalDefNumInputProcess",             0,                       MAX_INET_DEF,            PARM_ATTR_RWI_US,        PD_ATTR_V,               0,                       PD_GSG_ETHERNET     },
{"InetGlobalDefNumOutputProcess",            0,                       MAX_INET_DEF,            PARM_ATTR_RWI_US,        PD_ATTR_V,               0,                       PD_GSG_ETHERNET     },
{"InetGlobalConfigFlags",                    0,                       0,                       PARM_ATTR_RWNI_US,       PD_ATTR_BX,              0,                       PD_GSG_ETHERNET     },
{"InetGlobalInputBitsStart",                 0,                       63,                      PARM_ATTR_RWI_US,        PD_ATTR_V,               2,                       PD_GSG_ETHERNET     },
{"InetGlobalOutputBitsStart",                0,                       63,                      PARM_ATTR_RWI_US,        PD_ATTR_V,               2,                       PD_GSG_ETHERNET     },
{"InetGlobalOutputBitsStatusStart",          0,                       63,                      PARM_ATTR_RWI_US,        PD_ATTR_V,               18,                      PD_GSG_ETHERNET     },
{"InetGlobalInputWordsStart",                0,                       896,                     PARM_ATTR_RWI_US,        PD_ATTR_V,               128,                     PD_GSG_ETHERNET     },
{"InetGlobalOutputWordsStart",               0,                       896,                     PARM_ATTR_RWI_US,        PD_ATTR_V,               128,                     PD_GSG_ETHERNET     },
{"InetGlobalOutputWordsStatusStart",         0,                       896,                     PARM_ATTR_RWI_US,        PD_ATTR_V,               384,                     PD_GSG_ETHERNET     },
{"InetGlobalInputProcessStart",              0,                       896,                     PARM_ATTR_RWI_US,        PD_ATTR_V,               384,                     PD_GSG_ETHERNET     },
{"InetGlobalOutputProcessStart",             0,                       896,                     PARM_ATTR_RWI_US,        PD_ATTR_V,               640,                     PD_GSG_ETHERNET     },
{"NumDrivesRequired",                        0,                       MAX_AXES,                PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_SG_NONE          },
{"ClockResMultiplier",                       1,                       1000,                    PARM_ATTR_RWI,           PD_ATTR_V,               1,                       PD_SG_NONE          },
{"1394CommLostNumber",                       0,                       USHRT_MAX,               PARM_ATTR_RWI,           PD_ATTR_V,               50,                      PD_SG_NONE          },
{"MaxNPlotPoints",                           0,                       32000,                   PARM_ATTR_RWI,           PD_ATTR_V,               14000,                   PD_SG_NONE          },
};
#endif

/* Fiber Parameter Min, Max, default, attributes */
#ifndef __INIT_AER_P_INFO_H__
	extern AER_PARM_INFO       FiberParmInfo[MAX_FIBERPARMS];
#else
AER_PARM_INFO     FiberParmInfo[MAX_FIBERPARMS]=
{// Parameter Name,                          Minimum,                 Maximum,                 Type Attribute,          Display Attribute,       DefaultValue,            DisplaySubGroup
{"HCScanIncrement",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWNS,          PD_ATTR_V,               0.0003937007874,         PD_FSG_HILLCLIMB    },
{"HCMaxDisplacement",                        0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0.03937007874,           PD_FSG_HILLCLIMB    },
{"HCThreshold",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_FSG_HILLCLIMB    },
{"HCAxis",                                   0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              0,                       PD_FSG_HILLCLIMB    },
{"HCInputMode",                              0,                       2,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_HILLCLIMB    },
{"HCInputChannelNum",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               NO_SUCH_IO_BIT,          PD_FSG_HILLCLIMB    },
{"HCInvertSearch",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_HILLCLIMB    },
{"HCWholeWindow",                            0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_HILLCLIMB    },
{"HCDataSaveMode",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_HILLCLIMB    },
{"HCDelayTime",                              0,                       0xFFFFFFFF,              PARM_ATTR_RWI,           PD_ATTR_V,               50,                      PD_FSG_HILLCLIMB    },
{"SRMaxRadius",                              0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0.03937007874,           PD_FSG_SPIRALROUGH  },
{"SRNumSpirals",                             0,                       0xFFFFFFFF,              PARM_ATTR_RWI,           PD_ATTR_V,               10,                      PD_FSG_SPIRALROUGH  },
{"SRSegmentLength",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0.0003937007874,         PD_FSG_SPIRALROUGH  },
{"SRThreshold",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               1,                       PD_FSG_SPIRALROUGH  },
{"SRAxis1",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              0,                       PD_FSG_SPIRALROUGH  },
{"SRAxis2",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              1,                       PD_FSG_SPIRALROUGH  },
{"SRInputMode",                              0,                       2,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_SPIRALROUGH  },
{"SRInputChannelNum",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               NO_SUCH_IO_BIT,          PD_FSG_SPIRALROUGH  },
{"SRInvertSearch",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_SPIRALROUGH  },
{"SRMotionType",                             0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_SPIRALROUGH  },
{"SRDataSaveMode",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_SPIRALROUGH  },
{"SRDelayTime",                              0,                       0xFFFFFFFF,              PARM_ATTR_RWI,           PD_ATTR_V,               50,                      PD_FSG_SPIRALROUGH  },
{"SFEndRadius",                              0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0.03937007874,           PD_FSG_SPIRALFINE   },
{"SFNumSpirals",                             0,                       0xFFFFFFFF,              PARM_ATTR_RWI,           PD_ATTR_V,               10,                      PD_FSG_SPIRALFINE   },
{"SFSegmentLength",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0.0003937007874,         PD_FSG_SPIRALFINE   },
{"SFAxis1",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              0,                       PD_FSG_SPIRALFINE   },
{"SFAxis2",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              1,                       PD_FSG_SPIRALFINE   },
{"SFInputMode",                              0,                       2,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_SPIRALFINE   },
{"SFInputChannelNum",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               NO_SUCH_IO_BIT,          PD_FSG_SPIRALFINE   },
{"SFInvertSearch",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_SPIRALFINE   },
{"SFMotionType",                             0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_SPIRALFINE   },
{"SFDataSaveMode",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_SPIRALFINE   },
{"SFDelayTime",                              0,                       0xFFFFFFFF,              PARM_ATTR_RWI,           PD_ATTR_V,               50,                      PD_FSG_SPIRALFINE   },
{"FASelectAxis1",                            0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              0,                       PD_FSG_FASTALIGN    },
{"FASelectAxis2",                            0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              1,                       PD_FSG_FASTALIGN    },
{"FASelectAxis3",                            0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              2,                       PD_FSG_FASTALIGN    },
{"FASelectAxis4",                            0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              3,                       PD_FSG_FASTALIGN    },
{"FASelectAxis5",                            0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              4,                       PD_FSG_FASTALIGN    },
{"FASelectAxis6",                            0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              5,                       PD_FSG_FASTALIGN    },
{"FAOffsetAxis1",                            0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.003937007874,          PD_FSG_FASTALIGN    },
{"FAOffsetAxis2",                            0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.003937007874,          PD_FSG_FASTALIGN    },
{"FAOffsetAxis3",                            0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.003937007874,          PD_FSG_FASTALIGN    },
{"FAOffsetAxis4",                            0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,                0.003937007874,         PD_FSG_FASTALIGN    },
{"FAOffsetAxis5",                            0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.003937007874,          PD_FSG_FASTALIGN    },
{"FAOffsetAxis6",                            0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0.003937007874,          PD_FSG_FASTALIGN    },
{"FAPosLimitAxis1",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FAPosLimitAxis2",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FAPosLimitAxis3",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FAPosLimitAxis4",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FAPosLimitAxis5",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FAPosLimitAxis6",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FANegLimitAxis1",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FANegLimitAxis2",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FANegLimitAxis3",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FANegLimitAxis4",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FANegLimitAxis5",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FANegLimitAxis6",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FATermTolerance",                          0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0.01,                    PD_FSG_FASTALIGN    },
{"FAMaxNumIterations",                       0,                       0xFFFFFFFF,              PARM_ATTR_RWI,           PD_ATTR_V,               500,                     PD_FSG_FASTALIGN    },
{"FASaturationValue",                        0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FAReturnToStart",                          0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FAInputMode",                              0,                       2,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FAInputChannelNum",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               NO_SUCH_IO_BIT,          PD_FSG_FASTALIGN    },
{"FAInvertSearch",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_FASTALIGN    },
{"FADelayTime",                              0,                       0xFFFFFFFF,              PARM_ATTR_RWI,           PD_ATTR_V,               50,                      PD_FSG_FASTALIGN    },
{"GCScanSize",                               0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0.03937007874,           PD_FSG_GEOCENTER    },
{"GCScanIncrement",                          0,                       0xFFFFFFFF,              PARM_ATTR_RWNS,          PD_ATTR_V,               0.00003937007874,        PD_FSG_GEOCENTER    },
{"GCScanLines",                              0,                       0xFFFFFFFF,              PARM_ATTR_RWI,           PD_ATTR_V,               10,                      PD_FSG_GEOCENTER    },
{"GCEdgeValue",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               4,                       PD_FSG_GEOCENTER    },
{"GCAxis1",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              0,                       PD_FSG_GEOCENTER    },
{"GCAxis2",                                  0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              1,                       PD_FSG_GEOCENTER    },
{"GCInputMode",                              0,                       2,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_GEOCENTER    },
{"GCInputChannelNum",                        0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               NO_SUCH_IO_BIT,          PD_FSG_GEOCENTER    },
{"GCInvertSearch",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_GEOCENTER    },
{"GCSingleRasterMode",                       0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_GEOCENTER    },
{"GCMotionType",                             0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_GEOCENTER    },
{"GCDataSaveMode",                           0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_GEOCENTER    },
{"GCDelayTime",                              0,                       0xFFFFFFFF,              PARM_ATTR_RWI,           PD_ATTR_V,               50,                      PD_FSG_GEOCENTER    },
{"CMaxDisplacement1",                        0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0.03937007874,           PD_FSG_CENTROID     },
{"CMaxDisplacement2",                        0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0.03937007874,           PD_FSG_CENTROID     },
{"CMaxDisplacement3",                        0,                       0xFFFFFFFF,              PARM_ATTR_RWS,           PD_ATTR_V,               0.03937007874,           PD_FSG_CENTROID     },
{"CScanIncrement",                           0,                       0xFFFFFFFF,              PARM_ATTR_RWNS,          PD_ATTR_V,               0.00003937007874,        PD_FSG_CENTROID     },
{"CEdgeValue",                               0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               4,                       PD_FSG_CENTROID     },
{"CAxis1",                                   0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              0,                       PD_FSG_CENTROID     },
{"CAxis2",                                   0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              1,                       PD_FSG_CENTROID     },
{"CAxis3",                                   0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              2,                       PD_FSG_CENTROID     },
{"CInputMode",                               0,                       2,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_CENTROID     },
{"CInputChannelNum",                         0,                       0,                       PARM_ATTR_RWNI,          PD_ATTR_V,               NO_SUCH_IO_BIT,          PD_FSG_CENTROID     },
{"CInvertSearch",                            0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_CENTROID     },
{"CReturnToCenter",                          0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_CENTROID     },
{"CDataSaveMode",                            0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_FSG_CENTROID     },
{"CDelayTime",                               0,                       0xFFFFFFFF,              PARM_ATTR_RWI,           PD_ATTR_V,               50,                      PD_FSG_CENTROID     },
{"HCPercentDrop",                            0,                       100,                     PARM_ATTR_RW,            PD_ATTR_V,               0,                       PD_FSG_HILLCLIMB    },
};
#endif

/* Virtual Pivot Point Parameter Min, Max, default, attributes */
#ifndef __INIT_AER_P_INFO_H__
	extern AER_PARM_INFO       VppParmInfo[MAX_VPPPARMS];
#else
AER_PARM_INFO     VppParmInfo[MAX_VPPPARMS]=
{// Parameter Name,                          Minimum,                 Maximum,                 Type Attribute,          Display Attribute,       DefaultValue,            DisplaySubGroup
{"Model",                                    0,                       11,                      PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_VSG_GENERAL      },
{"FixedToolTip",                             0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_VSG_GENERAL      },
{"UserToolTip",                              0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_VSG_GENERAL      },
{"SecondToolTip",                            0,                       1,                       PARM_ATTR_RWI,           PD_ATTR_V,               0,                       PD_VSG_GENERAL      },
{"ToolOffsetX",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_TOOL         },
{"ToolOffsetY",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_TOOL         },
{"ToolOffsetZ",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_TOOL         },
{"ToolRotateX",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_TOOL         },
{"ToolRotateY",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_TOOL         },
{"ToolRotateZ",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_TOOL         },
{"Tip1OffsetX",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_TIP1         },
{"Tip1OffsetY",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_TIP1         },
{"Tip1OffsetZ",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_TIP1         },
{"Tip1RotateX",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_TIP1         },
{"Tip1RotateY",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_TIP1         },
{"Tip1RotateZ",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_TIP1         },
{"Tip2OffsetX",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_TIP2         },
{"Tip2OffsetY",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_TIP2         },
{"Tip2OffsetZ",                              0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_TIP2         },
{"Tip2RotateX",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_TIP2         },
{"Tip2RotateY",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_TIP2         },
{"Tip2RotateZ",                              0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_TIP2         },
{"Axis1Type",                                0,                       6,                       PARM_ATTR_RWI,           PD_ATTR_V,               1,                       PD_VSG_AXIS1        },
{"Axis1OffsetX",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS1        },
{"Axis1OffsetY",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS1        },
{"Axis1OffsetZ",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS1        },
{"Axis1RotateX",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS1        },
{"Axis1RotateY",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS1        },
{"Axis1RotateZ",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS1        },
{"Axis1MotionDirection",                     -1,                      1,                       PARM_ATTR_RWI,           PD_ATTR_V,               1,                       PD_VSG_AXIS1        },
{"Axis1MotionMin",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               -50,                     PD_VSG_AXIS1        },
{"Axis1MotionMax",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               50,                      PD_VSG_AXIS1        },
{"Axis1PhysicalAxis",                        0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              0,                       PD_VSG_AXIS1        },
{"Axis2Type",                                0,                       6,                       PARM_ATTR_RWI,           PD_ATTR_V,               2,                       PD_VSG_AXIS2        },
{"Axis2OffsetX",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS2        },
{"Axis2OffsetY",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS2        },
{"Axis2OffsetZ",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS2        },
{"Axis2RotateX",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS2        },
{"Axis2RotateY",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS2        },
{"Axis2RotateZ",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS2        },
{"Axis2MotionDirection",                     -1,                      1,                       PARM_ATTR_RWI,           PD_ATTR_V,               1,                       PD_VSG_AXIS2        },
{"Axis2MotionMin",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               -50,                     PD_VSG_AXIS2        },
{"Axis2MotionMax",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               50,                      PD_VSG_AXIS2        },
{"Axis2PhysicalAxis",                        0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              1,                       PD_VSG_AXIS2        },
{"Axis3Type",                                0,                       6,                       PARM_ATTR_RWI,           PD_ATTR_V,               3,                       PD_VSG_AXIS3        },
{"Axis3OffsetX",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS3        },
{"Axis3OffsetY",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS3        },
{"Axis3OffsetZ",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS3        },
{"Axis3RotateX",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS3        },
{"Axis3RotateY",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS3        },
{"Axis3RotateZ",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS3        },
{"Axis3MotionDirection",                     -1,                      1,                       PARM_ATTR_RWI,           PD_ATTR_V,               1,                       PD_VSG_AXIS3        },
{"Axis3MotionMin",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               -50,                     PD_VSG_AXIS3        },
{"Axis3MotionMax",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               50,                      PD_VSG_AXIS3        },
{"Axis3PhysicalAxis",                        0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              2,                       PD_VSG_AXIS3        },
{"Axis4Type",                                0,                       6,                       PARM_ATTR_RWI,           PD_ATTR_V,               4,                       PD_VSG_AXIS4        },
{"Axis4OffsetX",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS4        },
{"Axis4OffsetY",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS4        },
{"Axis4OffsetZ",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS4        },
{"Axis4RotateX",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS4        },
{"Axis4RotateY",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS4        },
{"Axis4RotateZ",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS4        },
{"Axis4MotionDirection",                     -1,                      1,                       PARM_ATTR_RWI,           PD_ATTR_V,               1,                       PD_VSG_AXIS4        },
{"Axis4MotionMin",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               -50,                     PD_VSG_AXIS4        },
{"Axis4MotionMax",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               50,                      PD_VSG_AXIS4        },
{"Axis4PhysicalAxis",                        0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              3,                       PD_VSG_AXIS4        },
{"Axis5Type",                                0,                       6,                       PARM_ATTR_RWI,           PD_ATTR_V,               5,                       PD_VSG_AXIS5        },
{"Axis5OffsetX",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS5        },
{"Axis5OffsetY",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS5        },
{"Axis5OffsetZ",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS5        },
{"Axis5RotateX",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS5        },
{"Axis5RotateY",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS5        },
{"Axis5RotateZ",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS5        },
{"Axis5MotionDirection",                     -1,                      1,                       PARM_ATTR_RWI,           PD_ATTR_V,               1,                       PD_VSG_AXIS5        },
{"Axis5MotionMin",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               -50,                     PD_VSG_AXIS5        },
{"Axis5MotionMax",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               50,                      PD_VSG_AXIS5        },
{"Axis5PhysicalAxis",                        0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              4,                       PD_VSG_AXIS5        },
{"Axis6Type",                                0,                       6,                       PARM_ATTR_RWI,           PD_ATTR_V,               6,                       PD_VSG_AXIS6        },
{"Axis6OffsetX",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS6        },
{"Axis6OffsetY",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS6        },
{"Axis6OffsetZ",                             0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               0,                       PD_VSG_AXIS6        },
{"Axis6RotateX",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS6        },
{"Axis6RotateY",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS6        },
{"Axis6RotateZ",                             0,                       0,                       PARM_ATTR_RWN,           PD_ATTR_V,               0,                       PD_VSG_AXIS6        },
{"Axis6MotionDirection",                     -1,                      1,                       PARM_ATTR_RWI,           PD_ATTR_V,               1,                       PD_VSG_AXIS6        },
{"Axis6MotionMin",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               -50,                     PD_VSG_AXIS6        },
{"Axis6MotionMax",                           0,                       0,                       PARM_ATTR_RWNS,          PD_ATTR_V,               50,                      PD_VSG_AXIS6        },
{"Axis6PhysicalAxis",                        0,                       MAX_AXES-1,              PARM_ATTR_RWI,           PD_ATTR_AS,              5,                       PD_VSG_AXIS6        },
};
#endif


#endif /* __AERPINFO_H__ */
