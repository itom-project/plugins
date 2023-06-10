#ifndef __AER_LEGACY_H__
#define __AER_LEGACY_H__
//
//  These legacy functions and associated structures and ARE FROZEN and CANNOT chaange
//  for the purposes of back compatibility

//////////////////////////////////////////////////////////////////////////////////
//
// This structure is maintained for back compatibility only, pre-2.13,
// needed by CAerDataCenter::GetAxisData()    and    AerStatusGetAxisInfo()
// NOTE: for back compatibility reasons, AER_AXIS_DATA_EX cannot be changed.
typedef struct tagAER_AXIS_DATA_EX
{
   LONG     lPos;                /* Position (without calibration), reflects PositionCnts Axis Parameter            (counts)  */
   LONG     lPosCmd;             /* Position command (without calibration), reflects PositionCmdCnts Axis Parameter (counts) */
   LONG     lPosCal;             /* Calibration value added to position command  (counts)  */
   LONG     lPosCam;             /* Cam value added to position command  (counts)  */
   LONG     lPosSecondary;       /* Position (without calibration), from secondary encoder feedback (counts)  */
   DOUBLE   dAvgVelCnts;         /* Average velocity (without calibration)       (cnts/msec) */
   DOUBLE   dPosProgCmd;         /* Position command             (user-prog-dist-units) */
   DOUBLE   dFixtureOffset;      /* Active Fixture offset value  (user-prog-dist-units) */
   DOUBLE   dTargetPos;          /* Target position for current move  (user-prog-dist-units) */
   DOUBLE   dRollOver;           /* Rollover distance, if not 0.0, user-program units rollover to zero at this value. (user-virt-dist-units) */
   DOUBLE   dPosFactor;          /* Position conversion factor (multiply by to convert */
   DOUBLE   dAvgVel;             /* Average velocity (without calibration), takes into account G70/G71,G75,G76  (user-virt-dist-units/user-prog-time-units) */
   DWORD    dwFaultStatus;       /* Reflects Axis parameter - Fault */
   DWORD    dwDriveStatus;       /* Reflects Axis parameter - DriveStatus */
   DWORD    dwAxisStatus;        /* Reflects Axis parameter - AxisStatus */
   WORD     wCurrent;            /* Feedback current */
   WORD     wCurrentCommand;     /* Commanded current */
   DWORD    dwType;              /* =AXISTYPE_xxxx */
   WORD     wNumDecimals;        /* Number of Decimals (NumDecimalsxxx axis parameter value, where xxx based on dwType of axis, ad G70/G71 mode) */
   WORD     wUnits;              /* Whether owning task is in Metric(0->G71) or English(1->G70) or Counts(2->G72) */
   WORD     wMinutes;            /* Whether owning task is in per/min(G75) or not (G76) */
   AER_AXIS_DATA_IO IOData;
} AER_AXIS_DATA_EX;
typedef AER_AXIS_DATA_EX
   *PAER_AXIS_DATA_EX;

//////////////////////////////////////////////////////////////////////////////////
//
// NOTE: for back compatibility reasons, AER_AXIS_DATA cannot be changed, up to the IOData member. (pre 2.14)
// (can switch things around, but it must be that
// (AerAxisData.byType == AerAxisDataEx.dwType) and (AerAxisData.IOData == AerAxisDataEx.IOData)
//    "user-prog-dist-units"  means the coordinates the user programs in (are determined by the owning task's current G70/G71/G72 setting, as well as by current transforms)
//    "user-virt-dist-units"  means the encoder position with conversion to mm/inch based on task's current G70/G71/G72 setting, and rolledover (transorms not done)
//       So if in mirroring about X axis (G83 X) and user says: G1X10, then at end of move user-prog-dist-units=10, user-virt-dist-units=-10
//   "user-prog-time-units"  sec/min, based on the current G75/G76 setting
//    If the axis is NOT bound to a task:
//    If axis not bound to a task, G70, G76 is assummed
//
typedef struct tagAER_AXIS_DATA
{
   LONG     lPos;                /* Position (without calibration), reflects PositionCnts Axis Parameter            (counts)  */
   LONG     lPosCmd;             /* Position command (without calibration), reflects PositionCmdCnts Axis Parameter (counts) */
   LONG     lPosCal;             /* Calibration value added to position command  (counts)  */
   LONG     lPosCam;             /* Cam value added to position command  (counts)  */
   LONG     lPosSecondary;       /* Position (without calibration), from secondary encoder feedback (counts)  */

   DOUBLE   dAvgVelCnts;         /* Average velocity (without calibration)       (cnts/msec) */
   DOUBLE   dPosProgCmd;         /* Position command             (user-prog-dist-units) */
   DOUBLE   dFixtureOffset;      /* Active Fixture offset value  (user-prog-dist-units) */

   DOUBLE   dSpare;


   DOUBLE   dRollOver;           /* Rollover distance, if not 0.0, user-program units rollover to zero at this value. (user-virt-dist-units) */
   DOUBLE   dPosFactor;          /* Position conversion factor (multiply by to convert:
                                          (counts --> user-virt-dist-units)) */
   DOUBLE   dAvgVel;             /* Average velocity (without calibration), takes into account G70/G71,G75,G76  (user-virt-dist-units/user-prog-time-units) */

   DWORD    dwFaultStatus;       /* Reflects Axis parameter - Fault */
   DWORD    dwDriveStatus;       /* Reflects Axis parameter - DriveStatus */
   DWORD    dwAxisStatus;        /* Reflects Axis parameter - AxisStatus */

   WORD     wCurrent;            /* Feedback current */
   WORD     wCurrentCommand;     /* Commanded current */

   BYTE     byType;              /* =AXISTYPE_xxxx */
   BYTE     byNumDecimals;       /* Number of Decimals (NumDecimalsxxx axis parameter value, where xxx based on dwType of axis, ad G70/G71 mode) */
   BYTE     byUnits;             /* Whether owning task is in Metric(0->G71) or English(1->G70) or Counts(2->G72) */
   BYTE     byMinutes;           /* Whether owning task is in per/min(G75) or not (G76) */
   LONG     lGantryOffset;       /* Gantry offset (if any) in cnts */
   WORD     wSpare;

   AER_AXIS_DATA_IO IOData;

} AER_AXIS_DATA;
typedef AER_AXIS_DATA   *PAER_AXIS_DATA;


//////////////////////////////////////////////////////////////////////////////////

// WARNING! must be same size as AER_GSTRIP_AXIS_DATA     (pre 2.14)
typedef struct tagAER_GSTRIP_SYSTEM_DATA
{
   WORD  wWastedSpace[10];
   DWORD dwWastedSpaceAgain;
   LONG  lClock;
   LONG  lNotUsed[9];
} AER_GSTRIP_SYSTEM_DATA;
typedef AER_GSTRIP_SYSTEM_DATA   *PAER_GSTRIP_SYSTEM_DATA;

typedef struct tagAER_GSTRIP_AXIS_STD_DATA
{
   LONG  lPos;               // cnts
   LONG  lPosCommand;        // cnts
   LONG  lRawPosCommand;     // cnts
   LONG  lRawVelCommand;     // cnts/msec
   LONG  lVelocity;          // cnts/msec
   LONG  lVelocityCommand;   // cnts/msec
   LONG  lPosExt;            // cnts
   SHORT sAcceleration;      // cnts/msec/msec
   SHORT sDigitalInputs;     // by default, is digital input data
   SHORT sTorque;
   SHORT sTorqueCmd;
   SHORT sAnalog0;
   SHORT sAnalog1;
   SHORT sDigitalOutputs;    // by default, is digital output data
   SHORT sDataCollectRate;   // should

   LONG  lCNCLineNumber;     // by default, is CNC line number -- used, but not displayed
   //LONG  lOptionalData0;     // for back compatibility only, (2.13 and before)
   //LONG  lOptionalData1;     // for back compatibility only, (2.13 and before)
   DOUBLE  dOptionalData0;   // optional data slot (by default, is nothing)
   DOUBLE  dOptionalData1;   // optional data slot (by default, is nothing)

} AER_GSTRIP_AXIS_STD_DATA;
typedef AER_GSTRIP_AXIS_STD_DATA  *PAER_GSTRIP_AXIS_STD_DATA;

typedef struct tagAER_GSTRIP_AXIS_OPT_DATA

{
   LONG  lOptionalData0;
   LONG  lOptionalData1;
   LONG  lOptionalData2;
   LONG  lOptionalData3;
   LONG  lOptionalData4;
   LONG  lOptionalData5;
   LONG  lOptionalData6;
   LONG  lOptionalData7;
   LONG  lSpare0;
   SHORT sDataCollectRate;    // used to identify the rate for this sample 2, 4, or 8 kHz ??
   SHORT sSpare0;
   SHORT sOptionalDataCode0;
   SHORT sOptionalDataCode1;
   SHORT sOptionalDataCode2;
   SHORT sOptionalDataCode3;
   SHORT sOptionalDataCode4;
   SHORT sOptionalDataCode5;
   SHORT sOptionalDataCode6;
   SHORT sOptionalDataCode7;
   LONG  lMoreSpares[2];
} AER_GSTRIP_AXIS_OPT_DATA;
typedef AER_GSTRIP_AXIS_OPT_DATA  *PAER_GSTRIP_AXIS_OPT_DATA;

typedef union tagAER_GSTRIP_AXIS_DATA
{
   AER_GSTRIP_AXIS_STD_DATA   stdData;
   AER_GSTRIP_AXIS_OPT_DATA   optData;
} AER_GSTRIP_AXIS_DATA;
typedef AER_GSTRIP_AXIS_DATA  *PAER_GSTRIP_AXIS_DATA;

typedef struct tagAER_GSTRIP_SAMPLE
{
   PAER_GSTRIP_SYSTEM_DATA  pSystem;
   PAER_GSTRIP_AXIS_DATA    pAxis[MAX_AXES];
} AER_GSTRIP_SAMPLE;
typedef AER_GSTRIP_SAMPLE *PAER_GSTRIP_SAMPLE;

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus          /* Needed to prevent Name mangling of function prototypes */
extern "C" {
#endif

// This function is maintained for back compatibility only, pre-2.13, now use AerStatusGetAxisInfo2()
AERERR_CODE AER_DLLENTRY AerStatusGetAxisInfo( HAERCTRL hAerCtrl, AXISMASK mAxis, PAER_AXIS_DATA_EX pData );
//
//////////////////////////////////////////////////////////////////////////////////
// pre 2.14
AERERR_CODE AER_DLLENTRY AerDCGetAxisDirectEx( HAERCTRL hAerCtrl, AXISMASK mAxis, PAER_AXIS_DATA_EX pData );

AERERR_CODE AER_DLLENTRY AerStatusGetAxisInfo2( HAERCTRL hAerCtrl, AXISMASK mAxis, PAER_AXIS_DATA pData );

AERERR_CODE AER_DLLENTRY AerStatusGetPosition( HAERCTRL hAerCtrl, AXISMASK mAxis, DWORD dwUnits,
                                               PAODOUBLE pdPosition,
                                               PAODOUBLE pdPositionCmd,
                                               PAODOUBLE pdVelocityAvg);

//////////////////////////////////////////////////////////////////////////////////
// pre 2.14
AERERR_CODE AER_DLLENTRY  AerStripGlobalGetSample( HAERCTRL hAerCtrl, WORD wFirst,
					    WORD wCount, AXISMASK mAxis,
					    PAER_GSTRIP_SAMPLE pData );

// pre 2.14
AERERR_CODE AER_DLLENTRY  AerStripGlobalGetQueue( HAERCTRL hAerCtrl, AXISMASK mAxis,
					   WORD wReq, PAER_GSTRIP_SAMPLE pData, PWORD pwRec );

//////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
}
#endif


#endif
