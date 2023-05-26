
/*
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_STRP_H__
#define __AER_STRP_H__

//
// WARNING! must be same size as AER_GSTRIP_AXIS_STD_DATAD
typedef struct tagAER_GSTRIP_SYSTEM_DATAD
{
   WORD  wNotUsed[10];
   DWORD dwNotUsed;
   LONG  lClock;        // clock value (milliseconds)
   LONG  lNotUsed[27];
} AER_GSTRIP_SYSTEM_DATAD;
typedef AER_GSTRIP_SYSTEM_DATAD   *PAER_GSTRIP_SYSTEM_DATAD;

// WARNING! must be same size as AER_GSTRIP_SYSTEM_DATAD
// this structure used only in 1 khts, see AER_GSTRIP_AXIS_OPT_DATAD for 2,4, or 8 khtz
typedef struct tagAER_GSTRIP_AXIS_STD_DATAD
{
   DOUBLE   dPos;               // cnts
   DOUBLE   dPosCommand;        // cnts
   DOUBLE   dRawPosCommand;     // cnts
   DOUBLE   dVelocity;          // cnts/msec
   DOUBLE   dVelocityCommand;   // cnts/msec
   DOUBLE   dRawVelCommand;     // cnts
   DOUBLE   dPosExt;            // cnts
   DOUBLE   dAcceleration;      // cnts/msec/msec

   //WARNING! there must be a even number of shorts below, or packet alignment destroys us !
   SHORT sDigitalInputs;       // by default, is digital input data
   SHORT sTorque;              // this is IQ current feedback (16 bit signed)
   SHORT sTorqueCmd;           // this is IQ current command (16 bit signed)
   SHORT sAnalog0;             // analog "counts" (16 bit signed)
   SHORT sAnalog1;             // analog "counts" (16 bit signed)
   SHORT sDigitalOutputs;      // by default, is digital output data
   //SHORT sDataCollectRate;   // not used (in this struct its always 1 khtz)

   LONG  lCNCLineNumber;     // by default, is CNC line number -- used, but not displayed
   DOUBLE  dOptionalData0;   // optional data slot (by default, is nothing)
   DOUBLE  dOptionalData1;   // optional data slot (by default, is nothing)
   DOUBLE  dOptionalData2;   // optional data slot (by default, is nothing)
   DOUBLE  dOptionalData3;   // optional data slot (by default, is nothing)

   DWORD   dwMoreSpare[5];

} AER_GSTRIP_AXIS_STD_DATAD;
typedef AER_GSTRIP_AXIS_STD_DATAD  *PAER_GSTRIP_AXIS_STD_DATAD;

// WARNING! must be same size as AER_GSTRIP_AXIS_STD_DATAD
typedef struct tagAER_GSTRIP_AXIS_OPT_DATAD
{
   DOUBLE  dOptionalData0;
   DOUBLE  dOptionalData1;
   DOUBLE  dOptionalData2;
   DOUBLE  dOptionalData3;
   DOUBLE  dOptionalData4;
   DOUBLE  dOptionalData5;
   DOUBLE  dOptionalData6;
   DOUBLE  dOptionalData7;
   LONG  lSpare0;
   SHORT sDataCollectRate;     // The rate this sample was taken at (2, 4, or 8 kHz)
   SHORT sSpare0;
   SHORT sOptionalDataCode0;
   SHORT sOptionalDataCode1;
   SHORT sOptionalDataCode2;
   SHORT sOptionalDataCode3;
   SHORT sOptionalDataCode4;
   SHORT sOptionalDataCode5;
   SHORT sOptionalDataCode6;
   SHORT sOptionalDataCode7;
   DWORD  dwMoreSpares[12];
} AER_GSTRIP_AXIS_OPT_DATAD;
typedef AER_GSTRIP_AXIS_OPT_DATAD  *PAER_GSTRIP_AXIS_OPT_DATAD;

typedef union tagAER_GSTRIP_AXIS_DATAD
{
   AER_GSTRIP_AXIS_STD_DATAD   stdData;
   AER_GSTRIP_AXIS_OPT_DATAD   optData;
} AER_GSTRIP_AXIS_DATAD;
typedef AER_GSTRIP_AXIS_DATAD  *PAER_GSTRIP_AXIS_DATAD;

typedef struct tagAER_GSTRIP_SAMPLED
{
   PAER_GSTRIP_SYSTEM_DATAD  pSystem;
   PAER_GSTRIP_AXIS_DATAD    pAxis[MAX_AXES];
} AER_GSTRIP_SAMPLED;
typedef AER_GSTRIP_SAMPLED *PAER_GSTRIP_SAMPLED;

// prevents "name decoration" ("name mangling") of functions by C++
#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY  AerStripGlobalAllocate( HAERCTRL hAerCtrl, AXISMASK mAxis,
					   WORD wSize );
AERERR_CODE AER_DLLENTRY  AerStripGlobalAllocateTrigger( HAERCTRL hAerCtrl, WORD wSize );
AERERR_CODE AER_DLLENTRY  AerStripGlobalGetTriggerStatus( HAERCTRL hAerCtrl,
						   PWORD pwStatus, PWORD pwSize );
AERERR_CODE AER_DLLENTRY  AerStripGlobalSetTriggerPoint( HAERCTRL hAerCtrl,
						  WORD wPoint, LONG lData );
AERERR_CODE AER_DLLENTRY  AerStripGlobalGetTriggerPoint( HAERCTRL hAerCtrl,
						  WORD wPoint, PLONG plData,
						  PWORD pwStatus );
AERERR_CODE AER_DLLENTRY  AerStripGlobalSetInputType( HAERCTRL hAerCtrl, DWORD dwInputType,
					     AXISINDEX iAxis, DWORD dwInputNumber, DWORD dwInputEdge, DWORD dwNumPointsOnEdge );
AERERR_CODE AER_DLLENTRY  AerStripGlobalSetTrigger( HAERCTRL hAerCtrl, WORD wMode,
					     WORD wTime, WORD wSize,
					     LONG lParm1, LONG lParm2 );
AERERR_CODE AER_DLLENTRY  AerStripGlobalGetStatus( HAERCTRL hAerCtrl, PWORD pwStatus,
					    PWORD pwAllocated, PWORD pwDepth,
					    PWORD pwCollected, PAXISMASK pmAxis );

AERERR_CODE AER_DLLENTRY  AerStripGlobalGetSampleD( HAERCTRL hAerCtrl, WORD wFirst,
					    WORD wCount, AXISMASK mAxis,
					    PAER_GSTRIP_SAMPLED pData );
//AERERR_CODE AER_DLLENTRY  AerStripGlobalGetSampleG( HAERCTRL hAerCtrl, WORD wFirst,
//					    WORD wCount, AXISMASK mAxis,
//					    PAER_GSTRIP_SAMPLED pData );

AERERR_CODE AER_DLLENTRY  AerStripGlobalGetQueueD( HAERCTRL hAerCtrl, AXISMASK mAxis,
					                                    WORD wReq, PAER_GSTRIP_SAMPLED pData, PWORD pwRec );
AERERR_CODE AER_DLLENTRY  AerStripGlobalGetImmediate( HAERCTRL hAerCtrl, AXISMASK mAxis,
                  DWORD dwParm1, PDWORD pStatus1,
                  DWORD dwParm2, PDWORD pStatus2,
                  DWORD dwParm3, PDWORD pStatus3,
                  DWORD dwParm4, PDWORD pStatus4,
                  DWORD dwParm5, PDWORD pStatus5,
                  DWORD dwParm6, PDWORD pStatus6,
                  DWORD dwParm7, PDWORD pStatus7,
                  DWORD dwParm8, PDWORD pStatus8);

// For .dot net / labview support, Upload loads all data into a staticly mallocaed structure, others
// fetch data from that structure
AERERR_CODE AER_DLLENTRY  AerStripGlobalUploadData( HAERCTRL hAerCtrl, WORD wFirst,
                                                    WORD wCount, AXISMASK mAxis );
AERERR_CODE AER_DLLENTRY  AerStripQueueUploadData( HAERCTRL hAerCtrl, WORD wCount, AXISMASK mAxis, PWORD pointsUploaded);
AERERR_CODE AER_DLLENTRY  AerStripGlobalGetSystemData( HAERCTRL hAerCtrl, AXISINDEX iAxis, WORD wCount,
                                                       PAER_GSTRIP_SYSTEM_DATAD pData );
AERERR_CODE AER_DLLENTRY  AerStripGlobalGetSTDData( HAERCTRL hAerCtrl, AXISINDEX iAxis, WORD wCount,
                                                    PAER_GSTRIP_AXIS_STD_DATAD pData );
AERERR_CODE AER_DLLENTRY  AerStripGlobalGetOPTData( HAERCTRL hAerCtrl, AXISINDEX iAxis, WORD wCount,
                                                    PAER_GSTRIP_AXIS_OPT_DATAD pData );
AERERR_CODE AER_DLLENTRY  AerStripGlobalGetData( HAERCTRL hAerCtrl, AXISINDEX iAxis, WORD wCount, WORD wDataCode,
                                                 PAODOUBLE pDataValues);


//AERERR_CODE AER_DLLENTRY  AerStripGlobalGetQueueDepth( HAERCTRL hAerCtrl, PDWORD pdwValue);
//AERERR_CODE AER_DLLENTRY  AerStripGlobalGetQueueRecent( HAERCTRL hAerCtrl, AXISMASK mAxis,
//					   WORD wUserNumReq, PAER_GSTRIP_SAMPLED pData,
//					   PWORD pwNumRec );
//AERERR_CODE AER_DLLENTRY  AerStripGlobalGetQueueDecimate( HAERCTRL hAerCtrl, AXISMASK mAxis,
//					   WORD wUserNumReq, PAER_GSTRIP_SAMPLED pData,
//					   PWORD pwNumRec,WORD wStep, WORD wType);
//AERERR_CODE AER_DLLENTRY  AerStripSetIOPosLatch(HAERCTRL hAerCtrl, AXISINDEX iAxis,
//					   WORD wMode, WORD wType, WORD wBit, WORD wLevel);
//AERERR_CODE AER_DLLENTRY  AerStripGetIOPosLatchStatus(HAERCTRL hAerCtrl, AXISINDEX iAxis,
//					   PWORD pwMode, PWORD pwType, PWORD pwBit, PWORD pwLevel);
AERERR_CODE AER_DLLENTRY AerStripGlobalFree( HAERCTRL hAerCtrl );
AERERR_CODE AER_DLLENTRY AerStripGlobalHold( HAERCTRL hAerCtrl );
AERERR_CODE AER_DLLENTRY AerStripGlobalRelease( HAERCTRL hAerCtrl );
AERERR_CODE AER_DLLENTRY AerStripGlobalHalt( HAERCTRL hAerCtrl );
AERERR_CODE AER_DLLENTRY AerStripGlobalFreeTrigger( HAERCTRL hAerCtrl );
AERERR_CODE AER_DLLENTRY AerStripGlobalIsFree( HAERCTRL hAerCtrl, PDWORD pdwIsFree );
//AERERR_CODE AER_DLLENTRY  AerStripGlobalLogic( HAERCTRL hAerCtrl, WORD wMode, WORD wTime,
//                  WORD wSize, WORD wParm1, WORD wParm2, WORD wParm3, WORD wParm4,
//                  WORD wCollectType1, WORD wCollectOffset1,
//                  WORD wCollectType2, WORD wCollectOffset2,
//                  WORD wCollectType3, WORD wCollectOffset3,
//                  WORD wCollectType4, WORD wCollectOffset4);
AERERR_CODE AER_DLLENTRY  AerStripGlobalClearOverflowStatus( HAERCTRL hAerCtrl);

AERERR_CODE AER_DLLENTRY AerStripGlobalSetOptionalDataItem( HAERCTRL hAerCtrl,
                                                            WORD wType,
                                                            WORD wNum,
                                                            WORD wOptionalDataCode,
                                                            DWORD dwAddress );

AERERR_CODE AER_DLLENTRY AerStripGlobalSetOptionalDataItemAxis( HAERCTRL hAerCtrl,
                                                               AXISINDEX iAxis,
                                                               WORD wType,
                                                               WORD wNum,
                                                               WORD wOptionalDataCode,
                                                               DWORD dwAddress );

AERERR_CODE AER_DLLENTRY AerStripGlobalSetRTDataCollectRate( HAERCTRL hAerCtrl,
                                                          AXISINDEX iAxis,
                                                          DWORD dwRate );

AERERR_CODE AER_DLLENTRY AerStripGlobalSetRTDataFdbk( HAERCTRL hAerCtrl,
                                                      AXISINDEX iAxis,
                                                      DWORD dwBuffer,
                                                      DWORD dwDataType );

AERERR_CODE AER_DLLENTRY AerStripGlobalGetRTDataStatus(  HAERCTRL hAerCtrl,
                                                         AXISINDEX iAxis,
                                                         PDWORD pdwEnableStatus,
                                                         PDWORD pdwRate );

AERERR_CODE AER_DLLENTRY  AerStripGlobalGetStartPosition( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                                          LONG lType, PLONG plPosition);

LONG AER_DLLENTRY formatOptionalDataCode(SHORT sOptionalDataCode, LONG lRawData);

#ifdef __cplusplus
}
#endif


#endif
// __AER_STRP_H__
