#ifndef __AER_QUEUE_H__
#define __AER_QUEUE_H__

#ifdef __cplusplus          /* Needed to prevent Name mangling of function prototypes */
extern "C" {
#endif

//
//////////////////////////////////////////////////////////////////////////////////////////
//
//  Initialization
//
//////////////////////////////////////////////////////////////////////////////////////////

#define QUEUE_NUM_TEMP_VARAIBLES 12    // tehse are needed if AerQueueSendCommand() arguments are too complex

//////////////////////////////////////////////////////////////////////////////////////////
//
//  Queued Program manegement
//  Operates a circular queue program execution. Lines are executing in succession, program does not
//  stop execution, so user must keep it supplied with new lines to avoid starvation.
//
//////////////////////////////////////////////////////////////////////////////////////////
//
AERERR_CODE AER_DLLENTRY AerQueueInit(HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwNumQueueLines,
                              PAER_PROG_HANDLE pProgHandle, PDWORD pdwProgNum);
AERERR_CODE AER_DLLENTRY AerQueueFree( HAERCTRL hAerCtrl, TASKINDEX iTask, PAER_PROG_HANDLE pProgHandle);
AERERR_CODE AER_DLLENTRY AerQueueAbort(HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwStopType);
AERERR_CODE AER_DLLENTRY AerQueueRestart(HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwFullRestart,
                                           DWORD dwStopType, DWORD dwNewLineUser, PAER_PROG_HANDLE pProgHandle, PDWORD pdwProgNum);

AERERR_CODE AER_DLLENTRY AerQueueGetTaskStatusCheckMode(HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, PDWORD pdwMode);
AERERR_CODE AER_DLLENTRY AerQueueSetTaskStatusCheckMode(HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, DWORD dwMode);

//////////////////////////////////////////////////////////////////////////////////////////
//
//  Queued Program line loading. All need as input:
//    hAerCtrl   - communications handle
//    iTask      - task number
//    dwProg     - program number (identifies the program)
//    dwLineUser - user line number of line downloaded
//
//////////////////////////////////////////////////////////////////////////////////////////
//
AERERR_CODE AER_DLLENTRY AerQueueM2( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser );
AERERR_CODE AER_DLLENTRY AerQueueEnable( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                  AXISMASK mAxes, BOOL bSwitch);
AERERR_CODE AER_DLLENTRY AerQueueHome( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                AXISMASK mAxes);

AERERR_CODE AER_DLLENTRY AerQueueSoftwareHome( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                                AXISMASK mAxes, PAIDOUBLE pdPositions, DWORD dwClear, DWORD dwTaskMode);

AERERR_CODE AER_DLLENTRY AerQueueG1( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwProgNum, DWORD dwLineUser,
                              AXISMASK mAxes, PAIDOUBLE pdDist, DOUBLE dFeedrate,
                              DWORD dwTaskMode, BOOL bIntAccel, BOOL bIntDecel );
AERERR_CODE AER_DLLENTRY AerQueueG2( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwProgNum, DWORD dwLineUser,
                                     AXISMASK mAxes, PAIDOUBLE pdTarget,
                                     PAIDOUBLE pdOffset, DOUBLE dRadius, DOUBLE dFeedrate,
                                     DWORD dwTaskMode);
AERERR_CODE AER_DLLENTRY AerQueueG3( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwProgNum, DWORD dwLineUser,
                                     AXISMASK mAxes, PAIDOUBLE pdTarget,
                                     PAIDOUBLE pdOffset, DOUBLE dRadius, DOUBLE dFeedrate,
                                     DWORD dwTaskMode);

AERERR_CODE AER_DLLENTRY AerQueueWaitOnInput( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                              AXISINDEX iAxis, DWORD dwInputType, DWORD dwInputNum,
                              DWORD dwCompType, DOUBLE fdCompValue, DWORD dwTimeout, DWORD dwTimeoutFlag);

AERERR_CODE AER_DLLENTRY AerQueueWaitOnStatus(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                              AXISINDEX iAxis, DWORD dwStatusType,
                                              DWORD dwCompType, DOUBLE fdCompValue, DWORD dwTimeout,
                                              DWORD dwTimeoutFlag);

AERERR_CODE AER_DLLENTRY AerQueueFaultAck( HAERCTRL hAerCtrl, AXISMASK mAxes, DWORD dwProgNum, DWORD dwLineUser);
AERERR_CODE AER_DLLENTRY AerQueueG4( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser, DOUBLE fdDwellTime);

AERERR_CODE AER_DLLENTRY AerQueueSetDriveOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwProgNum, DWORD dwLineUser, WORD wOutputNum, BOOL bOutputVal);
AERERR_CODE AER_DLLENTRY AerQueueSetDriveOutputWord( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwProgNum, DWORD dwLineUser, WORD wOutputVal);
AERERR_CODE AER_DLLENTRY AerQueueSetAnalogOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwProgNum, DWORD dwLineUser, WORD wOutputNum, DOUBLE fdOutputVal);
AERERR_CODE AER_DLLENTRY AerQueueSetVirtOutputBit( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser, WORD wOutputNum, BOOL bOutputVal);
AERERR_CODE AER_DLLENTRY AerQueueSetVirtOutputReg( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser, WORD wOutputNum, WORD wOutputVal);
AERERR_CODE AER_DLLENTRY AerQueueSetEtherBinOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwProgNum, DWORD dwLineUser, WORD wOutputNum, BOOL bOutputVal);
AERERR_CODE AER_DLLENTRY AerQueueSetEtherRegOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwProgNum, DWORD dwLineUser, WORD wOutputNum, WORD wOutputVal);
AERERR_CODE AER_DLLENTRY AerQueueSetEtherPrcOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwProgNum, DWORD dwLineUser, WORD wOutputNum, WORD wOutputVal);
AERERR_CODE AER_DLLENTRY AerQueueSetVirtInputBit( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser, WORD wInputNum, BOOL bInputVal);
AERERR_CODE AER_DLLENTRY AerQueueSetVirtInputReg( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser, WORD wInputNum, WORD wInputVal);

AERERR_CODE AER_DLLENTRY AerQueueGetStatusInfo( HAERCTRL hAerCtrl, TASKINDEX iTask, PAER_PROG_HANDLE phProgHandle,
                                                PDWORD pdwCurrentUserLine,
                                                PBYTE  pbProgramExecuting,
                                                PBYTE  pbQBufferStarted,
                                                PBYTE  pbQBufferFull,
                                                PBYTE  pbQBufferEmpty);

AERERR_CODE AER_DLLENTRY AerQueueGetStatusInfoEx( HAERCTRL hAerCtrl, TASKINDEX iTask, PAER_PROG_HANDLE phProgHandle,
                                                PDWORD pdwCurrentUserLine,
                                                PDWORD pdwProgramExecuting,
                                                PDWORD pdwQBufferStarted,
                                                PDWORD pdwQBufferFull,
                                                PDWORD pdwQBufferEmpty);

AERERR_CODE AER_DLLENTRY AerQueueProbeSetInput( HAERCTRL hAerCtrl, DWORD dwProgNum,
                                           DWORD dwLineUser, DWORD dwInputType,
                                           AXISINDEX iAxis, DWORD dwInputNum);

AERERR_CODE AER_DLLENTRY AerQueueProbeSetMode( HAERCTRL hAerCtrl, DWORD dwProgNum,
                                           DWORD dwLineUser, DWORD dwMode, DWORD dwInputLevel,
                                           WORD wStartTaskVarNum);

AERERR_CODE AER_DLLENTRY AerQueueProbeEnable( HAERCTRL hAerCtrl, DWORD dwProgNum,
                                           DWORD dwLineUser, AXISMASK mAxes);

AERERR_CODE AER_DLLENTRY AerQueuePsoPulse(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                          AXISINDEX iAxis, DOUBLE fdTotalTime, DOUBLE fdOnTime,
                                          DWORD dwNumCycles);

AERERR_CODE AER_DLLENTRY AerQueuePsoControl(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                            AXISINDEX iAxis, DWORD dwCmdMode, DWORD dwOutputMode,
                                            DWORD dwOutputPin, DWORD dwWindowMode);

AERERR_CODE AER_DLLENTRY AerQueuePsoDistance(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                             AXISINDEX iAxis, DWORD dwCmdMode, DOUBLE fdIncDistance,
                                             DWORD dwStartIndex, DWORD dwNumElements);

AERERR_CODE AER_DLLENTRY AerQueuePsoOutput(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                           AXISINDEX iAxis, DWORD dwOutputSelect,
                                           DWORD dwCmdMode, DWORD dwStartIndex, DWORD dwNumElements,
                                           DWORD dwEdgeMode);

AERERR_CODE AER_DLLENTRY AerQueuePsoWindow(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                           AXISINDEX iAxis, DWORD dwWindowNum, DWORD dwCmdMode,
                                           DOUBLE fdLowVal, DOUBLE fdHighVal, DOUBLE fdLoadVal,
                                           DWORD dwStartIndex, DWORD dwNumElements, DWORD dwEdgeCode,
                                           DWORD dwInputChannel, DWORD dwResetMode);

AERERR_CODE AER_DLLENTRY AerQueuePsoTrack(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                          AXISINDEX iAxis, DWORD dwCmdMode,
                                          PAIDWORD pdwChannels, PAIDWORD pdwDivisors, DWORD dwResetMode,
                                          DWORD dwDirectionMode);

AERERR_CODE AER_DLLENTRY AerQueueArray(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                       AXISINDEX iAxis, DWORD dwCmdMode, DWORD dwTaskVarNum,
                                       DWORD dwStartIndex, DWORD dwNumElements);

AERERR_CODE AER_DLLENTRY AerQueueDataAcq(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                         AXISINDEX iAxis, DWORD dwWindowNum, DWORD dwCmdMode,
                                         DWORD dwSource, DWORD dwArrayIndex, DWORD dwNumSamples);

AERERR_CODE AER_DLLENTRY AerQueueSetParam( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                           DWORD dwParamType, DWORD dwIndex, DWORD dwParamName, DOUBLE fdValue);

AERERR_CODE AER_DLLENTRY AerQueueWaitForQueueEmpty(
                           HAERCTRL hAerCtrl, PAER_PROG_HANDLE pProgHandle,
                           TASKINDEX iTask, DWORD dwTimeOut10Msec,
                           AXISMASK mAxes, LONG lFlags,
                           PAERERR_CODE pTaskFault,
                           PAERERR_CODE pTaskWarning);

AERERR_CODE AER_DLLENTRY AerQueueG0( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                     AXISMASK mAxes, PAIDOUBLE pdDist, PAIDOUBLE pdFeedrate,
                                     DWORD dwTaskMode);

AERERR_CODE AER_DLLENTRY AerQueueSlew( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                       DWORD dwSlewDimension, DWORD dwSlewSet);

AERERR_CODE AER_DLLENTRY AerQueueSetPosFbk( HAERCTRL hAerCtrl, DWORD dwProgNum,
                                            DWORD dwLineUser, AXISINDEX iAxis, DOUBLE dPos);
AERERR_CODE AER_DLLENTRY AerQueueSetPosCmd( HAERCTRL hAerCtrl, DWORD dwProgNum,
                                            DWORD dwLineUser, AXISINDEX iAxis, DOUBLE dPos);
AERERR_CODE AER_DLLENTRY AerQueuePVT( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser, AXISMASK mAxes,
                                      ULONG ulTimeMSec, PAIDOUBLE pdDist, PAIDOUBLE pdVeloc, DWORD dwFlags);

AERERR_CODE AER_DLLENTRY AerQueueMoveInc( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                          AXISINDEX iAxis, DOUBLE dDistance, DOUBLE dVelocity,
                                          DWORD dwTaskMode);

AERERR_CODE AER_DLLENTRY AerQueueMoveAbs( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                          AXISINDEX iAxis, DOUBLE dPos, DOUBLE dVelocity,
                                          DWORD dwTaskMode);

AERERR_CODE AER_DLLENTRY AerQueueMoveFreerun( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                                AXISINDEX iAxis, DOUBLE dDirection, DOUBLE dVelocity,
                                                DWORD dwTaskMode);

AERERR_CODE AER_DLLENTRY AerQueueMoveHalt( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser, AXISINDEX iAxis);

AERERR_CODE AER_DLLENTRY AerQueueMoveOscillate( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser,
                                                AXISINDEX iAxis, DOUBLE dAmplitude, DOUBLE dFrequency,
                                                DOUBLE dNumCycles, DWORD dwTaskMode);

AERERR_CODE AER_DLLENTRY AerQueueSlice( HAERCTRL hAerCtrl, TASKINDEX iTask,
                                        DWORD dwProgNum, DWORD dwLineUser,
                                        AXISINDEX iStepAxis, AXISINDEX iScanAxis,
                                        DOUBLE dStepJumpPos, DOUBLE dScanJumpPos,
                                        DOUBLE dStepEndPos, DOUBLE dScanEndPos,
                                        DOUBLE dVectorSpeed,
                                        DOUBLE dStepAxisJumpSpeed, DOUBLE dScanAxisJumpSpeed,
                                        DWORD dwTaskMode, BOOL bIntAccel, BOOL bIntDecel);

AERERR_CODE AER_DLLENTRY AerQueueFiberSpiralRough(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser);
AERERR_CODE AER_DLLENTRY AerQueueFiberHillClimb(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser);
AERERR_CODE AER_DLLENTRY AerQueueFiberSpiralFine(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser);
AERERR_CODE AER_DLLENTRY AerQueueFiberGeoCenter(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser);
AERERR_CODE AER_DLLENTRY AerQueueFiberCentroid(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser, DWORD dwNumDimensions);
AERERR_CODE AER_DLLENTRY AerQueueFiberFastAlign(HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser, DWORD dwNumDimensions);

AERERR_CODE AER_DLLENTRY AerQueueGetTaskMode(HAERCTRL hAerCtrl, TASKINDEX iTask, PDWORD pdwTaskMode);
AERERR_CODE AER_DLLENTRY AerQueueSetTaskMode(HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwTaskMode);

AERERR_CODE AER_DLLENTRY AerQueueCfgTaskMoveMode(TASKINDEX iTask, DWORD dwAbsolute, PDWORD pdwTaskMode);
AERERR_CODE AER_DLLENTRY AerQueueCfgTaskUnitsMode(TASKINDEX iTask, DWORD dwCounts, DWORD dwEnglish, DWORD dwMinutes, PDWORD pdwTaskMode);
AERERR_CODE AER_DLLENTRY AerQueueCfgTaskProfilingMode(TASKINDEX iTask, DWORD dwMotionContinuous, PDWORD pdwTaskMode);
AERERR_CODE AER_DLLENTRY AerQueueCfgTaskWaitMode(TASKINDEX iTask, DWORD dwWaitMode, PDWORD pdwTaskMode);

AERERR_CODE AER_DLLENTRY AerQueueBrake( HAERCTRL hAerCtrl, DWORD dwProgNum, DWORD dwLineUser, AXISMASK mAxes, DWORD dwEnable);


#ifdef __cplusplus
}
#endif

#endif
