/*+++

   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_TASK_H__
#define __AER_TASK_H__


#ifdef __cplusplus
extern "C" {
#endif


AERERR_CODE AER_DLLENTRY AerTaskProgramAssociate( HAERCTRL hAerCtrl, TASKINDEX iTask, PAER_PROG_HANDLE pHandle );
AERERR_CODE AER_DLLENTRY AerTaskProgramSetLineUser( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwLineUser );
AERERR_CODE AER_DLLENTRY AerTaskProgramGetLineUser( HAERCTRL hAerCtrl, TASKINDEX iTask, PDWORD pdwLineUser );
AERERR_CODE AER_DLLENTRY AerTaskProgramExecute( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwType );
AERERR_CODE AER_DLLENTRY AerTaskProgramStop( HAERCTRL hAerCtrl, TASKINDEX iTask );
AERERR_CODE AER_DLLENTRY AerTaskProgramStopWait( HAERCTRL hAerCtrl, TASKINDEX iTask );
AERERR_CODE AER_DLLENTRY AerTaskProgramAbort( HAERCTRL hAerCtrl, TASKINDEX iTask );
AERERR_CODE AER_DLLENTRY AerTaskProgramAbortWait( HAERCTRL hAerCtrl, TASKINDEX iTask );   // waits till abort really done
AERERR_CODE AER_DLLENTRY AerTaskProgramReset( HAERCTRL hAerCtrl, TASKINDEX iTask );
AERERR_CODE AER_DLLENTRY AerTaskProgramResetSafe( HAERCTRL hAerCtrl, TASKINDEX iTask );
AERERR_CODE AER_DLLENTRY AerTaskReset( HAERCTRL hAerCtrl, TASKINDEX iTask );
AERERR_CODE AER_DLLENTRY AerTaskProgramDeAssociate( HAERCTRL hAerCtrl, TASKINDEX iTask );
AERERR_CODE AER_DLLENTRY AerTaskProgramIsExecuting( HAERCTRL hAerCtrl, TASKINDEX iTask, PDWORD pIsExecuting);

AERERR_CODE AER_DLLENTRY AerTaskImmediateExecuteLine( HAERCTRL hAerCtrl, TASKINDEX iTask, PCODE_PACKET pCode );
AERERR_CODE AER_DLLENTRY AerTaskImmediateGetLine( HAERCTRL hAerCtrl, TASKINDEX iTask, PCODE_PACKET pCode );

//AERERR_CODE AER_DLLENTRY AerTaskNormalExecuteLine( HAERCTRL hAerCtrl, PCODE_PACKET pCode, TASKINDEX iTask);

AERERR_CODE AER_DLLENTRY AerTaskSetRetraceMode( HAERCTRL hAerCtrl, TASKINDEX iTask,
                                                DWORD  dwMode );
AERERR_CODE AER_DLLENTRY AerTaskSetSlewMode( HAERCTRL hAerCtrl, TASKINDEX iTask,
                                             DWORD  dwMode );
AERERR_CODE AER_DLLENTRY AerTaskSetExecuteMode( HAERCTRL hAerCtrl, TASKINDEX iTask,
                                                DWORD dwMode );


AERERR_CODE AER_DLLENTRY AerTaskOnGosubGetData( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD iIndex, PONGOSUB_DATA pData);
AERERR_CODE AER_DLLENTRY AerTaskMonitorGetData( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD iIndex, PMONITOR_DATA pData);

#ifdef __cplusplus
}
#endif

#endif
