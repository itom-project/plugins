/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is the PUBLIC Aerotech U600 include, and is used by
   Win apps, (but not the firmware or device driver)
   and must also be included by all customer apps (library, SDK, VB).

+++*/

#ifndef __AERSYS_H__
#define __AERSYS_H__

/*
   On 960 internal/library data must be 1 byte aligned (not sure why)
   On 3200 internal/library data is 2 byte aligned  ???
*/
#pragma pack(push, enter_includeAerSys ,2)
//#else
//#pragma pack(push, enter_includeAerSys ,1)
//#endif


#include "aeraer.h"     // Simple basic U600 stuff common to ALL U600 apps, and user apps

#include "aermacro.h"
#include "aersafe.h"    // Safety checking macros
#include "Aerc32.h"     // Digital Drive interface
#include "Aerpinfo.h"   // Parameter info
#include "Aer960.h"     /* Include common (960+PC+user) header files */
#include "AerLegac.h"   /* old stuff */

//
//  The below stuff should not be visible to user
#ifdef __AER_SYS_DLL__
    #include <cmdcodes.h>
    #include <Sys960.h>     /* internal structure files, used by AERSYS.DLL, and image */
    #include "aersysp.h"    // prototypres of routines private to AERSYS.DLL, not for user apps
    #include <drsys960.h>   // Firmware<->DeviceDriver<->AERSYS.DLL stuff
    #include <drvsys.h>     // DeviceDriver<->AERSYS.DLL stuff
#endif

/////////////////////////////////////////////////////////////////////////

#include "aerdrv.h"        // Device driver related stuff needed by Win applications
//
//
// prevents "name decoration" ("name mangling") of functions by C++
#ifdef __cplusplus
extern "C" {
#endif


// main ones
AERERR_CODE AER_DLLENTRY AerSysStart(DWORD dwReset, HAERCTRL *phAerCtrl );
AERERR_CODE AER_DLLENTRY AerSysStop( HAERCTRL hAerCtrl );
AERERR_CODE	AER_DLLENTRY AerSysIsHardwareCompatible( HAERCTRL hAerCtrl,	PDWORD pdwCompatible );
AERERR_CODE AER_DLLENTRY AerSysInitSystem( HAERCTRL hAerCtrl, DWORD dwReset,
                                           AXISMASK mAxis, TASKMASK mTask, LPCTSTR pszFile,
                                           PDWORD pdwParmType, PDWORD pdwParmNum, PDWORD piIndex );
AERERR_CODE AER_DLLENTRY AerSysReset( HAERCTRL hAerCtrl, DWORD dwResetFireWire );
AERERR_CODE AER_DLLENTRY AerSysFaultAck(HAERCTRL hAerCtrl, AXISMASK mAxis, TASKMASK mTask, DWORD dwMoveOutLimit );
//  AerSysFaultAck() calls the below three routines, waiting for completion of them
AERERR_CODE AER_DLLENTRY AerSysMoveOutLimitAll(HAERCTRL hAerCtrl, TASKINDEX iTask, PAXISMASK pmMoved);
AERERR_CODE AER_DLLENTRY AerSysAxisFaultAckAll(HAERCTRL hAerCtrl, TASKINDEX iTask);
AERERR_CODE AER_DLLENTRY AerSysTaskFaultAckAll(HAERCTRL hAerCtrl, TASKINDEX iTask);
//
// other stuff
AERERR_CODE AER_DLLENTRY AerSysOpen( DWORD dwDeviceID, DWORD dwCard, HAERCTRL *phAerCtrl );

AERERR_CODE AER_DLLENTRY AerSysOpenEx( HAERCTRL hAerCtrl );
AERERR_CODE AER_DLLENTRY AerSysClose( HAERCTRL hAerCtrl );

//AERERR_CODE AER_DLLENTRY AerXDeviceIOControl2( HAERCTRL hAerCtrl, DWORD dwCode,
//                                               void* pSend, DWORD dwSendBytes, PVOID pOut, DWORD dwOutBytes );
//AERERR_CODE AER_DLLENTRY AerSysCloseAll( HAERCTRL hAerCtrl );
AERERR_CODE AER_DLLENTRY AerSysGetDeviceID( HAERCTRL hAerCtrl,
                                            PDWORD pdwDeviceID, PDWORD pdwCard );

AERERR_CODE AER_DLLENTRY AerSysGetProcessNumber( HAERCTRL hAerCtrl, PDWORD pdwProcessNumber );

AERERR_CODE AER_DLLENTRY AerSysIsResetDone( HAERCTRL hAerCtrl, PDWORD pdwDone);
AERERR_CODE AER_DLLENTRY AerSysIsSystemInitialized(PDWORD pdwInitialized);
//AERERR_CODE AER_DLLENTRY AerSysValidateLicense( DWORD dwLevel );
//AERERR_CODE AER_DLLENTRY AerSysDebugOpen( DWORD dwAerDeviceID, DWORD dwCard, HAERCTRL *phAerCtrl );
//
AERERR_CODE AER_DLLENTRY AerSysSendCommandRaw(HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                              USHORT usCommandCode, DWORD dwCmdParameter0,
                                              DWORD dwCmdParameter1, DWORD dwCmdParameter2,
                                              DWORD dwCmdParameter3, DWORD dwCmdParameter4,
                                              DWORD dwCmdParameter5, DWORD dwCmdParameter6,
                                              WORD wReturnSizeBytes, PBYTE pbReturnData );
AERERR_CODE AER_DLLENTRY AerSysSendReadDSPMem(  HAERCTRL hAerCtrl,
                                                AXISINDEX iAxis,
                                                DWORD dwType,
                                                DWORD dwAddress,
                                                PDWORD pdwValue );
AERERR_CODE AER_DLLENTRY AerSysSendReadDSPMemFloat( HAERCTRL hAerCtrl,
                                                    AXISINDEX iAxis,
                                                    DWORD dwType,
                                                    DWORD dwAddress,
                                                    PFLOAT pfValue );
AERERR_CODE AER_DLLENTRY AerSysSendReadDSPMemDouble( HAERCTRL hAerCtrl,
                                                     AXISINDEX iAxis,
                                                     DWORD dwType,
                                                     DWORD dwAddress,
                                                     PDOUBLE pdValue );

AERERR_CODE AER_DLLENTRY AerSysSendWriteDSPMem( HAERCTRL hAerCtrl,
                                                AXISINDEX iAxis,
                                                DWORD dwType,
                                                DWORD dwAddress,
                                                DWORD dwValue );
AERERR_CODE AER_DLLENTRY AerSysSendWriteDSPMemFloat( HAERCTRL hAerCtrl,
                                                     AXISINDEX iAxis,
                                                     DWORD dwType,
                                                     DWORD dwAddress,
                                                     FLOAT fValue );
AERERR_CODE AER_DLLENTRY AerSysSendWriteDSPMemDouble( HAERCTRL hAerCtrl,
                                                      AXISINDEX iAxis,
                                                      DWORD dwType,
                                                      DWORD dwAddress,
                                                      DOUBLE dValue );

AERERR_CODE AER_DLLENTRY AerSysSendWriteFlashMem( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwSave, DWORD dwAddress, DOUBLE dValue );
AERERR_CODE AER_DLLENTRY AerSysSendCalibrateIOffsets(HAERCTRL hAerCtrl, AXISINDEX iAxis);
AERERR_CODE AER_DLLENTRY AerSysSendEtherNetMACAddr(HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwAddr16Upper, DWORD dwAddr16Middle, DWORD dwAddr16Lower);
AERERR_CODE AER_DLLENTRY AerSysSendMinimumDeadBandTime(HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwDeadTimeMinimum);
AERERR_CODE AER_DLLENTRY AerSysDrvSetTimeOut( HAERCTRL hAerCtrl, DWORD dwTimeOut );
AERERR_CODE AER_DLLENTRY AerSysDrvGetTimeOut( HAERCTRL hAerCtrl, PDWORD pdwTimeOut );

// Below for back compatibility only
AERERR_CODE AER_DLLENTRY AerSysCommandSendRaw(HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                              USHORT usCommandCode, DWORD dwCmdParameter0,
                                              DWORD dwCmdParameter1, DWORD dwCmdParameter2,
                                              DWORD dwCmdParameter3, DWORD dwCmdParameter4,
                                              DWORD dwCmdParameter5, DWORD dwCmdParameter6,
                                              WORD wReturnSizeBytes, PBYTE pbReturnData );
AERERR_CODE AerSysTest(HAERCTRL hAerCtrl, AXISINDEX iAxis, WORD wSubCode,
                       DRV_COMM_LENG wSendSizeBytes, PBYTE pbSendData,
                       DRV_COMM_LENG wReturnSizeBytes, PBYTE pbReturnData );
AERERR_CODE AerSysTestDriverCommun (HAERCTRL hAerCtrl, DRV_COMM_LENG wTranLen, DRV_COMM_LENG wRecvLen, BOOL bDoAxes, BOOL bCheckResults);
AERERR_CODE AerSysTestDriverCommun2(HAERCTRL hAerCtrl);
AERERR_CODE AER_DLLENTRY AerSysGetDebugTitle(HAERCTRL hAerCtrl, LPTSTR szString);
AERERR_CODE AER_DLLENTRY AerSysTestGetOHCIRegister(HAERCTRL hAerCtrl, DWORD dwNum, PDWORD dwValue);

// Used to obtain maximum bandwidth size for library commands
WORD AER_DLLENTRY getMaxXmitSizeAllowed(HAERCTRL hAerCtrl);
WORD AER_DLLENTRY getMaxRecvSizeAllowed(HAERCTRL hAerCtrl);

void AER_DLLENTRY AerDrvGetSharedStatusWord(DWORD dwThreadNum, PDLL_SHARE_SINGLE pData);
void AER_DLLENTRY AerDrvSetSharedStatusWord(DWORD dwThreadNum, DWORD dwFunctNum, DWORD dwValue);

void AER_DLLENTRY AerDrvGetSharedStatusWordDIO(HAERCTRL hAerCtrl, DWORD dwThreadNum, PDLL_SHARE_SINGLE pData);

BOOL AER_DLLENTRY logNewFileHandle(HANDLE hFile, TASKINDEX iTask, DWORD dwFlag);
void AER_DLLENTRY closeAllFileHandles(TASKINDEX iTask);

AERERR_CODE AER_DLLENTRY aerSysCommand( HAERCTRL hAerCtrl, WORD wOpCode, WORD wSubCode, DWORD pvTran, WORD wTranLen, DWORD pvRecv, WORD wRecvLen );

#include "aerserv.h"

#ifdef __cplusplus
}
#endif

///////////////// includes in INCLUDE directory ///////////////////////////////////

#include "aererr.h"     // Error utility prototypes and typedefs
#include "aerevent.h"
#include "aerver.h"
#include "aermem.h"

#include "aerdc.h"
#include "aerparm.h"
#include "aermove.h"
#include "aerio.h"
#include "aertorq.h"
#include "aerphase.h"
//#include "aerdebug.h"
#include "aerprob.h"
#include "aerprof.h"
#include "aercal.h"
#include "aeraux.h"
//#include "aerlaser.h"
#include "aercommon.h"
#include "aerutil.h"
#include "aerwatch.h"
#include "aerport.h"
#include "aercam.h"
#include "aerstrip.h"
#include "aeruser.h"
#include "aerreg.h"
#include "aercnc.h"
#include "aertask.h"
#include "aercback.h"      /* callback related stuff */
#include "aervar.h"
#include "aercmplr.h"
#include "aerat.h"
#include "aerauto.h"
#include "aertool.h"
#include "aertime.h"
#include "aershape.h"
#include "AerStat.h"
#include "AerQueue.h"
#include "aerFiber.h"

#pragma pack( pop, enter_includeAerSys )

#endif
// __AERSYS_H__
