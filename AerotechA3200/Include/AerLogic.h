#ifndef __AERLOGIC_H__
#define __AERLOGIC_H__

#define AER_NLOGIC_TIMEOUT 5000

#ifdef __cplusplus          /* Needed to prevent Name mangling of function prototypes */
extern "C" {
#endif

//nLogic
AERERR_CODE AER_DLLENTRY AerNLogicStart( HAERCTRL hAerCtrl );
AERERR_CODE AER_DLLENTRY AerNLogicStop( HAERCTRL hAerCtrl, PAERERR_CODE pExitCode);

BOOL AER_DLLENTRY AerNLogicServerIsRunning( HAERCTRL hAerCtrl );
BOOL AER_DLLENTRY AerNLogicExecIsRunning( HAERCTRL hAerCtrl, PAERERR_CODE pExitCode);
BOOL AER_DLLENTRY AerNLogicResourceIsRunning( HAERCTRL hAerCtrl );

AERERR_CODE AER_DLLENTRY AerNLogicGetCycleTime( HAERCTRL hAerCtrl, PDWORD pdwCycleTime );
AERERR_CODE AER_DLLENTRY AerNLogicSetCycleTime( HAERCTRL hAerCtrl, DWORD dwCycleTime );

AERERR_CODE AER_DLLENTRY AerNLogicGetLastError( HAERCTRL hAerCtrl, PDWORD pdwError,
                                                LPSTR pszString, PDWORD pdwErrorStamp);

#ifdef __cplusplus    /* Needed to prevent Name mangling of function prototypes */
}
#endif

#endif
