/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/
#ifndef __AER_CBACK_H__
#define __AER_CBACK_H__

typedef AERERR_CODE (STDCALL *PFN_AERTASKCALLBACK)( HAERCTRL hAerCtrl, TASKINDEX iTask, PCALLBACK_VALUE pValue );

#define NUM_CALLDLL_ARGS   2
#define USER_ARG(num)      (NUM_CALLDLL_ARGS + num)

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerTaskCallBackGetData(HAERCTRL hAerCtrl, TASKINDEX iTask, PCALLBACK_VALUE pValue);
AERERR_CODE AER_DLLENTRY AerTaskCallBackContinue( HAERCTRL hAerCtrl,
   TASKINDEX iTask, PCALLBACK_VALUE pValue, AERERR_CODE e960Rc,
   AERERR_CODE eTaskRc, DOUBLE dCNCReturnValue );

AERERR_CODE AER_DLLENTRY AerTaskCallBackContinueEx( HAERCTRL hAerCtrl,
   TASKINDEX iTask, PCALLBACK_VALUE pValue, AERERR_CODE e960Rc,
   AERERR_CODE eTaskRc );
AERERR_CODE AER_DLLENTRY AerTaskCallBackReturnSetDouble( HAERCTRL hAerCtrl,
   PCALLBACK_VALUE pValue, DOUBLE dCNCReturnValue );
AERERR_CODE AER_DLLENTRY AerTaskCallBackReturnSetString( HAERCTRL hAerCtrl,
   PCALLBACK_VALUE pValue, LPCTSTR pszCNCReturnValue );

AERERR_CODE AER_DLLENTRY AerTaskCallBackNumArgs( PCALLBACK_VALUE pValue, PDWORD dwNumArgs );
AERERR_CODE AER_DLLENTRY AerTaskCallBackValueValidateNumArgs( PCALLBACK_VALUE pValue, DWORD dwMin, DWORD dwMax );

AERERR_CODE AER_DLLENTRY AerTaskCallBackValueGetDouble( HAERCTRL hAerCtrl, PCALLBACK_VALUE pValue,
                                                        DWORD dwArg, PDOUBLE pdValue );
AERERR_CODE AER_DLLENTRY AerTaskCallBackValueSetDouble( HAERCTRL hAerCtrl,
                                                        PCALLBACK_VALUE pValue,
                                                        DWORD dwArg, DOUBLE dValue );
AERERR_CODE AER_DLLENTRY AerTaskCallBackValueGetDWORD( HAERCTRL hAerCtrl, PCALLBACK_VALUE pValue,
                                                       DWORD dwArg, PDWORD pdwValue );
AERERR_CODE AER_DLLENTRY AerTaskCallBackValueSetDWORD( HAERCTRL hAerCtrl,
                                                       PCALLBACK_VALUE pValue,
                                                       DWORD dwArg, DWORD dwValue );
AERERR_CODE AER_DLLENTRY AerTaskCallBackValueGetString( HAERCTRL hAerCtrl, PCALLBACK_VALUE pValue,
                                                        DWORD dwArg,
                                                        LPTSTR pszValue,
                                                        DWORD dwMaxBytes );
AERERR_CODE AER_DLLENTRY AerTaskCallBackValueGetStringEx(HAERCTRL hAerCtrl,
                                                         PCALLBACK_VALUE pValue,
                                                         LPTSTR pszStr,
                                                         DWORD dwFirstArg,
                                                         DWORD dwLastArg,
                                                         PDWORD pdwNextArg );
AERERR_CODE AER_DLLENTRY AerTaskCallBackValueSetString( HAERCTRL hAerCtrl,
                                                        PCALLBACK_VALUE pValue,
                                                        DWORD dwArg, LPTSTR pszString );

AERERR_CODE AER_DLLENTRY AerTaskCallBackValueMakeString( HAERCTRL hAerCtrl, PCALLBACK_VALUE pValue,
                                                         DWORD dwBlobIndex,
                                                         LPTSTR pszBigString,
                                                         DWORD dwBigStringLength,
                                                         DWORD dwFlags );

AERERR_CODE AER_DLLENTRY aerTaskGetBlobDataPointers(CALLBACK_DATA *pBlobs, DWORD dwBlobIndex, CALLBACK_BLOB ** ppBlobHeader,
                                                    BYTE** pBlobContainer);
AERERR_CODE AER_DLLENTRY aerTaskGetBlobValuePointers(CALLBACK_VALUE* pBlobs, DWORD dwBlobIndex, CALLBACK_BLOB** ppBlobHeader,
                                                     BYTE** pBlobContainer);

/*
Response Functions - Task Fault and Call Backs
*/
/*
User defined function type
*/
typedef AERERR_CODE (STDCALL *PFN_AERUSER)(HANDLE hUser, HAERCTRL hAerCtrl, UINT uCmd, LPARAM lp1, LPARAM lp2);
#define AERUSER_TASK_CALLBACK    1                 /* AerRespTaskCallBack...  */
         /* iTask = LOWORD(lp1)              */
         /* wType = HIWORD(lp1)              */
         /* pValue = (PCALLBACK_VALUE)(lp2)  */
#define AERUSER_TASK_FAULT       2                 /* AerRespTaskFault...     */
         /* iTask = LOWORD(lp1)              */
         /* dwFault = (AERERR_CODE)(lp2)     */

/*
Response functions
*/
AERERR_CODE AER_DLLENTRY AerRespTaskCallBackCreate(HAERCTRL hAerCtrl, TASKINDEX iTask, HANDLE hUser, PFN_AERUSER pfnUser);
AERERR_CODE AER_DLLENTRY AerRespTaskCallBackDestroy(HAERCTRL hAerCtrl, TASKINDEX iTask);

AERERR_CODE AER_DLLENTRY AerRespTaskFaultCreate(HAERCTRL hAerCtrl, TASKINDEX iTask, HANDLE hUser, PFN_AERUSER pfnUser);
AERERR_CODE AER_DLLENTRY AerRespTaskFaultDestroy(HAERCTRL hAerCtrl, TASKINDEX iTask);

#ifdef __cplusplus
}
#endif

#endif
