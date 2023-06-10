
#ifndef __AERLIBCBACK_H__
#define __AERLIBCBACK_H__

//
// defines for AclLibCBack.cpp
//
// C#	CLASS=Callback		REGION=Events
#define AER_LIB_CALLBACK_MESSAGE_EVENT          0
#define AER_LIB_CALLBACK_USER_EVENT             1
#define AER_LIB_CALLBACK_USER_DOTNET_EVENT      2
#define AER_LIB_CALLBACK_ERROR_EVENT            3
#define AER_LIB_CALLBACK_TERMINATE_EVENT        4
// C# END

AERERR_CODE STDCALL AerCBackInitiateLibCallback(TASKINDEX iTask);
AERERR_CODE STDCALL AerCBackLibSetTerminateEvent(TASKINDEX iTask);
AERERR_CODE STDCALL AerCBackTerminateLibCallback(TASKINDEX iTask);

void STDCALL AerCBackLibHandleDisplayCallbacks( TASKINDEX iTask, BOOL HandleFlag );
void STDCALL AerCBackLibHandleUserCallbacks( TASKINDEX iTask, BOOL HandleFlag );
void STDCALL AerCBackLibHandleUserDotNETCallbacks( TASKINDEX iTask, BOOL HandleFlag );
void STDCALL AerCBackLibHandleCallbackErrors( TASKINDEX iTask, BOOL HandleFlag );

AERERR_CODE STDCALL AerCBackLibWaitForCallbackEvent( TASKINDEX iTask, PDWORD pdwEvent );
AERERR_CODE STDCALL AerCBackLibDisplayCallbackGetEventHandle( TASKINDEX iTask, PHANDLE phEvent );
AERERR_CODE STDCALL AerCBackLibUserCallbackGetEventHandle( TASKINDEX iTask, PHANDLE phEvent );
AERERR_CODE STDCALL AerCBackLibUserDotNETCallbackGetEventHandle( TASKINDEX iTask, PHANDLE phEvent );

AERERR_CODE STDCALL AerCBackLibCallbackErrorGetEventHandle( TASKINDEX iTask, PHANDLE phEvent );

AERERR_CODE STDCALL AerCBackLibCallbackSetDoubleReturn( HAERCTRL hAerCtrl, TASKINDEX iTask, DOUBLE dCNCReturnValue );
AERERR_CODE STDCALL AerCBackLibCallbackSetStringReturn( HAERCTRL hAerCtrl, TASKINDEX iTask, LPCTSTR pszCNCReturnValue );

AERERR_CODE STDCALL AerCBackLibCallbackContinue( HAERCTRL hAerCtrl, TASKINDEX iTask, AERERR_CODE e960Rc, AERERR_CODE eTaskRc );

AERERR_CODE STDCALL AerCBackLibCallbackNumArgs( TASKINDEX iTask, PDWORD dwNumArgs );

AERERR_CODE STDCALL AerCBackLibCallbackValueGetDouble( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwArg, PDOUBLE pdValue );
AERERR_CODE STDCALL AerCBackLibCallbackValueGetDWORD( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwArg, PDWORD pdwValue );
AERERR_CODE STDCALL AerCBackLibCallbackValueGetString( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwArg, LPTSTR pszValue,
                                                       DWORD dwMaxBytes );

AERERR_CODE STDCALL AerCBackLibCallbackValueGetStringEx( HAERCTRL hAerCtrl, TASKINDEX iTask, LPTSTR pszStr, DWORD dwFirstArg,
                                                         DWORD dwLastArg, PDWORD pdwNextArg );

AERERR_CODE STDCALL AerCBackLibCallbackValueMakeString( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwBlobIndex,
                                                        LPTSTR pszBigString, DWORD dwBigStringLength, DWORD dwFlags );

AERERR_CODE STDCALL AerCBackLibCallbackErrorGetErrorCode( TASKINDEX iTask );
AERERR_CODE STDCALL AerCBackLibCallbackValueGetCallbackType( HAERCTRL hAerCtrl, TASKINDEX iTask, PDWORD pdwValue );

#endif
