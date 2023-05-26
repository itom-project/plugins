/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.

   NOTE: DO NOT PUT ANYTHING BUT DEFINES IN HERE (no structs, due to alignment problems)

+++*/

#ifndef __AER_ERR_H__
#define __AER_ERR_H__
//
///////////////////////////////////////////////////////////////
//
// PURPOSE: Error code translation (into strings or message boxs)
//    and error history logging.
//
// ??? need to make parent handles not desktop.
///////////////////////////////////////////////////////////////
//
#include "AerCode.h"   /* Error code definitions */


#define ERRORDLLNAME (LPCTSTR)"A32SYS.DLL"  /* Name of DLL where error strings found */
//
/********************************************************

  Error utility routines. All displays are one button message boxes. All
  functions are well bulletproofed from bad parameters passed. (regardless of
  DEBUG or RELEASE build). AERMESS and AerErrGetMessage() are the most useful ones.

AerErrGetMessage() - Given error code, copies message to passed ptr,
                          appends severity.
AERMESS()          - Allocates buffer, does a AerGetMessage() with passed
                          error code, displays, (just give it an RC id).
AERERRGETMESS()    - Same as AerGetMessage(), but you must call this
                          if calling function has variable arguments.
aerMessageBox()    - Appends severity to given string, displays.

AerErrLogFileOpen() - Opens a LOG file
AerErrLogError()    - Logs message to error file
AerErrLogFileClose()- Closes log file

***********************************************************/

#ifdef __cplusplus          /* Needed to prevent Name mangling of function prototypes */
extern "C" {
#endif
/*
    MessageBox() with "Aerotech Message" in the title.
*/
int AER_DLLENTRY aerMessageBox(HWND hWnd, LPCTSTR pText, UINT uType);
/*
   AERMESS() is simple Message box function (OK button only). OR logs to error log if not WIN95
      ERRORS: If it fails, lets you know what file and line in calling file where it did.
         But if it cant alloc even space to do that, it gives two boxes, one
         reporting a massive failure, the other giving the filename.
      NOTE: ??? This thing will eat up filename+pathname+20 bytes of static space for each call.
         It returns nothing.
*/
#define AERMESS(zz) {\
AERERR_CODE uierr = aerMessageBox(NULL,zz,(MB_APPLMODAL|MB_OK|MB_ICONSTOP));\
if (!uierr) aerMessageBoxFail(NULL,"AERMESS",__FILE__,__LINE__);}

#define AERMESSFAIL(zz) aerMessageBoxFail(NULL,zz,__FILE__,__LINE__);
//
// AERERRGETMESS() copies the RC string to passed pointer, with substitution.
// You must you must use this to get substitition, if you are in a function
// that recieves variable arguments. If substitution fails, it returns raw string.
// If copy fails entirely, it does nothing.
// If form is true, it uses long form of message. else short.
//
// works in C++ only (because of declarations within {}) !!!
//
#define AERERRGETMESS(dwID, pstr, form, lastArg, maxlen) {\
   TCHAR pszInternal0[2*maxlen];\
   LPTSTR pMess;\
   va_list vargs;\
   pMess = AerErrGetMessage (dwID, pszInternal0, -2*MAX_TEXT_LEN, form);\
   if (pMess != (LPTSTR)NULL) {\
      _TRY {\
         va_start( vargs, lastArg );\
         _vsnprintf( pmess, maxlen-1, pszInternal0, vargs );\
         va_end( vargs );\
      }\
      _CATCHOREXCEPT {\
         aerMessageBoxFail(NULL,"AERERRGETMESS", __FILE__, __LINE__);\
      }\
   }\
}
/*

  AerErrGetMessage() retrieves strings from RC file, with subsitutions,
    and itwrites message to error log, if erro log open. Pas wStrSize as 0
    to assumme size of MAX_TEXT_LEN. Add arguments at end, to be substituted
    with format specifiers in specified string. If the vsprintf substitution fails,
    it returns the string raw. Adds prefix with Aerotech name and severity.
    See AER_MACR.H for severity levels.
    It is very bulletproof, it returns NULL only in the worst failures,
    otherwise it returns pszStr.
*/
void AER_DLLENTRY AerErrGetMessageEx( AERERR_CODE eRc, LPTSTR pszStr,
                                         LONG wStrSize, BOOL bLongForm);

LPTSTR AER_DLLENTRY AerErrGetMessage( AERERR_CODE eRc, LPTSTR pszStr,
                                         LONG wStrSize, BOOL bLongForm, ... );

AERERR_CODE AER_DLLENTRY AerErrGetString( DWORD dwResID, LPTSTR pszText,
                                             DWORD dwLen );
AERERR_CODE AER_DLLENTRY AerErrGetSysErrorString( LPTSTR pszText, DWORD dwTextLen);

/*
   For logging errors to file. Old file destroyed when new one opened up.
   Note that AerOpenLogFile() return NULL or pointer to string explaining error.
*/
#define LOGFILENAME "A32Err.Log"
LPTSTR AER_DLLENTRY AerErrLogFileOpen(void);
AERERR_CODE AER_DLLENTRY AerErrLogError(LPCTSTR pStr);
void AER_DLLENTRY AerErrLogFileClose(void);
/*

   AerErrMessageBox() does a AerErrGetMessage(), then a AerMess() call.
   It does the string allocation for you, just pass it a error code, a severity
   and additional arguments, if desired, for substittion.
*/
void AER_DLLENTRY AerErrMessageBox( DWORD dwMessID, ... );
/*
  Gets severity
*/
DWORD AER_DLLENTRY AerErrGetSeverity(AERERR_CODE dwCode);
/*
   Returns pointer to string giving the exception.
*/
LPTSTR AER_DLLENTRY getExceptionString( DWORD dwCode );  /* Exception string from Exception code */


AERERR_CODE AER_DLLENTRY AerErrGetFaultMessage( HAERCTRL hAerCtrl, WORD wMsg, LPTSTR pszMsg );
AERERR_CODE AER_DLLENTRY AerErrGetAxisStatusMessage( HAERCTRL hAerCtrl, WORD wMsg, LPTSTR pszMsg );
AERERR_CODE AER_DLLENTRY AerErrGetDriveStatusMessage( HAERCTRL hAerCtrl, WORD wMsg, LPTSTR pszMsg );
AERERR_CODE AER_DLLENTRY AerErrGetStripStatusMessage( HAERCTRL hAerCtrl, WORD wMsg, LPTSTR pszMsg );

AERERR_CODE AER_DLLENTRY AerErrSetUserFault( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwFault);

void AER_DLLENTRY aerMessageBoxFail(HWND hWnd, LPCTSTR pExtra, LPCTSTR pFileName, DWORD dwLine);

BOOL aerErrdoesErrorMeanSMCisDead(AERERR_CODE dwErr);

/*
   Used by the AERMESS macro
*/
//
// Private functions
//
#ifdef __cplusplus    /* Needed to prevent Name mangling of function prototypes */
}
#endif

#endif     /* __AER_ERR_H__  */
