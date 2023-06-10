
/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

//
///////////////////////////////////////////////////////////////////////////
//
// Aercmplr.h : main header file for the AERCMPLR DLL
//
//    To run a CNC program you need to :
//      1. Compile source into object code
//      2. Download object code to axis processor.
//      3. Execute object code.
//    All routines here are concerned here with just steps 1. or  2. except
//    AerCompilerRunImmediate() which does steps 1, 2. and 3.
//
// NOTES:
//
///////////////////////////////////////////////////////////////////////////
//
// these must be protrected against bad passing of the pointer (all gets) ???
//
#ifndef __AER_CMPLR_H__
#define __AER_CMPLR_H__

#include "AerAer.h"       // Aerotech stuff
//#include "AerMacro.h"     // Aerotech windows stuff
#include "Aer960.h"
//
///////////////////////////////////////////////////////////////////////////////////////////////////////
// AerCompiler functions
///////////////////////////////////////////////////////////////////////////////////////////////////////
//
typedef ULONG LINENUM;           // All linenums are of this type
#define LINE_NULL (ULONG)-1L     // Returned when "NO_SUCH_LINE"

#define MAX_STATE_CHARS 30

typedef struct       // Type returned by AerCompilerGetStatus()
{
   DWORD dwState;           // (see states below)
   TCHAR pszState[MAX_STATE_CHARS];  // Verbal equivalent of dwState
   DWORD dwSourceLine;      // Current program line being compiled (is total num at end)
   DWORD dwFileLine;        // Current file line being compiled
   DWORD dwNErrs;           // Number of errors+warnings so far.
   DWORD dwNWarns;          // Number of warnings so far.
   TCHAR pszFile[MAX_PATH]; // Current file being scanned (only pass 0 relevent, else blank.)
   LONG ttime;              // time (milliseconds) spent on file so far
   LONG times[6];           // time (milliseconds) spent in varoiuus phases

} AER_COMPILE_STATUS_DATA;

typedef struct       // Type returned by AerCompilerErrData();
{
   AERERR_CODE dwCode;    // Code of error (see aercode.h + aererr.rc for code meanings) (AERERR_NOERR if no error)
   LONG dwSourceLine;     // SOURCE Line that error occurred. (0-based) (-1 if not relevent)
   LONG dwFileLine;       // FILE Line that error occurred. (0-based) (-1 if not relevent)
   LONG dwCharStart;      // First Character of offending token (0-based) (-1 if not applicable)
   LONG dwCharEnd;        // Last Character of offending token (0-based) (-1 if not applicable)
   TCHAR pszFile[MAX_PATH]; // File occurred in (or "" if not applicable)

} AER_COMPILE_ERROR_DATA;

#define COMPILER_STATE_NULL 0       // states must be sequential in occurerence (1 then 2 etc.)
#define COMPILER_STATE_PREPROC 1
#define COMPILER_STATE_PASS1 2
#define COMPILER_STATE_PASS2 3
#define COMPILER_STATE_DOWNLOADING 4
#define COMPILER_STATE_DONE_OK 5     // must be first completion state
#define COMPILER_STATE_DONE_WITH_WARNINGS 6
#define COMPILER_STATE_DONE_WITH_ERRORS 7
#define COMPILER_STATE_DOWNLOADED       8
#define COMPILER_STATE_DOWNLOADERROR    9


//
// Prototypes
//
#ifdef __cplusplus          /* Needed to prevent Name mangling of function prototypes */
extern "C" {
#endif
//
///////////////////////////////////////////////////////////////
//
//   AEROTECH CNC COMPILER "C" interface.
//
//  - Line/char/err numbers passed in, and returned are ZERO_BASED.
//  - Line/char numbers returned as NULL_INDEX indicate a NULL, or invalid number.
//  - All functions that can fail, return a AERERR_CODE, which is AERERR_NOERR if
//      the functions was succesful, else the return is one of the errors listed below.
//  - Functions with "Get" in thier name write data into a location indicated by a passed
//      pointer. If they return AERCMPLR_GENERAL_BAD_PASSED_BUFFER the pointer was found
//      to be suspect. Do not use the pointer.
//  - Error data consists of two parts, the error condition data, and error location data.
//      Error location data consists of the line number, char num, file etc.
//      Either error data can be returned in a formatted strings, or as values.
//
//    Possible error returns:
//       AERERR_NOERR
//       AERCMPLR_GENERAL_BAD_COMPILER_HANDLE
//       AERCMPLR_GENERAL_NO_SUCH_FILE
//       AERCMPLR_GENERAL_NO_SUCH_LINE
//       AERCMPLR_GENERAL_NO_SUCH_ERROR
//       AERCMPLR_GENERAL_NO_PROGRAM_LOADED
//       AERCMPLR_GENERAL_BAD_PASSED_BUFFER
//       AERCMPLR_GENERAL_NO_SUCH_ERROR
//       AERCMPLR_GENERAL_INSUFFICIENT_CONNECTION_DATA
//       AERCMPLR_GENERAL_OVERSPECIFIED_CONNECTION_DATA
//    or
//       any one of the of AERCMPLR_FILE_ errors. ???  make consistant
//
///////////////////////////////////////////////////////////////
//
AERERR_CODE AERCMPL_DLLENTRY AerCompilerOpen(HCOMPILER* phCompiler);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerOpenEx(HCOMPILER hCompile);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerClose(HCOMPILER hCompile);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerFileNameToHandle( LPCTSTR pszName,
                                                          LPTSTR pszHandle );

//
// Create and Destroy are left for bacwards compability -
//    Use Open/Close instead
AERERR_CODE AERCMPL_DLLENTRY AerCompilerCreate(HCOMPILER* phCompile);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerDestroy(HCOMPILER hCompile);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerLoadAxisNames(HAERCTRL hAerCtrl, HCOMPILER hCompile, LPCTSTR pszParamFile);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerLoadParmNames(HCOMPILER hCompile, LPCTSTR pszParamFile);
//
//  Compiles a file. It needs a filename. It returns AERERR_NOERR, or it returns
//  code for first compile error it found. Use "GetErr" functions below
//  to get more error data.  what about warnings ???
//
AERERR_CODE AERCMPL_DLLENTRY AerCompilerCompileFile(HCOMPILER hCompile, LPCTSTR pszFile, DWORD dwMode);
//
//  Compiles a single line. It returns AERERR_NOERR, or it returns
//
AERERR_CODE AERCMPL_DLLENTRY AerCompilerCompileLine(HCOMPILER hCompile, LPCTSTR pszLine, DWORD dwMode);
//
//  You can download (as a program) or run immediate data compiled from a program or a line.
//  Downloading needs a connection. Pass it the connection data or let it use
//  the default values.  If you pass newProgram as true, it allocates new lines.
// ??? no queue version
//
#define HAERCTRL_FAKE (HAERCTRL)-1

AERERR_CODE AERCMPL_DLLENTRY AerCompilerRunImmediate(HCOMPILER hCompile, HAERCTRL hAerCtrl, LPCTSTR pszTextLine, DWORD iTask);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerDownloadImmediate(HCOMPILER hCompile, HAERCTRL hAerCtrl,
                                                          TASKINDEX iTask, PDWORD pdwLastLineLoaded);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerSetQueueMode( HCOMPILER hCompile, BOOL bQueue,
   DWORD dwQueueSize, DWORD dwQueueRetain, BOOL bForceQueueAllocate );
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetQueueMode( HCOMPILER hCompile, PBOOL pbQueueFlag, PBOOL pbAllocated, PDWORD pdwQueueSize, PDWORD pdwQueueRetain );
AERERR_CODE AERCMPL_DLLENTRY AerCompilerDownload( HCOMPILER hCompile, HAERCTRL hAerCtrl, LPCTSTR psz960Name, DWORD dwFirstUserLineNumber);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerDownloadEx( HCOMPILER hCompile, HAERCTRL hAerCtrl,
   LPCTSTR psz960Name, DWORD dwStartPacket, DWORD dwStartUserLine,
   DWORD dwNumPackets, PDWORD pdwNumDownloaded, PDWORD pdwUserLine);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerAbort( HCOMPILER hCompile );

AERERR_CODE AERCMPL_DLLENTRY AerCompilerErrsGetNumOf(HCOMPILER hCompile, DWORD dwLowestSeverityOf, DWORD* pdwNErrs);
//
//  Error condition information
AERERR_CODE AERCMPL_DLLENTRY AerCompilerErrGetCode(HCOMPILER hCompile, DWORD dwErrNum, DWORD* code);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerErrGetText(HCOMPILER hCompile, DWORD dwErrNum, LPTSTR pszBuffer, DWORD dwBufferSize);
//
//  Error location information
AERERR_CODE AERCMPL_DLLENTRY AerCompilerErrGetLocText(HCOMPILER hCompile, DWORD dwErrNum, LPTSTR pszBuffer, DWORD dwBufferSize);
//
//  Both
AERERR_CODE AERCMPL_DLLENTRY AerCompilerErrGetData(HCOMPILER hCompile, DWORD dwErrNum, AER_COMPILE_ERROR_DATA* eData);
//
// Status
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetStatus(HCOMPILER hCompile, AER_COMPILE_STATUS_DATA* sData);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetNumOfLines(HCOMPILER hCompile, DWORD* nLines);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetLineText(HCOMPILER hCompile, LINENUM lineNum, LPTSTR pszBuffer, DWORD dwBufferSize);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerAddDefinesFileFast(HCOMPILER hCompile, LPCTSTR pszFile);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerAddDefinesFile(HCOMPILER hCompile, LPCTSTR pszFile);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerAddDefinesString(HCOMPILER hCompile, LPCTSTR pszString);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerRemoveDefinesFile(HCOMPILER hCompile, LPCTSTR pszFile);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerIsObjectOutOfDate(LPCTSTR pszFile, DWORD dwDateTime, PBOOL isOutofDate);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetPacketTotal(HCOMPILER hCompile, PDWORD pdwPackets);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetPacketUserLineNumber(HCOMPILER hCompile, DWORD iPacket, PDWORD pUserLineNumber);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetLabelTotal(HCOMPILER hCompile, PDWORD pdwLabels);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetLabel(HCOMPILER hCompile, DWORD iLabel, AER_PROG_LABEL_INFO* pCodeLabel);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetHeader(HCOMPILER hCompile, AER_PROG_HEADER* pHeader);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetHandle(HCOMPILER hCompile, AER_PROG_HANDLE* pHandle);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetPacket(HCOMPILER hCompile, DWORD iPacket, PCODE_PACKET pCodePacket);

//AERERR_CODE AERCMPL_DLLENTRY AerCompilerAxisNameDefaultGet(HCOMPILER hCompile, TASKAXISINDEX taskAxis, LPTSTR pszAxisName, DWORD dwAxisNameLeng);
//AERERR_CODE AERCMPL_DLLENTRY AerCompilerAxisNameDefaultSet(HCOMPILER hCompile, TASKAXISINDEX taskAxis, LPTSTR pszAxisName);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerCSParmNameGet(HCOMPILER hCompile, DWORD iParm, LPTSTR pszParmName, DWORD dwParmNameLeng);
//AERERR_CODE AERCMPL_DLLENTRY AerCompilerParmNameSet(HCOMPILER hCompile, DWORD iParm, LPTSTR pszParmName);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetProgVarTotal(HCOMPILER hCompile, PLONG plNumProgVars);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetProgVarByName(HCOMPILER hCompile, LPCTSTR pszVarName, PLONG plProgVarNum, PLONG plProgVarSize);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetProgVarByNumber(HCOMPILER hCompile, LONG lProgVarNum, LPTSTR pszVarName);
AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetProgVarSByNumber(HCOMPILER hCompile, LONG lProgVarNum, LPTSTR pszVarName);

AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetUniqueID(HCOMPILER hCompile, PDWORD pdwID );

AERERR_CODE AERCMPL_DLLENTRY AerCompilerAutoInclude( HCOMPILER hCompile, LPCTSTR pszParamFile  );
AERERR_CODE AERCMPL_DLLENTRY AerCompilerAutoIncludeEx( HCOMPILER hCompile, TASKMASK mTask, LPCTSTR pszParamFile );

AERERR_CODE AERCMPL_DLLENTRY AerCompilerAutoRun( HCOMPILER hCompile, HAERCTRL hAerCtrl, LPCTSTR pszParamFile  );
AERERR_CODE AERCMPL_DLLENTRY AerCompilerAutoRunEx( HCOMPILER hCompile,
                                                   HAERCTRL hAerCtrl,
                                                   LPCTSTR pszParamFile,
                                                   LPTSTR pszFileWithError,
                                                   LPTSTR pszProgTask1,
                                                   LPTSTR pszProgTask2,
                                                   LPTSTR pszProgTask3,
                                                   LPTSTR pszProgTask4 );
AERERR_CODE AERCMPL_DLLENTRY AerCompilerAutoRunEx2( HCOMPILER hCompile,
                                                    HAERCTRL hAerCtrl,
                                                    LPCTSTR pszParamFile,
                                                    LPTSTR pszFileWithError,
                                                    LPTSTR pszProgTask1,
                                                    LPTSTR pszProgTask2,
                                                    LPTSTR pszProgTask3,
                                                    LPTSTR pszProgTask4,
                                                    PBOOL pbShutdownProgTask1,
                                                    PBOOL pbShutdownProgTask2,
                                                    PBOOL pbShutdownProgTask3,
                                                    PBOOL pbShutdownProgTask4 );

//AERERR_CODE AERCMPL_DLLENTRY AerCompilerGetAxisMentioned(HCOMPILER hCompile, TASKAXISMASK* pmTaskAxisAll, TASKAXISMASK* pmTaskAxis);
//
///////////////////////////////////////////////////////////////////////////////////////////////////////
// AerProgram functions
///////////////////////////////////////////////////////////////////////////////////////////////////////
//
AERERR_CODE AERCMPL_DLLENTRY AerProgramCompilerCreate( HAERCTRL hAerCtrl, HCOMPILER* phCompiler);
AERERR_CODE AERCMPL_DLLENTRY AerProgramCompilerCreateEx( HAERCTRL hAerCtrl, HCOMPILER* phCompiler, LPCTSTR pszParamFile);
AERERR_CODE AERCMPL_DLLENTRY AerProgramCompilerDestroy( HCOMPILER hCompiler);
AERERR_CODE AERCMPL_DLLENTRY AerProgramRun( HAERCTRL hAerCtrl, LPCTSTR pszProgram, DWORD dwFlags, TASKINDEX iTask);
AERERR_CODE AERCMPL_DLLENTRY AerProgramRunEx( HAERCTRL hAerCtrl, LPCTSTR pszProgram, DWORD dwFlags, TASKINDEX iTask, HCOMPILER hCompiler );
AERERR_CODE AERCMPL_DLLENTRY AerProgramIsRunning( HAERCTRL hAerCtrl, TASKINDEX iTask, LPTSTR pszProgNameRunning, DWORD dwProgNameSize);
AERERR_CODE AERCMPL_DLLENTRY AerProgramDownload( HAERCTRL hAerCtrl, LPCTSTR pszProgram, DWORD dwFlags, TASKINDEX iTask);
AERERR_CODE AERCMPL_DLLENTRY AerProgramDownloadEx( HAERCTRL hAerCtrl, LPCTSTR pszProgram, DWORD dwFlags, TASKINDEX iTask, HCOMPILER hCompiler );

AERERR_CODE AERCMPL_DLLENTRY AerProgramGetStatus( HAERCTRL hAerCtrl, HCOMPILER hCompiler, TASKINDEX iTask,
                                                  PAER_PROG_INFO pProgInfo, PDWORD pdwProgramExecuting,
                                                  LPTSTR pszFile, DWORD dwFileSize,
                                                  LPTSTR pszLineText, DWORD dwLineTextSize,
                                                  LPTSTR pszFaultText, DWORD dwFaultTextSize );

AERERR_CODE AERCMPL_DLLENTRY AerProgramGetStatusEx( HAERCTRL hAerCtrl, HCOMPILER hCompiler, TASKINDEX iTask,
                                                    PDWORD pdwNumLinesSMC, PDWORD pdwCurrentLineSMC,
                                                    PDWORD pdwCurrentLineUser, PDWORD pdwCurrentQueueLines,
                                                    PDWORD pdwQueueBufferFull, PDWORD pdwQueueBufferEmpty,
                                                    PDWORD pdwProgramExecuting,
                                                    LPTSTR pszFile, DWORD dwFileSize,
                                                    LPTSTR pszLineText, DWORD dwLineTextSize,
                                                    LPTSTR pszFaultText, DWORD dwFaultTextSize );

AERERR_CODE AERCMPL_DLLENTRY AerAutoProgRun( HAERCTRL hAerCtrl, LPCTSTR pszParamFile );

AERERR_CODE AERCMPL_DLLENTRY AerAutoProgRunEx( HAERCTRL hAerCtrl,
                                               LPCTSTR pszFile,
                                               LPTSTR pszFileWithError,
                                               LPTSTR pszProgTask1,
                                               LPTSTR pszProgTask2,
                                               LPTSTR pszProgTask3,
                                               LPTSTR pszProgTask4 );

AERERR_CODE AERCMPL_DLLENTRY AerAutoProgRunEx2( HAERCTRL hAerCtrl,
                                                LPCTSTR pszFile,
                                                LPTSTR pszFileWithError,
                                                LPTSTR pszProgTask1,
                                                LPTSTR pszProgTask2,
                                                LPTSTR pszProgTask3,
                                                LPTSTR pszProgTask4,
                                                PBOOL pbShutdownProgTask1,
                                                PBOOL pbShutdownProgTask2,
                                                PBOOL pbShutdownProgTask3,
                                                PBOOL pbShutdownProgTask4 );

AERERR_CODE AERCMPL_DLLENTRY AerSysInitialize( HAERCTRL hAerCtrl,
                                               LPCTSTR pszFile,
                                               DWORD dwCheckHardware,
                                               HAERCTRL *phAerCtrl,
                                               PDWORD pdwInitErrorState,
                                               PDWORD pdwFirstError,
                                               LPTSTR pszErrorInfo,
                                               LPTSTR pszStatusInfo,
                                               LPTSTR pszProgTask1,
                                               LPTSTR pszProgTask2,
                                               LPTSTR pszProgTask3,
                                               LPTSTR pszProgTask4 );

AERERR_CODE AERCMPL_DLLENTRY AerSysTaskInitialization( HCOMPILER hCompile,
                                                       HAERCTRL hAerCtrl,
                                                       LPCTSTR pszParamFile );
AERERR_CODE AERCMPL_DLLENTRY AerProgramFree( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle );
AERERR_CODE AERCMPL_DLLENTRY AerProgramGetHandle( HAERCTRL hAerCtrl, DWORD dwNum, PAER_PROG_HANDLE pHandle );
AERERR_CODE AERCMPL_DLLENTRY AerProgramGetInfo( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle,
                                                PAER_PROG_INFO pInfo );
AERERR_CODE AERCMPL_DLLENTRY AerProgramGetNumber( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle,
                                                  PDWORD pdwNumber );
AERERR_CODE AERCMPL_DLLENTRY AerProgramSetBreakPoint( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle,
                                                      DWORD dwLineUser, DWORD dwOn_Off );
AERERR_CODE AERCMPL_DLLENTRY AerProgramGetBreakPoint( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle,
                                                      DWORD dwLineUser, PBOOL pbOn );
AERERR_CODE AERCMPL_DLLENTRY AerProgramSetFlags( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, DWORD dwFlags);

AERERR_CODE AERCMPL_DLLENTRY AerQueueSendCommand(HAERCTRL hAerCtrl,  HCOMPILER hCompiler,
                                                 DWORD dwProgNum, DWORD dwLineUser,
                                                 LPCTSTR pszTextLine);

#ifdef __cplusplus    /* Needed to prevent Name mangling of function prototypes */
}
#endif


#endif
