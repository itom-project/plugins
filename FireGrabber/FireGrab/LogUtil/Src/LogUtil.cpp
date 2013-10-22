////////////////////////////////////////////////////////////////////////////////
// FireGrab Log utility.
// intek, c. kuehnel, 18.01.2008
////////////////////////////////////////////////////////////////////////////////

#include "logutil.h"
#include <windows.h>
#include <stdarg.h>
#include <stdio.h>
#include <share.h>

static FILE *pFile=NULL;

const char StartMsg[] = "<<<<<<<<<<< FGCamera logging starts right here <<<<<<<<<<<\n";
const char StopMsg[]  = ">>>>>>>>>>> FGCamera logging stops right here  >>>>>>>>>>>\n";

CRITICAL_SECTION Cs;

////////////////////////////////////////////////////////////////////////////////
// Init log utility and open file.

BOOL LogInit()
{
  char  cbuf[256];
  int   i;                           

  if(pFile)
   return false;
   
  // If linked we use directory from application, otherwise DLL
  GetModuleFileName(GetModuleHandle("logutil.dll"),cbuf,sizeof(cbuf));

  i=(int)strlen(cbuf);
  while(cbuf[i]!='\\')
   cbuf[i--]=0;
  strcat(cbuf,"fgcamera.log");
    
  // Open file without write sharing
  pFile=_fsopen(cbuf,"aS",_SH_DENYWR);
  if(!pFile)
   return false;
     
  fwrite(StartMsg,1,sizeof(StartMsg)-1,pFile);
  
  InitializeCriticalSection(&Cs);
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy log utility and close file.

void LogExit()
{
  if(pFile)
  {
    if(pFile)
     fwrite(StopMsg,1,sizeof(StopMsg)-1,pFile);

    fclose(pFile);
  }
  
  DeleteCriticalSection(&Cs);
}

////////////////////////////////////////////////////////////////////////////////
// Log a message.

void LogMsg(char *pFormat,...)
{
  char       cbuf[128];
  SYSTEMTIME Time;
  int        Cnt;
  va_list    args;

  EnterCriticalSection(&Cs);
  if(pFile)
  {
    GetLocalTime(&Time);
    Cnt=wsprintf(cbuf,"%02d:%02d:%02d.%03d[0x%p] ",
                 Time.wHour,Time.wMinute,Time.wSecond,Time.wMilliseconds,
                 GetCurrentThread());
    fwrite(cbuf,1,Cnt,pFile);
    
    va_start(args,pFormat);
    Cnt=vsprintf(cbuf,pFormat,args)  ;
    
    fwrite(cbuf,1,Cnt,pFile);
  }
  LeaveCriticalSection(&Cs);
}

////////////////////////////////////////////////////////////////////////////////
// Main entry function.
////////////////////////////////////////////////////////////////////////////////

BOOL APIENTRY DllMain( HANDLE hModule,DWORD  ul_reason_for_call,LPVOID lpReserved)
{
  switch(ul_reason_for_call)
  {
    case DLL_PROCESS_ATTACH:
      if(!LogInit())
       return false;
      return true;

    case DLL_PROCESS_DETACH:
      LogExit();
      return true;

    case DLL_THREAD_ATTACH:
      return true;

    case DLL_THREAD_DETACH:
      return true;
  }

  return true;
}

