/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.

   NOTE: DO NOT PUT ANYTHING BUT DEFINES IN HERE (no structs, due to alignment problems)

+++*/

#ifndef __AER_MACR__
#define __AER_MACR__
/*
////////////////////////////////////////////////////////////////////
//
// Constants, macros.
//
////////////////////////////////////////////////////////////////////
*/
#define UNREFERENCED_VAR( var )  var

#ifdef _WIN32
   #define DLLEXPORT __declspec(dllexport)
   #define DLLIMPORT __declspec(dllimport)
   #define STDCALL   _stdcall
   /*
   // Exception handling macros
   //  (_TRY and _CATCHOREXCEPT work equally in C and C++)
   */
   #ifdef __cplusplus
      #define _TRY      try
      #define _CATCHOREXCEPT  catch(...)
   #else
      #define _TRY      __try
      #define _CATCHOREXCEPT  _EXCEPT (EXCEPTION_EXECUTE_HANDLER)
   #endif
   #define _FINALLY  __finally
   #define _LEAVE    __leave
   #define _EXCEPT   __except
#endif
/* _WIN32
//
// Declare handles (basically just (void*) pointers)
//
*/

/*
DECLARE_HANDLE(HDEVICE);
DECLARE_HANDLE(HAERCTRL);
DECLARE_HANDLE(HSECURE);
DECLARE_HANDLE(HCOMPILER);
*/
typedef void*  HDEVICE;
typedef void*  HAERCTRL;
typedef void*  HSECURE;
typedef void*  HCOMPILER;

/*////////////////// Aerotech build related //////////////////////////////
//
// If __AER_x_DLL__ (for any x) is defined, then the functions declared
// AER_DLLENTRY or AER_DLLCLASSENTRY are exported by the x DLL. Otherwise
// they are imported.
//     Use AER_DLLENTRY for C-Functions
//     Use AER_DLLCLASSENTRY for C++ Classes
//
*/
#ifdef __AER_SYSTEM_DLL__
   #define AER_DLLENTRY       DLLEXPORT STDCALL
#else
   #define AER_DLLENTRY       DLLIMPORT STDCALL
#endif

//#ifdef __AER_ERR_DLL__       /* Define this to export error functions */
//   #define AERERR_DLLENTRY       DLLEXPORT STDCALL
//   #define AERERR_DLLCLASSENTRY  DLLEXPORT
//#else
//   #define AERERR_DLLENTRY       DLLIMPORT STDCALL
//   #define AERERR_DLLCLASSENTRY  DLLIMPORT
//#endif

#ifdef __AER_CMPL_DLL__
   #define AERCMPL_DLLENTRY       DLLEXPORT STDCALL
   #define AERCMPL_DLLCLASSENTRY  DLLEXPORT
#else
   #define AERCMPL_DLLENTRY       DLLIMPORT STDCALL
   #define AERCMPL_DLLCLASSENTRY  DLLIMPORT
#endif

//////////////////// constants //////////////////////////////

// C#	CLASS=Wait	REGION=Wait Constants
// Define for Syncronization objects
#define AER_WAIT_INFINITE 0xFFFFFFFF
// C# END

/////// Simple abbreviations ///////////////////////////////////////
#define SNULL (LPTSTR)NULL

//////////////////// function wrappers //////////////////////////////

//Memory operations (include string.h)
#define MEMALLOC( tSize )          malloc( tSize )
#define MEMREALLOC( hMem, tSize )  realloc( hMem, tSize )
#define MEMFREE( hMem )            free( hMem )
#define MEMSIZE( hMem )            msize( hMem )
#define MEMLOCK( hMem )            hMem
#define MEMUNLOCK( hMem )          0
#define MEMSET( lpVoid, nChar, tSize )       memset( lpVoid, nChar, tSize )
#define MEMCPY( lpDest, lpSource, tSize )    memcpy( lpDest, lpSource, tSize )
#define MEMCMP( lpBuf1, lpBuf2, tSize )      memcmp( lpBuf1, lpBuf2, tSize )

// I/O operations (include stdio.h)
#define FOPEN( lpsName, lpsOpt )         fopen( (LPSTR) lpsName, (LPSTR) lpsOpt )
#define FGETS( lpsBuffer, uBytes, fp )   fgets( lpsBuffer, uBytes, fp )
#define FPUTS( lpsBuffer, fp )           fputs( lpsBuffer, fp )
#define FGETC( fp )                      fgetc( fp )
#define FTELL( fp )                      ftell( fp )
#define FSEEK( fp, lOffset, nOrig )      fseek( fp, lOffset, nOrig )
#define FWRITE( lpsBuf, size, num, fp )  fwrite( (LPSTR) lpsBuf, size, num, fp )
#define FREAD( lpsBuf, size, num, fp )   fread( (LPSTR) lpsBuf, size, num, fp )
#define FCLOSE( fp )                     fclose( fp )
#define FEOF( fp )                       feof( fp )

// Character operations  (include ctype.h)
#define ISUPPER( wChar )                 (BOOL)isupper( wChar )
#define ISSPACE( wChar )                 (BOOL)isspace( wChar )
#define ISALPHA( wChar )                 (BOOL)isalpha( wChar )
#define ISDIGIT( wChar )                 (BOOL)isdigit( wChar )
#define ISCNTRL( wChar )                 (BOOL)iscntrl( wChar )
#define ISGRAPH( wChar )                 (BOOL)isgraph( wChar )
#define ISPRINT( wChar )                 (BOOL)isprint( wChar )

// But compiler "proper" ignores '[\x20\t\b\x7F\x1A\x0B\x0C]+'   ???
#define ISWHITESPACE( wChar )            (BOOL)((wChar == ' ') || (wChar == '\t'))
#define ISNOTWHITESPACE( wChar )         (BOOL)((wChar != ' ') && (wChar != '\t'))

// Type conversions operations  (stdlib.h)
#define ATOI( psStr )   atoi( psStr )
#define ATOL( psStr )   atol( psStr )
#define ATOF( psStr )   atof( psStr )

// Port Functions
#define OUTP( uPort, cData )  (int) outp( (USHORT) uPort, (int)((BYTE) cData) )
#define OUTPW( uPort, uData ) (USHORT) outpw( (USHORT) uPort, (USHORT) uData )
#define OUTPD( uPort, lData ) (ULONG) outpd( (USHORT) uPort, (ULONG) lData )
#define INP( uPort )   (int) inp( (USHORT) uPort )
#define INPW( uPort )  (USHORT) inpw( (USHORT) uPort )
#define INPD( uPort )  (ULONG) inpd( (USHORT) uPort )



//////////////////// complex macros //////////////////////////////
//
// MACRO functions
// Returns offset to member m, within the structure s.
//
#define OFFSETOF(s,m)	(size_t)&(((s *)0)->m)

#define ON_ERR( eRc )               ( eRc != AERERR_NOERR)
#define RETURN_ON_ERR( eRc )        if( eRc != AERERR_NOERR) return( eRc )
#define BREAK_ON_ERR( eRc )         if( eRc != AERERR_NOERR) break
#define JUMP_ON_ERR( eRc, label )   if( eRc != AERERR_NOERR) goto label
#define LEAVE_ON_ERR( eRc )         if( eRc != AERERR_NOERR) _LEAVE

#define JUMP_ON_FAIL( hr, label )  if( FAILED( hr ) ) goto label

#define FREE_POINTER( p )     \
{                             \
   PVOID  pTmp = (PVOID)p;    \
   p = NULL;                  \
   if( pTmp != NULL )         \
      MEMFREE( pTmp );        \
}

#define DELETE_POINTER( p )   \
{                             \
   PVOID  pTmp = (PVOID)p;    \
   p = NULL;                  \
   if( pTmp != NULL )         \
      delete pTmp;            \
}

#define RELEASE_INTERFACE( p )      \
{                                   \
   IUnknown *pTmp = (IUnknown *)p;  \
   p = NULL;                        \
   if( pTmp != NULL )               \
      pTmp->Release();              \
}
//
//
// SAFE va_start/va_sprintf/va_end dissection of variable argument lists (it really works) to use:
//   MUST declare : va_list vargs; and include <stdarg.h>. New last argument
//  (lastarg) is what you would pass to va_start.
// Biggest SAFETY problem with vsprintf is: with vsprintf is that if there are more substitution "%" thingees
// then provided arguments, it gets an access vio, thus the TRY/CATCH is needed. There seems to be no
// way to count the number of arguments passed to avoid the access vio.
//
#define VSPRINTF(msg,formatline,lastarg) _TRY {\
      va_start(vargs, lastarg);vsprintf(msg, formatline, vargs);} _CATCHOREXCEPT { _TRY {\
         STRCPY(msg,formatline);} _CATCHOREXCEPT {_TRY {\
             STRCPY(msg,"");} _CATCHOREXCEPT {aerMessageBoxFail(NULL,"VSPRINTF",__FILE__,__LINE__);\
      }}} va_end(vargs)

#endif

#define NEW(x,y,z)\
    try {\
       x = (y)new z;\
    } catch (CmemoryException* eeeeeee) {\
       return (y)NULL;\
    }
