/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.

   NOTE: DO NOT PUT ANYTHING BUT DEFINES IN HERE (no structs, due to alignment problems)

+++*/

#ifndef __AERSAFE_H___
#define __AERSAFE_H___

//
//////////////////////////////////////////////////////////////////////////
//
// Conditional debug macros. These are extra safety check macros
// that can be turned off by the "DEFINE_SAFE" defines below:
//
//////////////////////////////////////////////////////////////////////////
//
// If DEFINE_SAFE_STR is not set, STRLEN() etc. is strlen() etc.
// if it is set, STRLEN etc. is "safe" (exception catching)
//
//#ifndef NDEBUG
#define DEFINE_SAFE_STR
#define DEFINE_SAFE_PTR
//#endif
//
//
// If DEFINE_SAFE_PTR is set, RETURN_ON_INVALID macros check the pointer for validity.
// if not set, the RETURN_ON_INVALID macros do nothing.
//
//#define DEFINE_SAFE_PTR

///////////////////////////////////////////////////////////////////////////

#ifdef  DEFINE_SAFE_PTR
// Disallows NULL and junk
#  define RETURN_ON_INVALID_PTR(pp) \
   if (aerIsOkPointer((void *)pp) != AERERR_NOERR) return AERERR_INVALID_PASSED_POINTER

// Disallows junk, returns 1 if its null, 0 if its not junk
#define IS_PTR_NULL( pszStrin, dwNul ) \
   if      (pszStrin==NULL)                                dwNul = 1; \
   else if (aerIsOkPointer((void*)pszStrin)!=AERERR_NOERR) return AERERR_INVALID_PASSED_POINTER; \
   else                                                    dwNul = 0;

#  define RETURN_ON_INVALID_HAERCTL( hAerCtrl) \
   if (aerIsOkHaerCtrl(hAerCtrl) != AERERR_NOERR) return AERERR_INVALID_HAERCTRL

#  define RETURN_ON_INVALID_HCOMPILER( ph) \
   if (aerIsOkHCompiler(ph) != AERERR_NOERR) return AERCMPLR_GENERAL_BAD_COMPILER_HANDLE
#else
#  define RETURN_ON_INVALID_PTR(pp)
#  define RETURN_ON_INVALID_HAERCTL(pp)
#  define RETURN_ON_INVALID_HCOMPILER(pp)
#endif
//////////////////////////////////////////////////////////////////////
//
// If DEFINE_SAFE_STR not set, then STRLEN etc. translate to normal strlen
// etc.  If DEFINE_SAFE_STR is set, we use the fancy exception catching ones
// (if bad pointer passsed, they throw up messages boxes, announing line and
//  file where bad call made)
//
//////////////////////////////////////////////////////////////////////
//
#ifdef DEFINE_SAFE_STR
//////////////////////////////////////////////////////////////////////
//  use custom exception catching string routines
//////////////////////////////////////////////////////////////////////
//

#define STRLEN(msg)                      aerstrlen(msg,__FILE__,__LINE__)
#define STRCPY( psStr1, psStr2 )         aerstrcpy( psStr1, psStr2, __FILE__,__LINE__)
#define STRNCPY( psStr1, psStr2, nLen )  aerstrncpy( psStr1, psStr2, nLen, __FILE__,__LINE__)
#define STRCAT( psStr1, psStr2 )         aerstrcat( psStr1, psStr2, __FILE__,__LINE__)
#define STRNCAT( psStr1, psStr2, nLen )  aerstrncat( psStr1, psStr2, nLen, __FILE__,__LINE__)
#define STRCMP( psStr1, psStr2 )         aerstrcmp( psStr1, psStr2,__FILE__,__LINE__ )
#define STRCMPI( psStr1, psStr2 )        aerstrcmpi( psStr1, psStr2,__FILE__,__LINE__ )
#define STRNCMP( psStr1, psStr2, nLen )  aerstrncmp( psStr1, psStr2, nLen, __FILE__,__LINE__ )
#define STRUPR( psStr )                  aerstrupr( psStr, __FILE__,__LINE__)
#define STRCHR( psStr, nChar )           aerstrchr( psStr, nChar, __FILE__,__LINE__)
#define STRSTR( psStr1, psStr2 )         aerstrstr( psStr1, psStr2, __FILE__,__LINE__)

#else
//////////////////////////////////////////////////////////////////////
// use standard (inferior) string routines
//////////////////////////////////////////////////////////////////////

#define STRLEN( psStr1)                  strlen( psStr1)
#define STRCMP( psStr1, psStr2 )         strcmp( psStr1, psStr2 )
#define STRCMPI( psStr1, psStr2 )        strcmpi( psStr1, psStr2 )
#define STRNCMP( psStr1, psStr2, nLen )  strncmp( psStr1, psStr2, nLen )
#define STRCPY( psStr1, psStr2 )         strcpy( psStr1, psStr2 )
#define STRNCPY( psStr1, psStr2, nLen )  strncpy( psStr1, psStr2, nLen )
#define STRCAT( psStr1, psStr2 )         strcat( psStr1, psStr2 )
#define STRNCAT( psStr1, psStr2, nLen )  strncat( psStr1, psStr2, nLen )
#define STRUPR( psStr )                  strupr( psStr )
#define STRLWR( psStr )                  strlwr( psStr )
#define STRCHR( psStr, nChar )           strchr( psStr, nChar )
#define STRSTR( psStr1, psStr2 )         strstr( psStr1, psStr2 )

#endif

// Stupid sprintf causes monster chaos (kernal exceptions etc.)if the number in exponent is "too high" and %f
// format is used. (%f prints one digit for each exponent). Even "catch" or exceptions cant seem to trap it.
// So here for safety, we trap it manually by doing scientific notation instead for these cases.
#define SPRINTF                          sprintf
#define SPRINTF_DOUBLE0(aaa,bbb) ((fabs(bbb)>1e+32) ? sprintf(aaa,"%e",bbb) : sprintf(aaa,"%f",bbb))
#define SPRINTF_DOUBLE1(aaa,ccc,bbb) ((fabs(bbb)>1e+32) ? sprintf(aaa,"%s%e",ccc,bbb) : sprintf(aaa,"%s%f",ccc,bbb))
#define SPRINTF_DOUBLE2(aaa,bbb,ccc) ((fabs(bbb)>1e+32) ? sprintf(aaa,"%e%s",bbb,ccc) : sprintf(aaa,"%f%s",bbb,ccc))
#define SPRINTF_DOUBLE0A(aaa,bbb) ((fabs(bbb)>1e+32) ? sprintf(aaa,"%e",bbb) : sprintf(aaa,"%.16f",bbb))

///////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus    /* Needed to prevent Name mangling of function prototypes */
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY aerIsOkPointer(void* hc);
AERERR_CODE AER_DLLENTRY aerIsOkHaerCtrl(HAERCTRL hc);
AERERR_CODE AER_DLLENTRY aerIsOkHCompiler(HCOMPILER hc);
//void AER_DLLENTRY aerMessageBoxFail(HWND hWnd, LPCTSTR pextra, LPCTSTR pFileName, DWORD dwLine);

size_t AER_DLLENTRY aerstrlen(LPCTSTR msg, LPCTSTR pszFILE, DWORD dwLINE);
LPTSTR AER_DLLENTRY aerstrcpy( LPTSTR psStr1, LPCTSTR psStr2 ,LPCTSTR pszFILE, DWORD dwLINE);
LPTSTR AER_DLLENTRY aerstrncpy( LPTSTR psStr1, LPCTSTR psStr2, DWORD nLen ,LPCTSTR pszFILE, DWORD dwLINE);
LPTSTR AER_DLLENTRY aerstrcat( LPTSTR psStr1, LPCTSTR psStr2 ,LPCTSTR pszFILE, DWORD dwLINE);
LPTSTR AER_DLLENTRY aerstrncat( LPTSTR psStr1, LPCTSTR psStr2, DWORD nLen ,LPCTSTR pszFILE, DWORD dwLINE);

int AER_DLLENTRY aerstrcmp( LPCTSTR psStr1, LPCTSTR psStr2 ,LPCTSTR pszFILE, DWORD dwLINE);
int AER_DLLENTRY aerstrcmpi( LPCTSTR psStr1, LPCTSTR psStr2 ,LPCTSTR pszFILE, DWORD dwLINE);
int AER_DLLENTRY aerstrncmp( LPCTSTR psStr1, LPCTSTR psStr2, DWORD nLen ,LPCTSTR pszFILE, DWORD dwLINE);
LPTSTR AER_DLLENTRY aerstrupr( LPTSTR psStr ,LPCTSTR pszFILE, DWORD dwLINE);
LPTSTR AER_DLLENTRY aerstrchr( LPCTSTR psStr, TCHAR nChar, LPCTSTR pszFILE, DWORD dwLINE);
LPTSTR AER_DLLENTRY aerstrstr( LPCTSTR psStr1, LPCTSTR psStr2, LPCTSTR pszFILE, DWORD dwLINE);
//int AER_DLLENTRY aersprintf( LPTSTR psStr1, LPCTSTR psStr2,...);

#ifdef __cplusplus    /* Needed to prevent Name mangling of function prototypes */
}
#endif

#endif
// __AERSAFE_H___
