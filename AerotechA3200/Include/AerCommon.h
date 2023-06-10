#ifndef __AERCOMMON_H__
#define __AERCOMMON_H__

#include "aersys.h"   // AEROTECH public interface

#ifdef __cplusplus
extern "C" {
#endif

BOOL AER_DLLENTRY AerIsSeperator(LPTSTR iv_pString, ULONG indx);

LONG AER_DLLENTRY AerIsbadflt(LPTSTR iv_pString);
LONG AER_DLLENTRY AerIsBadLong(LPTSTR iv_pString);

DWORD AER_DLLENTRY AerWritePrivateProfileLong( LPCTSTR szSection, LPCTSTR szKey, LONG lValue, LPCTSTR szFile );
DWORD AER_DLLENTRY AerWritePrivateProfileDouble( LPCTSTR szSection, LPCTSTR szKey, DOUBLE dValue,
                                                 LPCTSTR szFile );
DWORD AER_DLLENTRY AerWritePrivateProfileString( LPCTSTR szSection, LPCTSTR szKey, LPCTSTR szValue,
                                                 LPCTSTR szFile );
DWORD AER_DLLENTRY AerWritePrivateProfileSection( LPCTSTR szSection, LPCTSTR szValue, LPCTSTR szFile );

DWORD AER_DLLENTRY AerGetPrivateProfileLong( LPCTSTR szSection, LPCTSTR szKey, LONG lDefValue, PLONG plValue,
                                             LPCTSTR szFile, LONG lMinValue, LONG lMaxValue );
DWORD AER_DLLENTRY AerGetPrivateProfileDouble( LPCTSTR szSection, LPCTSTR szKey, DOUBLE dDefValue,
                                               PDOUBLE pdValue, LPCTSTR szFile, DOUBLE dMinValue,
                                               DOUBLE dMaxValue );
DWORD AER_DLLENTRY AerGetPrivateProfileString( LPCTSTR szSection, LPCTSTR szKey, LPCTSTR szDefValue,
                                               LPTSTR pszValue, DWORD dwSize, LPCTSTR szFile );
DWORD AER_DLLENTRY AerGetPrivateProfileSection( LPCTSTR szSection, LPTSTR pszValue, DWORD dwSize,
                                                LPCTSTR szFile );



DWORD AER_DLLENTRY AerParmReadSectionString(LPCTSTR pszKeyText, LPTSTR pStr, ULONG ulStrLen, PAER_PARM_SECTION pSection);

DWORD AER_DLLENTRY AerParmReadSectionValue(LPCTSTR pszKeyText, PDOUBLE pdValue, PAER_PARM_SECTION pSection);

AERERR_CODE AER_DLLENTRY AerGetPrivateProfileSectionValue( LPCTSTR pszKeyText, LPTSTR szSection,
                                                           LONG lSectionLength, PDOUBLE pdValue );
AERERR_CODE AER_DLLENTRY AerGetPrivateProfileSectionString( LPCTSTR pszKeyText, LPTSTR pszSection,
                                                            LONG lSectionLength, LPTSTR pStr, LONG lStrLen );

#ifdef __cplusplus
}
#endif

#endif
// __AER_COMMON_H__
