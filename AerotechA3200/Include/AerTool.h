/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AERTOOL_H__
#define __AERTOOL_H__

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerToolTableAllocate( HAERCTRL hAerCtrl, DWORD dwNumTools );
AERERR_CODE AER_DLLENTRY AerToolTableFree( HAERCTRL hAerCtrl );
AERERR_CODE AER_DLLENTRY AerToolTableGetNumTools( HAERCTRL hAerCtrl,
                                                  PDWORD pdwNumTools );

AERERR_CODE AER_DLLENTRY AerToolTableSetTool( HAERCTRL hAerCtrl, DWORD dwTool,
                                              PAER_TOOL pTool );
AERERR_CODE AER_DLLENTRY AerToolTableSetToolEx( HAERCTRL hAerCtrl, DWORD dwTool,
                                                DWORD mFlags, DOUBLE dCutterRadius,
                                                DOUBLE dCutterLength, DOUBLE dCutterWear,
                                                DOUBLE dOffsetX, DOUBLE dOffsetY,
                                                DOUBLE dFWord, DOUBLE dSWord,
                                                LPCTSTR szName );

AERERR_CODE AER_DLLENTRY AerToolTableGetTool( HAERCTRL hAerCtrl, DWORD dwTool,
                                              PAER_TOOL pTool );
AERERR_CODE AER_DLLENTRY AerToolTableGetToolEx( HAERCTRL hAerCtrl, DWORD dwTool,
                                                PDWORD pmFlags, PDOUBLE pdCutterRadius,
                                                PDOUBLE pdCutterLength, PDOUBLE pdCutterWear,
                                                PDOUBLE pdOffsetX, PDOUBLE pdOffsetY,
                                                PDOUBLE pdFWord, PDOUBLE pdSWord,
                                                LPTSTR pszName );

AERERR_CODE AER_DLLENTRY AerToolFileDownload( HAERCTRL hAerCtrl, LPCTSTR pszFile );
AERERR_CODE AER_DLLENTRY AerToolFileGetNumTools( LPCTSTR pszFile, PDWORD pdwNumTools );

AERERR_CODE AER_DLLENTRY AerToolFileAddTool( LPCTSTR pszFile, DWORD dwTool,
                                             PAER_TOOL pTool );
AERERR_CODE AER_DLLENTRY AerToolFileAddToolEx( LPCTSTR pszFile, DWORD dwTool,
                                               DWORD mFlags, DOUBLE dCutterRadius,
                                               DOUBLE dCutterLength, DOUBLE dCutterWear,
                                               DOUBLE dOffsetX, DOUBLE dOffsetY,
                                               DOUBLE dFWord, DOUBLE dSWord,
                                               LPTSTR szName );


AERERR_CODE AER_DLLENTRY AerToolFileRemoveTool( LPCTSTR pszFile, DWORD dwTool );

AERERR_CODE AER_DLLENTRY AerToolFileGetTool( LPCTSTR pszFile, DWORD dwTool,
                                             PAER_TOOL pTool );
AERERR_CODE AER_DLLENTRY AerToolFileGetToolEx( LPCTSTR pszFile, DWORD dwTool,
                                               PDWORD pmFlags, PDOUBLE pdCutterRadius,
                                               PDOUBLE pdCutterLength, PDOUBLE pdCutterWear,
                                               PDOUBLE pdOffsetX, PDOUBLE pdOffsetY,
                                               PDOUBLE pdFWord, PDOUBLE pdSWord,
                                               LPTSTR pszName );

AERERR_CODE AER_DLLENTRY AerToolFileSetTool( LPCTSTR pszFile, DWORD dwTool,
                                             PAER_TOOL pTool );
AERERR_CODE AER_DLLENTRY AerToolFileSetToolEx( LPCTSTR pszFile, DWORD dwTool,
                                               DWORD mFlags, DOUBLE dCutterRadius,
                                               DOUBLE dCutterLength, DOUBLE dCutterWear,
                                               DOUBLE dOffsetX, DOUBLE dOffsetY,
                                               DOUBLE dFWord, DOUBLE dSWord,
                                               LPCTSTR szName );

#ifdef __cplusplus
}
#endif

#endif
/* __AERTOOL_H__ */
