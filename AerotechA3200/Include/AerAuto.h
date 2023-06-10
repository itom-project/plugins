/*+++
   Copyright (c) Aerotech Inc., 1998-1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AERAUTO_H__
#define __AERAUTO_H__

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerAutoProgGetNumPrograms( LPCTSTR pszFile,
                                                    PDWORD pdwNumPrograms  );
AERERR_CODE AER_DLLENTRY AerAutoProgGetProgram( LPCTSTR pszFile, DWORD dwProg,
                                                PDWORD pdwSystem, LPTSTR pszProg,
                                                PDWORD pdwType, PTASKMASK pmTask );
AERERR_CODE AER_DLLENTRY AerAutoProgSetProgram( LPCTSTR pszFile, DWORD dwProg,
                                                DWORD dwSystem, LPCTSTR pszProg,
                                                DWORD dwType, TASKMASK mTask );
AERERR_CODE AER_DLLENTRY AerAutoProgAddProgram( LPCTSTR pszFile, DWORD dwSystem,
                                                LPCTSTR pszProg, DWORD dwType,
                                                TASKMASK mTask );
AERERR_CODE AER_DLLENTRY AerAutoProgRemoveProgram( LPCTSTR pszFile, DWORD dwProg );

#ifdef __cplusplus
}
#endif

#endif
// __AERAUTO_H__
