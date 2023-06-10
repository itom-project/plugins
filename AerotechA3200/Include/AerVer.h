/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_VER_H__
#define __AER_VER_H__


#ifdef __cplusplus
extern "C" {
#endif


AERERR_CODE AER_DLLENTRY AerVerVersionToString( PAER_VERSION pVersion, LPTSTR pszVersion );
AERERR_CODE AER_DLLENTRY AerVerStringToVersion( LPTSTR pszVersion, PAER_VERSION pVersion );

AERERR_CODE AER_DLLENTRY AerVerGetLibVersion( HAERCTRL hAerCtrl, PAER_VERSION pVersion );
AERERR_CODE AER_DLLENTRY AerVerGetLibVersionEx( HAERCTRL hAerCtrl, PDWORD pdwUnidex,
                                                PDWORD pdwMajor, PDWORD pdwMinor,
                                                PDWORD pdwBuild );
AERERR_CODE AER_DLLENTRY AerVerGetLibVersionString( HAERCTRL hAerCtrl, LPTSTR pszVersion );

AERERR_CODE AER_DLLENTRY AerVerGetSMCVersion( HAERCTRL hAerCtrl, PAER_VERSION pVersion );
AERERR_CODE AER_DLLENTRY AerVerGetSMCVersionEx( HAERCTRL hAerCtrl, PDWORD pdwUnidex,
                                                PDWORD pdwMajor, PDWORD pdwMinor,
                                                PDWORD pdwBuild );
AERERR_CODE AER_DLLENTRY AerVerGetSMCVersionString( HAERCTRL hAerCtrl, LPTSTR pszVersion );

AERERR_CODE AER_DLLENTRY AerVerGetDriveVersion( HAERCTRL hAerCtrl,
                                                AXISINDEX iAxis,
                                                PDWORD pdwHardwareType,
                                                PDWORD pdwMajorVersion,
                                                PDWORD pdwMinorVersion,
                                                PDWORD pdwBuildVersion,
                                                PDWORD pdwFPGAVersion,
                                                PDWORD pdwMXHVersion );

#ifdef __cplusplus
}
#endif

#endif
// __AER_VER_H__
