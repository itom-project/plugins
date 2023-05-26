/*+++

   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.

+++*/

#ifndef __AER_PARM_H__
#define __AER_PARM_H__
//
// Sizes of INI file sections (read in by GetPrivateProfileSection())
#define MAX_PARM_SECTION_SIZE       10000   // for general use
#define MAX_PARM_SECTION_SIZE_SMALL  500    // for setup data only (axis name etc.)
//
// Names of INI sections
#define AXIS_PARM_SECTION              "AxisParm"
#define TASK_PARM_SECTION              "TaskParm"
#define GLOBAL_PARM_SECTION            "GlobalParm"
#define FIBER_PARM_SECTION             "FiberParm"
#define VPP_PARM_SECTION               "VppParm"
#define SETUP_PARM_SECTION             "AxisMisc"
#define AXIS_CAL_SECTION               "AxisCal"

typedef struct tagAER_AXIS_SETUP_PARAMETERS
{
   TCHAR             szAxisName[MAX_NAME_LEN + 2];             // MAX_NAME_LEN = 32.  Plus 2 for 2 byte packing
   TCHAR             szEnglishUnitsName[MAX_UNIT_LEN + 1];     // MAX_UNIT_LEN = 3.   Plus 1 for 2 byte packing
   TCHAR             szMetricUnitsName[MAX_UNIT_LEN + 1];      // MAX_UNIT_LEN = 3.   Plus 1 for 2 byte packing
   TCHAR             szRotaryUnitsName[MAX_UNIT_LEN + 1];      // MAX_UNIT_LEN = 3.   Plus 1 for 2 byte packing
   TCHAR             szRotaryVelUnitsName[MAX_UNIT_LEN + 1];   // MAX_UNIT_LEN = 3.   Plus 1 for 2 byte packing
} AER_AXIS_SETUP_PARAMETERS;
typedef AER_AXIS_SETUP_PARAMETERS   *PAER_AXIS_SETUP_PARAMETERS;

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerParmGetTypesInfo( PAER_PARM_TYPES_INFO pTypesInfo );
//
// Setting/getting/reading/writing "regular" parameter data
AERERR_CODE AER_DLLENTRY AerParmDownloadFile( HAERCTRL hAerCtrl, DWORD dwParmType, LPCTSTR pszFile, DWORD dwMask, long* plParmNum, long* plIndex,  BOOL bReportErrorWhenParmNotFound);
AERERR_CODE AER_DLLENTRY AerParmDownloadEnd(HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwStart);

// Used by AerParmVerifyDefaultFile()
#define PARMDEF_VALID   0x01  // the file is valid
#define PARMDEF_OODATE  0x02  // the file is out-of-date

//AERERR_CODE AER_DLLENTRY AerParmWriteDefaultFile( HAERCTRL hAerCtrl,
//                                                  LPCTSTR pszFile );
AERERR_CODE AER_DLLENTRY AerParmVerifyDefaultFile( HAERCTRL hAerCtrl,
                                                   LPCTSTR pszFile,
                                                   PDWORD pdwParmDef );

AERERR_CODE AER_DLLENTRY AerParmReadValue( LPCTSTR pszFile, DWORD dwParmType, DWORD dwIndex,
                                           LPCTSTR pszName, PDOUBLE pdValue, PDOUBLE pdDefValue );
AERERR_CODE AER_DLLENTRY AerParmReadValueString( LPCTSTR pszFile, DWORD dwParmType, DWORD dwIndex,
                                                 LPCTSTR pszName, LPTSTR pszValue, LPCTSTR pszDefValue );

AERERR_CODE AER_DLLENTRY AerParmWriteValue( LPCTSTR pszFile, DWORD dwParmType, DWORD dwIndex,
                                            LPCTSTR pszName, DOUBLE dValue );
AERERR_CODE AER_DLLENTRY AerParmWriteValueString( LPCTSTR pszFile, DWORD dwParmType, DWORD dwIndex,
                                                  LPCTSTR pszName, LPCTSTR pszValue );

AERERR_CODE AER_DLLENTRY AerParmGetCount( DWORD dwParmType, PDWORD pdwCount );
AERERR_CODE AER_DLLENTRY AerParmGetInfo( DWORD dwParmType, DWORD dwParm, PAER_PARM_INFO pInfo );
AERERR_CODE AER_DLLENTRY AerParmGetInfoEx( DWORD dwParmType, DWORD dwParm, LPTSTR pszName, PDOUBLE pdMin,
                                           PDOUBLE pdMax, PDWORD pdwAttr, PDWORD pdwDisplayAttr,
                                           PDOUBLE pdDefault, PDWORD pdwDisplaySubGroup);

AERERR_CODE AER_DLLENTRY AerParmGetValue( HAERCTRL hAerCtrl, DWORD dwParmType, DWORD dwIndex,
                                          DWORD dwParm, DWORD dwConvertToUserUnits, PDOUBLE pdValue );
AERERR_CODE AER_DLLENTRY AerParmGetValueFromDrive( HAERCTRL hAerCtrl, DWORD dwParmType, DWORD dwIndex,
                                                   DWORD dwParm, DWORD dwConvertToUserUnits, PDOUBLE pdValue );
//AERERR_CODE AER_DLLENTRY AerParmGetValueString( HAERCTRL hAerCtrl, DWORD dwParmType, DWORD dwIndex,
//                                               DWORD dwParm, LPTSTR pszValue );
//
AERERR_CODE AER_DLLENTRY AerParmSetValue( HAERCTRL hAerCtrl, DWORD dwParmType, DWORD dwIndex,
                                          DWORD dwParm, DWORD dwConvertFromUserUnits, DOUBLE dValue );
AERERR_CODE AER_DLLENTRY AerParmMSetValue( HAERCTRL hAerCtrl, DWORD dwParmType, DWORD dwMask,
                                           DWORD dwParm, PAIDOUBLE pdValues, PDWORD pdwIndexOfFirstError );
//AERERR_CODE AER_DLLENTRY AerParmSetValueString( HAERCTRL hAerCtrl, DWORD dwParmType, DWORD dwIndex,
//                                                DWORD dwParm, LPCTSTR pszValue );
//

// Read and write the setup parameters using GetPrivateProfileSection()
AERERR_CODE AER_DLLENTRY AerParmSetupReadSection( LPCTSTR pszFile, AXISINDEX iAxis, PAER_AXIS_SETUP_PARAMETERS pSetupParams );
AERERR_CODE AER_DLLENTRY AerParmSetupWriteSection( LPCTSTR pszFile, AXISINDEX iAxis, PAER_AXIS_SETUP_PARAMETERS pSetupParams );

AERERR_CODE AER_DLLENTRY AerParmCheckFile( LPCTSTR pszFileName, DWORD dwParmType, DWORD dwIndex, DWORD dwStartParmNum, PDWORD pdwBadWord, LPTSTR pszBadName);

AERERR_CODE AER_DLLENTRY AerParmBuildInputNumber(DWORD dwIOType, DWORD dwDriveNumber, DWORD dwIOAddress, PDOUBLE pdBuiltIONumber);
AERERR_CODE AER_DLLENTRY AerParmUnBuildInputNumber(DOUBLE dBuiltIONumber, PDWORD pdwIOType, PDWORD pdwDriveNumber, PDWORD pdwIOAddress);

AERERR_CODE AER_DLLENTRY AerParmGetConstantFromName(DWORD dwParmType, LPCTSTR pszParmName, PDWORD pdwConstant);

//AERERR_CODE AER_DLLENTRY AerParmBuildOutputNumber(DWORD dwIOType, DWORD dwDriveNumber, DWORD dwIOAddress, PDOUBLE pdBuiltIONumber);
//AERERR_CODE AER_DLLENTRY AerParmUnBuildOutputNumber(DOUBLE dBuiltIONumber, PDWORD pdwIOType, PDWORD pdwDriveNumber, PDWORD pdwIOAddress);

//
// For reading parms from INI files, the quick way.
typedef struct tagAER_PARM_SECTION
{
   LPTSTR pszTextStrt;
   LPTSTR pszTextEnd;
   LPTSTR pszTextPos;
} AER_PARM_SECTION, *PAER_PARM_SECTION;

AERERR_CODE AER_DLLENTRY AerParmReadSectionWhole(LPCTSTR pszFile, LPCTSTR pszParmType, DWORD dwIndex,
                                                 LPTSTR pszParmSection, ULONG ulParmSectionLen, PAER_PARM_SECTION pSection);
//AERERR_CODE AER_DLLENTRY AerParmReadSectionValue(LPCTSTR pszKeyText, PDOUBLE pdValue, PAER_PARM_SECTION pSection);
//AERERR_CODE AER_DLLENTRY AerParmReadSectionString(LPCTSTR pszKeyText, LPTSTR pStr, ULONG ulStrLen, PAER_PARM_SECTION pSection);

AERERR_CODE AER_DLLENTRY AerParmWriteSectionStart(LPTSTR pszParmSection, ULONG ulParmSectionLen, PAER_PARM_SECTION pSection);
AERERR_CODE AER_DLLENTRY AerParmWriteSectionValue(LPCTSTR pszKeyText, DOUBLE dValue, PAER_PARM_SECTION pSection);
AERERR_CODE AER_DLLENTRY AerParmWriteSectionString(LPCTSTR pszKeyText, LPTSTR pszStr, PAER_PARM_SECTION pSection);
AERERR_CODE AER_DLLENTRY AerParmWriteSectionWhole(LPCTSTR pszFile, LPCTSTR pszParmType, DWORD dwIndex, PAER_PARM_SECTION pSection);

AERERR_CODE AER_DLLENTRY AerParmConvertDriveUnits( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwParmNum, DWORD dwFlag, PDOUBLE pfdValue);
void AER_DLLENTRY aerAddAbsolutePath(LPTSTR pszFileName);

AERERR_CODE AerParmDownLoadAxisName(HAERCTRL hAerCtrl, AXISINDEX iAxis, LPTSTR pszName);
//
// Validators
BOOL isFixtureOffset(long iParm);
DWORD AerAxisIsInValidAxisName(LPCTSTR pszAxisName);
BOOL  AerAxisIsValidUnitsName(LPCTSTR pszUnitString);


#ifdef __cplusplus
}
#endif

#endif
// __AER_PARM_H__
