/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_REG_H__
#define __AER_REG_H__

#include <stdio.h>
#include "aersys.h"

//
/////////////////////////////////////////////////////////////////////////////////////
//
// All registry data found under this key
//
#define AER_REGISTRY_ROOT	"Software\\Aerotech"
#define	AER_ARDENCE_ROOT	"Software\\Ardence"
#define AER_VENTURCOM_ROOT	"Software\\VenturCom"

/////////////////////////////////////////////////////////////////////////////////////

#define _pszGeneral "General"
#define _pszProgramAutomation "A32Auto.ProgramAutomation"
/*
   Registry information is stored in the registry under
   HKEY_LOCAL_MACHINE
      Software
         Aerotech
            A3200
               Card1
               Card2
               Card3
               Card4
               Setup
                  AxesOfControl
                  CNC5
                  DeviceID
                  Fiber
                  installDir
                  NControlSDK
                  NFiber
                  NLogic
                  NMark
                  NViewMMI
                  NVision
                  ParameterMask
                  SMCArguments
                  VendorID
                  VersionBuild
                  VersionMajor
                  VersionMinor
                  VisionCBRespond
                  VPP
            A3210
               Card1
                  CallbackPortNumber
                  ClientIPAddress
                  ServerIPADdress
                  ServerPortNumber
               Card2
               Card3
               Card4
               Setup
                  AxesOfControl
                  CNC5
                  DeviceID
                  Fiber
                  installDir
                  NControlSDK
                  NFiber
                  NLogic
                  NMark
                  NViewMMI
                  NVision
                  ParameterMask
                  SMCArguments
                  VendorID
                  VersionBuild
                  VersionMajor
                  VersionMinor
                  VisionCBRespond
                  VPP
            Active
               Card
               DeviceID
               UseEnetForCallback
		 Ardence
			RTX
			   RTXDir
         VenturCom
            RTX
               RTXDir
*/

// Registry Key Names
#define AER_REG_DEF_DEVICEID  "DeviceID"              // Default DeviceID (DWORD)
#define AER_REG_DEF_CARD      "Card"                  // Default Card (DWORD)

#define AER_REG_BUILD_NUMBER        "Version Build"      // Version of the software installed
#define AER_REG_VERSION_MAJOR       "Version Major"      // Major version number of the software installed
#define AER_REG_VERSION_MINOR       "Version Minor"      // Minor version number of the software installed
#define AER_REG_AXES_OF_CONTROL     "AxesOfControl"      // Password which contains the number of axes of control
#define AER_REG_CNC5                "CNC5"               // Password which contains CNC5 Enabled Flag
#define AER_REG_NFIBER              "NFiber"             // Password which contains NFiber Enabled Flag
#define AER_REG_NLOGIC              "NLogic"             // Password which contains NLogic Enabled Flag
#define AER_REG_NMARK_VCT           "NMark"              // Password which contains NMark VCT Enabled Flag
#define AER_REG_NMARK_GRC           "NMark"              // Password which contains NMark GRC Enabled Flag
#define AER_REG_NMARK_LMS           "NMark"              // Password which contains NMark LMS Enabled Flag
#define AER_REG_NVISION             "NVision"            // Password which contains NVision Enabled Flag
#define AER_REG_SMCARGUMENTS        "SMCArguments"       // Command line arguments for the SMC
#define AER_REG_VISION_CB_RESPOND   "VisionCBRespond"    // Name of the COM object that responds to vision callbacks
#define AER_REG_PCI_VENDOR_ID       "PCIVendorID"        // Vendor ID for an unsupported firewire card
#define AER_REG_PCI_DEVICE_ID       "PCIDeviceID"        // Device ID for an unsupported firewire card
#define AER_REG_PCI_BUS_NUMBER      "PCIBusNumber"       // PCI Bus Number for a specific firewire card
#define AER_REG_PCI_SLOT_NUMBER     "PCISlotNumber"      // PCI Slot Number for a specific firewire card
#define AER_REG_PCI_FUNCTION_NUMBER "PCIFunctionNumber"  // PCI Function Number for a specific firewire card
#define AER_REG_SMC_OPTIONS         "NMotionSMCOptions"  // Bit mask for controling optional SMC software
                                                         //    1 -  Enable\Disable Global Ethernet IO
#define AER_REG_DEVDRIVERTIMEOUT    "DevDriveTimeOut"
#define AER_REG_DEVDRIVERTIMEOUTNUMTIMES    "DevDriveTimeOutNumTimes"
#define AER_REG_RTSSINSTALLDIR      "RtssInstallDir"
#define AER_REG_REMOTESERVERDEBUG   "RemoteServerDebug"  //Determine whether to run in client-server debug mode


#define AER_REG_ENET_CLIENT_IP   "ClientIPAddress"    // IP Address of the Client computer
#define AER_REG_ENET_SERVER_IP   "ServerIPAddress"    // IP Address of the Server computer
#define AER_REG_ENET_SERVER_PORT_NUM  "ServerPortNumber"    // Server listens on this port for EtherNet communication
#define AER_REG_ENET_CBACK_PORT_NUM   "CallbackPortNumber"  // Server uses this for sending callback interrupt

// Axes of control definitions
#define AER_REG_AXES_OF_CONTROL_2         0x000000
#define AER_REG_AXES_OF_CONTROL_4         0x000001
#define AER_REG_AXES_OF_CONTROL_6         0x000002
#define AER_REG_AXES_OF_CONTROL_8         0x000003
#define AER_REG_AXES_OF_CONTROL_10        0x000004
#define AER_REG_AXES_OF_CONTROL_12        0x000005
#define AER_REG_AXES_OF_CONTROL_14        0x000006
#define AER_REG_AXES_OF_CONTROL_16        0x000007
#define AER_REG_AXES_OF_CONTROL_18        0x000008
#define AER_REG_AXES_OF_CONTROL_20        0x000009
#define AER_REG_AXES_OF_CONTROL_22        0x00000A
#define AER_REG_AXES_OF_CONTROL_24        0x00000B
#define AER_REG_AXES_OF_CONTROL_26        0x00000C
#define AER_REG_AXES_OF_CONTROL_28        0x00000D
#define AER_REG_AXES_OF_CONTROL_30        0x00000E
#define AER_REG_AXES_OF_CONTROL_32        0x00000F

// Conditional install options
#define AER_REG_NVIEW_HMI                 0x000100
#define AER_REG_NCONTROL_SDK              0x000200
#define AER_REG_NVISION_ENABLED           0x000400
#define AER_REG_NFIBER_ENABLED            0x001000

// Runtime verifiable only options
#define AER_REG_CNC5_ENABLED              0x010000
#define AER_REG_NMARK_VCT_ENABLED         0x020000
#define AER_REG_NMARK_GRC_ENABLED         0x040000
#define AER_REG_NMARK_LMS_ENABLED         0x080000

#define AER_MAX_PROJECTS 10

typedef struct tagAER_REG_DEVICE_INFO
{
   TCHAR szSMCArguments[MAX_REG_SZ_SIZE + 1];
   DWORD dwPCIVendorID;
   DWORD dwPCIDeviceID;
   DWORD dwPCIBusNumber;
   DWORD dwPCISlotNumber;
   DWORD dwPCIFunctionNumber;
   DWORD dwSMCOptions;
   DWORD dwSpare1;
   DWORD dwSpare2;
   DWORD dwSpare3;
   DWORD dwSpare4;
} AER_REG_DEVICE_INFO;
typedef AER_REG_DEVICE_INFO   *PAER_REG_DEVICE_INFO;

typedef struct tagAER_REG_ETHERNET_INFO
{
   TCHAR szClientIPNumber[MAX_REG_SZ_SIZE+1];
   TCHAR szServerIPNumber[MAX_REG_SZ_SIZE+1];

   DWORD dwPortNumber;
   DWORD dwCallbackPortNumber;

} AER_REG_ETHERNET_INFO;
typedef AER_REG_ETHERNET_INFO    *PAER_REG_ETHERNET_INFO;

#define MAX_NUM_FIREWIRE_CARDS 5

typedef struct tagFIREWIRE_CARD_DATA
{
   DWORD          dwNumFirewireCards;
   DWORD          dwRTXInfSupport[5];
   TCHAR          szDeviceDescrip[5][MAX_REG_SZ_SIZE + 1];
   TCHAR          szLocationInfo[5][MAX_REG_SZ_SIZE + 1];
} FIREWIRE_CARD_DATA, *PFIREWIRE_CARD_DATA;

#ifdef __cplusplus
extern "C" {
#endif



AERERR_CODE AER_DLLENTRY AerRegSetDefDevice( DWORD dwDeviceID, DWORD dwCard );
AERERR_CODE AER_DLLENTRY AerRegGetDefDevice( PDWORD pdwDeviceID, PDWORD pdwCard );
AERERR_CODE AER_DLLENTRY AerRegConvertDefDevice( DWORD dwInDeviceID, DWORD dwInCard,
                                                 PDWORD pdwOutDeviceID, PDWORD pdwOutCard );

AERERR_CODE AER_DLLENTRY AerRegGetInstallDirPath( DWORD dwDeviceID, DWORD dwCard, LPTSTR pszFile );
AERERR_CODE AER_DLLENTRY AerRegGetParamFileName( DWORD dwDeviceID, DWORD dwCard, LPTSTR pszFile );
AERERR_CODE AER_DLLENTRY AerRegGetIniFileName( DWORD dwDeviceID, DWORD dwCard, LPTSTR pszFile );

AERERR_CODE AER_DLLENTRY AerRegGetFileName( DWORD dwDeviceID, DWORD dwCard,
                                            DWORD dwRegId, LPTSTR pszFile );
AERERR_CODE AER_DLLENTRY AerRegSetFileName( DWORD dwDeviceID, DWORD dwCard,
                                            DWORD dwRegId, LPCTSTR pszFile );

AERERR_CODE AER_DLLENTRY AerRegGetEtherNetInfo( DWORD dwDeviceID, DWORD dwCard,
                                                PAER_REG_ETHERNET_INFO pInfo );

AERERR_CODE AER_DLLENTRY AerRegSetEtherNetInfo( DWORD dwDeviceID, DWORD dwCard,
                                                AER_REG_ETHERNET_INFO Info );

AERERR_CODE AER_DLLENTRY AerRegQueryCardCount( DWORD dwDeviceID,
                                               PDWORD pdwCount );
AERERR_CODE AER_DLLENTRY AerRegQueryCardList( DWORD dwDeviceID, DWORD dwCount,
                                              PDWORD pdwList );

DWORD AER_DLLENTRY AerA3200IniGetBuildNumber(LPCTSTR szIniFileName);
void AER_DLLENTRY AerA3200IniGetCurrentProjectName(LPCTSTR szIniFileName, LPTSTR pszSection);
void AER_DLLENTRY AerA3200IniGetProjectName(LPCTSTR szIniFileName, DWORD dwProject, LPTSTR pszSection);


AERERR_CODE AER_DLLENTRY AerRegSetAxesOfControl (DWORD dwDeviceID, PTCHAR szAxesOfControl, PTCHAR szMinorVersion);
AERERR_CODE AER_DLLENTRY AerRegSetCNC5 ( DWORD dwDeviceID, PTCHAR szCNC5, PTCHAR szMajorVersion);

AERERR_CODE AER_DLLENTRY AerRegGetAxesOfControl (DWORD dwDeviceID, PDWORD pdwNumAxes);
AERERR_CODE AER_DLLENTRY AerRegGetAxesOfControlFromKey (DWORD dwDeviceID, PTCHAR szAxesOfControl, PTCHAR szMinorVersion, PDWORD pdwNumAxes );
AERERR_CODE AER_DLLENTRY AerRegGetCNC5Enabled( DWORD dwDeviceID, PBOOL pbCNC5Enabled );
AERERR_CODE AER_DLLENTRY AerRegGetCNC5EnabledFromKey ( DWORD dwDeviceID, PTCHAR szCNC5, PTCHAR szMajorVersion, PBOOL pbCNC5Enabled );
AERERR_CODE AER_DLLENTRY AerRegGetNFiberEnabled( DWORD dwDeviceID, PBOOL pbNFiberEnabled );
AERERR_CODE AER_DLLENTRY AerRegGetNMarkVCTEnabled( DWORD dwDeviceID, PBOOL pbNMarkVCTEnabled );
AERERR_CODE AER_DLLENTRY AerRegGetNMarkGRCEnabled( DWORD dwDeviceID, PBOOL pbNMarkGRCEnabled );
AERERR_CODE AER_DLLENTRY AerRegGetNMarkLMSEnabled( DWORD dwDeviceID, PBOOL pbNMarkLMSEnabled );
AERERR_CODE AER_DLLENTRY AerRegGetNVisionEnabled( DWORD dwDeviceID, PBOOL pbNVisionEnabled );
AERERR_CODE AER_DLLENTRY AerRegGetDeviceDriverTimeOutDefault( DWORD dwDeviceID, PDWORD pdwDeviceDriverTimeoutMsec, PDWORD pdwDeviceDriverTimeoutNumTimes);

AERERR_CODE AER_DLLENTRY AerRegGetSMCArguments (DWORD dwDeviceID, PTCHAR pszSMCArguments, DWORD dwSMCArgumentsBytes);


AERERR_CODE writeIniString( LPCTSTR pszFile, LPCTSTR pszSection, LPCTSTR pszKey,
                            LPCSTR pszValue );
BOOL        readIniString( LPCTSTR pszFile, LPCTSTR pszSection, LPCTSTR pszKey,
                           LPCTSTR pszDefault, LPTSTR pszValue );

AERERR_CODE writeIniDWord( LPCTSTR pszFile, LPCTSTR pszSection, LPCTSTR pszKey,
                           DWORD dwValue );
DWORD readIniDWord( LPCTSTR pszFile, LPCTSTR pszSection, LPCTSTR pszKey,
                    DWORD dwDefault );

AERERR_CODE writeIniDouble( LPCTSTR pszFile, LPCTSTR pszSection, LPCTSTR pszKey,
                            DOUBLE dValue );
DOUBLE readIniDouble( LPCTSTR pszFile, LPCTSTR pszSection, LPCTSTR pszKey,
                      DOUBLE dDefault );

AERERR_CODE AER_DLLENTRY AerRegGetDeviceInfo( DWORD dwDeviceID, DWORD dwCard,
                                              PAER_REG_DEVICE_INFO pInfo );
AERERR_CODE AER_DLLENTRY AerRegGetDeviceInfoEx( DWORD dwDeviceID, DWORD dwCard,
                                                LPTSTR pszSMCArguments, LPTSTR pszVisionCBRespond,
                                                PDWORD pdwPCIVendorID, PDWORD pdwPCIDeviceID,
                                                PDWORD pdwPCIBusNumber, PDWORD pdwPCIDeviceNumber,
                                                PDWORD pdwSMCOptions, PDWORD pdwPCIFunctionNumber);

AERERR_CODE AER_DLLENTRY AerRegSetDeviceInfo( DWORD dwDeviceID, DWORD dwCard,
                                              AER_REG_DEVICE_INFO Device );
AERERR_CODE AER_DLLENTRY AerRegSetDeviceInfoEx( DWORD dwDeviceID, DWORD dwCard,
                                                LPCTSTR pszSMCArguments, LPCTSTR pszVisionCBRespond,
                                                DWORD dwPCIVendorID, DWORD dwPCIDeviceID,
                                                DWORD dwPCIBusNumber, DWORD dwPCIDeviceNumber,
                                                DWORD dwSMCOptions, DWORD dwPCIFunctionNumber);

//AERERR_CODE AER_DLLENTRY AerRegGetU600DeviceInfoEx( DWORD dwCard,
//                                                    PDWORD pdwDSC );
//AERERR_CODE AER_DLLENTRY AerRegSetU600DeviceInfoEx( DWORD dwCard,
//                                                    DWORD dwDSC );

AERERR_CODE AER_DLLENTRY AerRegGetDefDeviceInfo( DWORD dwDeviceID,
                                                 PAER_REG_DEVICE_INFO pInfo );

//AERERR_CODE AER_DLLENTRY AerRegSystemSetup( DWORD dwDeviceID );

//AERERR_CODE AER_DLLENTRY AerRegGetPSOInfo( DWORD dwDeviceID, DWORD dwCard,
//                                           PAER_REG_PSO_INFO pInfo );
//AERERR_CODE AER_DLLENTRY AerRegGetPSOInfoEx( DWORD dwDeviceID, DWORD dwCard,
//                                             LPTSTR pszPSOImage,
//                                             PDWORD pdwIOBase );
//AERERR_CODE AER_DLLENTRY AerRegSetPSOInfo( DWORD dwDeviceID, DWORD dwCard,
//                                           PAER_REG_PSO_INFO pInfo );
//AERERR_CODE AER_DLLENTRY AerRegSetPSOInfoEx( DWORD dwDeviceID, DWORD dwCard,
//                                             LPTSTR pszPSOImage,
//                                             DWORD dwIOBase );

//AERERR_CODE AER_DLLENTRY AerDo132BackCompatibilityStuff(DWORD dwDeviceID, DWORD dwCard);

AERERR_CODE AER_DLLENTRY AerRegSetNLogicCycleTime( DWORD dwDeviceID, DWORD dwCard, DWORD dwCycleTime );
AERERR_CODE AER_DLLENTRY AerRegGetNLogicCycleTime( DWORD dwDeviceID, PDWORD pdwCycleTime );

AERERR_CODE AER_DLLENTRY AerRegCheckFWCardRTXInfSupport( DWORD dwPCIBusNumber, DWORD dwPCISlotNumber,
                                                         PDWORD pdwSupport );

AERERR_CODE AER_DLLENTRY AerRegCheckFWCardRTXInfSupportEx( DWORD dwPCIBusNumber, DWORD dwPCISlotNumber,
                                                           DWORD dwPCIFunctionNumber, PDWORD pdwSupport );
AERERR_CODE AER_DLLENTRY AerRegSearchForFWCards();
AERERR_CODE AER_DLLENTRY AerRegGetNumOfFWCards( PDWORD pdwNumberOfCards );
AERERR_CODE AER_DLLENTRY AerRegGetFWCardInfo( DWORD dwCardNumber, PDWORD pdwRTXInfSupport,
                                              LPTSTR pszDeviceDescription, LPTSTR pszLocationInfo );







#ifdef __cplusplus
}
#endif

#endif
// __AER_REG_H__
