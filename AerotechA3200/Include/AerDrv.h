/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
   Stuff needed in ALL device drivers
+++*/

#ifndef __AERDRV_H__
#define __AERDRV_H__

/*
 Lib Error data structure  (returned by AerDrvGetLastCmdErr())
*/
typedef struct tagAER_LASTCMD_INFO
{  // Used for holding library errors/lastCommand
   DWORD          dwIndex;          // Device Driver index
   DWORD          iAxis;            // Axis Index
   DRV_COMM_LENG     wTranLen;         // Transmit Len
   DRV_COMM_LENG     wRecvLen;         // Receive Len
   WORD           wOpCode;          // Opcode
   WORD           wSubCode;         // Subcode
   DWORD          dwCmdType;        // CMDTYPE_XXXX
   AERERR_CODE    eRc;              // AERERR_XXXX
   DWORD          dwActualRecvLen;
   BOOL           bWasResetting;
   DWORD          dwSpare1;
   DWORD          dwSpare2;
   DWORD          dwSpare3;
} AER_LASTCMD_INFO;
typedef AER_LASTCMD_INFO *PAER_LASTCMD_INFO;



/* Device driver Identifiers */
#define IsDeviceA3200( dwDeviceID )                ( (dwDeviceID == AUTOMATION_3200) || (dwDeviceID == AUTOMATION_3200_ETHERNET) )
#define IsDeviceA3200SoftwareOnly( dwDeviceID )    (dwDeviceID == AUTOMATION_3200)
#define IsDeviceA3200EtherNet( dwDeviceID )        (dwDeviceID == AUTOMATION_3200_ETHERNET)

//typedef struct tagAER_UNIDEX_INFO
//{
//   DWORD dwDeviceID;    /* Device ID */
//   DWORD dwATWindow;    /* AT Window */
//   DWORD dwIOBase;      /* IO Base address */
//   DWORD dwIRQ;         /* IRQ number */
//   DWORD dwDSC;         /* ??? */
//   DWORD dwATWindow2;   /* 2nd ATWindow */
//} AER_UNIDEX_INFO;
//typedef AER_UNIDEX_INFO *PAER_UNIDEX_INFO;
//
//
// DeviceIOControl OPCode's
// These AER_IOCTL codes conform as specified by Microsoft (see winioctl.h)
//
// U500 control codes defined in U500 NT headers (AerQuick.h) - will
// ultimately use some of this stuff, but must get it working right now

#define AER_U500_DEVICE_TYPE     0xA500   // Between 0x8000-0xFFFF
#define AER_U600_DEVICE_TYPE     0xA600   // Between 0x8000-0xFFFF
#define AER_U631_DEVICE_TYPE     (AER_U600_DEVICE_TYPE)

#define AER_IOCTL_CODE           0x800    // Between 0x800-0xFFF
                                          // Same for all controllers

// CTL_CODE is from winioctl.h
#define CTL_CODE( DeviceType, Function, Method, Access ) (                 \
    ((DeviceType) << 16) | ((Access) << 14) | ((Function) << 2) | (Method) \
)

#define AER_U600_CODE( ioctl )   \
   ((DWORD) (CTL_CODE( AER_U600_DEVICE_TYPE, AER_IOCTL_CODE+ioctl, 0, 0 )))

//#define AER_IOCTL_GETVERSION        AER_U600_CODE(1)  // Obsolete
//#define AER_IOCTL_BMCCOMMAND        AER_U600_CODE(2)  // Obsolete
//#define AER_IOCTL_READDIRECT        AER_U600_CODE(3)
//#define AER_IOCTL_WRITEDIRECT       AER_U600_CODE(4)
#define AER_IOCTL_AXISCOMMAND       AER_U600_CODE(5)
#define AER_IOCTL_SYSTEMCOMMAND     AER_U600_CODE(6)
#define AER_IOCTL_EVENTCOMMAND      AER_U600_CODE(7)
#define AER_IOCTL_DCCOMMAND         AER_U600_CODE(8)  // DataCenterCommand
//#define AER_IOCTL_COMMERRCOMMAND    AER_U600_CODE(9)  // Obsolete
#define AER_IOCTL_DRVCOMMAND        AER_U600_CODE(10)
#define AER_IOCTL_DRV_BLOBCOMMAND   AER_U600_CODE(11)
#define AER_IOCTL_SOFTCMD           AER_U600_CODE(12)
// PCI Configuration information
//#define U600PCI_VENDORID   0x11b0
//#define U600PCI_DEVICEID   4
//
//// Offsets in U600PCI_REGISTER_MAP
//#define U600PCI_OFFSET_VENDOR    0x00     // Vendor ID
//#define U600PCI_OFFSET_DEVICE    0x02     // Device
//#define U600PCI_OFFSET_CMD_STAT  0x04     // CMD and STAT
//#define U600PCI_OFFSET_CMD       0x04     // CMD
//#define U600PCI_OFFSET_STAT      0x06     // STAT
//#define U600PCI_OFFSET_CC_REV    0x08     // Revision
//#define U600PCI_OFFSET_IO_BASE   0x10
//#define U600PCI_OFFSET_BASE0     0x14     // ATWindow1
//#define U600PCI_OFFSET_BASE1     0x18     // ATWindow2
//#define U600PCI_OFFSET_BPARAM    0x3C
//#define U600PCI_OFFSET_MAP0      0x40     // Address Mapping
//#define U600PCI_LB_IO_BASE       0x6E
//#define U600PCI_OFFSET_SYSTEM    0x78     // System/Reset register
//#define U600PCI_OFFSET_CFG       0x7C     // Configuration Register
//
//
//// PCI Register Bit defines
//#define U600PCI_SYSTEM_UNLOCK    0xa05f   // need to write this value to
//                                          //     U600PCI_SYSTEM_OFFSET
//                                          // if LOCK bit (bit 14) is set
//#define U600PCI_SYSTEM_RST_OUT   0x8000   // SYSTEM REGISTER - Reset
//#define U600PCI_SYSTEM_LOCK      0x4000   //
//#define U600PCI_SYSTEM_SPROM_EN  0x2000   //
//#define U600PCI_SYSTEM_SCL       0x1000   //
//#define U600PCI_SYSTEM_SDA_OUT   0x0800   //
//
//#define U600PCI_SYSTEM_DEFAULT      (0xF800) // RST_OUT, LOCK, SPROM_EN, SCL, SDA_OUT
//#define U600PCI_CMD_STAT_DEFAULT    (0x7)
//#define U600PCI_MAP0_DEFAULT        (0x3)
//#define U600PCI_IO_BASE_DEFAULT     (0xE401)
//#define U600PCI_BPARAM_DEFAULT      (0xff)
//#define U600PCI_CFG_DEFAULT         (0x66)   // Configuration Register
//#define U600PCI_LB_IO_BASE_DEFAULT  (0x800)

// Revision mask
//#define AERDRV_REVC_MASK   0x0001
//#define AERDRV_66MHZ_MASK  0x0002
//#define AERDRV_REVF_MASK   0x0005 // (Rev D,E,F all the same)

// AT Window Numbers
//#define ATWIN_NUM_0 0
//#define ATWIN_NUM_1 1
//#define ATWIN_NUM_2 2
//#define ATWIN_NUM_3 3
//
//// AT Window
//#define ATWIN_D800  0xD8000000
//#define ATWIN_DC00  0xDC000000
//#define ATWIN_CC00  0xCC000000
//#define ATWIN_FFC0  0xFFC00000

// Dram Size Code
//#define DSC_2MEG    0
//#define DSC_8MEG    1
//#define DSC_32MEG   2
//#define DSC_64MEG   4

//#define AER_DRV600_DEFAULT_ATWIN      ATWIN_DC00
//#define AER_DRV600_DEFAULT_ATWIN2     0
//#define AER_DRV600PCI_DEFAULT_ATWIN   0xD0000000
//#define AER_DRV600PCI_DEFAULT_ATWIN2  0
//#define AER_DRV631_DEFAULT_ATWIN      0xE0000000
//#define AER_DRV631_DEFAULT_ATWIN2     0
//
//#define AER_DRV600_DEFAULT_IO      0x220
//#define AER_DRV600PCI_DEFAULT_IO   0x0
//#define AER_DRV631_DEFAULT_IO      0x8104
//
//#define AER_DRV600_DEFAULT_IOCOUNT     0xa
//#define AER_DRV600PCI_DEFAULT_IOCOUNT  0x0
//#define AER_DRV631_DEFAULT_IOCOUNT     0x5d
//
//#define  AER_DRV600_DEFAULT_IRQ     5
//#define  AER_DRV600PCI_DEFAULT_IRQ  5
//#define  AER_DRV631_DEFAULT_IRQ     10
//
//#define  AER_DRV600_DEFAULT_DSC        DSC_2MEG
//#define  AER_DRV631_DEFAULT_VME_ADDR   0x800000
//
//#define  AER_DRV600_DEFAULT_PSOIO   0

// DRV_OPCODE_CFG for U600
//#define  BMCCFG_INIT 0
//#define  BMCCFG_LOW  1
//#define  BMCCFG_HIGH 2

/* structure and defines to get command error info */

#define CMDTYPE_AXIS   1  // Axis Command
#define CMDTYPE_SYSTEM 2  // System Command

//
// This structure is the Register Map for the PCI - V3 chip
// It can be used in place of the PCI_COMMON_CONFIG structure in the NT DDK (ntddk.h)
// It should be 256 Bytes
//
//typedef struct tagAER_PCI_REGISTER_MAP
//{
//   WORD  PCI_VENDOR;          // 0x00 (offset in structure)
//   WORD  PCI_DEVICE;          // 0x02
//   WORD  PCI_CMD;             // 0x04
//   WORD  PCI_STAT;            // 0x06
//   DWORD PCI_CC_REV;          // 0x08
//   DWORD PCI_HDR_CFG;         // 0x0C
//   DWORD PCI_IO_BASE;         // 0x10
//   DWORD PCI_BASE0;           // 0x14
//   DWORD PCI_BASE1;           // 0x18
//   DWORD reserved1;           // 0x1C
//   DWORD reserved2;           // 0x20
//   DWORD reserved3;           // 0x24
//   DWORD reserved4;           // 0x28
//   WORD  PCI_SUB_VENDOR;      // 0x2C
//   WORD  PCI_SUB_DEVICE;      // 0x2e
//   DWORD PCI_ROM;             // 0x30
//   DWORD reserved5;           // 0x34
//   DWORD reserved6;           // 0x38
//   DWORD PCI_BPARAM;          // 0x3C
//   DWORD PCI_MAP0;            // 0x40
//   DWORD PCI_MAP1;            // 0x44
//   DWORD PCI_INT_STAT;        // 0x48
//   DWORD PCI_INT_CFG;         // 0x4C
//   DWORD reserved7;           // 0x50
//   DWORD LB_BASE0;            // 0x54
//   DWORD LB_BASE1;            // 0x58
//   WORD  reserved8;           // 0x5C
//   WORD  LB_MAP0;             // 0x5E
//   WORD  reserved9;           // 0x60
//   WORD  LB_MAP1;             // 0x62
//   WORD  LB_BASE2;            // 0x64
//   WORD  LB_MAP2;             // 0x66
//   DWORD LB_SIZE;             // 0x68
//   WORD  reserved10;          // 0x6C
//   WORD  LB_IO_BASE;          // 0x6E
//   WORD  FIFO_CFG;            // 0x70
//   WORD  FIFO_PRIORITY;       // 0x72
//   WORD  FIFO_STAT;           // 0x74
//   BYTE  LB_ISTAT;            // 0x76
//   BYTE  LB_IMASK;            // 0x77
//   WORD  SYSTEM;              // 0x78
//   WORD  LB_CFG;              // 0x7A
//   WORD  PCI_CFG;             // 0x7C
//   WORD  reserved11;          // 0x7E
//   BYTE  pad[128];            // make sure we're 256 bytes in length
//} AER_PCI_REGISTER_MAP;
//typedef AER_PCI_REGISTER_MAP  *PAER_PCI_REGISTER_MAP;

typedef struct tagAER_DRV_DEBUG_INFO
{
   DWORD       dwDeviceID;       /* Device ID */
   DWORD       dwIndex;          /* Identify myself */
   DWORD       dwATWindow;       /* AT Window */
   DWORD       dwATMapped;       /* Mapped AT Address (was dwPhysAddr) */
   DWORD       dwATPage;         /* AT Window page (was dwPageAddr) */
   DWORD       dwATMask;         /* AT Window mask (was ATWINDOW_MASK) */
   DWORD       dwATSize;         /* AT Window size (was WINDOW_SIZE) */
   DWORD       dwIOBase;         /* IO Base address */
   DWORD       dwIOCount;        /* IO address length */
   DWORD       dwIOMapped;       /* Mapped IO Address */
   DWORD       dwIRQ;            /* IRQ number */
   DWORD       dwIRQMapped;      /* Mapped IRQ number */
   DWORD       dwPSOIOBase;      /* PSO IO Base address */
   DWORD       dwPSOIOCount;     /* PSO IO address length */
   DWORD       dwPSOIOMapped;    /* PSO Mapped IO Address */
   DWORD       dwOpenCount;      /* # of times opened */
   DWORD       dwWait;           /* ProgError wait state */
   DWORD       dwRevision;       /* Hardware Revision */
   DWORD       dwATWindow2;      /* AT Window 2 */
   DWORD       dwATMapped2;      /* Mapped AT Address 2 */
   DWORD       dwResetOnTimeout; /* Reset on Timeout flag */
} AER_DRV_DEBUG_INFO;
typedef AER_DRV_DEBUG_INFO *PAER_DRV_DEBUG_INFO;

// AER_IOCTL_DRVCOMMAND op-codes (these commands do not go down to the firmware)
//#define DRV_OPCODE_PROG_SETWAIT        1
//#define DRV_OPCODE_PROG_GETWAIT        2
//#define DRV_OPCODE_OPEN                3
//#define DRV_OPCODE_CLOSE               4
//#define DRV_OPCODE_CONFIG              5
//#define DRV_OPCODE_EXECUTE             6
//#define DRV_OPCODE_RESET               7
#define DRV_OPCODE_CLR_LASTCMDERR      8
#define DRV_OPCODE_GET_LASTCMD         9
#define DRV_OPCODE_GET_LASTCMDERR      10
#define DRV_OPCODE_GET_VERSION         11
#define DRV_OPCODE_GET_DEBUG           12
#define DRV_OPCODE_SET_REVISION        13 // Hardware revision
//#define DRV_OPCODE_RESET_PSO           14
//#define DRV_OPCODE_OPEN_PSO            15 // Open PSO Card (95 specific)
#define DRV_OPCODE_SET_TIMEOUT         16
#define DRV_OPCODE_GET_TIMEOUT         17
//#define DRV_OPCODE_GET_SERIAL          18
#define DRV_OPCODE_SET_RESETONTIMEOUT  19
#define DRV_OPCODE_GET_RESETONTIMEOUT  20
#define DRV_OPCODE_SET_PCI_CFGDATA     21
#define DRV_OPCODE_GET_PCI_CFGDATA     22
#define DRV_OPCODE_GET_NUMLASTCMD      23
#define DRV_OPCODE_SET_LASTCMD_SPARE   24

#define DRV_OPCODE_GET_DLL_SHARED_SINGLE 32



//ALL entry points into AswDrv.dll
//This is required for Client-server
#define DRV_OPCODE_SYSTEM_SOFTOPEN                 24
#define DRV_OPCODE_SYSTEM_SOFTCLOSE                25
#define DRV_OPCODE_SYSTEM_SOFTRECLAIM              26
#define DRV_OPCODE_SYSTEM_SOFTKILL                 27
#define DRV_OPCODE_SYSTEM_SOFTISOPEN               28
#define DRV_OPCODE_SYSTEM_SOFTSERVSTART            29
//#define DRV_OPCODE_SYSTEM_SOFTFWOPEN               30
//#define DRV_OPCODE_SYSTEM_SOFTFWCLOSE              31
#define DRV_OPCODE_SYSTEM_SOFTDIOCTRL              32
//#define DRV_OPCODE_SYSTEM_SOFTEVENTCTRL            33
#define DRV_OPCODE_SYSTEM_SOFTGETLASTERR           34
//#define DRV_OPCODE_SYSTEM_DRVSETSTATUS             35
//#define DRV_OPCODE_SYSTEM_DRVGETSTATUS             36
#define DRV_OPCODE_SYSTEM_SOFTGETPROCESSNUM        37
#define DRV_OPCODE_SYSTEM_SOFTRESETANDWAIT         38
#define DRV_OPCODE_SYSTEM_SOFTNLOADRESETANDWAIT    39



//Watchdog
#define DRV_OPCODE_ENET_WATCHDOG_START  40
#define DRV_OPCODE_ENET_WATCHDOG_POLL   41
#define DRV_OPCODE_ENET_WATCHDOG_STOP   42
#define DRV_OPCODE_ENET_WATCHDOG_PAUSE  43
#define DRV_OPCODE_ENET_WATCHDOG_RESUME 44


//Heartbeat (replaces "WatchdogDog")
#define DRV_OPCODE_ENET_HEARTBEAT_POLL    47



// AER_IOCTL_DRV_BLOBCOMMAND op-codes
#define DRVBLOB_OPCODE_DOWNLOAD_PSO 1


#ifdef __cplusplus
extern "C" {
#endif

typedef struct tagAER_DRV_VERSION
{
   WORD  wUnidex;    /* AER_UNIDEX_xxxx */
   WORD  wMajor;     /* Major Version Number */
   WORD  wMinor;     /* Minor Version Number */
   WORD  wBuild;     /* Internal Build Number */
} AER_DRV_VERSION;
typedef AER_DRV_VERSION   *PAER_DRV_VERSION;

//AERERR_CODE AER_DLLENTRY AerDrvGetNumLastCmd( HAERCTRL hAerCtrl, PDWORD pdwNumCmd );
//AERERR_CODE AER_DLLENTRY AerDrvGetLastCmdEx( HAERCTRL hAerCtrl, DWORD dwCmd,
//                                             PAER_LASTCMD_INFO pLastCmd );
//AERERR_CODE AER_DLLENTRY AerDrvGetLastCmdErrEx( HAERCTRL hAerCtrl, DWORD dwCmd,
//                                                PAER_LASTCMD_INFO pLastCmdErr );

AERERR_CODE AER_DLLENTRY AerDrvSetLastCmdSpare( HAERCTRL hAerCtrl, DWORD dwSpareNum, DWORD dwSpareVal);
AERERR_CODE AER_DLLENTRY AerDrvGetLastCmd( HAERCTRL hAerCtrl,
                                           PAER_LASTCMD_INFO pLastCmd );
AERERR_CODE AER_DLLENTRY AerDrvGetLastCmdErr( HAERCTRL hAerCtrl,
                                              PAER_LASTCMD_INFO pLastCmdErr );
AERERR_CODE AER_DLLENTRY AerDrvClearLastCmdErr( HAERCTRL hAerCtrl );

AERERR_CODE AER_DLLENTRY AerDrvSetResetOnTimeOut( HAERCTRL hAerCtrl, BOOL bReset );
AERERR_CODE AER_DLLENTRY AerDrvGetResetOnTimeOut( HAERCTRL hAerCtrl, PBOOL pbReset );

//AERERR_CODE AER_DLLENTRY aerDrvOpen( HAERCTRL hAerCtrl, PAER_UNIDEX_INFO pUnidex,
//                                     PDWORD pdwIndex );
//AERERR_CODE AER_DLLENTRY aerDrvClose( HAERCTRL hAerCtrl );
//AERERR_CODE AER_DLLENTRY aerDrvConfig( HAERCTRL hAerCtrl, DWORD dwCfg );
//AERERR_CODE AER_DLLENTRY aerDrvExecute( HAERCTRL hAerCtrl, DWORD dwStartAddr );
//AERERR_CODE AER_DLLENTRY aerDrvFillMem( HAERCTRL hAerCtrl, DWORD dwStartAddr, DWORD dwBytes,
//                                        DWORD dwChar );
//AERERR_CODE AER_DLLENTRY aerDrvSetRevision( HAERCTRL hAerCtrl );

//AERERR_CODE AER_DLLENTRY aerDrvOpenPSO( HAERCTRL hAerCtrl );
//AERERR_CODE AER_DLLENTRY aerDrvResetPSO( HAERCTRL hAerCtrl );
//AERERR_CODE AER_DLLENTRY aerDrvDownloadPSO( HAERCTRL hAerCtrl, PVOID pvImage, DWORD dwSize );

//AERERR_CODE AER_DLLENTRY aerDrvGetDebugInfo( HAERCTRL hAerCtrl,
//                                             PAER_DRV_DEBUG_INFO pDebug );


AERERR_CODE AER_DLLENTRY aerDrvSetTimeOut( HAERCTRL hAerCtrl, DWORD dwTimeOut );
AERERR_CODE AER_DLLENTRY aerDrvGetTimeOut( HAERCTRL hAerCtrl, PDWORD pdwTimeOut );

//AERERR_CODE AER_DLLENTRY aerDrvGetSerialNumber( HAERCTRL hAerCtrl, PDWORD pdwSerial );

AERERR_CODE AER_DLLENTRY aerDrvPCISetConfig( HAERCTRL hAerCtrl, DWORD dwOffset,
                                             PVOID pvData, DWORD dwSize );
AERERR_CODE AER_DLLENTRY aerDrvPCIGetConfig( HAERCTRL hAerCtrl, DWORD dwOffset,
                                             PVOID pvData, DWORD dwSize );

AERERR_CODE AER_DLLENTRY aerDrvPCICfgSetByte( HAERCTRL hAerCtrl, DWORD dwOffset,
                                              BYTE byData );
AERERR_CODE AER_DLLENTRY aerDrvPCICfgSetWord( HAERCTRL hAerCtrl, DWORD dwOffset,
                                              WORD wData );
AERERR_CODE AER_DLLENTRY aerDrvPCICfgSetDWord( HAERCTRL hAerCtrl, DWORD dwOffset,
                                               DWORD dwData );

AERERR_CODE AER_DLLENTRY aerDrvPCICfgGetByte( HAERCTRL hAerCtrl, DWORD dwOffset,
                                              PBYTE pbyData );
AERERR_CODE AER_DLLENTRY aerDrvPCICfgGetWord( HAERCTRL hAerCtrl, DWORD dwOffset,
                                              PWORD pwData );
AERERR_CODE AER_DLLENTRY aerDrvPCICfgGetDWord( HAERCTRL hAerCtrl, DWORD dwOffset,
                                               PDWORD pdwData );
AERERR_CODE AER_DLLENTRY aerDrvSetWait( HAERCTRL hAerCtrl, BOOL bSet );
AERERR_CODE AER_DLLENTRY aerDrvGetWait( HAERCTRL hAerCtrl, PBOOL pbSet );
AERERR_CODE AER_DLLENTRY aerDrvGetVersion( HAERCTRL hAerCtrl, PAER_DRV_VERSION pVersion );

#ifdef __cplusplus
}
#endif

//////////////////////////////////

#define AER_STATUS_SHARED_MEM_MAX         10

// dwdThreadNum constants when using AerDrvGetSharedStatusWord()/AerDrvSetSharedStatusWord()
#define AER_STATUS_SHARED_ETHERNET1                        0x0  //used for remote server client
#define AER_STATUS_SHARED_ETHERNET2                        0x1  //used for remote server server
#define AER_STATUS_SHARED_TASK                             0x2  // 0x2 - 0x2+MAX_TASKS

// dwFunctNum constants when using AerDrvGetSharedStatusWord()//AerDrvSetSharedStatusWord()
#define AER_STATUS_SHARED_MEM_CNT                            0x01   // bump the hit counter
#define AER_STATUS_SHARED_MEM_RST                            0x02   // resets the hit counter
#define AER_STATUS_SHARED_MEM_ERR                            0x03   // sets the error code to dwValue
#define AER_STATUS_SHARED_MEM_STAT                           0x04   // sets the status to dwValue

typedef struct tagDLL_SHARE_SINGLE
{
   DWORD dwErr;
   DWORD dwCallbackError;     //only for callback errors
   DWORD dwCount;
   DWORD dwStatus;
}  DLL_SHARE_SINGLE;
typedef DLL_SHARE_SINGLE *PDLL_SHARE_SINGLE;

typedef struct tagDLL_SHARE
{
   int	            InstanceCount;	     //number of dll instances.
   int               OpenCount;
   int               g_OpenErrCode;      //global error code
   int               LastUsedCallBackPort;
   int               CallBackPorts[16];  //Instancecount 1 gets starting callback port,
                                         //next instance gets starting callback port+1 etc
   DLL_SHARE_SINGLE  Data[AER_STATUS_SHARED_MEM_MAX];
}  DLL_SHARE;
typedef DLL_SHARE *PDLL_SHARE;



#endif
// __AERVME_H__
