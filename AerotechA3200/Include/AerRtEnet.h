//////////////////////////////////////////////////////////////////////////
// Filename:             AswEnet.h
// Description:          Uses Rt-TCP/IP to do deterministic communication
//                       with remote ethernet devices. Supported post TCP
//                       protocols are Modbus, HTTP and Telnet.
//
// Date started:         10/27/2003
//
// Auther:               Mohammed M Rahman Ratin
// Operation:            One portion of the code acts as tcp client, fills
//                       in the Modbus data structure to send to the remote
//                       device that understands Modbus protocol. It then
//                       waits for an acknowledge and then sends the next
//                       modbus data. This is done in a state machine.
//
//
//
//
// Notes:
// History:
//
//
// Copyright (C) 2003 AEROTECH INC.
//                    All Rights Reserved.
//////////////////////////////////////////////////////////////////////////

#ifndef __AERRTENET_H_INCLUDED
#define __AERRTENET_H_INCLUDED

#pragma pack(push,enter_includeAerrtenet,2)

#include "AerVirtIO.h"

//Ethernet process init
DWORD RtEnetProcessInit(void);

//Ethernet process thread
DWORD RtEnetProcessLoop(void);


// Modbus specific defines

#define MODBUS_FC1    0x01
#define MODBUS_FC2    0x02
#define MODBUS_FC3    0x03
#define MODBUS_FC4    0x04
#define MODBUS_FC5    0x05
#define MODBUS_FC6    0x06
#define MODBUS_FC7    0x07
#define MODBUS_FC11   0x0B
#define MODBUS_FC15   0x0F
#define MODBUS_FC16   0x10
#define MODBUS_FC23   0x17

#define MODBUS_EXCEPTION_FC1   0x81
#define MODBUS_EXCEPTION_FC2   0x82
#define MODBUS_EXCEPTION_FC3   0x83
#define MODBUS_EXCEPTION_FC4   0x84
#define MODBUS_EXCEPTION_FC5   0x85
#define MODBUS_EXCEPTION_FC6   0x86
#define MODBUS_EXCEPTION_FC7   0x87
#define MODBUS_EXCEPTION_FC11  0x8B
#define MODBUS_EXCEPTION_FC15  0x8F
#define MODBUS_EXCEPTION_FC16  0x90
#define MODBUS_EXCEPTION_FC23  0x97

#define MODBUS_EXCEPTION_ILL_FUNC   0x01
#define MODBUS_EXCEPTION_ILL_ADD    0x02
#define MODBUS_EXCEPTION_ILL_VALUE  0x03
#define MODBUS_EXCEPTION_SALVE_FAIL 0x04

//status bits
#define GLOBAL_ETHERNET_FAULT_SYSTEM_ERROR                (1 << 0)
#define GLOBAL_ETHERNET_STATUS_PROCESS_TERMINATED         (1 << 1)
#define GLOBAL_ETHERNET_STATUS_RESERVED1                  (1 << 2)
#define GLOBAL_ETHERNET_STATUS_MODULE_CONNECTED           (1 << 3)
#define GLOBAL_ETHERNET_STATUS_NO_IO_TO_UPDATE            (1 << 4)
#define GLOBAL_ETHERNET_STATUS_A3200_INITIALIZING         (1 << 5)
#define GLOBAL_ETHERNET_STATUS_RESERVED2				  (1 << 6)
#define GLOBAL_ETHERNET_STATUS_MODULE_MODBUSEXCEPTION     (1 << 7)
#define GLOBAL_ETHERNET_STATUS_SOCKET_ERROR               (1 << 8)



#define MAXWORDUPDATE           120 //maximum word update per packet
#define VWOUTOFFSET             256  //start of process registers among 512 registers
#define CONFIGCOUNT             4
#define WAGOTCPWATCHDOGTIMER    200  //200 msec watchdog timer for the WAGO tcp/ip communication
#define WAGOTCPWATCHDOGADDRESS  4144

#define WAGO_FIELDBUS_IO          1
#define AUTOMATION_DERECT_IO      2
#define DVT_SENSOR                3
#define COGNEX                    4
#define STDMODBUS                 5
#define FLATADDRESSING            7 //Uses InetGlobalxxxStart to also determine offset over modbus

//offset into the index for differnt
//category of I/O

//#define DOUTOFFSET            0            //output bit offset
//#define DOUTSTATOFFSET       32            //status of output bits offset
//#define DINOFFSET            64            //discrete input offset
//#define WINOFFSET            96            //word input offset
//#define VWINOFFSET          352            //virtual word input offset
//#define WOUTOFFSET          608            //output word offset
//#define VWOUTOFFSET         864            //status of word output offset
//#define WOUTSTATOFFSET      1120            //virtual output word offset

//#define IOCOUNT      1376  //total number of WORDS
                           //allocated for IO (discrete I/O only uses
                           //lower half of the 16 bit words)

//#define CONFIGOFFSET IOCOUNT               //where config  info is stored

typedef struct
{
   HANDLE hStartEvent;
   HANDLE hEnetCloseComEvent;
   HANDLE hEndEvent;
} ENET_EVENT_DATA;

typedef struct tagENETErrorData
{
   int GenericError;                         //should be able to handle -1
   int GlobalEthernetUpdateState ;       //aerotech specific error
   DWORD GlobalEthernetStatus;           //status

} ENETErrorData;

typedef ENETErrorData *PENETErrorData;

#define AER_SHARED_MEMORY_ENET_NAME "_ENET_SHAREDMEM_"

typedef struct tagENETCOMMUNSPACE
{
   VIRTIO_DATA		GlobalIOTable;
   double			dGLOBPARM_InetGlobalIOIPAddress;
   double			dGLOBPARM_InetGlobalIOSubnetMask;
   double			dGLOBPARM_InetGlobalIOGateway;
   double			dGLOBPARM_InetGlobalDefNumInputWords;
   double			dGLOBPARM_InetGlobalDefNumOutputWords;
   double			dGLOBPARM_InetGlobalDefNumInputBits;
   double			dGLOBPARM_InetGlobalDefNumOutputBits;
   double			dGLOBPARM_InetGlobalDefNumInputProcess;
   double			dGLOBPARM_InetGlobalDefNumOutputProcess;
   double			dGLOBPARM_InetGlobalConfigFlags;
   double			dGLOBPARM_InetGlobalInputBitsStart;
   double			dGLOBPARM_InetGlobalOutputBitsStart;
   double			dGLOBPARM_InetGlobalOutputBitsStatusStart;
   double			dGLOBPARM_InetGlobalInputWordsStart;
   double			dGLOBPARM_InetGlobalOutputWordsStart;
   double			dGLOBPARM_InetGlobalOutputWordsStatusStart;
   double			dGLOBPARM_InetGlobalInputProcessStart;
   double			dGLOBPARM_InetGlobalOutputProcessStart;
   int				iFlagRtEnetProcessInitDone;
   int				iFlagEnetIONumParamChanged;
   int				iFlagEnetIOIPParamChanged;
   ENET_EVENT_DATA	hEnetEventData;
   ENETErrorData    ENETErrorDat;

} ENETCOMMUNSPACE ;

typedef ENETCOMMUNSPACE *PENETCOMMUNSPACE;


// Safe versions of the min() & max() macros for use on re-entrant code
// Ensures that any function arguments aren't called twice
#define minw(a, b)\
 (WORD)(a < b ? a : b )

#define maxw(a, b)\
 (WORD)(a > b ? a : b )

#define mini(a, b)\
 (int)((int)a < (int)b ? (int)a : (int)b)


#define maxi(a, b)\
 (int)((int)a > (int)b ? (int)a : (int)b)

/* byte swap a 16 bit word*/
#define swapw(w)\
   (WORD)((((int)w << 8) & 0xff00) | (((WORD)w >> 8) & 0x00ff))

/* byte swap a 32 bit word*/
#define swapl(lw)\
   (LWORD)((((LWORD) lw << 24) & 0xff000000L) | (((LWORD) lw <<8 ) & 0x00ff0000L) |\
   (((LWORD) lw >>8  ) & 0x0000ff00L) | (((LWORD) lw>>24) & 0x000000ffL))

//////////////////////////////////////////////
/////////Data structures /////////////////////

//typedef struct modhdr {
//	WORD     TransactionID,
//	         ProtocolID,
//				LengthField;
//	BYTE		UnitID,
//				FunctionCode ;
//}  MODTCPHDR ;


typedef struct modhdr {
   BYTE   TransactionIDHigh ,
          TransactionIDLow ,
          ProtocolIDHigh ,
          ProtocolIDLow ,
          LengthFieldHigh,
          LengthFieldLow,
          UnitID,
          FunctionCode ;
}  MODTCPHDR ;



typedef struct modpkt
{
   MODTCPHDR  m ;         // modbus header
   BYTE  data[512];       // Data area
}  MODPKT ;


typedef enum {
   INIT_NOT_DONE,       //MMI reset isn't done
   NOT_CONNECTED,       //Not conneted with remote device
   SET_WATCHDOG_TIMER,  //Set the watchdog timer on the WAGO
   INQUIRE_NUM_IO,      //Inquire about number of I/O to update
   UPDATE_DIG_INPUTS,   //Send packet to update digital inputs
   UPDATE_DIG_OUTPUTS,  //Send packet to update digital inputs
   UPDATE_DIG_OUTPUTS_STATUS, //Send packet to update digital inputs
   UPDATE_REG_INPUTS,         //Send packet to update digital inputs
   UPDATE_REG_OUTPUTS,        //Send packet to update digital inputs
   UPDATE_REG_OUTPUTS_STATUS, //Send packet to update digital inputs
   UPDATE_REG_PROCESS_INPUTS, //Send packet to update digital inputs
   UPDATE_REG_PROCESS_OUTPUTS //Send packet to update digital inputs

}  ENET_STATES;
#pragma pack(pop,enter_includeAerrtenet)

#endif
