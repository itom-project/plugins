/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_DC_H__
#define __AER_DC_H__


#define DATACENTER_OPCODE_REGISTER     1
#define DATACENTER_OPCODE_SET_RATE     2
#define DATACENTER_OPCODE_GET_VALUE    3  // DataCenter specific
#define DATACENTER_OPCODE_START        4
#define DATACENTER_OPCODE_STOP         5
#define DATACENTER_OPCODE_GET_DATA     6  // Axis or Task specific
#define DATACENTER_OPCODE_SET_EVENT    8
#define DATACENTER_OPCODE_GET_EVENT    9

#define AER_DEF_DATACENTER_RATE  250   // 250 msec

// Mask sent to DATACENTER_OPCODE_UNREGISTER to free all memory
// reguardless of how many times it has been opened
#define AER_DC_UNREGISTER_ALL 0xDEADFFFF

#define AER_DC_GET_RATE       0
#define AER_DC_GET_COUNTER    1
#define AER_DC_GET_TASKMASK   2
#define AER_DC_GET_AXISMASK   3

#define AER_DC_REGISTER_AXIS_MASK   1
#define AER_DC_UNREGISTER_AXIS_MASK 2
#define AER_DC_REGISTER_TASK_MASK   3
#define AER_DC_UNREGISTER_TASK_MASK 4

#define DCF_STARTED  0x0001   // DataCenter has received start command

// Structure is built internally by AerDCGetData and passed on to VxD
typedef struct tagAER_DC_GETDATA_RECV_PACKET
{
   DWORD dwVxDCounter;  // VxD Counter
   PVOID pvUserData;    // Pointer to users data buffer
} AER_DC_GETDATA_RECV_PACKET;
typedef AER_DC_GETDATA_RECV_PACKET  *PAER_DC_GETDATA_RECV_PACKET;

// The DECLARE_DC_GET macros help build defines for the VxD
// the value that is built is setup as follows
//     bits 31-28    Mask of what to get
//          27-16    Sizeof data
//          15-0     Offset into structure

#define DC_GET_MASK        0xF0000000
#define DC_SIZE_MASK       0x0FFF0000
#define DC_OFFSET_MASK     0x0000FFFF

#define DC_GET_AXIS_DATA   1
#define DC_GET_TASK_DATA   2

#define DEF_DC_GET_STRUCT( data, size ) \
   (((data << 28) & DC_GET_MASK) + ((size << 16) & DC_SIZE_MASK))

#define DEF_DC_GET( data, struct, element, size )  \
   (((data << 28) & DC_GET_MASK) + ((size << 16) & DC_SIZE_MASK) + OFFSETOF( struct, element ))
#define DEF_DC_GET_AXIS( element, size )   \
   DEF_DC_GET( DC_GET_AXIS_DATA, AER_AXIS_DATA, element, size )
#define DEF_DC_GET_TASK( element, size )   \
   DEF_DC_GET( DC_GET_TASK_DATA, AER_TASK_DATA, element, size )

// prevents "name decoration" ("name mangling") of functions by C++
#ifdef __cplusplus
extern "C" {
#endif

#define AERDCCOMMAND AerDataCenterCommand


AERERR_CODE AER_DLLENTRY AerDCGetAxisDirectD( HAERCTRL hAerCtrl, AXISMASK mAxis, PAER_AXIS_DATAD pDataD );
AERERR_CODE AER_DLLENTRY AerDCGetTaskDirect( HAERCTRL hAerCtrl, TASKMASK mTask, PAER_TASK_DATA pData );

#define AerDCRegister( hAerCtrl, dwValue, mAxis ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_REGISTER, dwValue, mAxis, NULL, 0 )
#define AerDCRegisterAxis( hAerCtrl, mAxis ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_REGISTER, AER_DC_REGISTER_AXIS_MASK, mAxis, NULL, 0 )
#define AerDCRegisterTask( hAerCtrl, dwTaskMask ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_REGISTER, AER_DC_REGISTER_TASK_MASK, dwTaskMask, NULL, 0 )
#define AerDCUnregisterAxis( hAerCtrl, mAxis ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_REGISTER, AER_DC_UNREGISTER_AXIS_MASK, mAxis, NULL, 0 )
#define AerDCUnregisterTask( hAerCtrl, dwTaskMask ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_REGISTER, AER_DC_UNREGISTER_TASK_MASK, dwTaskMask, NULL, 0 )
#define AerDCSetRate( hAerCtrl, dwRate ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_SET_RATE, dwRate, 0, NULL, 0 )
#define AerDCGetValue( hAerCtrl, dwValue, pValue, dwSize ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_GET_VALUE, dwValue, 0, pValue, dwSize )
#define AerDCGetRate( hAerCtrl, pdwRate ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_GET_VALUE, AER_DC_GET_RATE, 0, pdwRate, sizeof(DWORD) )
#define AerDCGetCounter( hAerCtrl, pdwCounter ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_GET_VALUE, AER_DC_GET_COUNTER, 0, pdwCounter, sizeof(DWORD) )
#define AerDCGetTaskMask( hAerCtrl, pdwTaskMask ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_GET_VALUE, AER_DC_GET_TASKMASK, 0, pdwTaskMask, sizeof(DWORD) )
#define AerDCGetAxisMask( hAerCtrl, pmAxis ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_GET_VALUE, AER_DC_GET_AXISMASK, 0, pmAxis, sizeof(DWORD) )
#define AerDCStart( hAerCtrl ) \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_START, 0, 0, NULL, 0 )
#define AerDCStop( hAerCtrl )  \
   AERDCCOMMAND( hAerCtrl, DATACENTER_OPCODE_STOP, 0, 0, NULL, 0 )

AERERR_CODE AER_DLLENTRY AerDCGetData( HAERCTRL hAerCtrl, DWORD dwValue,
                                       DWORD dwMask, PDWORD pdwVxDCounter,
                                       PVOID pvData, DWORD dwSize );

#ifdef __cplusplus
}
#endif

#endif
// __AER_DC_H__
