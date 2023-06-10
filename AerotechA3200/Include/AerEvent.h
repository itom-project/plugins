/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_EVNT_H__
#define __AER_EVNT_H__

typedef struct tagAERDRV_EVENT
{
   HANDLE   hDrvEvent;     // Handle from OpenVxDHandle(Win95)
                           // IoCreateSynchronizationEvent(WinNT)
   PVOID    pkEvent;       // Not Used (Win95)
                           // PKEVENT (WinNT)
   DWORD    dwOpenCount;   // Number of times event has been set
} AERDRV_EVENT;
typedef AERDRV_EVENT *PAERDRV_EVENT;

#define DRV_EVENTTABLE_UNKNOWN_EVENT      0
#define DRV_EVENTTABLE_VIRTIO_UPDATE      1
#define DRV_EVENTTABLE_IRQ_TIMER          2
#define DRV_EVENTTABLE_TASK_FAULT_1       3
#define DRV_EVENTTABLE_TASK_FAULT_2       4
#define DRV_EVENTTABLE_TASK_FAULT_3       5
#define DRV_EVENTTABLE_TASK_FAULT_4       6
#define DRV_EVENTTABLE_TASK_CALLBACK_1    7
#define DRV_EVENTTABLE_TASK_CALLBACK_2    8
#define DRV_EVENTTABLE_TASK_CALLBACK_3    9
#define DRV_EVENTTABLE_TASK_CALLBACK_4    10
#define DRV_EVENTTABLE_SERIAL_1           11
#define DRV_EVENTTABLE_SERIAL_2           12
#define DRV_EVENTTABLE_SERIAL_3           13
#define DRV_EVENTTABLE_SERIAL_4           14
#define DRV_EVENTTABLE_AXIS_FAULT_1       15
#define DRV_EVENTTABLE_AXIS_FAULT_2       16
#define DRV_EVENTTABLE_AXIS_FAULT_3       17
#define DRV_EVENTTABLE_AXIS_FAULT_4       18
#define DRV_EVENTTABLE_AXIS_FAULT_5       19
#define DRV_EVENTTABLE_AXIS_FAULT_6       20
#define DRV_EVENTTABLE_AXIS_FAULT_7       21
#define DRV_EVENTTABLE_AXIS_FAULT_8       22
#define DRV_EVENTTABLE_AXIS_FAULT_9       23
#define DRV_EVENTTABLE_AXIS_FAULT_10      24
#define DRV_EVENTTABLE_AXIS_FAULT_11      25
#define DRV_EVENTTABLE_AXIS_FAULT_12      26
#define DRV_EVENTTABLE_AXIS_FAULT_13      27
#define DRV_EVENTTABLE_AXIS_FAULT_14      28
#define DRV_EVENTTABLE_AXIS_FAULT_15      29
#define DRV_EVENTTABLE_AXIS_FAULT_16      30
#define DRV_EVENTTABLE_AXIS_FAULT_17      31
#define DRV_EVENTTABLE_AXIS_FAULT_18      32
#define DRV_EVENTTABLE_AXIS_FAULT_19      33
#define DRV_EVENTTABLE_AXIS_FAULT_20      34
#define DRV_EVENTTABLE_AXIS_FAULT_21      35
#define DRV_EVENTTABLE_AXIS_FAULT_22      36
#define DRV_EVENTTABLE_AXIS_FAULT_23      37
#define DRV_EVENTTABLE_AXIS_FAULT_24      38
#define DRV_EVENTTABLE_AXIS_FAULT_25      39
#define DRV_EVENTTABLE_AXIS_FAULT_26      40
#define DRV_EVENTTABLE_AXIS_FAULT_27      41
#define DRV_EVENTTABLE_AXIS_FAULT_28      42
#define DRV_EVENTTABLE_AXIS_FAULT_29      43
#define DRV_EVENTTABLE_AXIS_FAULT_30      44
#define DRV_EVENTTABLE_AXIS_FAULT_31      45
#define DRV_EVENTTABLE_AXIS_FAULT_32      46
#define DRV_EVENTTABLE_JOYSTICK_1         47
#define DRV_EVENTTABLE_JOYSTICK_2         48
#define DRV_EVENTTABLE_JOYSTICK_3         49
#define DRV_EVENTTABLE_JOYSTICK_4         50
#define DRV_EVENTTABLE_DRV_TIMEOUT        51
#define DRV_EVENTTABLE_END_G1_ACCEL_1     52
#define DRV_EVENTTABLE_END_G1_ACCEL_2     53
#define DRV_EVENTTABLE_END_G1_ACCEL_3     54
#define DRV_EVENTTABLE_END_G1_ACCEL_4     55
#define DRV_EVENTTABLE_START_G1_DECEL_1   56
#define DRV_EVENTTABLE_START_G1_DECEL_2   57
#define DRV_EVENTTABLE_START_G1_DECEL_3   58
#define DRV_EVENTTABLE_START_G1_DECEL_4   59
#define DRV_EVENTTABLE_ENET_WATCHDOG_TRIPPED 60
#define DRV_EVENTTABLE_MAX                   61

#define EVENT_OPCODE_CREATE_EX   20
#define EVENT_OPCODE_WAIT_EX     21
#define EVENT_OPCODE_CLOSE_EX    22

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerEventCreateEvent( HAERCTRL hAerCtrl, DWORD dwEvent,
                                              DWORD dwNum, LPTSTR psEventName,
                                              PHANDLE phEvent );
AERERR_CODE AER_DLLENTRY AerEventCloseEvent( HAERCTRL hAerCtrl, HANDLE hEvent,
                                             DWORD dwEvent, DWORD dwNum );
//AERERR_CODE AER_DLLENTRY AerEventTest( HAERCTRL hAerCtrl, DWORD dwEvent,
//                                       DWORD dwNum, DWORD dwWaitMSec );
//AERERR_CODE AER_DLLENTRY AerEventGenerateInt( HAERCTRL hAerCtrl, DWORD dwEvent,
//                                              DWORD dwNum );

#ifdef __cplusplus
}
#endif

#endif
// __AER_EVNT_H__
