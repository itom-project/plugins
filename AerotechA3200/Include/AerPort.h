/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_PORT_H__
#define __AER_PORT_H__

// defines for status returned in AerSerialGetReadStatus,
// AerSerialGetWriteStatus, AerPendantGetKey, AerPendantGetAnalog
#define SERIAL_TRANSMIT_ACTIVE      0x02  // transmit buffer not empty
#define SERIAL_PENDANT_ACTIVE       0x04  // serial pendant is active
#define SERIAL_UPDATE_PENDANT_LINE1 0x20  // pending update of line 1 of pendant
#define SERIAL_UPDATE_PENDANT_LINE2 0x40  // pending update of line 2 of pendant
#define SERIAL_UPDATE_PENDANT_LINE3 0x80  // pending update of line 3 of pendant
#define SERIAL_UPDATE_PENDANT_LINE4 0x100 // pending update of line 4 of pendant
#define SERIAL_PENDANT_TIMEOUT      0x200 // serial pendant communication failure

// defines for status returned in AerMouseGetStatus
#define MOUSE_PRESENT               0x01  // mouse active
#define TBALL_PRESENT               0x02  // trackball active
#define BUTTON_PRESS                0x04  // indicates mouse/tball button status change
#define MOUSE_ENABLED               0x08  // mouse/tball activated by user
#define X_ENABLED                   0x10  // set if x axis enabled by user
#define Y_ENABLED                   0x20  // set if y axis enabled by user
#define Z_ENABLED                   0x40  // set if z axis enabled by user

#define AER_BUTTON_RIGHT            0x01  // Status of right button
#define AER_BUTTON_LEFT             0x02  // Status of left button
#define AER_BUTTON_MIDDLE           0x04  // Status of middle buttton

#define MAX_SERIAL_TRANS_LEN  256
typedef struct tagAER_SERIAL_WRITE_PACKET
{
   WORD  wChannel;
   WORD  wCount;
   BYTE  byData[MAX_SERIAL_TRANS_LEN];
} AER_SERIAL_WRITE_PACKET;
typedef AER_SERIAL_WRITE_PACKET  *PAER_SERIAL_WRITE_PACKET;

typedef struct tagAER_SERIAL_READ_SEND_PACKET
{
   WORD  wChannel;
   WORD  wReqLen;
} AER_SERIAL_READ_SEND_PACKET;
typedef AER_SERIAL_READ_SEND_PACKET *PAER_SERIAL_READ_SEND_PACKET;

typedef struct tagAER_SERIAL_READ_RECV_PACKET
{
   WORD  wRecvLen;
   BYTE  byData[MAX_SERIAL_TRANS_LEN];
} AER_SERIAL_READ_RECV_PACKET;
typedef AER_SERIAL_READ_RECV_PACKET  *PAER_SERIAL_READ_RECV_PACKET;

typedef struct tagAER_SERIAL_STATUS_PACKET
{
   WORD  wStatus;
   WORD  wByteCount;
} AER_SERIAL_STATUS_PACKET;
typedef AER_SERIAL_STATUS_PACKET *PAER_SERIAL_STATUS_PACKET;

typedef struct tagAER_PENDANT_SETMODE_PACKET
{
   WORD  wChannel;
   WORD  wMode;
} AER_PENDANT_SETMODE_PACKET;

typedef struct tagAER_PENDANT_KEY_PACKET
{
   WORD  wSerialStatus;
   WORD  wPendantStatus;
   WORD  wKey;
} AER_PENDANT_KEY_PACKET;
typedef AER_PENDANT_KEY_PACKET   *PAER_PENDANT_KEY_PACKET;

#define MAX_PENDANT_ANALOG 4
typedef struct tagAER_PENDANT_ANALOG_PACKET
{
    WORD wSerialStatus;
    WORD wPendantStatus;
    WORD waAnalog[MAX_PENDANT_ANALOG];
} AER_PENDANT_ANALOG_PACKET;
typedef AER_PENDANT_ANALOG_PACKET   *PAER_PENDANT_ANALOG_PACKET;

typedef struct tagAER_PENDANT_JOG_PACKET
{
   WORD  wMode;
   DWORD dwSpeed;
   WORD  wPort;
   WORD  wChannel;
} AER_PENDANT_JOG_PACKET;
typedef AER_PENDANT_JOG_PACKET   *PAER_PENDANT_JOG_PACKET;

#define MAX_PENDANT_LINES  4
#define MAX_PENDANT_TEXT_LEN  20
typedef struct tagAER_PENDANT_TEXT_PACKET
{
    WORD wPort;
    WORD wLine;
    WORD wLen;
    char szText[MAX_PENDANT_TEXT_LEN + 1];
} AER_PENDANT_TEXT_PACKET;

typedef struct tagAER_PENDANT_DEADBAND_PACKET
{
   WORD  wPort;
   WORD  wDeadband;
} AER_PENDANT_DEADBAND_PACKET;

typedef struct tagAER_PENDANT_LED_PACKET
{
   WORD  wPort;
   WORD  wLEDs;
} AER_PENDANT_LED_PACKET;

typedef struct tagAER_MOUSE_STATUS_PACKET
{
   WORD  wStatus;
   WORD  wKey;
} AER_MOUSE_STATUS_PACKET;
typedef AER_MOUSE_STATUS_PACKET  *PAER_MOUSE_STATUS_PACKET;

typedef struct tagAER_MOUSE_POS_PACKET
{
   WORD  wStatus;
   DWORD dwPos;
} AER_MOUSE_POS_PACKET;
typedef AER_MOUSE_POS_PACKET  *PAER_MOUSE_POS_PACKET;

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerSerialWrite( HAERCTRL hAerCtrl, WORD wChannel,
                                   WORD wCount, PVOID pvData );
AERERR_CODE AER_DLLENTRY AerSerialRead( HAERCTRL hAerCtrl, WORD wChannel, WORD wReqLen,
                                  PAER_SERIAL_READ_RECV_PACKET pRecv );
AERERR_CODE AER_DLLENTRY AerSerialGetWriteStatusPacket( HAERCTRL hAerCtrl,
                                                  WORD wChannel,
                                                  PAER_SERIAL_STATUS_PACKET pStatus );
AERERR_CODE AER_DLLENTRY AerSerialGetWriteStatus( HAERCTRL hAerCtrl, WORD wChannel,
                                            PWORD pwWriteStatus, PWORD pwByteCount );
AERERR_CODE AER_DLLENTRY AerSerialGetReadStatusPacket( HAERCTRL hAerCtrl, WORD wChannel,
                                                 PAER_SERIAL_STATUS_PACKET pStatus );
AERERR_CODE AER_DLLENTRY AerSerialGetReadStatus( HAERCTRL hAerCtrl, WORD wChannel,
                                                 PWORD pwReadStatus, PWORD pwByteCount );

AERERR_CODE AER_DLLENTRY AerPendantSetMode( HAERCTRL hAerCtrl, WORD wPort, WORD wMode );
AERERR_CODE AER_DLLENTRY AerPendantGetMode( HAERCTRL hAerCtrl, WORD wPort, PWORD pwMode );
AERERR_CODE AER_DLLENTRY AerPendantGetKeyPacket( HAERCTRL hAerCtrl, WORD wPort,
                                                 PAER_PENDANT_KEY_PACKET pKey );
AERERR_CODE AER_DLLENTRY AerPendantGetKey( HAERCTRL hAerCtrl, WORD wPort,
                                           PWORD pwSerialStatus,
                                           PWORD pwPendantStatus, PWORD pwKey );
AERERR_CODE AER_DLLENTRY AerPendantGetAnalogPacket( HAERCTRL hAerCtrl, WORD wPort,
                                              PAER_PENDANT_ANALOG_PACKET pAnalog );
AERERR_CODE AER_DLLENTRY AerPendantGetAnalog( HAERCTRL hAerCtrl, WORD wPort,
                                              PWORD pwSerialStatus,
                                              PWORD pwPendantStatus,
                                              PWORD pwAnalog );
AERERR_CODE AER_DLLENTRY AerPendantSetJogModePacket( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                                     PAER_PENDANT_JOG_PACKET pJog );
AERERR_CODE AER_DLLENTRY AerPendantSetJogMode( HAERCTRL hAerCtrl,
                                               AXISINDEX iAxis,
                                               WORD wMode, DWORD dwSpeed,
                                               WORD wPort, WORD wChannel );
AERERR_CODE AER_DLLENTRY AerPendantGetJogModePacket( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                                     PAER_PENDANT_JOG_PACKET pJog );
AERERR_CODE AER_DLLENTRY AerPendantGetJogMode( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                               PWORD pwMode, PDWORD pdwSpeed,
                                               PWORD pwPort, PWORD pwChannel );
AERERR_CODE AER_DLLENTRY AerPendantSetText( HAERCTRL hAerCtrl, WORD wPort,
                                            WORD wLine, PSZ pszText );
AERERR_CODE AER_DLLENTRY AerPendantSetDeadband( HAERCTRL hAerCtrl, WORD wPort,
                                                WORD wDeadband );
AERERR_CODE AER_DLLENTRY AerPendantSetLeds( HAERCTRL hAerCtrl,
                                            WORD wPort, WORD wLEDs );

AERERR_CODE AER_DLLENTRY AerMouseGetKey( HAERCTRL hAerCtrl, WORD wChannel, PWORD wKey );
AERERR_CODE AER_DLLENTRY AerMouseGetStatusPacket( HAERCTRL hAerCtrl, WORD wChannel,
                                            PAER_MOUSE_STATUS_PACKET pStatus );
AERERR_CODE AER_DLLENTRY AerMouseGetStatus( HAERCTRL hAerCtrl, WORD wChannel,
                                      PWORD pwStatus, PWORD pwKey );
AERERR_CODE AER_DLLENTRY AerMouseGetPositionPacket( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                              PAER_MOUSE_POS_PACKET pPos );
AERERR_CODE AER_DLLENTRY AerMouseGetPosition( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                        PWORD pwStatus, PDWORD pdwPos );


#ifdef __cplusplus
}
#endif

#endif
// __AER_PORT_H__
