#ifndef __AERC32_H__
#define __AERC32_H__

#include "AerCDef.H"

/**********************************************************************************

  Communication structure definition
  (see U3200\SoftwareOnly\Specifications\Command Interface.doc)
   NOTE: Each entry contains data for all MAX_AXES axes

**********************************************************************************/

#define MCOMMAND_FLAGMASK_CVALID     0x1    /* if and only if this bit on in Cmd.usCommandFlags[0], lPositionCmd/lVelocityCmd sent to drive are valid */
//#define MCOMMAND_FLAGMASK_CINVALID   0xFFFE
#define MCOMMAND_FLAGMASK_TASK_ORIG  0xF00  /* Mask for Originating task bits (reserved for SMC use only) */
#define MCOMMAND_NUMBER_PARAMS_PER_COMMAND 7

typedef struct tagCOMMAND_SINGLE
{
   USHORT            usCommandCode;
   USHORT            usCommandFlags;
   /* Other command arguments (usage depends on command) */
   DWORD             dwCmdParameter0;    /* MUST BE FIRST COMMAND RELATED THINGEE IN STRUCTURE */
   DWORD             dwCmdParameter1;
   DWORD             dwCmdParameter2;
   DWORD             dwCmdParameter3;
   DWORD             dwCmdParameter4;
   DWORD             dwCmdParameter5;
   DWORD             dwCmdParameter6;
} COMMAND_SINGLE, *PCOMMAND_SINGLE;

#define MAX_SINGLE_COMMANDS 2

//#define PROFILE_DATA_SIZE_BYTES 16  // 4 4-byte words
//#define COMMAND_DATA_SIZE_BYTES 32  // 8 4-byte words
#define BY1394XMIT_ALREADY_SENT     (BYTE)99    // Xmit record was sent down firewire
#define BY1394XMIT_READY_TO_SEND    (BYTE)0     // Xmit record created, but not yet sent down firewire
#define BY1394XMIT_AXIS_DEAD        (BYTE)1     // Xmit record was never created by GenerateDriveCommand() (but a null record was still sent)

typedef struct tagCOMMAND_ONE_AXIS
{

   /* Profile point command arguments */
   LONG              lPositionCmd;       /* signed, (cnts)   MUST BE FIRST ONE IN STRUCTURE */
   LONG              lVelocityCmd;       /* signed, (cnts/msec) */
   LONG              lAccelerationCmd;   /* signed, (cnts/msec/msec) */
   LONG              lCalibration;       /* calibration value */
   COMMAND_SINGLE    Cmd[MAX_SINGLE_COMMANDS];
   DWORD             dwAnalogStreamData; /* values to set the the 16bit analog outputs to (see command: COMMAND_STREAM_SETUP=57) */
   BYTE              byAxisNumber;       /* zero based */
   BYTE              by1394XmitDirtyBit; /* a BY1394XMIT* constant */
   BYTE              bySpare[2];
   DWORD             dwSpare[2];

} COMMAND_ONE_AXIS, *PCOMMAND_ONE_AXIS;


#define STATUS_RETURN_DATA_SIZE_WORDS 8
#define NUMBER_OPTIONAL_FEEDBACK_ITEMS 16
typedef struct tagSTATUS_ONE_AXIS
{
   DWORD             lPositionCmdMSW;    /* signed, (cnts) */
   DWORD             lPositionCmdLSW;    /* unsigned, (cnts) */
   DWORD             lPositionFbkMSW;    /* signed, (cnts) */
   DWORD             lPositionFbkLSW;    /* unsigned, (cnts) */
   DWORD             lPositionFbkExtMSW; /* signed, (cnts) */
   DWORD             lPositionFbkExtLSW; /* unsigned, (cnts) */

   DWORD             dwFault;
   DWORD             dwStatus;
   WORD              wAnalogIn[ANALOGS_IN_EACH_DRIVE];
   WORD              wDigitalOut;
   WORD              wDigitalIn;
   WORD              wTorqueFbk;
   WORD              wTorqueCmd;
   BYTE              bEtherBinInAddr;
   BYTE              bEtherBinOutAddr;
   WORD              wExpandedIn;
   DWORD             dwEtherBinIn;
   DWORD             dwEtherBinOut;
   DWORD             dwOptionalFeedBack[NUMBER_OPTIONAL_FEEDBACK_ITEMS];
   WORD              wCmdReturnCode1;                              /* Response content code */
   WORD              wExpandedOut;
   WORD              wCmdReturn1[STATUS_RETURN_DATA_SIZE_WORDS];   /* Response data from commands */
   WORD              wCmdReturnCode2;                              /* Response content code (second) */
   WORD              wSpare2;
   WORD              wCmdReturn2[STATUS_RETURN_DATA_SIZE_WORDS];   /* Response data from commands */
} STATUS_ONE_AXIS, *PSTATUS_ONE_AXIS;

#define STATUS_TOTAL_SIZE_DWORDS 40
typedef struct tagSTATUS_ONE_AXIS2
{
  DWORD              dwStatWord[STATUS_TOTAL_SIZE_DWORDS];
} STATUS_ONE_AXIS2, *PSTATUS_ONE_AXIS2;


#endif
