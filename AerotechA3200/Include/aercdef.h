#ifndef __AERCDEF_H__
#define __AERCDEF_H__
//
// NOTE: NPAQ has 4 analog inputs, first two show up axis 1's analog ins, second two show up as axis 2's analog ins
#define ANALOGS_IN_EACH_DRIVE    2  // (16 bit) in,
#define ANALOGS_OUT_EACH_DRIVE  10  // (16 bit) out

#define DIG_IN_BITS_EACH_DRIVE_ONBOARD  16  // 12 on for NDriveHP RevA (last 2 are "user interrupt" or "high speed") (8 on Rev- NDriveHP, (no high speeds) 16 on NServo
#define DIG_OUT_BITS_EACH_DRIVE_ONBOARD 16  // 8 on Rev- NDriveHP, 12 on RevA NDriveHP, 14 on Rev- NPaq, 16 on NServo
#define DIG_IN_BITS_EACH_DRIVE  32     // NDriveCP has an extra 8in 8out on the expansio I/O board
#define DIG_OUT_BITS_EACH_DRIVE 32

// C#   CLASS=MinimumMaximum      REGION=EthernetIO
#define ETHER_BIN_EACH_DRIVE 256    // for both in and out
#define ETHER_REG_EACH_DRIVE 256    // for both in and out
#define ETHER_PRC_EACH_DRIVE 256    // for both in and out
#define ETHER_CFG_EACH_DRIVE 4      // input only
// C# END

#define ETHER_BIN_EACH_DRIVE_DWORDS ETHER_BIN_EACH_DRIVE/32

#define NPAQ_AMPS_ID_ADDRESS 0x00009EB8    // here is where lowerr 8 bits indicating number of amps actuaslly in the NPaq rack are

#define WAIT_ON_RESET_FOR_DRIVE_DISABLE_10MSEC 10

/***********************************************************************

   Command codes
  (see U3200\SoftwareOnly\Specifications\Command Interface.doc)
   NOTE: CHANGES HERE MUST BE REFLECTED IN:
         FIRMWARE/SHAREMEM/AERCMND.C
         FIRMWARE/SHAREMEM/AERPEND.C   (only if the command gets data back from the drive)

***********************************************************************/

#define COMMAND_NULL                       0   /* this MUST be 0 */
#define COMMAND_GET_PARM                   1
#define COMMAND_SET_PARM                   2
#define COMMAND_SET_OUTPUT_DIG             3  /* no get, digital outputs always come back in status */
#define COMMAND_SET_OUTPUT_ANA             4  /* no get, analog  outputs always come back in status */
#define COMMAND_BRAKE_CONTROL              5
#define COMMAND_AMP_CONTROL                6
#define COMMAND_SET_CMD_POS                7
#define COMMAND_MOVE_TO_LIMIT              8
#define COMMAND_MOVE_OUT_OF_LIMIT          9
#define COMMAND_FIND_MARKER               10
#define COMMAND_FREERUN                   11
#define COMMAND_INDEX                     12
#define COMMAND_ABORT_MOTION              13
#define COMMAND_FAULT_ACK                 14
#define COMMAND_MSET                      15
#define COMMAND_COMMUTATION_TEST          16
#define COMMAND_SINUSOID_EXCITE           17
#define COMMAND_SIMULATION                18
#define COMMAND_PSO                       19
#define COMMAND_AUTO_FOCUS                20
#define COMMAND_MASTER_MODE               21  /* syncronizes a slave to a master */
#define COMMAND_SET_AUTOCAL_MODE          22
#define COMMAND_SET_RT_DATA_FDBK          23
#define COMMAND_SET_AUX                   24
#define COMMAND_READ_DSP_MEM              25
#define COMMAND_SET_OUTBIT_ETHER          26
#define COMMAND_GET_INBIT_ETHER           27
#define COMMAND_SET_OUTREG_ETHER          28
#define COMMAND_GET_INREG_ETHER           29
#define COMMAND_SET_OUTPRC_ETHER          30
#define COMMAND_GET_INREG_CFG_ETHER       31
#define COMMAND_GET_DRIVE_INFO            32
#define COMMAND_CALIBRATE_I_OFFSETS       33
#define COMMAND_SET_MAC_ADDR_ETHER        34
#define COMMAND_GET_OUTBIT_STAT_ETHER     35
#define COMMAND_GET_OUTREG_STAT_ETHER     36
#define COMMAND_GET_OUTPRC_STAT_ETHER     37
#define COMMAND_GET_INPRC_ETHER           38
#define COMMAND_THROW_AXIS_FAULT          39  /* SMC requests drive throw an axis fault (0 param is ESTOP) */
#define COMMAND_SET_MINIMUM_DEADBAND      40  /* reset drive minimum for the AXISPARM_IGainDTimeUSec parameter */
#define COMMAND_DATA_ACQ                  41  /* data acquisation  */
#define COMMAND_THROW_TASK_FAULT          42  /* SMC requests drive throw a task fault */
#define COMMAND_END_PARAMETER_DOWNLOAD    43  /* all parameters downloaded from PC */
#define COMMAND_WRITE_DSP_MEM             44
#define COMMAND_ERASE_FLASH_PARAMS        45
#define COMMAND_LOOP_TRANSMISSION         46
#define COMMAND_PROBE_SETUP               47
#define COMMAND_PROBE_GET_POSITION        48
#define COMMAND_WRITE_FLASH               49
#define COMMAND_SET_RTD_COLLECT           50
#define COMMAND_ARM_MARKER_LATCH            51
#define COMMAND_RETRIEVE_LATCHED            52
#define COMMAND_RELAY_CONTROL               53
#define COMMAND_SET_OUTPUT_ANA_MULTIPLE   54  /* no get, analog  outputs always come back in status */
#define COMMAND_SET_OUTBIT_ETHER_MULTIPLE 55  /* no get, digital outputs always come back in status */
#define COMMAND_RESET_MXH_CAL_COUNTER     56
#define COMMAND_STREAM_SETUP              57
#define COMMAND_CALIBRATE_ANALOGS         58
#define COMMAND_THROW_SAFEZONE_FAULT      59
#define COMMAND_DIGITAL_LOOP_TRANS        60
#define COMMAND_GET_RTD_COLLECT           61
#define COMMAND_PORT_SET_BYTE             62
#define COMMAND_PORT_GET_BYTE             63
#define COMMAND_PORT_SET_BIT              64
#define COMMAND_PORT_GET_BIT              65
#define COMMAND_PORT_SET_DIRECTION        66
#define COMMAND_JOKER                     67
#define COMMAND_STEPPER_ENC_VERIFY        68
#define COMMAND_MX_SETUP                  69
#define COMMAND_SOFTLIMITMODE             70
#define COMMAND_ANALOG_FDFWD_CONFIG       71
#define COMMAND_HOME_CYCLE_START_END      72
#define COMMAND_LASER_SYNC_CLK            73
#define COMMAND_ENC_SIG_ROUTE             74
#define COMMAND_RESET_PID_INTEGRATERS     75
#define COMMAND_GAIN_SCHEDULE             76
#define COMMAND_SLAVE_CURRENT_SCALE       77
#define COMMAND_SET_CURRENT_CLAMP_EDGE    78
#define COMMAND_LOAD_ICMD_CORRECTION      79
#define COMMAND_NPAQ_LASER_AUTOTRACK      80
#define COMMAND_CHANGE_DUAL_LOOP_STATE    81
#define COMMAND_ARRAY_READ_FAST           82
#define COMMAND_INPOS_OUTPUT_SET          83
#define COMMAND_EXT_CLOCK_FREQ_SET        84
#define COMMAND_DATA_COLLECT_TRIG         85
#define COMMAND_RESET_LOOP_TIMERS         86
#define COMMAND_LAST                      87    /* DO NOT FORGET TO UPDATE ME WHEN ADDING NEW COMMANDS ! */

#define COMMAND_MAX                 0xFFFF  /* (2**16 - 1) */

/* Some parameter commands */
#define COMMAND_READ_DSP_MEM_TYPE_P 0   /* P memory on NDriveHP */
#define COMMAND_READ_DSP_MEM_TYPE_X 1   /* X memory on NDriveHP */
#define COMMAND_READ_DSP_MEM_TYPE_Y 2   /* Y memory on NDriveHP */

/* Interpration of flags parameter (if one used in the commands above) */
#define MPARAMETER_FLAGS_CW_DIRECT     0x00000001  /* direction, 0=CCW          */
#define MPARAMETER_FLAGS_SINUSOID      0x00000002  /* else linear curve         */
#define MPARAMETER_FLAGS_HOMELIM       0x00000004  /* used only in mov to limit */
#define MPARAMETER_FLAGS_CCWLIM        0x00000008  /* used only in mov to limit */
#define MPARAMETER_FLAGS_CWLIM         0x00000010  /* used only in mov to limit */
#define MPARAMETER_FLAGS_GANTRY_INDEX  0x00000020  /* reserved for SMC internal use, drives never see this */

/* PSO command codes */
#define PULSE_SETUP            1
#define OUTPUT_MODE            2
#define WINDOW_RANGE_SETUP     3
#define TRACKING_SETUP         4
#define PRESCALE_SETUP         5
#define DISTANCE_SETUP         6
#define ARRAY_DATA_WRITE       7
#define ARRAY_DATA_READ        8
#define PSO_RESET              9
#define PSO_CONFIGURE         10
#define FIRE_OUTPUT_SELECT    11
#define FIRE_CONTROL_MODE     12
#define WINDOW_ENABLE         13
#define DISTANCE_ENABLE       14
#define TRACKING_ADVANCED_SETUP 15
#define WINDOW_TRIGGER_SETUP    16
#define WINDOW_RESET_SETUP      17
#define WINDOW_SOURCE_SETUP     18
#define WINDOW_LOAD_SETUP       19
#define TRACKING_DIRECTION      20
#define WINDOW_MODE_SETUP       21
#define HALT_INPUT              22
#define HALT_ENABLE             23
#define HALT_LATCH_RETRIEVE     24
#define OUTPUT_COMBINE          25
#define ANALOG_COMMAND          26
/* PSO OUTPUT sub command codes */
#define PSOO_SUBCODE_PULSE          0   // Normal Pulse Firing
#define PSOO_SUBCODE_WINDOW         1   // direct window output to PSO output
#define PSOO_SUBCODE_TOGGLE         2   // toggle PSO output value on pulse events
#define PSOO_SUBCODE_BITMAP         3   // Array data values directly mapped to output value
#define PSOO_SUBCODE_PULSE_BIT_MASK 4   // Array data values mask pulse output events
#define PSOO_SUBCODE_WINDOW_MASK    5   // Window output masks PSO output
#define PSOO_SUBCODE_WINDOW_EVENT   6   // Window masks pulse events
#define PSOO_SUBCODE_PULSE2         8   // Normal Pulse2 Firing
#define PSOO_SUBCODE_WINDOW_EVENT2  9   // Window masks pulse2 events

#define PSOO_SUBCODE_ANALOG_OUTPUT   1
#define PSOO_SUBCODE_ANALOG_TRIGGER  2
#define PSOO_SUBCODE_ANALOG_ARRAY    3
#define PSOO_SUBCODE_ANALOG_OFF      4
#define PSOO_SUBCODE_ANALOG_ON       5

/* PSO OUTPUT edge codes */
#define PSOO_EDGECODE_NONE          0   // No Pulse Output  (default)
#define PSOO_EDGECODE_EXIT          1   // Pulse Output Triggers on exit from Window
#define PSOO_EDGECODE_ENTER         2   // Pulse Output Triggers on enter into Window
#define PSOO_EDGECODE_BOTH          3   // Pulse Output Triggers on both enter into and exit from Window

/* ARRAY read/write stuff */
#define ARRAYWRITES_PER_COMMAND 5
#define ARRAYREADS_PER_COMMAND 3
#define ARRAYREADS_FAST_PER_COMMAND 19

/* DATA ACK functions codes */
#define DATA_ACQ_ENABLE                  1
#define DATA_ACQ_TRIGGER_MODE            2
#define DATA_ACQ_INPUT_SOURCE            3
#define DATA_ACQ_INPUT2_SOURCE           4

/* Locations on drive */
#define DSP_STICKY_BITS_ADDRESS 0x30
#define COMMAND_READ_DSP_MEM_PEAK_ADDRESS 0x2D  /* Address of peak rating in X memory (units=0x7FFFFF/peak) */

/***********************************************************************

   Drive fault definitions
  (see U3200\SoftwareOnly\Specifications\Command Interface.doc)
   NOTE: CHANGES HERE MUST BE REFLECTED IN:
         INCLUDE\AerDrFlt.H
         INI\AERPARAM.PGM
         A32SYS\A32SYS.ODL
***********************************************************************/
// C#   CLASS=Fault      REGION=Drive Fault Masks
#define MDRV_FAULT_POSERR       0x00000001
#define MDRV_FAULT_CURERR       0x00000002
#define MDRV_FAULT_CWHARDLIM    0x00000004
#define MDRV_FAULT_CCWHARDLIM   0x00000008
#define MDRV_FAULT_CWSOFTLIM    0x00000010
#define MDRV_FAULT_CCWSOFTLIM   0x00000020
#define MDRV_FAULT_AMPPOWER     0x00000040
#define MDRV_FAULT_POSFBK       0x00000080
#define MDRV_FAULT_VELFBK       0x00000100
#define MDRV_FAULT_HALLFBK      0x00000200   /* Invalid hall feedback sensor states */
#define MDRV_FAULT_MAXVEL       0x00000400
#define MDRV_FAULT_ESTOP        0x00000800   /* Drive ESTOP (unrelated to software ESTOP) from drive, or from */
#define MDRV_FAULT_MAXVELERR    0x00001000
#define MDRV_FAULT_TASK         0x00002000   /* Task fault ordered by front end () */
#define MDRV_FAULT_PROBE        0x00004000   // Drive Probe Fault
#define MDRV_FAULT_AUXILIARY    0x00008000
#define MDRV_FAULT_SAFEZONE     0x00010000
#define MDRV_FAULT_MOTOR_TEMP   0x00020000
#define MDRV_FAULT_AMP_TEMP     0x00040000
#define MDRV_FAULT_EXTERNAL     0x00080000
#define MDRV_FAULT_COMMUN       0x00100000
#define MDRV_FAULT_MAX          0xFFFFFFFF
// C# END

// C#   CLASS=Fault      REGION=Drive Fault Bits
#define BDRV_FAULT_POSERR        0
#define BDRV_FAULT_CURERR        1
#define BDRV_FAULT_CWHARDLIM     2
#define BDRV_FAULT_CCWHARDLIM    3
#define BDRV_FAULT_CWSOFTLIM     4
#define BDRV_FAULT_CCWSOFTLIM    5
#define BDRV_FAULT_AMPPOWER      6
#define BDRV_FAULT_POSFBK        7
#define BDRV_FAULT_VELFBK        8
#define BDRV_FAULT_HALLFBK       9
#define BDRV_FAULT_MAXVEL       10
#define BDRV_FAULT_ESTOP        11
#define BDRV_FAULT_MAXVELERR    12
#define BDRV_FAULT_TASK         13
#define BDRV_FAULT_PROBE        14
#define BDRV_FAULT_AUXILIARY    15
#define BDRV_FAULT_SAFEZONE     16
#define BDRV_FAULT_MOTOR_TEMP   17
#define BDRV_FAULT_AMP_TEMP     18
#define BDRV_FAULT_EXTERNAL     19
#define BDRV_FAULT_COMMUN       20
// C# END

/***********************************************************************

   Drive status bit definitions
     (see U3200\SoftwareOnly\Specifications\Command Interface.doc)
      Axis status bits defined in AERSTAT.H
     (axis status is internal to controller, not on drive, as opposed to drive status)
   NOTE: CHANGES HERE MUST BE REFLECTED IN:
         FIRMWARE\SHAREMEM\ASWPARM.C   (albGetProgErrorMsg())
         INI\AERPARAM.PGM
         A32SYS\A32SYS.ODL
***********************************************************************/
// C#   CLASS=Status      REGION=Drive Status Masks
#define MDRV_STATUS_DRIVE                 0x00000001
#define MDRV_STATUS_CWLIMIT               0x00000002
#define MDRV_STATUS_CCWLIMIT              0x00000004
#define MDRV_STATUS_HOMELIMIT             0x00000008
#define MDRV_STATUS_MARKER                0x00000010
#define MDRV_STATUS_HALLA                 0x00000020
#define MDRV_STATUS_HALLB                 0x00000040
#define MDRV_STATUS_HALLC                 0x00000080
#define MDRV_STATUS_SINENCODER_ERR        0x00000100
#define MDRV_STATUS_COSENCODER_ERR        0x00000200
#define MDRV_STATUS_ESTOPINPUT            0x00000400
#define MDRV_STATUS_BRAKEOUTPUT           0x00000800
#define MDRV_STATUS_SPARE1                0x00001000
#define MDRV_STATUS_SPARE2                0x00002000
#define MDRV_STATUS_SOFT_START_DISENGAGE  0x00004000
#define MDRV_STATUS_CURRENT_CLAMP         0x00008000
#define MDRV_STATUS_MARKER_LATCH          0x00010000
#define MDRV_STATUS_AUTOFOCUS             0x00200000
#define MDRV_STATUS_FLASH_PROGRAM         0x00400000
#define MDRV_STATUS_PROGRAMMING_MXH       0x00800000
#define MDRV_STATUS_BADDRIVECOMMAND       0x01000000
#define MDRV_STATUS_INPOSITION            0x02000000
#define MDRV_STATUS_MOVEACTIVE            0x04000000
#define MDRV_STATUS_ACCELPHASE            0x08000000
#define MDRV_STATUS_DECELPHASE            0x10000000
#define MDRV_STATUS_ENCODER_CLIP          0x20000000
#define MDRV_STATUS_DUALLOOP              0x40000000
#define MDRV_STATUS_RESERVED              0x80000000     // reserved for drive use only
#define MDRV_STATUS_MAX                   0xFFFFFFFF
// C# END

// C#   CLASS=Status      REGION=Drive Status Bits
#define BDRV_STATUS_DRIVE                 0x00
#define BDRV_STATUS_CWLIMIT               0x01
#define BDRV_STATUS_CCWLIMIT              0x02
#define BDRV_STATUS_HOMELIMIT             0x03
#define BDRV_STATUS_MARKER                0x04
#define BDRV_STATUS_HALLA                 0x05
#define BDRV_STATUS_HALLB                 0x06
#define BDRV_STATUS_HALLC                 0x07
#define BDRV_STATUS_SINENCODER_ERR        0x08
#define BDRV_STATUS_COSENCODER_ERR        0x09
#define BDRV_STATUS_ESTOPINPUT            0x0A
#define BDRV_STATUS_BRAKEOUTPUT           0x0B
#define BDRV_STATUS_SPARE1                0x0C
#define BDRV_STATUS_SPARE2                0x0D
#define BDRV_STATUS_SOFT_START_DISENGAGE  0x0E
#define BDRV_STATUS_CURRENT_CLAMP         0x0F
#define BDRV_STATUS_MARKER_LATCH          0x10
#define BDRV_STATUS_PSOHALT_LATCH         0x12
#define BDRV_STATUS_AUTOFOCUS             0x15
#define BDRV_STATUS_FLASH_PROGRAM         0x16
#define BDRV_STATUS_PROGRAMMING_MXH       0x17
#define BDRV_STATUS_BADDRIVECOMMAND       0x18
#define BDRV_STATUS_INPOSITION            0x19
#define BDRV_STATUS_MOVEACTIVE            0x1A
#define BDRV_STATUS_ACCELPHASE            0x1B
#define BDRV_STATUS_DECELPHASE            0x1C
#define BDRV_STATUS_ENCODER_CLIP          0x1D
#define BDRV_STATUS_DUALLOOP              0x1E
#define BDRV_STATUS_RESERVED              0x1F           // reserved for drive use only
// C# END

// NOTE: these drivestatus bits are "shared" by SMC (it can set them to 1)
#define SHAREDDRIVESTATUSBITS (MDRV_STATUS_ACCELPHASE | MDRV_STATUS_DECELPHASE)

/***********************************************************************

   Configuration Drive Parameter definitions
  (see U3200\SoftwareOnly\Specifications\Command Interface.doc)

***********************************************************************/

#define CFG_MOTOR_AC_BRUSHLESS              0
#define CFG_MOTOR_AC_BRUSHLESS_NO_HALLS     1
#define CFG_MOTOR_DC_BRUSH                  2
#define CFG_MOTOR_STEPPER                   3
#define CFG_MOTOR_CERAMIC                   4
#define CFG_MOTOR_AC_BRUSHLESS_WITH_ENCODER 5
#define CFG_MOTOR_PARALLEL_GALVO            6
#define CFG_MOTOR_MAX                       7

#define CFG_FDBCK_TYPE_NULL               0
#define CFG_FDBCK_TYPE_ENCODER            1
#define CFG_FDBCK_TYPE_MX                 2
#define CFG_FDBCK_TYPE_MXU                3
#define CFG_FDBCK_TYPE_AD                 4
#define CFG_FDBCK_TYPE_ABS_ENCODER        5
#define CFG_FDBCK_TYPE_RD_CONVERTER       6
#define CFG_FDBCK_TYPE_HALL_FEEDBACK      7
#define CFG_FDBCK_TYPE_LASER              8
#define CFG_FDBCK_TYPE_MAX                9

/***********************************************************************

   Optional Data feedback stuff
   (see U3200\SoftwareOnly\Specifications\Command Interface.doc)

***********************************************************************/
//
// NOTE: these codes are to be differentiated from the OPTIONAL_DATA_CODEL_* constants,
// only in that the OPTIONAL_FEEDBACK_* constant are a different numbering scheme than the SMC
// Userinterface used OPTIONAL_DATA_CODEL_* constants. In theory there should be a one-to-one correspondence
// between the two sets.
//
#define OPTIONAL_FEEDBACK_CODE_NONE               0
#define OPTIONAL_FEEDBACK_CODE_POSCMD_1MSEC       1
#define OPTIONAL_FEEDBACK_CODE_POSFBK_1MSEC       2
#define OPTIONAL_FEEDBACK_CODE_ID                 3
#define OPTIONAL_FEEDBACK_CODE_IQ                 4
#define OPTIONAL_FEEDBACK_CODE_CURRENT_FBK        5
#define OPTIONAL_FEEDBACK_CODE_MXH_SIN_COS        6
#define OPTIONAL_FEEDBACK_CODE_AD                 7
#define OPTIONAL_FEEDBACK_CODE_LT_BEFORE          8
#define OPTIONAL_FEEDBACK_CODE_LT_AFTER           9
#define OPTIONAL_FEEDBACK_CODE_ZHEIGHT_DIFF       10
#define OPTIONAL_FEEDBACK_CODE_ZHEIGHT_SUM        11
#define OPTIONAL_FEEDBACK_CODE_ZHEIGHT            12
#define OPTIONAL_FEEDBACK_CODE_ZHEIGHT_LPF        13
#define OPTIONAL_FEEDBACK_CODE_TEST               14
#define OPTIONAL_FEEDBACK_CODE_DIGITALIO          15   // Not implemented
#define OPTIONAL_FEEDBACK_CODE_DSPMEM             16
#define OPTIONAL_FEEDBACK_CODE_DSPMEM64           17
#define OPTIONAL_FEEDBACK_CODE_PSO_STATUS         18
#define OPTIONAL_FEEDBACK_CODE_TIMER              19
#define OPTIONAL_FEEDBACK_CODE_MAX                20

/***********************************************************************

   Asynchronous command stuff
   (see U3200\SoftwareOnly\Specifications\Command Interface.doc)

***********************************************************************/

// C#   CLASS=Drive      REGION=HardwareType
#define DRIVE_HARDWARE_TYPE_INVALID          0x0000
#define DRIVE_HARDWARE_REPEATER              0x0001      // not a real drivetype - reserved for telling frontend
#define DRIVE_HARDWARE_THIRDPARTY            0x0002      // not a real drivetype - reserved for telling frontend
#define DRIVE_HARDWARE_PC                    0x0003      // not a real drivetype - reserved for telling frontend
#define DRIVE_HARDWARE_NDRIVEHL_REV_A        0x0004      // reserved for telling frontend

#define DRIVE_HARDWARE_TYPE_NDRIVEHP_REV_DASH  0x8001
#define DRIVE_HARDWARE_TYPE_NPAQ_REV_DASH      0x8002
#define DRIVE_HARDWARE_TYPE_NDRIVEHP_REV_A     0x8003     // NdriveHL also
#define DRIVE_HARDWARE_TYPE_NDRIVECP_REV_DASH  0x8004
#define DRIVE_HARDWARE_TYPE_NSERVO2_REV_DASH   0x8005
#define DRIVE_HARDWARE_TYPE_NSERVO4_REV_DASH   0x8006
#define DRIVE_HARDWARE_TYPE_NPAQ_REV_A         0x8007
#define DRIVE_HARDWARE_TYPE_NSERVO1_REV_DASH   0x8008
#define DRIVE_HARDWARE_TYPE_NSERVO3_REV_DASH   0x8009
#define DRIVE_HARDWARE_TYPE_NDRIVECP_REV_A     0x800A
#define DRIVE_HARDWARE_TYPE_USER_001           0x800B
#define DRIVE_HARDWARE_TYPE_USER_002           0x800C
#define DRIVE_HARDWARE_TYPE_USER_003           0x800D      // always treated as a repeater
#define DRIVE_HARDWARE_TYPE_NDRIVEMP_REV_DASH  0x800E
#define DRIVE_HARDWARE_TYPE_NSTEP1_REV_DASH    0x800F
#define DRIVE_HARDWARE_TYPE_NSTEP2_REV_DASH    0x8010
#define DRIVE_HARDWARE_TYPE_NSTEP3_REV_DASH    0x8011
#define DRIVE_HARDWARE_TYPE_NSTEP4_REV_DASH    0x8012
#define DRIVE_HARDWARE_TYPE_NDRIVECL           0x8013
#define DRIVE_HARDWARE_TYPE_NDRIVECP_REV_B     0x8014
#define DRIVE_HARDWARE_TYPE_NDRIVEHP_REV_B     0x8015
#define DRIVE_HARDWARE_TYPE_NMARK2             0x8016
#define DRIVE_HARDWARE_TYPE_NMARK3             0x8017
#define DRIVE_HARDWARE_TYPE_NMARK5             0x8018
#define DRIVE_HARDWARE_TYPE_NMARK6             0x8019

#define DRIVE_BOOT_CONFIG_WORD_ADDRESS 0xF0000400
#define DRIVE_Unit_Specifier_ID        0xF0000448
// C# END

//
// NdriveHP/NdriveHL related Asyncronous FireWire Command Defines
//
#define NDRVCMD_NO_COMMAND      0
#define NDRVCMD_LOAD_P_SPACE    1
#define NDRVCMD_LOAD_Y_SPACE    2
#define NDRVCMD_EXECUTE_EXPORTED_STORE_THE_SYSTEM  3
#define NDRVCMD_LOAD_Y_BLOCK    4
#define NDRVCMD_START_BOOT_LOAD 5            // must = NPAQCMD_RELEASE
#define NDRVCMD_GET_BOARD_INFO  6            // dip switch setting
//#define NDRVCMD_STARTING_AXIS   9            // not availible on Rev - Ndrives
#define NDRVCMD_GET_FLASH_INFO  7
#define NDRVCMD_GET_BOOT_REV    8


//
// NDriveHP FLASH memory locations for FPGA Version and Firmware Version
//

#define NDRVFLASH_LOC_FPGA_VERSION  0x5d0001
#define NDRVFLASH_LOC_FIRWARE_VERSION  0x510009


//
// Defines for Asyncronous FireWire Commands
// NOTE: Shared by NPaq, NServo, NdriveCP, NStep and NdriveMP.
//       Each drive should ignore the commands it does not support.
//       All except ASYNCFWCMD_NO_COMMAND are one dword data xmits (tCode=0)
//
// Added By Emrah Diril (05/12/2005)
//
#define ASYNCFWCMD_NO_COMMAND         0            // not a NULL command. Meaning of this depends on context of what drive is doing
#define ASYNCFWCMD_UPLOAD             1
#define	ASYNCFWCMD_BURN				  2
#define ASYNCFWCMD_RELEASE            5            // must = NDRVCMD_START_BOOT_LOAD
#define ASYNCFWCMD_BOOTCODEVERSION    6
#define ASYNCFWCMD_BOARDREVISION      7
#define ASYNCFWCMD_FPGAVERSION        8
#define ASYNCFWCMD_STARTING_AXIS      9
#define ASYNCFWCMD_FEATURES          10
#define ASYNCFWCMD_FIRMWAREVERSION   11
#define ASYNCFWCMD_CPLDVERSION       12
#define ASYNCFWCMD_FWBASEADDX_FLASH  13
#define ASYNCFWCMD_RELEASE_BE_MASTER 14


#define NSERVO_MAX_PORTS 6

#endif
