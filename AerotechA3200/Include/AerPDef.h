/*+++

	Copyright (c) Aerotech Inc., 1996 - 2006

	This header file is an  PUBLIC Aerotech A3200 include file, and is used by
	all (Win apps, device drivers, library, firmware).
	and must also be included by all customer apps (library, SDK, VB).

	NOTE:	A change will usually mean a change in two or three places at once, as data is stored in
			parallel here. For example adding a task parameter means a change in both the TASKPARM_
			constants, and a change in the TaskParmInfo data structure. Especially true when changing
			status constants, when you may need to change 4 places or so.

	NOTE:	This file is auto generated.  Changes made to this file will be lost the next time the
			file is generated.

+++*/


#ifndef __AERPDEF_H__
#define __AERPDEF_H__


// C#	CLASS=Parameter		REGION=Parameter Types
/*
   Stores the parameter types contained within this file
*/
#define AER_PARMTYPE_NONE    -1
#define AER_PARMTYPE_AXIS     0            // must be the first type
#define AER_PARMTYPE_TASK     1
#define AER_PARMTYPE_GLOBAL   2
#define AER_PARMTYPE_FIBER    3
#define AER_PARMTYPE_VPP      4
#define AER_PARMTYPE_SETUP    5
#define AER_PARMTYPE_MAX      6
// C# END

// C#	CLASS=MinimumMaximum	REGION=Digital Drive DSP
/*
 Maximum and Minimum values for the Digital Drive DSP parameters
*/
#define DD_24_BIT_MAX                       8388607    //  0x7FFFFF
#define DD_24_BIT_MIN                      -8388607    //  0x800000 - 1
// NOTE: in best of all worlds, 24 bit unsigned max should be 2^24, but NDriveHP limits it to  2^23 (for gains at least)
// (NDriveHP probably handles it internally as a signed 24 bit)
#define DD_24_BIT_UNSGN_MAX                 8388607
#define DD_24_BIT_UNSGN_MIN                       0

#define DD_40_BIT_MAX                  549755813887    //  0x7FFFFFFFFF
#define DD_40_BIT_MIN                 -549755813887    //  0x8000000000 - 1
#define DD_40_BIT_UNSGN_MAX           1099511627776
#define DD_40_BIT_UNSGN_MIN                       0

#define DD_48_BIT_MAX               140737488355327    //  0x7FFFFFFFFFFF
#define DD_48_BIT_MIN              -140737488355327    //  0x800000000000 - 1
// NOTE: in best of all worlds, 48 bit unsigned max should be 2^48, but NDriveHP limits it to  2^47
// (NDriveHP probably handles it internally as a signed 48 bit)
#define DD_48_BIT_UNSGN_MAX         140737488355327    // 281474976710656
#define DD_48_BIT_UNSGN_MIN                       0
// NOTE: NDriveHP shifts certain 48 bit parameters down 8 bits internally, thus, truncating at 47-8=39 bits.
#define DD_39_BIT_UNSGN_MAX            549755813887    //  0x7FFFFFFFFF
#define DD_39_BIT_UNSGN_MIN           -549755813887    //  0x8000000000 - 1
// C# END

/*
  used for RolloverDistanceCnts axis parameter (do not change)
*/
#define ROLLOVER_NONE -1
#define ROLLOVER_360 0
#define ROLLOVER_360_NO_FASTEST -2
/*
   Miscellnous defines used in default/max/min paramater values
*/
#define MAX_INET_DEF 255

#define DD_MINIMUM_8_BIT_RESOLUTION               0.00390625     // 2<<8
#define DD_MINIMUM_24_BIT_RESOLUTION              4.76837158E-7  // 2<<21
#define FILT_MAX                                  (4.0-DD_MINIMUM_24_BIT_RESOLUTION)
//
//#define ANALOG_BIT_NUM_MAX (((ANALOGS_IN_EACH_DRIVE-1)*100)+(MAX_AXES-1))
//#define DRIVE_BIT_NUM_MAX  (((DIG_BITS_IN_EACH_DRIVE-1)*100)+(MAX_AXES-1))
#define FAULT_ESTOP_DEDICATED -1  /* else its a zero based drive input num */
#define FAULT_ESTOP_NONE      -2
/*
    Various IO types, that can be used as inputs (digital/analog) for parameter input coding
    (see AerParmBuildInputNumber())
*/
#define NO_SUCH_IO_BIT      0    // Has to be zero

// C#	CLASS=Parameter		REGION=Input Type
#define INPUT_TYPE_NONE         0
#define INPUT_TYPE_DRVIN        1   // digital
#define INPUT_TYPE_ANAIN        2   // analog
#define INPUT_TYPE_ETHDIN       3   // digital
#define INPUT_TYPE_ETHRIN       4   // analog
#define INPUT_TYPE_ETHPIN       5   // analog
#define INPUT_TYPE_VIRTBIN      6   // digital
#define INPUT_TYPE_VIRTREG      7   // analog
#define INPUT_TYPE_MAX          8
// C# END

//#define OUTPUT_TYPE_NONE         0
//#define OUTPUT_TYPE_DRVOUT       1   // digital
//#define OUTPUT_TYPE_ANAOUT       2   // analog
//#define OUTPUT_TYPE_ETHDOUT      3   // digital
//#define OUTPUT_TYPE_ETHROUT      4   // analog
//#define OUTPUT_TYPE_ETHPOUT      5   // analog
//#define OUTPUT_TYPE_VIRTBOUT     6   // digital
//#define OUTPUT_TYPE_VIRTREG      7   // analog
//#define OUTPUT_TYPE_MAX          8

// C#	CLASS=Parameter		REGION=Gantry Mode
#define GANTRY_MODE_NONE                0
#define GANTRY_MODE_LOOSE               1
#define GANTRY_MODE_CURRENT_SLAVE       2
#define GANTRY_MODE_VOLTAGE_SLAVE       3
#define GANTRY_MODE_MAX                 4
// C# END

// C#	CLASS=Parameter		REGION=Gear Mode
#define GEARMODE_OFF            0
#define GEARMODE_ON             1
#define GEARMODE_ON_FILTERED    2
#define GEARMODE_ON_GANTRY      3
#define GEARMODE_ON_GANTRY_PAUSE     4   // internal use only
#define GEARMODE_MAX            5
// C# END

// C#	CLASS=Parameter		REGION=Safe Mode
// these cannot be changed (they are masks)
#define SAFE_MODE_INTERNAL_USE         0xFFFFFFFF
#define SAFE_MODE_OFF                  0x0
#define SAFE_MODE_NO_EXIT              0x1
#define SAFE_MODE_NO_ENTER             0x2
#define SAFE_MODE_NO_EXIT_STRICT       0x3
#define SAFE_MODE_NO_ENTER_STRICT      0x4
// C# END

// C#	CLASS=Parameter		REGION=SoftLimitMode
// these cannot be changed (they are masks)
#define SOFTLIMIT_MODE_NOT_ACTIVE_UNTIL_HOMED  0x1
#define SOFTLIMIT_MODE_TARGET_SCANNING         0x2
#define SOFTLIMIT_MODE_CLAMPING                0x4
#define SOFTLIMIT_MODE_NO_DRIVE_CHECK          0x8
// C# END


// C#	CLASS=Parameter		REGION=Parameter Attributes
/*
 Parameter Attributes
*/
#define PARM_ATTR_READ           0x00000001     /* can read it */
#define PARM_ATTR_WRITE          0x00000002     /* can write it */
#define PARM_ATTR_UPDATE         0x00000004     /* parm editor ignores these: they are either updated by firmware, or used only by CNC compiler */
/*#define PARM_ATTR_RESERVED     0x00000008    */
#define PARM_ATTR_UNSIGNED       0x00000010
#define PARM_ATTR_NOLIMIT        0x00000020    /* has no minimum or maximum value (min/max values in xxxParmInfo[] structures below are ignored */
#define PARM_ATTR_INTEGER        0x00000040
/*#define PARM_ATTR_RESERVED2    0x00000080    */
#define PARM_ATTR_DRIVE          0x00000100     /* goes down to digital drive */
#define PARM_ATTR_32_BIT         0x00000200
#define PARM_ATTR_40_BIT         0x00000400
#define PARM_ATTR_64_BIT         0x00000800
#define PARM_ATTR_DISTANCE       0x00001000    /* units are distance (NOTE: HAS NO EFFECT ON GLOBAL PARMS) */
#define PARM_ATTR_VELOCITY       0x00002000    /* units are velocity (NOTE: HAS NO EFFECT ON GLOBAL PARMS) */
#define PARM_ATTR_ACCELERATION   0x00004000    /* units are acceleration (NOTE: HAS NO EFFECT ON GLOBAL PARMS) */
#define PARM_ATTR_TASK_ROTARY    0x00008000    /* is task parm, and units are "rotary" (use CntsPerRotaryaUnit) */
// C# END

/*
 Parameter Attribute shortcut codes/macros
*/
#define PARM_ATTR_NU         (PARM_ATTR_NOLIMIT | PARM_ATTR_UPDATE)
#define PARM_ATTR_RI         (PARM_ATTR_READ | PARM_ATTR_INTEGER)
#define PARM_ATTR_RN         (PARM_ATTR_READ | PARM_ATTR_NOLIMIT)
#define PARM_ATTR_RW         (PARM_ATTR_READ | PARM_ATTR_WRITE)
#define PARM_ATTR_RU         (PARM_ATTR_READ | PARM_ATTR_UPDATE)
#define PARM_ATTR_RIU        (PARM_ATTR_READ | PARM_ATTR_INTEGER | PARM_ATTR_UPDATE)
#define PARM_ATTR_RNU        (PARM_ATTR_READ | PARM_ATTR_NOLIMIT | PARM_ATTR_UPDATE)
#define PARM_ATTR_RWS        (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_DISTANCE)
#define PARM_ATTR_RWI        (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_INTEGER)
#define PARM_ATTR_RWN        (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT)
#define PARM_ATTR_RWU        (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_UPDATE)
#define PARM_ATTR_RWSU       (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_DISTANCE | PARM_ATTR_UPDATE)
#define PARM_ATTR_RWIU       (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_INTEGER | PARM_ATTR_UPDATE)
//#define PARM_ATTR_RWNA       (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_ACCELERATION)
#define PARM_ATTR_RWNS       (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_DISTANCE)
#define PARM_ATTR_RWNI       (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_INTEGER)
#define PARM_ATTR_RWNV       (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT |PARM_ATTR_VELOCITY)
#define PARM_ATTR_RWNU       (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_UPDATE)
#define PARM_ATTR_RNIU       (PARM_ATTR_READ | PARM_ATTR_NOLIMIT | PARM_ATTR_INTEGER | PARM_ATTR_UPDATE)
#define PARM_ATTR_RW32D      (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_32_BIT | PARM_ATTR_DRIVE)
#define PARM_ATTR_RWNIU      (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_INTEGER | PARM_ATTR_UPDATE)
#define PARM_ATTR_RWN32D     (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_32_BIT | PARM_ATTR_DRIVE)
#define PARM_ATTR_RWN64D     (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE)
#define PARM_ATTR_RWI64D     (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_INTEGER | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE)
#define PARM_ATTR_RWNI64D    (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_INTEGER | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE )
#define PARM_ATTR_RWN64UD    (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UPDATE)
#define PARM_ATTR_RWNSU      (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_DISTANCE | PARM_ATTR_UPDATE)
#define PARM_ATTR_RWNVU      (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_VELOCITY | PARM_ATTR_UPDATE)
#define PARM_ATTR_RNSU       (PARM_ATTR_READ | PARM_ATTR_NOLIMIT | PARM_ATTR_DISTANCE | PARM_ATTR_UPDATE )
#define PARM_ATTR_RNVU       (PARM_ATTR_READ | PARM_ATTR_NOLIMIT | PARM_ATTR_VELOCITY | PARM_ATTR_UPDATE )

#define PARM_ATTR_RWNVT      (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_VELOCITY | PARM_ATTR_TASK_ROTARY)
#define PARM_ATTR_RWNVUT     (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_VELOCITY | PARM_ATTR_UPDATE | PARM_ATTR_TASK_ROTARY)
#define PARM_ATTR_RWNUT      (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_UPDATE | PARM_ATTR_TASK_ROTARY)
#define PARM_ATTR_RWNT       (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_TASK_ROTARY)
#define PARM_ATTR_RNUVT      (PARM_ATTR_READ | PARM_ATTR_NOLIMIT | PARM_ATTR_UPDATE | PARM_ATTR_VELOCITY | PARM_ATTR_TASK_ROTARY)
//#define PARM_ATTR_RWNAT      (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_ACCELERATION | PARM_ATTR_TASK_ROTARY)
#define PARM_ATTR_RNUT       (PARM_ATTR_READ | PARM_ATTR_NOLIMIT | PARM_ATTR_UPDATE | PARM_ATTR_TASK_ROTARY)

#define PARM_ATTR_RI_US      (PARM_ATTR_READ | PARM_ATTR_INTEGER | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RW_US      (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RIU_US     (PARM_ATTR_READ | PARM_ATTR_INTEGER | PARM_ATTR_UPDATE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWV_US     (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_VELOCITY | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWA_US     (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_ACCELERATION | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWI_US     (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_INTEGER | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWN_US     (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWNV_US    (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT |PARM_ATTR_VELOCITY | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWIU_US    (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_INTEGER | PARM_ATTR_UPDATE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWNI_US    (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_INTEGER | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RNIU_US    (PARM_ATTR_READ | PARM_ATTR_NOLIMIT | PARM_ATTR_INTEGER | PARM_ATTR_UPDATE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RW32_USD   (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_32_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RW64_USD   (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWNIU_US   (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_INTEGER | PARM_ATTR_UPDATE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWN32_USD  (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_32_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWN32U_USD (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_32_BIT | PARM_ATTR_UPDATE | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWN64_USD  (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWNI64_USD (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_INTEGER | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWN64U_USD (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UPDATE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWNI64U_USD (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_INTEGER | PARM_ATTR_64_BIT | PARM_ATTR_UPDATE | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWNA_US    (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_ACCELERATION | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWNAT_US   (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_ACCELERATION | PARM_ATTR_TASK_ROTARY | PARM_ATTR_UNSIGNED)
#define PARM_ATTR_RWI64_USD  (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_INTEGER | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED)

#define PARM_ATTR_RWS64D      (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE | PARM_ATTR_DISTANCE)
#define PARM_ATTR_RWNS64D     (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE | PARM_ATTR_DISTANCE)
#define PARM_ATTR_RWNS64_USD  (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED | PARM_ATTR_DISTANCE)
#define PARM_ATTR_RWNA32_USD  (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_32_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED | PARM_ATTR_ACCELERATION )
#define PARM_ATTR_RWNV64_USD  (PARM_ATTR_READ | PARM_ATTR_WRITE | PARM_ATTR_NOLIMIT | PARM_ATTR_64_BIT | PARM_ATTR_DRIVE | PARM_ATTR_UNSIGNED | PARM_ATTR_VELOCITY )


// Parameter Display Attributes
//
// Format: 0xCCCEEEEE
//
// Where EEEEE - is the editor to edit the parameter in
//       CCC   - is the calculator to calculate the parameter in (if a calculator exists for this parameter)

// C#	CLASS=Parameter		REGION=Parameter Display Attributes
// Parameter Editors
#define PARMDISPLAY_ATTR_HIDDEN          0x00000001
#define PARMDISPLAY_ATTR_VALUE           0x00000002
#define PARMDISPLAY_ATTR_BITMASK         0x00000004      /* display as bit mapped */
#define PARMDISPLAY_ATTR_BITMASK2        0x00000008      /* display as both choices per bit bit mapped */
#define PARMDISPLAY_ATTR_CHOICE          0x00000010      /* display as choice */
#define PARMDISPLAY_ATTR_HEX             0x00000020      /* display as a HEX value */
#define PARMDISPLAY_ATTR_ETHERNET        0x00000040      /* display as IP octets */
#define PARMDISPLAY_ATTR_INPUT_DIG       0x00000080      /* display as digital input selector */
#define PARMDISPLAY_ATTR_INPUT_ANA       0x00000100      /* display as analog input selector */
#define PARMDISPLAY_ATTR_AXIS_SELECT     0x00000200      /* display as a list box of axis names */
#define PARMDISPLAY_ATTR_FILE            0x00000400      /* display as a file name */
#define PARMDISPLAY_ATTR_NAME            0x00000800      /* display as a axis name */
#define PARMDISPLAY_ATTR_STRING          0x00001000      /* display as a string */
#define PARMDISPLAY_ATTR_SLEWPAIR        0x00002000      /* display as a slew pair selector */
#define PARMDISPLAY_ATTR_SLEW3DMASK      0x00004000      /* display as a slew pair selector */
// C# END

// C#	CLASS=Parameter		REGION=Parameter Display Calculator
// Parameter Calculators
#define PARMDISPLAY_ATTR_CALC_FDBK       0x00100000      /* calculate using the Feedback Parameter calculator */
#define PARMDISPLAY_ATTR_CALC_MOTOR      0x00200000      /* calculate using the Motor Parameter calcultor */
#define PARMDISPLAY_ATTR_CALC_PI         0x00400000      /* calculate using the PI Loop Gain calculator */
#define PARMDISPLAY_ATTR_CALC_PID        0x00800000      /* calculate using the PID Loop Gain calculator */
#define PARMDISPLAY_ATTR_CALC_FILTER     0x01000000      /* calculate using the Filter Parameter Calculator */
// C# END

#define PD_ATTR_H              (PARMDISPLAY_ATTR_HIDDEN)
#define PD_ATTR_V              (PARMDISPLAY_ATTR_VALUE)
#define PD_ATTR_B              (PARMDISPLAY_ATTR_BITMASK)
#define PD_ATTR_BX             (PARMDISPLAY_ATTR_BITMASK | PARMDISPLAY_ATTR_HEX)
#define PD_ATTR_B2             (PARMDISPLAY_ATTR_BITMASK2)
#define PD_ATTR_B2X            (PARMDISPLAY_ATTR_BITMASK2 | PARMDISPLAY_ATTR_HEX)
#define PD_ATTR_C              (PARMDISPLAY_ATTR_CHOICE)
#define PD_ATTR_AS             (PARMDISPLAY_ATTR_AXIS_SELECT)
#define PD_ATTR_X              (PARMDISPLAY_ATTR_HEX)
#define PD_ATTR_ID             (PARMDISPLAY_ATTR_INPUT_DIG)
#define PD_ATTR_IA             (PARMDISPLAY_ATTR_INPUT_ANA)
#define PD_ATTR_E              (PARMDISPLAY_ATTR_ETHERNET)
#define PD_ATTR_SPX            (PARMDISPLAY_ATTR_SLEWPAIR | PARMDISPLAY_ATTR_HEX)
#define PD_ATTR_S3DX           (PARMDISPLAY_ATTR_SLEW3DMASK | PARMDISPLAY_ATTR_HEX)

#define PD_ATTR_V_FDBK          (PARMDISPLAY_ATTR_CALC_FDBK | PARMDISPLAY_ATTR_VALUE)
#define PD_ATTR_C_FDBK          (PARMDISPLAY_ATTR_CALC_FDBK | PARMDISPLAY_ATTR_CHOICE)
#define PD_ATTR_V_MTR          (PARMDISPLAY_ATTR_CALC_MOTOR | PARMDISPLAY_ATTR_VALUE)
#define PD_ATTR_C_MTR          (PARMDISPLAY_ATTR_CALC_MOTOR | PARMDISPLAY_ATTR_CHOICE)
#define PD_ATTR_B2X_MTR        (PARMDISPLAY_ATTR_CALC_MOTOR | PARMDISPLAY_ATTR_BITMASK2 | PARMDISPLAY_ATTR_HEX)
#define PD_ATTR_V_PI           (PARMDISPLAY_ATTR_CALC_PI | PARMDISPLAY_ATTR_VALUE)
#define PD_ATTR_V_PID          (PARMDISPLAY_ATTR_CALC_PID | PARMDISPLAY_ATTR_VALUE)
#define PD_ATTR_V_FLT          (PARMDISPLAY_ATTR_CALC_FILTER | PARMDISPLAY_ATTR_VALUE)

// Parameter Display Sub Groups
//
// Format: 0xTTTTSSSS
//
// Where TTTT - is the type of parameter
//       SSSS - is the sub group under a give parameter type

// C#	CLASS=Parameter		REGION=Parameter Display Type
//    Parameter Display Types
#define PARMDISPLAY_TYPE_SETUP            0x00010000
#define PARMDISPLAY_TYPE_AXIS             0x00020000
#define PARMDISPLAY_TYPE_TASK             0x00040000
#define PARMDISPLAY_TYPE_GLOBAL           0x00080000
#define PARMDISPLAY_TYPE_FIBER            0x00100000
#define PARMDISPLAY_TYPE_VPP              0x00200000
// C# END

// Setup Sub Groups

// C#	CLASS=Parameter		REGION=Parameter Display Axis Subgroup
// Axis Sub Groups
#define PARMDISPLAY_AXIS_SG_UNITS         0x00000001
#define PARMDISPLAY_AXIS_SG_CONFIG        0x00000002
#define PARMDISPLAY_AXIS_SG_FAULT         0x00000004
#define PARMDISPLAY_AXIS_SG_SERVO         0x00000008
#define PARMDISPLAY_AXIS_SG_FILTER        0x00000010
#define PARMDISPLAY_AXIS_SG_MOTION        0x00000020
#define PARMDISPLAY_AXIS_SG_ETHERNET      0x00000040
#define PARMDISPLAY_AXIS_SG_PSO           0x00000080
#define PARMDISPLAY_AXIS_SG_JOGGING       0x00000100
#define PARMDISPLAY_AXIS_SG_MISC          0x00000200
// C# END

// C#	CLASS=Parameter		REGION=Parameter Display Task Subgroup
// Task Sub Groups
#define PARMDISPLAY_TASK_SG_AXES          0x00000001
#define PARMDISPLAY_TASK_SG_MOTION        0x00000002
#define PARMDISPLAY_TASK_SG_CIRCULAR      0x00000004
#define PARMDISPLAY_TASK_SG_PROGRAM       0x00000008
#define PARMDISPLAY_TASK_SG_SPINDLES      0x00000010
#define PARMDISPLAY_TASK_SG_JOGGING       0x00000020
#define PARMDISPLAY_TASK_SG_MISC          0x00000040
// C# END

// C#	CLASS=Parameter		REGION=Parameter Display Global Subgroup
// Global Sub Groups
#define PARMDISPLAY_GLOBAL_SG_ETHERNET    0x00000001
// C# END

// C#	CLASS=Parameter		REGION=Parameter Display Fiber Subgroup
// Fiber Sub Groups
#define PARMDISPLAY_FIBER_SG_HILLCLIMB    0x00000001
#define PARMDISPLAY_FIBER_SG_SPIRALROUGH  0x00000002
#define PARMDISPLAY_FIBER_SG_SPIRALFINE   0x00000004
#define PARMDISPLAY_FIBER_SG_FASTALIGN    0x00000008
#define PARMDISPLAY_FIBER_SG_GEOCENTER    0x00000010
#define PARMDISPLAY_FIBER_SG_CENTROID     0x00000020
// C# END

// C#	CLASS=Parameter		REGION=Parameter Display VPP Subgroup
// VPP SubGroups
#define PARMDISPLAY_VPP_SG_GENERAL        0x00000001
#define PARMDISPLAY_VPP_SG_TOOL           0x00000002
#define PARMDISPLAY_VPP_SG_TIP1           0x00000004
#define PARMDISPLAY_VPP_SG_TIP2           0x00000008
#define PARMDISPLAY_VPP_SG_AXIS1          0x00000010
#define PARMDISPLAY_VPP_SG_AXIS2          0x00000020
#define PARMDISPLAY_VPP_SG_AXIS3          0x00000040
#define PARMDISPLAY_VPP_SG_AXIS4          0x00000080
#define PARMDISPLAY_VPP_SG_AXIS5          0x00000100
#define PARMDISPLAY_VPP_SG_AXIS6          0x00000200
// C# END

// Display only under the All tab, not under a sub group
#define PD_SG_NONE                        0x00000000

#define PD_ASG_UNITS              (PARMDISPLAY_TYPE_AXIS | PARMDISPLAY_AXIS_SG_UNITS)         // 0x00020001
#define PD_ASG_CONFIG             (PARMDISPLAY_TYPE_AXIS | PARMDISPLAY_AXIS_SG_CONFIG)        // 0x00020002
#define PD_ASG_FAULT              (PARMDISPLAY_TYPE_AXIS | PARMDISPLAY_AXIS_SG_FAULT)         // 0x00020004
#define PD_ASG_SERVO              (PARMDISPLAY_TYPE_AXIS | PARMDISPLAY_AXIS_SG_SERVO)         // 0x00020008
#define PD_ASG_FILTER             (PARMDISPLAY_TYPE_AXIS | PARMDISPLAY_AXIS_SG_FILTER)        // 0x00020010
#define PD_ASG_MOTION             (PARMDISPLAY_TYPE_AXIS | PARMDISPLAY_AXIS_SG_MOTION)        // 0x00020020
#define PD_ASG_ETHERNET           (PARMDISPLAY_TYPE_AXIS | PARMDISPLAY_AXIS_SG_ETHERNET)      // 0x00020040
#define PD_ASG_PSO                (PARMDISPLAY_TYPE_AXIS | PARMDISPLAY_AXIS_SG_PSO)           // 0x00020080
#define PD_ASG_JOG                (PARMDISPLAY_TYPE_AXIS | PARMDISPLAY_AXIS_SG_JOGGING)       // 0x00020100
#define PD_ASG_MISC               (PARMDISPLAY_TYPE_AXIS | PARMDISPLAY_AXIS_SG_MISC)          // 0x00020200

#define PD_TSG_AXES               (PARMDISPLAY_TYPE_TASK | PARMDISPLAY_TASK_SG_AXES)          // 0x00040001
#define PD_TSG_MOTION             (PARMDISPLAY_TYPE_TASK | PARMDISPLAY_TASK_SG_MOTION)        // 0x00040002
#define PD_TSG_CIRCULAR           (PARMDISPLAY_TYPE_TASK | PARMDISPLAY_TASK_SG_CIRCULAR)      // 0x00040004
#define PD_TSG_PROGRAM            (PARMDISPLAY_TYPE_TASK | PARMDISPLAY_TASK_SG_PROGRAM)       // 0x00040008
#define PD_TSG_SPINDLES           (PARMDISPLAY_TYPE_TASK | PARMDISPLAY_TASK_SG_SPINDLES)      // 0x00040010
#define PD_TSG_JOGGING            (PARMDISPLAY_TYPE_TASK | PARMDISPLAY_TASK_SG_JOGGING)       // 0x00040020
#define PD_TSG_MISC               (PARMDISPLAY_TYPE_TASK | PARMDISPLAY_TASK_SG_MISC)          // 0x00040040

#define PD_GSG_ETHERNET           (PARMDISPLAY_TYPE_GLOBAL | PARMDISPLAY_GLOBAL_SG_ETHERNET)  // 0x00080001

#define PD_FSG_HILLCLIMB          (PARMDISPLAY_TYPE_FIBER | PARMDISPLAY_FIBER_SG_HILLCLIMB)   // 0x00100001
#define PD_FSG_SPIRALROUGH        (PARMDISPLAY_TYPE_FIBER | PARMDISPLAY_FIBER_SG_SPIRALROUGH) // 0x00100002
#define PD_FSG_SPIRALFINE         (PARMDISPLAY_TYPE_FIBER | PARMDISPLAY_FIBER_SG_SPIRALFINE)  // 0x00100004
#define PD_FSG_FASTALIGN          (PARMDISPLAY_TYPE_FIBER | PARMDISPLAY_FIBER_SG_FASTALIGN)   // 0x00100008
#define PD_FSG_GEOCENTER          (PARMDISPLAY_TYPE_FIBER | PARMDISPLAY_FIBER_SG_GEOCENTER)   // 0x00100010
#define PD_FSG_CENTROID           (PARMDISPLAY_TYPE_FIBER | PARMDISPLAY_FIBER_SG_CENTROID)    // 0x00100020

#define PD_VSG_GENERAL            (PARMDISPLAY_TYPE_VPP | PARMDISPLAY_VPP_SG_GENERAL)         // 0x00200001
#define PD_VSG_TOOL               (PARMDISPLAY_TYPE_VPP | PARMDISPLAY_VPP_SG_TOOL)            // 0x00200002
#define PD_VSG_TIP1               (PARMDISPLAY_TYPE_VPP | PARMDISPLAY_VPP_SG_TIP1)            // 0x00200004
#define PD_VSG_TIP2               (PARMDISPLAY_TYPE_VPP | PARMDISPLAY_VPP_SG_TIP2)            // 0x00200008
#define PD_VSG_AXIS1              (PARMDISPLAY_TYPE_VPP | PARMDISPLAY_VPP_SG_AXIS1)           // 0x00200010
#define PD_VSG_AXIS2              (PARMDISPLAY_TYPE_VPP | PARMDISPLAY_VPP_SG_AXIS2)           // 0x00200020
#define PD_VSG_AXIS3              (PARMDISPLAY_TYPE_VPP | PARMDISPLAY_VPP_SG_AXIS3)           // 0x00200040
#define PD_VSG_AXIS4              (PARMDISPLAY_TYPE_VPP | PARMDISPLAY_VPP_SG_AXIS4)           // 0x00200080
#define PD_VSG_AXIS5              (PARMDISPLAY_TYPE_VPP | PARMDISPLAY_VPP_SG_AXIS5)           // 0x00200100
#define PD_VSG_AXIS6              (PARMDISPLAY_TYPE_VPP | PARMDISPLAY_VPP_SG_AXIS6)           // 0x00200200

// C#	CLASS=Parameter		REGION=Servo Mask
/* ServoMask definitions (options for the AXISPARM_ServoMask value) */
#define SERVOMASK_KPOS       0x00000001
#define SERVOMASK_VELLOOPFBK 0x00000002
#define SERVOMASK_FILTER1ON  0x00000004
#define SERVOMASK_FILTER2ON  0x00000008
#define SERVOMASK_FILTER3ON  0x00000010
#define SERVOMASK_FILTER4ON  0x00000020
#define SERVOMASK_FILTER5ON  0x00000100
#define SERVOMASK_FILTER6ON  0x00000200
#define SERVOMASK_FILTER7ON  0x00000400
#define SERVOMASK_FILTER8ON  0x00000800
// C# END

// C#	CLASS=Parameter		REGION=Axis Type
/* AxisType definitions (options for the AXISPARM_Type value) */
/* WARNING ! AXISTYPE_xxxx constants are limited to 256, because they are stored in a byte */
#define AXISTYPE_LINEAR             0
#define AXISTYPE_ROTARY             1
#define AXISTYPE_MAX                2
// C# END

// C#	CLASS=Parameter		REGION=Home Type
/* Home type definitions (options for the AXISPARM_HomeType parameter) */
#define HOMETYPE_PASTLIMIT          0
#define HOMETYPE_REVERSE            1
#define HOMETYPE_NOLIMIT            2
#define HOMETYPE_QUICK              3
#define HOMETYPE_VIRTUAL            4
#define HOMETYPE_VIRTUAL2           5
#define HOMETYPE_MAX                6
// C# END

// C#	CLASS=Parameter		REGION=Home Direction
/* Home Direction is actually a mask  (options for the AXISPARM_HomeDirection parameter) */
#define HOMEDIRECTION_DIRECTION    0x1      /* if on, go CW (else go CCW) */
#define HOMEDIRECTION_LIMIT        0x2      /* if on, use HOME limit (else use EOT limit indicated by above direction bit) */
#define HOMEDIRECTION_U600_STYLE   0x4      /* if on, HomeOffset is a SETPOSCMD, else its a real move  */
#define HOMEDIRECTION_MAX          0x7      /* max value for the the AXISPARM_HomeDirection parameter */
// C# END

// C#	CLASS=Parameter		REGION=Physical Axis State
/* Physical Axis State definitions (options for the AXISPARM_AxisState parameter) */
#define PHYSAXIS_STATE_FREE         0     /* MUST be zero */
#define PHYSAXIS_STATE_BOUND        1
#define PHYSAXIS_STATE_MAX          2
// C# END

// C#	CLASS=Parameter		REGION=Accel Mode Axis
// AccelModeAxis axis parameter mask definitions (same goes for DecelModeAxis axis parameter)
#define ACCELMODEAXIS_LINEAR    0x1
#define ACCELMODEAXIS_RATE      0x2
#define ACCELMODEAXIS_SCURVE    0x4
#define ACCELMODEAXIS_MAX       0x6
// C# END

// C#	CLASS=Parameter		REGION=Drive IO Config
// Bit Mask for Drive IO Config Parameter
#define DRIVE_IO_CONFIG_OUTPUT_I_SINK           0x00000001
#define DRIVE_IO_CONFIG_AI_0_RANGE_BIT_1        0x00000002
#define DRIVE_IO_CONFIG_AI_0_RANGE_BIT_2        0x00000004
#define DRIVE_IO_CONFIG_AI_1_RANGE_BIT_1        0x00000008
#define DRIVE_IO_CONFIG_AI_1_RANGE_BIT_2        0x00000010
#define DRIVE_IO_CONFIG_AI_2_RANGE_BIT_1        0x00000020
#define DRIVE_IO_CONFIG_AI_2_RANGE_BIT_2        0x00000040
#define DRIVE_IO_CONFIG_AI_3_RANGE_BIT_1        0x00000080
#define DRIVE_IO_CONFIG_AI_3_RANGE_BIT_2        0x00000100
#define DRIVE_IO_CONFIG_PSO_INTR_DEFEAT         0x00000200
#define DRIVE_IO_CONFIG_USE_POS_LIMITS          0x00000400
#define DRIVE_IO_CONFIG_USE_POS_HALLS           0x00000800
#define DRIVE_IO_CONFIG_USE_INT_BRAKE           0x00001000
#define DRIVE_IO_CONFIG_MOTOR_THERMISTOR        0x00002000
#define DRIVE_IO_CONFIG_DIFF_ICMD_OUTPUT        0x00004000
#define DRIVE_IO_CONFIG_AMP_FAULT_POL_LOW       0x00008000
#define DRIVE_IO_CONFIG_HALLS_ON_AUX_CONN       0x00010000
#define DRIVE_IO_CONFIG_NO_AI_OVERSAMPLE        0x00020000
#define DRIVE_IO_CONFIG_ANALOG_ICMD_OUT         0x00040000
#define DRIVE_IO_CONFIG_SYNCOUT_ACT_LOW         0x00080000
#define DRIVE_IO_CONFIG_AI_SRC_MUX              0x00100000
#define DRIVE_IO_CONFIG_UNIP_ICMD_OUTPUT        0x00200000
#define DRIVE_IO_CONFIG_OPTO_LIMIT_INPUTS       0x00400000
#define DRIVE_IO_CONFIG_EXTRN_FLT_ACTIVE_HI     0x00800000
#define DRIVE_IO_CONFIG_USE_VEL_MARKER          0x01000000
#define DRIVE_IO_CONFIG_MXR_VEL_FILTER			0x02000000
#define DRIVE_IO_CONFIG_MXR_AUTO_TUNE           0x04000000
#define DRIVE_IO_CONFIG_IGNORE_SLAVE_MRK        0x08000000
#define DRIVE_IO_CONFIG_SPARE1                  0x10000000
#define DRIVE_IO_CONFIG_SPARE2                  0x20000000
#define DRIVE_IO_CONFIG_SPARE3                  0x40000000
#define DRIVE_IO_CONFIG_SPARE4                  0x80000000
// C# END

/* Special flag for the TASKPARM_RotateAngleDeg parameter */
#define TASKPARM_ROTATEANGLEDEG_VALUE_OFF   -99999

// C#	CLASS=Parameter		REGION=RI Action Opcodes
/* Action definitions (options for the TASKPARM_RIAction1 parameter) */
#define RIACTION_OPCODE_NONE                 0
#define RIACTION_OPCODE_SPINDLE_CW           1
#define RIACTION_OPCODE_SPINDLE_CCW          2
#define RIACTION_OPCODE_SPINDLE_OFF          3
#define RIACTION_OPCODE_SPINDLE_REORIENT     4
#define RIACTION_OPCODE_ASYNCTYPE_MOVETO     5
#define RIACTION_OPCODE_ASYNCTYPE_HOME       6
#define RIACTION_OPCODE_ASYNCTYPE_INDEX      7
#define RIACTION_OPCODE_ASYNCTYPE_START      8
#define RIACTION_OPCODE_ASYNCTYPE_INFEED     9
//#define RIACTION_OPCODE_ASYNCTYPE_QINDEX     10
//#define RIACTION_OPCODE_ASYNCTYPE_QMOVETO    11
//#define RIACTION_OPCODE_ASYNCTYPE_QHOME      12
//#define RIACTION_OPCODE_ASYNCTYPE_ALTHOME    13
//#define RIACTION_OPCODE_ASYNCTYPE_NOHOME     14
#define RIACTION_OPCODE_ASYNCTYPE_OSCILLATE  15
#define RIACTION_OPCODE_ASYNCTYPE_HALT       16
#define RIACTION_OPCODE_HANDWHEEL            17
#define RIACTION_OPCODE_SPINDLE_CW_ASYNC     18
#define RIACTION_OPCODE_SPINDLE_CCW_ASYNC    19
// C# END

// C#	CLASS=Parameter		REGION=RO Action
/* Action definitions (options for the TASKPARM_ROAction1 parameter) */
#define RIO_1_CYCLESTART         0x0001
#define RIO_1_CYCLESTEP          0x0002
#define RIO_1_CYCLERETRACE_ON    0x0004
#define RIO_1_CYCLERETRACE_OFF   0x0008
#define RIO_1_CYCLESTOP          0x0010
#define RIO_1_CYCLERESET         0x0020
#define RIO_1_CYCLEABORT         0x0040
#define RIO_1_ASYNC_MOVE         0x0080
#define RIO_1_SLEWSTART          0x0100
#define RIO_1_SLEWSTOP           0x0200
#define RIO_1_RESERVED_1         0x0400
#define RIO_1_RESERVED_2         0x0800
#define RIO_1_AUTOMODE_ON        0x1000
#define RIO_1_AUTOMODE_OFF       0x2000
#define RIO_1_SLEW3DSTART        0x4000
// C# END

#define MAX_RIO_ENTRIES    1         /* RI/RO capability only activated on first task.*/

/* Plane definitions (options for the TASKPARM_Coord1Plane, TASKPARM_Coord2Plane parameters) */
#define PLANE_XY           1       // must be last one
#define PLANE_ZX           2
#define PLANE_YZ           3       // must be last one

/* Positions display definitions (options for the GLOBPARM_PositionsDisplay parameter) */
#define POSITIONS_DISPLAY_SHOW_CAL      1  // do NOT subtract calibration (decalibrate) from positions coming back from drive (also effects velocities)

// for Slew3DConfig parameter
#define SLEW3DCONFIG_MAX 5

/*** Status word definitions ***************************************************************************/
//
// TO MAKE A CHANGE IN THESE YOU MUST CHANGE ALL THE BELOW PLACES IN PARALLEL ! :
// See FIRST set (bit names) for comments on bit meanings
//    1. (AERPINFO.H) Structure members (to allow easy bit access to the bits)
//         (change both the bit numbers and the masks)
//    2. (AERPINFO.H) Names for the bits (used by aerstat, etc.)
//    3. (AERPDEF.H) Bit numbers
//    4. (INI/AERPARAM.PGM) CNC bit numbers
//         (change the bit numbers, there are no masks)
//    5. (A32SYS/A32SYS.ODL) CNC bit numbers
//         (change the masks, there are no bit numbers)
//
/* Task Status and Task Mode names */
#define MAX_TASKSTATUS_DWORD     3
#define MAX_TASKMODE_DWORD       1

// C#	CLASS=Status		REGION=Task Status 1
/* Defines are bit numbers for TASKPARM_Status1 */
#define TASKSTATUS1_ProgramAssociated                   0
#define TASKSTATUS1_ProgramActive                       1
#define TASKSTATUS1_ProgramExecuting                    2
#define TASKSTATUS1_ImmediateCodeExecuting              3
#define TASKSTATUS1_ReturnMotionExecuting               4
#define TASKSTATUS1_ProgramAborted                      5
#define TASKSTATUS1_SingleStepInto                      6
#define TASKSTATUS1_SingleStepOver                      7
#define TASKSTATUS1_ProgramReset                        8
#define TASKSTATUS1_PendingAxesStop                     9
#define TASKSTATUS1_EStopActive                        10
#define TASKSTATUS1_FeedHoldActive                     11
#define TASKSTATUS1_CallBackHoldActive                 12
#define TASKSTATUS1_CallBackResponding                 13
#define TASKSTATUS1_SpindleActive1                     14
#define TASKSTATUS1_SpindleActive2                     15
#define TASKSTATUS1_SpindleActive3                     16
#define TASKSTATUS1_SpindleActive4                     17
#define TASKSTATUS1_ProbeCycle                         18
#define TASKSTATUS1_Retrace                            19
#define TASKSTATUS1_SoftHomeActive                     20
#define TASKSTATUS1_InterruptMotionActive              21
#define TASKSTATUS1_SlewActive                         22
#define TASKSTATUS1_CornerRounding                     23
#define TASKSTATUS1_ROReq1Active                       24
#define TASKSTATUS1_SlewLowFeedModeActive              25
#define TASKSTATUS1_CannedFunctionActive               26
#define TASKSTATUS1_CannedFunctionExecuting            27
#define TASKSTATUS1_Spare3                             28
#define TASKSTATUS1_Spare4                             29
#define TASKSTATUS1_Spare5                             30
#define TASKSTATUS1_Spare6                             31
// C# END

// C#	CLASS=Status		REGION=Task Status 1 Mask
#define TASKSTATUS1_M_ProgramAssociated        0x00000001
#define TASKSTATUS1_M_ProgramActive            0x00000002
#define TASKSTATUS1_M_ProgramExecuting         0x00000004
#define TASKSTATUS1_M_ImmediateCodeExecuting   0x00000008
#define TASKSTATUS1_M_ReturnMotionExecuting    0x00000010
#define TASKSTATUS1_M_ProgramAborted           0x00000020
#define TASKSTATUS1_M_SingleStepInto           0x00000040
#define TASKSTATUS1_M_SingleStepOver           0x00000080
#define TASKSTATUS1_M_ProgramReset             0x00000100
#define TASKSTATUS1_M_PendingAxesStop          0x00000200
#define TASKSTATUS1_M_EStopActive              0x00000400
#define TASKSTATUS1_M_FeedHoldActive           0x00000800
#define TASKSTATUS1_M_CallBackHoldActive       0x00001000
#define TASKSTATUS1_M_CallBackResponding       0x00002000
#define TASKSTATUS1_M_SpindleActive1           0x00004000
#define TASKSTATUS1_M_SpindleActive2           0x00008000
#define TASKSTATUS1_M_SpindleActive3           0x00010000
#define TASKSTATUS1_M_SpindleActive4           0x00020000
#define TASKSTATUS1_M_ProbeCycle               0x00040000
#define TASKSTATUS1_M_Retrace                  0x00080000
#define TASKSTATUS1_M_SoftHomeActive           0x00100000
#define TASKSTATUS1_M_InterruptMotionActive    0x00200000
#define TASKSTATUS1_M_SlewActive               0x00400000
#define TASKSTATUS1_M_CornerRounding           0x00800000
#define TASKSTATUS1_M_ROReq1Active             0x01000000
#define TASKSTATUS1_M_SlewLowFeedModeActive    0x02000000
#define TASKSTATUS1_M_CannedFunctionActive     0x04000000
#define TASKSTATUS1_M_CannedFunctionExecuting  0x08000000
#define TASKSTATUS1_M_Spare3                   0x10000000
#define TASKSTATUS1_M_Spare4                   0x20000000
#define TASKSTATUS1_M_Spare5                   0x40000000
#define TASKSTATUS1_M_Spare6                   0x80000000       // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
// C# END

// C#	CLASS=Status		REGION=Task Status 2
/* Defines are bit numbers for TASKPARM_Status2 */
#define TASKSTATUS2_MotionModeAbsOffsets                0
#define TASKSTATUS2_ASyncSMCMotionAbortPending          1
#define TASKSTATUS2_ProfileQueRunning                   2
#define TASKSTATUS2_RetraceRequested                    3
#define TASKSTATUS2_MSOChange                           4
#define TASKSTATUS2_SpindleFeedHeld                     5
#define TASKSTATUS2_FeedHeldAxesStopped                 6
#define TASKSTATUS2_CutterEnabling                      7
#define TASKSTATUS2_CutterDisabling                     8
#define TASKSTATUS2_CutterOffsetsEnablingPos            9
#define TASKSTATUS2_CutterOffsetsEnablingNeg           10
#define TASKSTATUS2_CutterOffsetsDisabling             11
#define TASKSTATUS2_MFOChange                          12
#define TASKSTATUS2_InterruptFaultPending              13
#define TASKSTATUS2_RetraceNegativeHold                14
#define TASKSTATUS2_OnGosubPending                     15
#define TASKSTATUS2_ProgramAbortPending                16
#define TASKSTATUS2_CannedFunctionPending              17
#define TASKSTATUS2_NoMFOFloor                         18
#define TASKSTATUS2_Interrupted                        19
#define TASKSTATUS2_ProgramStepOnce                    20
#define TASKSTATUS2_Spare10                            21
#define TASKSTATUS2_Spare11                            22
#define TASKSTATUS2_Spare12                            23
#define TASKSTATUS2_Spare13                            24
#define TASKSTATUS2_Spare14                            25
#define TASKSTATUS2_Spare15                            26
#define TASKSTATUS2_Spare16                            27
#define TASKSTATUS2_Spare17                            28
#define TASKSTATUS2_Spare18                            29
#define TASKSTATUS2_Spare19                            30
#define TASKSTATUS2_Spare20                            31
// C# END

// C#	CLASS=Status		REGION=Task Status 2 Mask
#define TASKSTATUS2_M_MotionModeAbsOffsets         0x00000001
#define TASKSTATUS2_M_ASyncSMCMotionAbortPending   0x00000002
#define TASKSTATUS2_M_ProfileQueRunning            0x00000004
#define TASKSTATUS2_M_RetraceRequested             0x00000008
#define TASKSTATUS2_M_MSOChange                    0x00000010
#define TASKSTATUS2_M_SpindleFeedHeld              0x00000020
#define TASKSTATUS2_M_FeedHeldAxesStopped          0x00000040
#define TASKSTATUS2_M_CutterEnabling               0x00000080
#define TASKSTATUS2_M_CutterDisabling              0x00000100
#define TASKSTATUS2_M_CutterOffsetsEnablingPos     0x00000200
#define TASKSTATUS2_M_CutterOffsetsEnablingNeg     0x00000400
#define TASKSTATUS2_M_CutterOffsetsDisabling       0x00000800
#define TASKSTATUS2_M_MFOChange                    0x00001000
#define TASKSTATUS2_M_InterruptFaultPending        0x00002000
#define TASKSTATUS2_M_RetraceNegativeHold          0x00004000
#define TASKSTATUS2_M_OnGosubPending               0x00008000
#define TASKSTATUS2_M_ProgramAbortPending          0x00010000
#define TASKSTATUS2_M_CannedFunctionPending        0x00020000
#define TASKSTATUS2_M_NoMFOFloor                   0x00040000
#define TASKSTATUS2_M_Interrupted                  0x00080000
#define TASKSTATUS2_M_ProgramStepOnce              0x00100000
#define TASKSTATUS2_M_Spare10                      0x00200000
#define TASKSTATUS2_M_Spare11                      0x00400000
#define TASKSTATUS2_M_Spare12                      0x00800000
#define TASKSTATUS2_M_Spare13                      0x01000000
#define TASKSTATUS2_M_Spare14                      0x02000000
#define TASKSTATUS2_M_Spare15                      0x04000000
#define TASKSTATUS2_M_Spare16                      0x08000000
#define TASKSTATUS2_M_Spare17                      0x10000000
#define TASKSTATUS2_M_Spare18                      0x20000000
#define TASKSTATUS2_M_Spare19                      0x40000000
#define TASKSTATUS2_M_Spare20                      0x80000000     // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
// C# END

// C#	CLASS=Status		REGION=Task Status 3
/* Defines are bit numbers for TASKPARM_Status3 */
#define TASKSTATUS3_RotationActive                      0
#define TASKSTATUS3_RThetaPolarActive                   1
#define TASKSTATUS3_RThetaCylindricalActive             2
#define TASKSTATUS3_ScalingActive                       3
#define TASKSTATUS3_OffsetFixtureActive                 4
#define TASKSTATUS3_ProfileActive                       5
#define TASKSTATUS3_MotionTypePtToPt                    6
#define TASKSTATUS3_MotionTypeInterp                    7
#define TASKSTATUS3_MotionTypeProfilePoint              8
#define TASKSTATUS3_MotionContinuous                    9
#define TASKSTATUS3_MotionNoAccel                      10
#define TASKSTATUS3_VppActive                          11
#define TASKSTATUS3_CutterOffsetsActivePos             12
#define TASKSTATUS3_CutterActiveLeft                   13
#define TASKSTATUS3_CutterActiveRight                  14
#define TASKSTATUS3_CutterOffsetsActiveNeg             15
#define TASKSTATUS3_NormalcyActiveLeft                 16
#define TASKSTATUS3_NormalcyActiveRight                17
#define TASKSTATUS3_NormalcyAlignment                  18
#define TASKSTATUS3_MotionTypeCW                       19
#define TASKSTATUS3_MotionTypeCCW                      20
#define TASKSTATUS3_LimitFeedRateActive                21
#define TASKSTATUS3_LimitMFOActive                     22
#define TASKSTATUS3_Coord1Plane1                       23
#define TASKSTATUS3_Coord1Plane2                       24
#define TASKSTATUS3_Coord1Plane3                       25
#define TASKSTATUS3_Coord2Plane1                       26
#define TASKSTATUS3_Coord2Plane2                       27
#define TASKSTATUS3_Coord2Plane3                       28
#define TASKSTATUS3_NewG1                              29
#define TASKSTATUS3_MirrorActive                       30
#define TASKSTATUS3_RESERVED                           31
// C# END

// C#	CLASS=Status		REGION=Task Status 3 Mask
#define TASKSTATUS3_M_RotationActive           0x00000001
#define TASKSTATUS3_M_RThetaPolarActive        0x00000002
#define TASKSTATUS3_M_RThetaCylindricalActive  0x00000004
#define TASKSTATUS3_M_ScalingActive            0x00000008
#define TASKSTATUS3_M_OffsetFixtureActive      0x00000010
#define TASKSTATUS3_M_ProfileActive            0x00000020
#define TASKSTATUS3_M_MotionTypePtToPt         0x00000040
#define TASKSTATUS3_M_MotionTypeInterp         0x00000080
#define TASKSTATUS3_M_MotionTypeProfilePoint   0x00000100
#define TASKSTATUS3_M_MotionContinuous         0x00000200
#define TASKSTATUS3_M_MotionNoAccel            0x00000400
#define TASKSTATUS3_M_VppActive                0x00000800
#define TASKSTATUS3_M_CutterOffsetsActivePos   0x00001000
#define TASKSTATUS3_M_CutterActiveLeft         0x00002000
#define TASKSTATUS3_M_CutterActiveRight        0x00004000
#define TASKSTATUS3_M_CutterOffsetsActiveNeg   0x00008000
#define TASKSTATUS3_M_NormalcyActiveLeft       0x00010000
#define TASKSTATUS3_M_NormalcyActiveRight      0x00020000
#define TASKSTATUS3_M_NormalcyAlignment        0x00040000
#define TASKSTATUS3_M_MotionTypeCW             0x00080000
#define TASKSTATUS3_M_MotionTypeCCW            0x00100000
#define TASKSTATUS3_M_LimitFeedRateActive      0x00200000
#define TASKSTATUS3_M_LimitMFOActive           0x00400000
#define TASKSTATUS3_M_Coord1Plane1             0x00800000
#define TASKSTATUS3_M_Coord1Plane2             0x01000000
#define TASKSTATUS3_M_Coord1Plane3             0x02000000
#define TASKSTATUS3_M_Coord2Plane1             0x04000000
#define TASKSTATUS3_M_Coord2Plane2             0x08000000
#define TASKSTATUS3_M_Coord2Plane3             0x10000000
#define TASKSTATUS3_M_NewG1                    0x20000000
#define TASKSTATUS3_M_MirrorActive             0x40000000
#define TASKSTATUS3_M_RESERVED                   0x80000000      // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
// C# END

// C#	CLASS=Status		REGION=Task Mode 1
/* Defines are bit numbers for TASKPARM_Mode */
#define TASKMODE1_English                               0
#define TASKMODE1_Absolute                              1
#define TASKMODE1_AccelModeLinear                       2
#define TASKMODE1_AccelModeRate                         3
#define TASKMODE1_RotaryDominant                        4
#define TASKMODE1_MotionContinuous                      5
#define TASKMODE1_InverseCircular                       6
#define TASKMODE1_SpindleStopOnProgHalt                 7
#define TASKMODE1_BlockDelete                           8
#define TASKMODE1_OptionalStop                          9
#define TASKMODE1_Spare1                               10
#define TASKMODE1_MFOLock                              11
#define TASKMODE1_MSOLock                              12
#define TASKMODE1_DryRunFeedRate                       13
#define TASKMODE1_Spare2                               14
#define TASKMODE1_AutoMode                             15
#define TASKMODE1_ProgramFeedRateMPU                   16
#define TASKMODE1_ProgramFeedRateUPR                   17
#define TASKMODE1_ProgramSFeedRateSurf1                18
#define TASKMODE1_ProgramSFeedRateSurf2                19
#define TASKMODE1_ProgramSFeedRateSurf3                20
#define TASKMODE1_ProgramSFeedRateSurf4                21
#define TASKMODE1_BlockDelete2                         22
#define TASKMODE1_RunOverMode                          23
#define TASKMODE1_MultiBlockLookAhead                  24
#define TASKMODE1_HighSpeedLookAhead                   25
#define TASKMODE1_MFOActiveOnJog                       26
#define TASKMODE1_WaitForInPos                         27
#define TASKMODE1_Minutes                              28
#define TASKMODE1_Counts                               29
#define TASKMODE1_WaitNone                             30
#define TASKMODE1_RESERVED                             31       // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
// C# END

// C#	CLASS=Status		REGION=Task Mode 1 Mask
#define TASKMODE1_M_English                    0x00000001
#define TASKMODE1_M_Absolute                   0x00000002
#define TASKMODE1_M_AccelModeLinear            0x00000004
#define TASKMODE1_M_AccelModeRate              0x00000008
#define TASKMODE1_M_RotaryDominant             0x00000010
#define TASKMODE1_M_MotionContinuous           0x00000020
#define TASKMODE1_M_InverseCircular            0x00000040
#define TASKMODE1_M_SpindleStopOnProgHalt      0x00000080
#define TASKMODE1_M_BlockDelete                0x00000100
#define TASKMODE1_M_OptionalStop               0x00000200
#define TASKMODE1_M_AccelModeScurve            0x00000400
#define TASKMODE1_M_MFOLock                    0x00000800
#define TASKMODE1_M_MSOLock                    0x00001000
#define TASKMODE1_M_DryRunFeedRate             0x00002000
#define TASKMODE1_M_Spare2                     0x00004000
#define TASKMODE1_M_AutoMode                   0x00008000
#define TASKMODE1_M_ProgramFeedRateMPU         0x00010000
#define TASKMODE1_M_ProgramFeedRateUPR         0x00020000
#define TASKMODE1_M_ProgramSFeedRateSurf1      0x00040000
#define TASKMODE1_M_ProgramSFeedRateSurf2      0x00080000
#define TASKMODE1_M_ProgramSFeedRateSurf3      0x00100000
#define TASKMODE1_M_ProgramSFeedRateSurf4      0x00200000
#define TASKMODE1_M_BlockDelete2               0x00400000
#define TASKMODE1_M_RunOverMode                0x00800000
#define TASKMODE1_M_MultiBlockLookAhead        0x01000000
#define TASKMODE1_M_HighSpeedLookAhead         0x02000000
#define TASKMODE1_M_MFOActiveOnJog             0x04000000
#define TASKMODE1_M_WaitForInPos               0x08000000
#define TASKMODE1_M_Minutes                    0x10000000
#define TASKMODE1_M_Counts                     0x20000000
#define TASKMODE1_M_WaitNone                   0x40000000
#define TASKMODE1_M_RESERVED                   0x80000000       // DO NOT USE, DUE TO SIGN BIT PROBLEMS in VB
// C# END

/*** Setup Parameter data ***************************************************************************/

// C#	CLASS=Parameter		REGION=Setup Parameters
#define SETUPPARM_Name                    0
#define SETUPPARM_EnglishUnitsName        1
#define SETUPPARM_MetricUnitsName         2
#define SETUPPARM_RotaryUnitsName         3
#define SETUPPARM_RotaryVelUnitsName      4
#define MAX_SETUPPARMS                    5
// C# END

// C#	CLASS=Parameter		REGION=Axis Parameters
/*** Axis Parameter data ***************************************************************************/
/*
	NOTE: CHANGES HERE MUST BE MIRRORED IN THE AxisParmInfo STRUCTURE BELOW, AND IN AERSYS\AERSYS.ODL,
			 AND IN PROGRAMS\AERPARAM.PGM

	NOTE: ALL "MACHINE" TYPE AXIS PARAMETERS MUST APPEAR BEFORE THE "DRIVE" TYPE AXIS PARAMATERS.
*/
#define AXISPARM_Type                                    0  // WARNING: "type" MUST be first axis param
#define AXISPARM_CntsPerEnglishUnit                      1  // WARNING: the "CntsPerEng" MUST be second axis parms (follows type, but precedes everything else)
#define AXISPARM_CntsPerMetricUnit                       2  // WARNING: the "CntsPerMetr" MUST be third axis parms (follows type, but precedes everything else)
#define AXISPARM_CntsPerRotaryUnit                       3  // WARNING: the "CntsPerRota" MUST be fourth axis parms (follows type, but precedes everything else)
#define AXISPARM_RotaryFeedRateScaleFactor               4  // WARNING: the "RotaryFeedRateScaleFactor" must be fifth axis parm
#define AXISPARM_RolloverDistanceCnts                    5
#define AXISPARM_HomeType                                6
#define AXISPARM_HomeDirection                           7
#define AXISPARM_HomeFeedRate                            8
#define AXISPARM_HomeOffset                              9
#define AXISPARM_NumDecimalsEnglish                     10
#define AXISPARM_NumDecimalsMetric                      11
#define AXISPARM_NumDecimalsRotary                      12
#define AXISPARM_AxisState                              13
#define AXISPARM_ControllingTask                        14
#define AXISPARM_PositionUnits                          15
#define AXISPARM_PositionCmdUnits                       16
#define AXISPARM_PositionProgCmdUnits                   17
#define AXISPARM_AvgVelTimeMsec                         18
#define AXISPARM_ScaleFactor                            19
#define AXISPARM_PositionProgUnits                      20
#define AXISPARM_FixtureOffset1                         21
#define AXISPARM_FixtureOffset2                         22
#define AXISPARM_FixtureOffset3                         23
#define AXISPARM_FixtureOffset4                         24
#define AXISPARM_FixtureOffset5                         25
#define AXISPARM_FixtureOffset6                         26
#define AXISPARM_JogDistance                            27
#define AXISPARM_JogVelocity                            28
#define AXISPARM_UnusedAxis                             29
#define AXISPARM_ReverseSlewDir                         30
#define AXISPARM_VelTimeConst                           31
#define AXISPARM_Clock                                  32
#define AXISPARM_MaxFeedRate                            33
#define AXISPARM_RapidFeedRate                          34
#define AXISPARM_AccelModeAxis                          35
#define AXISPARM_DecelModeAxis                          36
#define AXISPARM_AccelRateAxis                          37
#define AXISPARM_DecelRateAxis                          38
#define AXISPARM_AccelTimeSecAxis                       39
#define AXISPARM_DecelTimeSecAxis                       40
#define AXISPARM_PositionCnts                           41
#define AXISPARM_PositionCmdCnts                        42
#define AXISPARM_Fault                                  43
#define AXISPARM_DriveStatus                            44
#define AXISPARM_AxisStatus                             45
#define AXISPARM_CalDisable1D                           46
#define AXISPARM_SlaveAdvance                           47
#define AXISPARM_SlaveOffset                            48
#define AXISPARM_GearMaster                             49
#define AXISPARM_GearSlave                              50
#define AXISPARM_GearMode                               51
#define AXISPARM_VelocityCntsSecAvg                     52
#define AXISPARM_ATAmplitude                            53
#define AXISPARM_ATStartFrequency                       54
#define AXISPARM_ATBandWidth                            55
#define AXISPARM_ATDamping                              56
#define AXISPARM_ATPhaseMargin                          57
#define AXISPARM_JoyStickLowFeedRate                    58
#define AXISPARM_JoyStickHighFeedRate                   59
#define AXISPARM_HomeAccelDecelRateAxis                 60
#define AXISPARM_AffDecelScale                          61
#define AXISPARM_SafeLimitMode                          62
#define AXISPARM_SafeZoneCWCnts                         63
#define AXISPARM_SafeZoneCCWCnts                        64
#define AXISPARM_PositionExtUnits                       65
#define AXISPARM_PositionExtCnts                        66
#define AXISPARM_SoftLimitMode                          67
#define AXISPARM_PositionErrUnits                       68
#define AXISPARM_PositionErrCnts                        69
#define AXISPARM_EnableGainCalibration                  70
#define AXISPARM_InPos2PeakToPeakError                  71
#define AXISPARM_InPos2WindowLengthMsec                 72
#define AXISPARM_HomePositionSet                        73
#define AXISPARM_CalBackLashTimeConst                   74
#define AXISPARM_Reserved5                              75
#define AXISPARM_Reserved6                              76
#define AXISPARM_Reserved7                              77
#define AXISPARM_Reserved8                              78
#define AXISPARM_Reserved9                              79
#define AXISPARM_Reserved10                             80
#define AXISPARM_Reserved11                             81
#define AXISPARM_Reserved12                             82
#define AXISPARM_Reserved13                             83
#define AXISPARM_Reserved14                             84
#define AXISPARM_Reserved15                             85
#define AXISPARM_Reserved16                             86
#define AXISPARM_Reserved17                             87
#define AXISPARM_Reserved18                             88
#define AXISPARM_Reserved19                             89
#define AXISPARM_Reserved20                             90
#define AXISPARM_Reserved21                             91
#define AXISPARM_Reserved22                             92
#define AXISPARM_Reserved23                             93
#define AXISPARM_Reserved24                             94
#define AXISPARM_Reserved25                             95
#define AXISPARM_Reserved26                             96
#define AXISPARM_Reserved27                             97
#define AXISPARM_Reserved28                             98
#define AXISPARM_Reserved29                             99

// "machine" parameters end
#define LAST_AXISPARM_MACH_PARAM           100

// "drive"parameters start (do go down to drive)
#define AXISPARM_ServoUpdateRate                       (  0 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ServoMask                             (  1 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKpos                              (  2 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKi                                (  3 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKp                                (  4 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainVff                               (  5 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainAff                               (  6 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKv                                (  7 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKpi                               (  8 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent1N0                        (  9 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent1N1                        ( 10 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent1N2                        ( 11 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent1D1                        ( 12 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent1D2                        ( 13 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent2N0                        ( 14 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent2N1                        ( 15 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent2N2                        ( 16 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent2D1                        ( 17 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent2D2                        ( 18 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_IGainDTimeUsec                        ( 19 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_IGainDTimeCorrectUSec                 ( 20 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_IGainK                                ( 21 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainReserved                          ( 22 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_IGainKi                               ( 23 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_IGainKp                               ( 24 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_IGainIaOffset                         ( 25 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_IGainIbOffset                         ( 26 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_IGainVaOffset                         ( 27 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_IGainVbOffset                         ( 28 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_IGainVcOffset                         ( 29 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FaultMask                             ( 30 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FaultMaskDisable                      ( 31 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FaultMaskDecel                        ( 32 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FaultMaskStop                         ( 33 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FaultMaskInterrupt                    ( 34 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_BrakeOnDriveDisable                   ( 35 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FaultMaskAux                          ( 36 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FaultEStopInput                       ( 37 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ThresholdPosErr                       ( 38 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ThresholdAvgIAmp                      ( 39 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ThresholdAvgITimeMsec                 ( 40 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ThresholdVelCmd                       ( 41 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ThresholdVelError                     ( 42 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ThresholdSoftCCW                      ( 43 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ThresholdSoftCW                       ( 44 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ThresholdClampIAmp                    ( 45 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ThresholdInPosDist                    ( 46 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotType                            ( 47 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotCyclesRev                       ( 48 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotCntsRev                         ( 49 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotCntsInternal                    ( 50 + (DWORD) LAST_AXISPARM_MACH_PARAM)  // MUST follow AXISPARM_CfgMotCntsRev AND AXISPARM_CfgMotCyclesRev
#define AXISPARM_CfgMotOffsetAng                       ( 51 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotMSetTimeMsec                    ( 52 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotMSetIAmps                       ( 53 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkPosType                         ( 54 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkPosChan                         ( 55 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkPosMultiplier                   ( 56 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkVelType                         ( 57 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkVelChan                         ( 58 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkVelMultiplier                   ( 59 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncMxhSetup                     ( 60 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncMultFactorMXH                ( 61 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncMultFactorMXU                ( 62 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncSineGain                     ( 63 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncSineOffset                   ( 64 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncCosGain                      ( 65 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncCosOffset                    ( 66 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncPhase                        ( 67 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgGantryMasterAxis                   ( 68 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgGantrySlaveAxis                    ( 69 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetDriveIPAddress                    ( 70 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetDriveSubnetMask                   ( 71 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetDriveGateway                      ( 72 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetIOIPAddress                       ( 73 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetIOSubnetMask                      ( 74 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetIOGateway                         ( 75 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetDefNumInputWords                  ( 76 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetDefNumOutputWords                 ( 77 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetDefNumInputBits                   ( 78 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetDefNumOutputBits                  ( 79 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetDefNumInputProcess                ( 80 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetDefNumOutputProcess               ( 81 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InetConfigFlags                       ( 82 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_DecelRateMoveAbort                    ( 83 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_LimitDecelDistCnts                    ( 84 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_LimitDebounceTimeMsec                 ( 85 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_LimitLevelMask                        ( 86 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_BacklashDistCnts                      ( 87 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_AuxOutBitMask                         ( 88 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_AuxOutBitLevel                        ( 89 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_DriveIOConfig                         ( 90 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_BrakeOutput                           ( 91 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_PsoSsi1Config                         ( 92 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_PsoSsi2Config                         ( 93 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_PsoMrk1Config                         ( 94 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_PsoMrk2Config                         ( 95 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_PsoMrk3Config                         ( 96 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_EncoderDivider                        ( 97 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FaultAuxInput                         ( 98 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FaultDisableDelay                     ( 99 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgGantryMode                         (100 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotStepperRes                      (101 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotStepperHighCur                  (102 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotStepperLowCur                   (103 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotStepperDGain                    (104 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgPhsAdvMaxAngle                     (105 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgPhsAdvMaxVel                       (106 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgPhsAdvSegAngle                     (107 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgPhsAdvSegVel                       (108 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncQuadDivider                  (109 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncQuadChan                     (110 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainDff                               (111 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainSFComp                            (112 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent3N0                        (113 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent3N1                        (114 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent3N2                        (115 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent3D1                        (116 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent3D2                        (117 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent4N0                        (118 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent4N1                        (119 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent4N2                        (120 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent4D1                        (121 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent4D2                        (122 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkRDGain                          (123 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkRDCosPhase                      (124 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkRDConfig                        (125 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ExtAmpMaxCurrent                      (126 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_LimitDebounceDistCnts                 (127 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgEnableDelay                        (128 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgMotStepperVerSpeed                 (129 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_ThresholdInPosTimeMSec                (130 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgFbkEncMxMrkLoc                     (131 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent5N0                        (132 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent5N1                        (133 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent5N2                        (134 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent5D1                        (135 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent5D2                        (136 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent6N0                        (137 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent6N1                        (138 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent6N2                        (139 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent6D1                        (140 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent6D2                        (141 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent7N0                        (142 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent7N1                        (143 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent7N2                        (144 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent7D1                        (145 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent7D2                        (146 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent8N0                        (147 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent8N1                        (148 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent8N2                        (149 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent8D1                        (150 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FiltCurrent8D2                        (151 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_EnableGainScaling                     (152 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainScaleThreshLow                    (153 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainScaleThreshHigh                   (154 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKpos2                             (155 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKi2                               (156 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKp2                               (157 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKpi2                              (158 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_FaultMaskDisableDelay                 (159 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_InPosDisableTimeoutMsec               (160 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_BrakeDisableDelay                     (161 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainScaleThreshLow2                   (162 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainScaleThreshHigh2                  (163 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKpos3                             (164 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKi3                               (165 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKp3                               (166 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_GainKpi3                              (167 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgGantryCurrLimit                    (168 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define AXISPARM_CfgGantryDiffPosLimit                 (169 + (DWORD) LAST_AXISPARM_MACH_PARAM)
#define MAX_AXISPARMS                                  (170 + (DWORD) LAST_AXISPARM_MACH_PARAM)
// C# END


// C#	CLASS=Parameter		REGION=Task Parameters
/*** Task Parameter data ***************************************************************************/
/*
	NOTE: All task parameters marked with DISTANCE, ACCELERATION, or VELOCITY MUST HAVE A DEFAULT OF ZERO,
		OR IT WILL NOT BE ABLE TO CONVERT THE VALUE PROPERLY (NO AXIS OF THAT TYPE MAY BE BOUND TO THE TASK)

	NOTE: CHANGES HERE MUST BE MIRRORED IN THE TaskParmInfo STRUCTURE BELOW, AND IN AERSYS\AERSYS.ODL,
		AND IN PROGRAMS\AERPARAM.PGM
*/
#define TASKPARM_Number                                  0
#define TASKPARM_TaskFault                               1
#define TASKPARM_TaskWarning                             2
#define TASKPARM_BoundAxesMask                           3
#define TASKPARM_DisplayAxesMask                         4
#define TASKPARM_RotateX                                 5
#define TASKPARM_RotateY                                 6
#define TASKPARM_RotateAngleDeg                          7
#define TASKPARM_RThetaX                                 8
#define TASKPARM_RThetaY                                 9
#define TASKPARM_RThetaR                                10
#define TASKPARM_RThetaT                                11
#define TASKPARM_RThetaRadius                           12
#define TASKPARM_RThetaEnabled                          13
#define TASKPARM_AccelRateLinear                        14
#define TASKPARM_DecelRateLinear                        15
#define TASKPARM_AccelRateRotary                        16
#define TASKPARM_DecelRateRotary                        17
#define TASKPARM_AccelTimeSec                           18
#define TASKPARM_DecelTimeSec                           19
#define TASKPARM_DecelOnProgramAbortMask                20
#define TASKPARM_FeedRateLinear                         21
#define TASKPARM_FeedRateRotary                         22
#define TASKPARM_FeedRateLinearDefault                  23
#define TASKPARM_FeedRateRotaryDefault                  24
#define TASKPARM_DryRunFeedRateLinear                   25
#define TASKPARM_DryRunFeedRateRotary                   26
#define TASKPARM_FeedRateLinearActual                   27
#define TASKPARM_FeedRateRotaryActual                   28
#define TASKPARM_BlendMaxAccelLinearIPS2                29
#define TASKPARM_BlendMaxAccelRotaryDPS2                30
#define TASKPARM_BlendMaxAccelCircleIPS2                31
#define TASKPARM_MaxLookAheadMoves                      32
#define TASKPARM_MFO                                    33
#define TASKPARM_UserFeedRateMode                       34
#define TASKPARM_FeedHold                               35
#define TASKPARM_Coord1I                                36
#define TASKPARM_Coord1J                                37
#define TASKPARM_Coord1K                                38
#define TASKPARM_Coord1Plane                            39
#define TASKPARM_Coord2I                                40
#define TASKPARM_Coord2J                                41
#define TASKPARM_Coord2K                                42
#define TASKPARM_Coord2Plane                            43
#define TASKPARM_MaxRadiusAdjustDeg                     44
#define TASKPARM_MaxRadiusError                         45
#define TASKPARM_CutterX                                46
#define TASKPARM_CutterY                                47
#define TASKPARM_CutterZ                                48
#define TASKPARM_CutterLength                           49
#define TASKPARM_CutterWear                             50
#define TASKPARM_CutterRadius                           51
#define TASKPARM_CutterOffsetX                          52
#define TASKPARM_CutterOffsetY                          53
#define TASKPARM_CutterActive                           54
#define TASKPARM_CutterToleranceDeg                     55
#define TASKPARM_NormalcyX                              56
#define TASKPARM_NormalcyY                              57
#define TASKPARM_NormalcyAxis                           58
#define TASKPARM_NormalcyToleranceDeg                   59
#define TASKPARM_Status1                                60
#define TASKPARM_Status2                                61
#define TASKPARM_Status3                                62
#define TASKPARM_Mode1                                  63
#define TASKPARM_ErrCode                                64
#define TASKPARM_HaltTaskOnAxisFault                    65
#define TASKPARM_InterruptMotion                        66
#define TASKPARM_InterruptMotionReturnType              67
#define TASKPARM_SoftEStopInput                         68
#define TASKPARM_FeedHoldInput                          69
#define TASKPARM_FeedHoldEdgeInput                      70
#define TASKPARM_AnalogMFOInput                         71
#define TASKPARM_S1_AnalogMSOInput                      72
#define TASKPARM_S2_AnalogMSOInput                      73
#define TASKPARM_S3_AnalogMSOInput                      74
#define TASKPARM_S4_AnalogMSOInput                      75
#define TASKPARM_S1_Index                               76  // These MUST be alternating consective: Index,Feedrate, Index,Feedrate, Etc.
#define TASKPARM_S1_FeedRate                            77
#define TASKPARM_S2_Index                               78
#define TASKPARM_S2_FeedRate                            79
#define TASKPARM_S3_Index                               80
#define TASKPARM_S3_FeedRate                            81
#define TASKPARM_S4_Index                               82
#define TASKPARM_S4_FeedRate                            83
#define TASKPARM_S1_MSO                                 84  // MMI needs spindle MSO's to be consecutive numbered
#define TASKPARM_S2_MSO                                 85
#define TASKPARM_S3_MSO                                 86
#define TASKPARM_S4_MSO                                 87
#define TASKPARM_S1_SpindleRadialAxis                   88
#define TASKPARM_S2_SpindleRadialAxis                   89
#define TASKPARM_S3_SpindleRadialAxis                   90
#define TASKPARM_S4_SpindleRadialAxis                   91
#define TASKPARM_ROReq1                                 92
#define TASKPARM_RIAction1                              93
#define TASKPARM_ROAction1                              94
#define TASKPARM_ROReq1Mask                             95
#define TASKPARM_RIActionOpCode                         96
#define TASKPARM_RIActionAxis                           97
#define TASKPARM_RIActionParm1                          98
#define TASKPARM_RIActionParm2                          99
#define TASKPARM_ActiveFixtureOffset                   100
#define TASKPARM_MaxCallStack                          101
#define TASKPARM_MaxModeStack                          102
#define TASKPARM_NumTaskDoubles                        103
#define TASKPARM_NumTaskStrings                        104
#define TASKPARM_NumTaskAxisPts                        105
#define TASKPARM_MaxMonitorData                        106
#define TASKPARM_MaxOnGosubData                        107
#define TASKPARM_UpdateTimeSec                         108
#define TASKPARM_UpdateNumEntries                      109
#define TASKPARM_ExecuteNumLines                       110
#define TASKPARM_ExecuteNumMonitors                    111
#define TASKPARM_ExecuteNumSpindles                    112
#define TASKPARM_JogPair1EnableIn                      113
#define TASKPARM_JogPair1Mode                          114
#define TASKPARM_JogPair1Axis1                         115
#define TASKPARM_JogPair1Axis1PlusIn                   116
#define TASKPARM_JogPair1Axis1MinusIn                  117
#define TASKPARM_JogPair1Axis2                         118
#define TASKPARM_JogPair1Axis2PlusIn                   119
#define TASKPARM_JogPair1Axis2MinusIn                  120
#define TASKPARM_JogPair2EnableIn                      121
#define TASKPARM_JogPair2Mode                          122
#define TASKPARM_JogPair2Axis1                         123
#define TASKPARM_JogPair2Axis1PlusIn                   124
#define TASKPARM_JogPair2Axis1MinusIn                  125
#define TASKPARM_JogPair2Axis2                         126
#define TASKPARM_JogPair2Axis2PlusIn                   127
#define TASKPARM_JogPair2Axis2MinusIn                  128
#define TASKPARM_JoyStickDeadband                      129
#define TASKPARM_SlewPair1                             130
#define TASKPARM_SlewPair2                             131
#define TASKPARM_SlewPair3                             132
#define TASKPARM_SlewPair4                             133
#define TASKPARM_SlewPair5                             134
#define TASKPARM_SlewPair6                             135
#define TASKPARM_SlewPair7                             136
#define TASKPARM_SlewPair8                             137
#define TASKPARM_LineNumberUser                        138
#define TASKPARM_LineNumberSMC                         139
#define TASKPARM_CannedFunctionID                      140
#define TASKPARM_IgnoreAxesMask                        141
#define TASKPARM_ChordicalToleranceInch                142
#define TASKPARM_ChordicalSlowdownMsec                 143
#define TASKPARM_CommandVelocityVariance               144
#define TASKPARM_Group1GCodeMode                       145
#define TASKPARM_SlewPair1Invert                       146
#define TASKPARM_SlewPair2Invert                       147
#define TASKPARM_SlewPair3Invert                       148
#define TASKPARM_SlewPair4Invert                       149
#define TASKPARM_SlewPair5Invert                       150
#define TASKPARM_SlewPair6Invert                       151
#define TASKPARM_SlewPair7Invert                       152
#define TASKPARM_SlewPair8Invert                       153
#define TASKPARM_JoyStickAnalogHorizInput              154
#define TASKPARM_JoyStickAnalogVertInput               155
#define TASKPARM_JoyStickDigitalAxisPairSelInput       156
#define TASKPARM_JoyStickDigitalFeedRateSelInput       157
#define TASKPARM_JoyStickDigitalInterlockInput         158
#define TASKPARM_JoyStickMinVoltage                    159
#define TASKPARM_JoyStickMaxVoltage                    160
#define TASKPARM_Slew3DAxisMask1                       161
#define TASKPARM_Slew3DAxisMask2                       162
#define TASKPARM_Slew3DAxisMask3                       163
#define TASKPARM_Slew3DAxisMask4                       164
#define TASKPARM_Slew3DAxisMask5                       165
#define TASKPARM_Slew3DAxisMask6                       166
#define TASKPARM_Slew3DAxisMask7                       167
#define TASKPARM_Slew3DAxisMask8                       168
#define TASKPARM_Slew3DConfig1                         169
#define TASKPARM_Slew3DConfig2                         170
#define TASKPARM_Slew3DConfig3                         171
#define TASKPARM_Slew3DConfig4                         172
#define TASKPARM_Slew3DConfig5                         173
#define TASKPARM_Slew3DConfig6                         174
#define TASKPARM_Slew3DConfig7                         175
#define TASKPARM_Slew3DConfig8                         176
#define TASKPARM_JoyStickAnalog3DInput                 177
#define TASKPARM_PciIODelayMode                        178
#define TASKPARM_AnalogMFOMinInputVoltage              179
#define TASKPARM_AnalogMFOMaxInputVoltage              180
#define TASKPARM_MFOMax                                181
#define TASKPARM_JoyStickVertMinVoltage                182
#define TASKPARM_JoyStickVertMaxVoltage                183
#define TASKPARM_JoyStickVertDeadband                  184
#define TASKPARM_JoyStick3DMinVoltage                  185
#define TASKPARM_JoyStick3DMaxVoltage                  186
#define TASKPARM_JoyStick3DDeadband                    187
#define TASKPARM_MFOMin                                188
#define TASKPARM_DwellPercentComplete                  189
#define TASKPARM_WaitTimeMsec                          190
#define TASKPARM_MFOStep                               191
#define TASKPARM_DfltSCurve                            192
#define MAX_TASKPARMS                                  193
// C# END


// C#	CLASS=Parameter		REGION=Global Parameters
/*** Global Parameter data ***************************************************************************/
/*
	NOTE: CHANGES HERE MUST BE MIRRORED IN THE GlobParmInfo STRUCTURE BELOW, AND IN AERSYS\AERSYS.ODL,
		AND IN PROGRAMS\AERPARAM.PGM
*/
#define GLOBPARM_AvgPollTimeSec                          0
#define GLOBPARM_Version                                 1
#define GLOBPARM_NumGlobalDoubles                        2
#define GLOBPARM_NumGlobalStrings                        3
#define GLOBPARM_NumGlobalAxisPts                        4
#define GLOBPARM_CallBackTimeoutSec                      5
#define GLOBPARM_BuildNumber                             6
#define GLOBPARM_UserMode                                7
#define GLOBPARM_ThrowTaskWarningsAsFaults               8
#define GLOBPARM_Enable2DCalibration                     9
#define GLOBPARM_NumCannedFunctions                     10
#define GLOBPARM_CompatibilityMode                      11
#define GLOBPARM_NumDecimalsCompare                     12
#define GLOBPARM_CNCMeasurementMode                     13
#define GLOBPARM_MaxNPlotAxes                           14
#define GLOBPARM_PositionsDisplayMask                   15
#define GLOBPARM_InetGlobalIOIPAddress                  16
#define GLOBPARM_InetGlobalIOSubnetMask                 17
#define GLOBPARM_InetGlobalIOGateway                    18
#define GLOBPARM_InetGlobalDefNumInputWords             19
#define GLOBPARM_InetGlobalDefNumOutputWords            20
#define GLOBPARM_InetGlobalDefNumInputBits              21
#define GLOBPARM_InetGlobalDefNumOutputBits             22
#define GLOBPARM_InetGlobalDefNumInputProcess           23
#define GLOBPARM_InetGlobalDefNumOutputProcess          24
#define GLOBPARM_InetGlobalConfigFlags                  25
#define GLOBPARM_InetGlobalInputBitsStart               26
#define GLOBPARM_InetGlobalOutputBitsStart              27
#define GLOBPARM_InetGlobalOutputBitsStatusStart        28
#define GLOBPARM_InetGlobalInputWordsStart              29
#define GLOBPARM_InetGlobalOutputWordsStart             30
#define GLOBPARM_InetGlobalOutputWordsStatusStart       31
#define GLOBPARM_InetGlobalInputProcessStart            32
#define GLOBPARM_InetGlobalOutputProcessStart           33
#define GLOBPARM_NumDrivesRequired                      34
#define GLOBPARM_ClockResMultiplier                     35
#define GLOBPARM_1394CommLostNumber                     36
#define GLOBPARM_MaxNPlotPoints                         37
#define MAX_GLOBPARMS                                   38
// C# END


// C#	CLASS=Parameter		REGION=Fiber Parameters
/*** Fiber Parameter data ***************************************************************************/
/*
	NOTE: CHANGES HERE MUST BE MIRRORED IN THE TaskParmInfo STRUCTURE BELOW, AND IN AERSYS\AERSYS.ODL,
		AND IN PROGRAMS\AERPARAM.PGM
*/
#define FIBERPARM_HCScanIncrement                        0  // needs to be checked for != 0
#define FIBERPARM_HCMaxDisplacement                      1  // needs to be checked for > 0
#define FIBERPARM_HCThreshold                            2
#define FIBERPARM_HCAxis                                 3
#define FIBERPARM_HCInputMode                            4
#define FIBERPARM_HCInputChannelNum                      5
#define FIBERPARM_HCInvertSearch                         6  // replaces HCDelay
#define FIBERPARM_HCWholeWindow                          7
#define FIBERPARM_HCDataSaveMode                         8  // NEW
#define FIBERPARM_HCDelayTime                            9  // NEW
#define FIBERPARM_SRMaxRadius                           10  // needs to be checked for > 0
#define FIBERPARM_SRNumSpirals                          11
#define FIBERPARM_SRSegmentLength                       12  // needs to be checked for > 0
#define FIBERPARM_SRThreshold                           13
#define FIBERPARM_SRAxis1                               14
#define FIBERPARM_SRAxis2                               15
#define FIBERPARM_SRInputMode                           16  // may or may not be implemented in the future - GPIB?
#define FIBERPARM_SRInputChannelNum                     17
#define FIBERPARM_SRInvertSearch                        18  // replaces SRDelay
#define FIBERPARM_SRMotionType                          19  // may or may not be implemented in the future
#define FIBERPARM_SRDataSaveMode                        20  // NEW
#define FIBERPARM_SRDelayTime                           21  // NEW
#define FIBERPARM_SFEndRadius                           22  // needs to be checked for > 0
#define FIBERPARM_SFNumSpirals                          23
#define FIBERPARM_SFSegmentLength                       24  // needs to be checked for > 0
#define FIBERPARM_SFAxis1                               25
#define FIBERPARM_SFAxis2                               26
#define FIBERPARM_SFInputMode                           27  // may or may not be implemented in the future - GPIB?
#define FIBERPARM_SFInputChannelNum                     28
#define FIBERPARM_SFInvertSearch                        29  // replaces SFDelay
#define FIBERPARM_SFMotionType                          30  // Implemented
#define FIBERPARM_SFDataSaveMode                        31  // NEW
#define FIBERPARM_SFDelayTime                           32  // NEW
#define FIBERPARM_FASelectAxis1                         33
#define FIBERPARM_FASelectAxis2                         34
#define FIBERPARM_FASelectAxis3                         35
#define FIBERPARM_FASelectAxis4                         36
#define FIBERPARM_FASelectAxis5                         37
#define FIBERPARM_FASelectAxis6                         38
#define FIBERPARM_FAOffsetAxis1                         39  // needs to be checked for != 0
#define FIBERPARM_FAOffsetAxis2                         40  // needs to be checked for != 0
#define FIBERPARM_FAOffsetAxis3                         41  // needs to be checked for != 0
#define FIBERPARM_FAOffsetAxis4                         42  // needs to be checked for != 0
#define FIBERPARM_FAOffsetAxis5                         43  // needs to be checked for != 0
#define FIBERPARM_FAOffsetAxis6                         44  // needs to be checked for != 0
#define FIBERPARM_FAPosLimitAxis1                       45  // needs to be checked for >= 0
#define FIBERPARM_FAPosLimitAxis2                       46  // needs to be checked for >= 0
#define FIBERPARM_FAPosLimitAxis3                       47  // needs to be checked for >= 0
#define FIBERPARM_FAPosLimitAxis4                       48  // needs to be checked for >= 0
#define FIBERPARM_FAPosLimitAxis5                       49  // needs to be checked for >= 0
#define FIBERPARM_FAPosLimitAxis6                       50  // needs to be checked for >= 0
#define FIBERPARM_FANegLimitAxis1                       51  // needs to be checked for >= 0
#define FIBERPARM_FANegLimitAxis2                       52  // needs to be checked for >= 0
#define FIBERPARM_FANegLimitAxis3                       53  // needs to be checked for >= 0
#define FIBERPARM_FANegLimitAxis4                       54  // needs to be checked for >= 0
#define FIBERPARM_FANegLimitAxis5                       55  // needs to be checked for >= 0
#define FIBERPARM_FANegLimitAxis6                       56  // needs to be checked for >= 0
#define FIBERPARM_FATermTolerance                       57  // needs to be checked for > 0
#define FIBERPARM_FAMaxNumIterations                    58
#define FIBERPARM_FASaturationValue                     59
#define FIBERPARM_FAReturnToStart                       60
#define FIBERPARM_FAInputMode                           61  // may or may not be implemented in the future - GPIB?
#define FIBERPARM_FAInputChannelNum                     62
#define FIBERPARM_FAInvertSearch                        63  // replaces FADelay
#define FIBERPARM_FADelayTime                           64  // new
#define FIBERPARM_GCScanSize                            65  // needs to be checked for > 0
#define FIBERPARM_GCScanIncrement                       66  // needs to be checked for > 0
#define FIBERPARM_GCScanLines                           67
#define FIBERPARM_GCEdgeValue                           68
#define FIBERPARM_GCAxis1                               69
#define FIBERPARM_GCAxis2                               70
#define FIBERPARM_GCInputMode                           71  // may or may not be implemented in the future - GPIB?
#define FIBERPARM_GCInputChannelNum                     72
#define FIBERPARM_GCInvertSearch                        73  // replaces GCDelay
#define FIBERPARM_GCSingleRasterMode                    74
#define FIBERPARM_GCMotionType                          75  // NEW
#define FIBERPARM_GCDataSaveMode                        76  // Need testing
#define FIBERPARM_GCDelayTime                           77
#define FIBERPARM_CMaxDisplacement1                     78  // needs to be checked for > 0
#define FIBERPARM_CMaxDisplacement2                     79  // needs to be checked for > 0
#define FIBERPARM_CMaxDisplacement3                     80  // needs to be checked for > 0
#define FIBERPARM_CScanIncrement                        81  // needs to be checked for > 0
#define FIBERPARM_CEdgeValue                            82
#define FIBERPARM_CAxis1                                83
#define FIBERPARM_CAxis2                                84
#define FIBERPARM_CAxis3                                85
#define FIBERPARM_CInputMode                            86  // may or may not be implemented in the future - GPIB?
#define FIBERPARM_CInputChannelNum                      87
#define FIBERPARM_CInvertSearch                         88  // replaces CDelay
#define FIBERPARM_CReturnToCenter                       89
#define FIBERPARM_CDataSaveMode                         90  // NEW
#define FIBERPARM_CDelayTime                            91  // NEW
#define FIBERPARM_HCPercentDrop                         92  // NEW
#define MAX_FIBERPARMS                                  93
// C# END


// C#	CLASS=Parameter		REGION=VPP Parameters
/*** VPP Parameter data ***************************************************************************/
/*
	NOTE: CHANGES HERE MUST BE MIRRORED IN THE TaskParmInfo STRUCTURE BELOW, AND IN AERSYS\AERSYS.ODL,
		AND IN PROGRAMS\AERPARAM.PGM
*/
#define VPP_Model                                        0
#define VPP_FixedToolTip                                 1
#define VPP_UserToolTip                                  2
#define VPP_SecondToolTip                                3
#define VPP_ToolOffsetX                                  4
#define VPP_ToolOffsetY                                  5
#define VPP_ToolOffsetZ                                  6
#define VPP_ToolRotateX                                  7
#define VPP_ToolRotateY                                  8
#define VPP_ToolRotateZ                                  9
#define VPP_Tip1OffsetX                                 10
#define VPP_Tip1OffsetY                                 11
#define VPP_Tip1OffsetZ                                 12
#define VPP_Tip1RotateX                                 13
#define VPP_Tip1RotateY                                 14
#define VPP_Tip1RotateZ                                 15
#define VPP_Tip2OffsetX                                 16
#define VPP_Tip2OffsetY                                 17
#define VPP_Tip2OffsetZ                                 18
#define VPP_Tip2RotateX                                 19
#define VPP_Tip2RotateY                                 20
#define VPP_Tip2RotateZ                                 21
#define VPP_Axis1Type                                   22
#define VPP_Axis1OffsetX                                23
#define VPP_Axis1OffsetY                                24
#define VPP_Axis1OffsetZ                                25
#define VPP_Axis1RotateX                                26
#define VPP_Axis1RotateY                                27
#define VPP_Axis1RotateZ                                28
#define VPP_Axis1MotionDirection                        29
#define VPP_Axis1MotionMin                              30
#define VPP_Axis1MotionMax                              31
#define VPP_Axis1PhysicalAxis                           32
#define VPP_Axis2Type                                   33
#define VPP_Axis2OffsetX                                34
#define VPP_Axis2OffsetY                                35
#define VPP_Axis2OffsetZ                                36
#define VPP_Axis2RotateX                                37
#define VPP_Axis2RotateY                                38
#define VPP_Axis2RotateZ                                39
#define VPP_Axis2MotionDirection                        40
#define VPP_Axis2MotionMin                              41
#define VPP_Axis2MotionMax                              42
#define VPP_Axis2PhysicalAxis                           43
#define VPP_Axis3Type                                   44
#define VPP_Axis3OffsetX                                45
#define VPP_Axis3OffsetY                                46
#define VPP_Axis3OffsetZ                                47
#define VPP_Axis3RotateX                                48
#define VPP_Axis3RotateY                                49
#define VPP_Axis3RotateZ                                50
#define VPP_Axis3MotionDirection                        51
#define VPP_Axis3MotionMin                              52
#define VPP_Axis3MotionMax                              53
#define VPP_Axis3PhysicalAxis                           54
#define VPP_Axis4Type                                   55
#define VPP_Axis4OffsetX                                56
#define VPP_Axis4OffsetY                                57
#define VPP_Axis4OffsetZ                                58
#define VPP_Axis4RotateX                                59
#define VPP_Axis4RotateY                                60
#define VPP_Axis4RotateZ                                61
#define VPP_Axis4MotionDirection                        62
#define VPP_Axis4MotionMin                              63
#define VPP_Axis4MotionMax                              64
#define VPP_Axis4PhysicalAxis                           65
#define VPP_Axis5Type                                   66
#define VPP_Axis5OffsetX                                67
#define VPP_Axis5OffsetY                                68
#define VPP_Axis5OffsetZ                                69
#define VPP_Axis5RotateX                                70
#define VPP_Axis5RotateY                                71
#define VPP_Axis5RotateZ                                72
#define VPP_Axis5MotionDirection                        73
#define VPP_Axis5MotionMin                              74
#define VPP_Axis5MotionMax                              75
#define VPP_Axis5PhysicalAxis                           76
#define VPP_Axis6Type                                   77
#define VPP_Axis6OffsetX                                78
#define VPP_Axis6OffsetY                                79
#define VPP_Axis6OffsetZ                                80
#define VPP_Axis6RotateX                                81
#define VPP_Axis6RotateY                                82
#define VPP_Axis6RotateZ                                83
#define VPP_Axis6MotionDirection                        84
#define VPP_Axis6MotionMin                              85
#define VPP_Axis6MotionMax                              86
#define VPP_Axis6PhysicalAxis                           87
#define MAX_VPPPARMS                                    88
// C# END



#endif /* __AERPDEF_H__ */
