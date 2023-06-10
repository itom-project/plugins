#ifndef __AERTDEF_H__
#define __AERTDEF_H__
//
/************************************************/
/* General axis index (when its not known if its an axis or a task index) */
//

// C#	CLASS=MinimumMaximum	REGION=Axes
#define MAX_AXES        32  // can set this only to 16 or 32
// C# END


#define MAX_32_AXES         // better define me, if you set MAX_AXES to 32

#define INVALID_INDX -1

#define AER_UNIDEX_NONE             ((DWORD) (-1))

// C#	CLASS=RegistryID	REGION=Device ID
#define AER_UNIDEX_DEFAULT          0
#define AUTOMATION_3200             3200
#define AUTOMATION_3200_ETHERNET    3210
// C# END

/* Card Identifiers  - Valid Card's Should be > 0 */
#define AER_CARD_NONE      ((DWORD) (-1))

// C#	CLASS=RegistryID	REGION=Card Identifiers
#define AER_CARD_DEFAULT   0
#define AER_CARD_1         1
#define AER_CARD_2         2
#define AER_CARD_3         3
#define AER_CARD_4         4
// C# END

// Sizes
// C#	CLASS=MinimumMaximum	REGION=Strings
#define MAX_TEXT_LEN          255      // General Text buffer size
#define MAX_AERSTRING128_LEN  128
#define MAX_NAME_LEN           32      // Thread names, etc misc stuff
#define MAX_UNIT_LEN            3      // Unit string in MMI position display
#define MAX_NUMBUFF            64      // Used to hold dwords converted to char
#define MAX_REG_SZ_SIZE       255      // limitation on size of REG-SZ strings retrieved from registry
// WARNING! 12/12/06 PRD there may be dependancies in the code such that MAX_REG_SZ_SIZE and MAX_TEXT_LEN must be the same.

// C# END
// NOTE:  Use MAX_PATH for filename lengths


// C#	CLASS=Indices		REGION=Axis Indices
#define AXISINDEX_1     0
#define AXISINDEX_2     1
#define AXISINDEX_3     2
#define AXISINDEX_4     3
#define AXISINDEX_5     4
#define AXISINDEX_6     5
#define AXISINDEX_7     6
#define AXISINDEX_8     7
#define AXISINDEX_9     8
#define AXISINDEX_10    9
#define AXISINDEX_11    10
#define AXISINDEX_12    11
#define AXISINDEX_13    12
#define AXISINDEX_14    13
#define AXISINDEX_15    14
#define AXISINDEX_16    15

#ifdef MAX_32_AXES
   #define AXISINDEX_17    16
   #define AXISINDEX_18    17
   #define AXISINDEX_19    18
   #define AXISINDEX_20    19
   #define AXISINDEX_21    20
   #define AXISINDEX_22    21
   #define AXISINDEX_23    22
   #define AXISINDEX_24    23
   #define AXISINDEX_25    24
   #define AXISINDEX_26    25
   #define AXISINDEX_27    26
   #define AXISINDEX_28    27
   #define AXISINDEX_29    28
   #define AXISINDEX_30    29
   #define AXISINDEX_31    30
   #define AXISINDEX_32    31
// C# END
   #define AXISINDEX_LAST   (AXISINDEX_32)
#else
   #define AXISINDEX_LAST   (AXISINDEX_16)
#endif
#define AXISINDEX_MAX   (AXISINDEX_LAST+1)
#define AXISINDEX_NONE  (AXISINDEX)INVALID_INDX
#define AXISINDEX_INVALID  (AXISINDEX)INVALID_INDX

// C#	CLASS=Masks		REGION=Axis Masks
#define AXISMASK_1     0x00000001L
#define AXISMASK_2     0x00000002L
#define AXISMASK_3     0x00000004L
#define AXISMASK_4     0x00000008L
#define AXISMASK_5     0x00000010L
#define AXISMASK_6     0x00000020L
#define AXISMASK_7     0x00000040L
#define AXISMASK_8     0x00000080L
#define AXISMASK_9     0x00000100L
#define AXISMASK_10    0x00000200L
#define AXISMASK_11    0x00000400L
#define AXISMASK_12    0x00000800L
#define AXISMASK_13    0x00001000L
#define AXISMASK_14    0x00002000L
#define AXISMASK_15    0x00004000L
#define AXISMASK_16    0x00008000L

#ifdef MAX_32_AXES
   #define AXISMASK_17    0x00010000L
   #define AXISMASK_18    0x00020000L
   #define AXISMASK_19    0x00040000L
   #define AXISMASK_20    0x00080000L
   #define AXISMASK_21    0x00100000L
   #define AXISMASK_22    0x00200000L
   #define AXISMASK_23    0x00400000L
   #define AXISMASK_24    0x00800000L
   #define AXISMASK_25    0x01000000L
   #define AXISMASK_26    0x02000000L
   #define AXISMASK_27    0x04000000L
   #define AXISMASK_28    0x08000000L
   #define AXISMASK_29    0x10000000L
   #define AXISMASK_30    0x20000000L
   #define AXISMASK_31    0x40000000L
   #define AXISMASK_32    0x80000000L
   #define AXISMASK_ALL   0xFFFFFFFFL
// C# END
   #define AXISMASK_LAST  (AXISMASK_32)
//   #define AXISMASK_NONE  @!@!!!!
#else
   #define AXISMASK_LAST  (AXISMASK_16)
   #define AXISMASK_ALL   0x0000FFFFL
//   #define AXISMASK_NONE  0x00010000L    /* Can't be zero */
#endif

#define IndexToMask(i)  (1<<(i))
#define IsValidAxisIndex(ax)   ( ((AXISINDEX)ax < MAX_AXES) ? 1 : 0 )
#define IsInvalidAxisIndex(ax) ( ((AXISINDEX)ax >= MAX_AXES) ? 1 : 0 )
#define MaskToIndex(iiii,mmm) {iiii=AXISINDEX_1;mBit=mmm>>1;while(mBit) {mBit>>=1;iiii++;}}
#define NumAxesInMask(iiii,mmm) {iiii=0;mBit=mmm;while(mBit) {if (mBit&0x1){iiii++;};mBit>>=1;}}

/* Physical axis index */
#define PHYSAXISINDEX_1   AXISINDEX_1      //  these must be in ascending order, and start from 0
#define PHYSAXISINDEX_2   AXISINDEX_2
#define PHYSAXISINDEX_3   AXISINDEX_3
#define PHYSAXISINDEX_4   AXISINDEX_4
#define PHYSAXISINDEX_5   AXISINDEX_5
#define PHYSAXISINDEX_6   AXISINDEX_6
#define PHYSAXISINDEX_7   AXISINDEX_7
#define PHYSAXISINDEX_8   AXISINDEX_8
#define PHYSAXISINDEX_9   AXISINDEX_9
#define PHYSAXISINDEX_10  AXISINDEX_10
#define PHYSAXISINDEX_11  AXISINDEX_11
#define PHYSAXISINDEX_12  AXISINDEX_12
#define PHYSAXISINDEX_13  AXISINDEX_13
#define PHYSAXISINDEX_14  AXISINDEX_14
#define PHYSAXISINDEX_15  AXISINDEX_15
#define PHYSAXISINDEX_16  AXISINDEX_16

#ifdef MAX_32_AXES
   #define PHYSAXISINDEX_17    AXISINDEX_17
   #define PHYSAXISINDEX_18    AXISINDEX_18
   #define PHYSAXISINDEX_19    AXISINDEX_19
   #define PHYSAXISINDEX_20    AXISINDEX_20
   #define PHYSAXISINDEX_21    AXISINDEX_21
   #define PHYSAXISINDEX_22    AXISINDEX_22
   #define PHYSAXISINDEX_23    AXISINDEX_23
   #define PHYSAXISINDEX_24    AXISINDEX_24
   #define PHYSAXISINDEX_25    AXISINDEX_25
   #define PHYSAXISINDEX_26    AXISINDEX_26
   #define PHYSAXISINDEX_27    AXISINDEX_27
   #define PHYSAXISINDEX_28    AXISINDEX_28
   #define PHYSAXISINDEX_29    AXISINDEX_29
   #define PHYSAXISINDEX_30    AXISINDEX_30
   #define PHYSAXISINDEX_31    AXISINDEX_31
   #define PHYSAXISINDEX_32    AXISINDEX_32
   #define PHYSAXIS_MASK_ALL       AXISMASK_ALL
#else
   #define PHYSAXIS_MASK_ALL       AXISMASK_ALL
#endif
#define PHYSAXISINDEX_INVALID  (AXISINDEX)INVALID_INDX

/************************************************/
/* Task axis index */
#define TASKAXISINDEX_1   AXISINDEX_1      //  these must be in ascending order, and start from 0
#define TASKAXISINDEX_2   AXISINDEX_2
#define TASKAXISINDEX_3   AXISINDEX_3
#define TASKAXISINDEX_4   AXISINDEX_4
#define TASKAXISINDEX_5   AXISINDEX_5
#define TASKAXISINDEX_6   AXISINDEX_6
#define TASKAXISINDEX_7   AXISINDEX_7
#define TASKAXISINDEX_8   AXISINDEX_8
#define TASKAXISINDEX_9   AXISINDEX_9
#define TASKAXISINDEX_10  AXISINDEX_10
#define TASKAXISINDEX_11  AXISINDEX_11
#define TASKAXISINDEX_12  AXISINDEX_12
#define TASKAXISINDEX_13  AXISINDEX_13
#define TASKAXISINDEX_14  AXISINDEX_14
#define TASKAXISINDEX_15  AXISINDEX_15
#define TASKAXISINDEX_16  AXISINDEX_16

#ifdef MAX_32_AXES
   #define TASKAXISINDEX_17    AXISINDEX_17
   #define TASKAXISINDEX_18    AXISINDEX_18
   #define TASKAXISINDEX_19    AXISINDEX_19
   #define TASKAXISINDEX_20    AXISINDEX_20
   #define TASKAXISINDEX_21    AXISINDEX_21
   #define TASKAXISINDEX_22    AXISINDEX_22
   #define TASKAXISINDEX_23    AXISINDEX_23
   #define TASKAXISINDEX_24    AXISINDEX_24
   #define TASKAXISINDEX_25    AXISINDEX_25
   #define TASKAXISINDEX_26    AXISINDEX_26
   #define TASKAXISINDEX_27    AXISINDEX_27
   #define TASKAXISINDEX_28    AXISINDEX_28
   #define TASKAXISINDEX_29    AXISINDEX_29
   #define TASKAXISINDEX_30    AXISINDEX_30
   #define TASKAXISINDEX_31    AXISINDEX_31
   #define TASKAXISINDEX_32    AXISINDEX_32
   #define TASKAXIS_MASK_ALL       AXISMASK_ALL
#endif
#define TASKAXISINDEX_MAX  AXISINDEX_LAST+1
#define TASKAXIS_MASK_ALL       AXISMASK_ALL
#define TASKAXISINDEX_INVALID  (AXISINDEX)INVALID_INDX

/* Task Index definitions */
// C#	CLASS=Indices		REGION=Task Indices
#define TASKINDEX_1     0        //  these must be in ascending order, and start from 0
#define TASKINDEX_2     1
#define TASKINDEX_3     2
#define TASKINDEX_4     3
// C# END

// C#	CLASS=MinimumMaximum	REGION=Tasks
#define MAX_TASKS      4     // number of tasks that can run programs in the SMC
#define MAX_TASKS_VB   4     // number of tasks that can be displayed (are visible) to user
// C# END

#define TASKINDEX_NONE  (TASKINDEX) INVALID_INDX    // must be different then all other task, and different from MAX_TASKS

#define IsValidTaskIndex(iTask)   ( ((TASKINDEX)iTask< MAX_TASKS) ? 1 : 0 )
#define IsInvalidTaskIndex(iTask) ( ((TASKINDEX)iTask>=MAX_TASKS) ? 1 : 0 )

// C#	CLASS=Masks		REGION=Task Masks
#define TASKMASK_NONE    0x00000000L
#define TASKMASK_ALL     0x0000000FL
#define TASKMASK_1       0x00000001L
#define TASKMASK_2       0x00000002L
#define TASKMASK_3       0x00000004L
#define TASKMASK_4       0x00000008L
#define TASKMASK_ALL_VB  0x0000000FL
// C# END

/************************************************/
/* Call stack parameters index */
#define CSPARMINDEX_1     AXISINDEX_1                     //  these must be in ascending order, and start from 0
#define CSPARMINDEX_2     AXISINDEX_2
#define CSPARMINDEX_3     AXISINDEX_3
#define CSPARMINDEX_4     AXISINDEX_4
#define CSPARMINDEX_5     AXISINDEX_5
#define CSPARMINDEX_6     AXISINDEX_6
#define CSPARMINDEX_7     AXISINDEX_7
#define CSPARMINDEX_8     AXISINDEX_8
#define CSPARMINDEX_9     AXISINDEX_9
#define CSPARMINDEX_10    AXISINDEX_10
#define CSPARMINDEX_11    AXISINDEX_11
#define CSPARMINDEX_12    AXISINDEX_12
#define CSPARMINDEX_13    AXISINDEX_13
#define CSPARMINDEX_14    AXISINDEX_14
#define CSPARMINDEX_15    AXISINDEX_15
#define CSPARMINDEX_16    AXISINDEX_16

#ifdef MAX_32_AXES
   #define CSPARMINDEX_17    AXISINDEX_17
   #define CSPARMINDEX_18    AXISINDEX_18
   #define CSPARMINDEX_19    AXISINDEX_19
   #define CSPARMINDEX_20    AXISINDEX_20
   #define CSPARMINDEX_21    AXISINDEX_21
   #define CSPARMINDEX_22    AXISINDEX_22
   #define CSPARMINDEX_23    AXISINDEX_23
   #define CSPARMINDEX_24    AXISINDEX_24
   #define CSPARMINDEX_25    AXISINDEX_25
   #define CSPARMINDEX_26    AXISINDEX_26
   #define CSPARMINDEX_27    AXISINDEX_27
   #define CSPARMINDEX_28    AXISINDEX_28
   #define CSPARMINDEX_29    AXISINDEX_29
   #define CSPARMINDEX_30    AXISINDEX_30
   #define CSPARMINDEX_31    AXISINDEX_31
   #define CSPARMINDEX_32    AXISINDEX_32
#endif

/* The remaindor are call stack paramaters. These must remain in consecutive order */
#define CSPARMINDEX_S_1    (AXISINDEX_MAX)
#define CSPARMINDEX_S_2    (AXISINDEX_MAX+1)
#define CSPARMINDEX_S_3    (AXISINDEX_MAX+2)
#define CSPARMINDEX_S_4    (AXISINDEX_MAX+3)
#define CSPARMINDEX_S_5    (AXISINDEX_MAX+4)
#define CSPARMINDEX_S_6    (AXISINDEX_MAX+5)
#define CSPARMINDEX_S_7    (AXISINDEX_MAX+6)
#define CSPARMINDEX_S_8    (AXISINDEX_MAX+7)
#define CSPARMINDEX_S_9    (AXISINDEX_MAX+8)
#define CSPARMINDEX_S_10   (AXISINDEX_MAX+9)
#ifdef MAX_32_AXES
   #define CSPARMINDEX_S_11   (AXISINDEX_MAX+10)
   #define CSPARMINDEX_S_12   (AXISINDEX_MAX+11)
   #define CSPARMINDEX_S_13   (AXISINDEX_MAX+12)
   #define CSPARMINDEX_S_14   (AXISINDEX_MAX+13)
   #define CSPARMINDEX_S_15   (AXISINDEX_MAX+14)
   #define CSPARMINDEX_S_16   (AXISINDEX_MAX+15)
   #define CSPARMINDEX_S_17   (AXISINDEX_MAX+16)
   #define CSPARMINDEX_S_18   (AXISINDEX_MAX+17)
   #define CSPARMINDEX_S_19   (AXISINDEX_MAX+18)
   #define CSPARMINDEX_S_20   (AXISINDEX_MAX+19)
   #define MAX_CSPARMS        (AXISINDEX_MAX+20)    // limited to 0xFF
#else
   #define MAX_CSPARMS        (AXISINDEX_MAX+10)    // limited to 0xFF
#endif
#define IsInvalidCSParmIndex(iCSParm) ( (iCSParm>=MAX_CSPARMS) ? 1 : 0 )
#ifdef _DEBUG
#ifdef  MAX_32_AXES         // this only works for 16 or 32 axes
   #define MASK1_ALL    0xFFFFFFFF
   #define MASK2_ALL    0x000003FF
#else
   #define MASK1_ALL    0x03FFFFFF
   #define MASK2_ALL    0x0
#endif
#endif

// But only first 5 allowed as parameters
#define O_LETTER CSPARMINDEX_S_1
#define P_LETTER CSPARMINDEX_S_2
#define Q_LETTER CSPARMINDEX_S_3
#define R_LETTER CSPARMINDEX_S_4
#define L_LETTER CSPARMINDEX_S_5
#ifdef MAX_32_AXES
   #define I_LETTER CSPARMINDEX_S_6
   #define J_LETTER CSPARMINDEX_S_7
   #define K_LETTER CSPARMINDEX_S_8
#endif

#ifdef MAX_32_AXES
   #define MAX_PARM_ARGS       (AXISINDEX_MAX+8)
#else
   #define MAX_PARM_ARGS       (AXISINDEX_MAX+5)
#endif

#define IsInvalidParmArgIndex(iParm) ( (iParm>=MAX_PARM_ARGS) ? 1 : 0 )

// And compiler likes to reserve the rest for internal use (temporary variables)
#ifdef MAX_32_AXES
   #define A32CMPLR_TEMPVAR_FIRST CSPARMINDEX_S_13
   #define A32CMPLR_TEMPVAR_LAST  CSPARMINDEX_S_20
#else
   #define A32CMPLR_TEMPVAR_FIRST CSPARMINDEX_S_6
   #define A32CMPLR_TEMPVAR_LAST  CSPARMINDEX_S_10
#endif

/************************************************/
/* Spindle index */
#define SPINDLEINDEX_1    0
#define SPINDLEINDEX_2    1
#define SPINDLEINDEX_3    2
#define SPINDLEINDEX_4    3
#define MAX_SPINDLES      4

#define IsInvalidSpindleIndex(iSpindle) ( (iSpindle>=MAX_SPINDLES) ? 1 : 0 )

/************************************************/
/* Real digital I/O definitions */
#define MAX_DRIVE_DIGITAL      (MAX_AXES*DIG_BITS_IN_EACH_DRIVE)
//
//#define ANALOGINDEX_1   0
//#define ANALOGINDEX_2   1
//#define ANALOGINDEX_3   2
//#define ANALOGINDEX_4   3
//#define ANALOGINDEX_5   4
//#define ANALOGINDEX_6   5
//#define ANALOGINDEX_7   6
//#define ANALOGINDEX_8   7
//#define ANALOGINDEX_9   8
//#define ANALOGINDEX_10  9
//#define ANALOGINDEX_11  10
//#define ANALOGINDEX_12  11
//#define ANALOGINDEX_13  12
//#define ANALOGINDEX_14  13
//#define ANALOGINDEX_15  14
//#define ANALOGINDEX_16  15
//#define ANALOGINDEX_MAX   (ANALOGINDEX_16)
#define MAX_ANALOG      (MAX_AXES*ANALOGS_IN_EACH_DRIVE)        // max inputs (and max outputs)

//
//#define ANALOG_MASK     0x0000FFFFL
//#define ANALOGMASK_1    0x00000001L
//#define ANALOGMASK_2    0x00000002L
//#define ANALOGMASK_3    0x00000004L
//#define ANALOGMASK_4    0x00000008L
//#define ANALOGMASK_5    0x00000010L
//#define ANALOGMASK_6    0x00000020L
//#define ANALOGMASK_7    0x00000040L
//#define ANALOGMASK_8    0x00000080L
//#define ANALOGMASK_9    0x00000100L
//#define ANALOGMASK_10   0x00000200L
//#define ANALOGMASK_11   0x00000400L
//#define ANALOGMASK_12   0x00000800L
//#define ANALOGMASK_13   0x00001000L
//#define ANALOGMASK_14   0x00002000L
//#define ANALOGMASK_15   0x00004000L
//#define ANALOGMASK_16   0x00008000L


/* Binary/Registor I/O */
// C#	CLASS=MinimumMaximum	   REGION=VirtualBinaryIO
#define MAX_VIRT_BINARY_BITS        2048
// C# END
#define MAX_VIRT_BINARY_WORDS       (MAX_VIRT_BINARY_BITS/16) /* 64 */
// C#	CLASS=MinimumMaximum	REGION=VirtualRegisterIO
#define MAX_VIRT_REGISTERS          896
// C# END

#define MAX_DRIVE_WORDS             MAX_AXES         // one for each drive
#define MAX_ETHER_IO_READ           8

// Units constants
// DO NOT CHANGE ORDER !!
#define AER_UNITTYPE_NONE    -1
#define AER_UNITTYPE_METRIC   0
#define AER_UNITTYPE_ENGLISH  1
#define AER_UNITTYPE_COUNTS   2
#define AER_UNITTYPE_ROTARY   3
//
/************************************************/
/*
   Axis status bits (FAULT and STATUS bits defined in AERC32.H )
   (axis status is internal to controller, not on drive, as opposed to drive status)
*/
/*
   NOTE: CHANGES HERE MUST BE REFLECTED IN:
         FIRMWARE\ASWPARM.C   (albGetProgErrorMsg())
*/
// C#	CLASS=Status		REGION=Axis Status Masks
#define MAXS_STATUS_HOMED              0x00000001   // have homed succesfully since last reset/home command/encoder fault
#define MAXS_STATUS_PROFILING          0x00000002   // On when generating command points from a G0/G1/G2/G3 (see below)
#define MAXS_STATUS_MOVEDONE           0x00000004   // TRUE if motion is "done" (see below)
#define MAXS_STATUS_CMDVALID           0x00000008   // sending valid pos/vel command to drive
#define MAXS_STATUS_HOMING             0x00000010   // axis is homing now
#define MAXS_STATUS_DRIVECONTROLLED    0x00000020   // drive is controlling motion
#define MAXS_STATUS_SOFTWARE_ESTOP     0x00000040   // software ESTOP occurred (see TASKPARM_SoftEStopInput)
#define MAXS_STATUS_SOFTWARE_USER      0x00000080   // ???
#define MAXS_STATUS_JOGGING            0x00000100   // On when generating command points from a INDEX/FREERUN
#define MAXS_STATUS_DRIVECONTROL_PEND  0x00000200   // motion command sent to drive, but drive not taken control yet
#define MAXS_STATUS_SIMUL              0x00000400   // SMC simul
#define MAXS_STATUS_VELTIMECONST       0x00000800   // 1=on
#define MAXS_STATUS_INTERRUPT          0x00001000   // interrupt to frontend sent
#define MAXS_STATUS_ALIVE              0x00002000   // axis is connected to firewire (not virtual)
#define MAXS_STATUS_1DCALIBRATION      0x00004000   // axis is being 1 dimensionally calibrated
#define MAXS_STATUS_2DCALIBRATION      0x00008000   // axis is being 2 dimensionally calibrated
#define MAXS_STATUS_MASTER_SLAVE       0x00010000   // axis is being cammed
#define MAXS_STATUS_SLEW_ACTIVE        0x00020000   // axis is being slewed
#define MAXS_STATUS_BACKLASH_ACTIVE    0x00040000   // axis has backlash comp
#define MAXS_STATUS_GAINCALIBRATION    0x00080000   // axis is being gain calibrated
#define MAXS_STATUS_INPOSITION2        0x00100000   // secondary in-position status bit
#define MAXS_STATUS_BLOCKED            0x00200000   // application has blocked motion on this axis
#define MAXS_STATUS_DONE_NOTINPOS      0x00400000   // move is "mostly" done (i.e. done but not in position)
#define MAXS_STATUS_CLAMPED            0x00800000   // axis is clamped on a software limit

// C# END

// C#	CLASS=Status		REGION=Axis Status Bits
#define BAXS_STATUS_HOMED              0x00
#define BAXS_STATUS_PROFILING          0x01
#define BAXS_STATUS_MOVEDONE           0x02
#define BAXS_STATUS_CMDVALID           0x03
#define BAXS_STATUS_HOMING             0x04
#define BAXS_STATUS_DRIVECONTROLLED    0x05
#define BAXS_STATUS_SOFTWARE_ESTOP     0x06
#define BAXS_STATUS_SOFTWARE_USER      0x07
#define BAXS_STATUS_JOGGING            0x08
#define BAXS_STATUS_DRIVECONTROL_PEND  0x09
#define BAXS_STATUS_SIMUL              0x0A
#define BAXS_STATUS_VELTIMECONST       0x0B
#define BAXS_STATUS_INTERRUPT          0x0C
#define BAXS_STATUS_ALIVE              0x0D
#define BAXS_STATUS_1DCALIBRATION      0x0E
#define BAXS_STATUS_2DCALIBRATION      0x0F
#define BAXS_STATUS_MASTER_SLAVE       0x10
#define BAXS_STATUS_SLEW_ACTIVE        0x11
#define BAXS_STATUS_BACKLASH_ACTIVE    0x12
#define BAXS_STATUS_GAINCALIBRATION    0x13
#define BAXS_STATUS_INPOSITION2        0x14
#define BAXS_STATUS_BLOCKED            0x15
#define BAXS_STATUS_DONE_NOTINPOS      0x16
#define BAXS_STATUS_CLAMPED            0x17
// C# END

//
// Motion is "profiling" when
//   1. The axis is in the profile mask, and the profile queue is not empty
//
// Motion is not "profiling" when
//   1. Drive is disabled
//   2. Call to AbortAxes() made
//
// Motion is "done" when either:
//   1. Falling edge of "motionActive" drive bit seen from drive, OR
//   2. Motion queue empties (goes from not-empty to empty state) for a task that owns this axes.
//   3. Call to AbortAxes() made
//
// Motion is not "done" when either:
//   1. Rising edge of "motionActive" drive bit seen from drive, OR
//   2. Motion queue is non empty for a task that owns this axes
//   3. Command sent to drive that will force a rising edge of "motionActive" drive bit (when drive gets it)
//

/**************************************************************/
/* Options for third parameter of AerStatusGetStatusWord(hAerCtrl, 0, AER_STATUS_*, &dwStatusword) */
#define AER_STATUS_FAULT                          0x01     /* Fault status returned from the drive   */
#define AER_STATUS_DRIVE                          0x02     /* Axis status returned from the drive    */
#define AER_STATUS_AXIS							        0x03     /* Axis status maintained locally         */

/************************************************/
/* Task definitions
*/

#define TASKEXEC_DEFAULT         (DWORD) INVALID_INDX  /* Execute default Mode (Run/Step) */
// C#	CLASS=CNCProgram	REGION=Program Execute Constants
#define TASKEXEC_RUN_INTO        0  /* run into subroutines */
#define TASKEXEC_STEP_INTO       1  /* step into subroutines */
#define TASKEXEC_STEP_OVER       2  /* step over subroutines */
#define TASKEXEC_RUN_OVER        3  /* run over subroutines */
// C# END
#define TASKEXEC_RUN             TASKEXEC_RUN_INTO


/**************************************************************/
/* Pointer Type Definitions (restricted to 2 bytes !)
*/

/* Pointer type defines */
/* WARNING ! PTRTYPE_xxxx constants are limited to 256, because they are stored in a byte (PTR_DATA.bType) */
#define  PTRTYPE_NULL                  0x00      // MUST be 0

/* Call Stack Parameter Types */
         /* Variables */
#define  PTRTYPE_CSP_CSPARM_VAR        0x01

/* Axis Point Pointer Types */
         /* Variables */
#define  PTRTYPE_APT_GLOBAL_VAR        0x02
#define  PTRTYPE_APT_TASK_VAR          0x03
#define  PTRTYPE_APT_CSPARM_VAR        0x04

/* String Pointer Types */
         /* Variables */
#define  PTRTYPE_STR_GLOBAL_VAR        0x05
#define  PTRTYPE_STR_TASK_VAR          (PTRTYPE_STR_GLOBAL_VAR+1)
#define  PTRTYPE_STR_PROGRAM_VAR       (PTRTYPE_STR_GLOBAL_VAR+2)

/* Numeric Pointer Types */
         /* Variables */
#define  PTRTYPE_DBL_GLOBAL_VAR        (PTRTYPE_STR_GLOBAL_VAR+3)
#define  PTRTYPE_DBL_TASK_VAR          (PTRTYPE_STR_GLOBAL_VAR+4)     /* needs Task Axis index in addition    */
#define  PTRTYPE_DBL_PROGRAM_VAR       (PTRTYPE_STR_GLOBAL_VAR+5)
#define  PTRTYPE_DBL_CSPARM_VAR        0x0B     /* needs CSParm index in addition */
#define  PTRTYPE_MASK_CSPARM_VAR       0x0C     /* Read only , optional CSParm index    */
#define  PTRTYPE_DBL_APT_GLOBAL_VAR    0x0D     /* Task Axis index                      */
#define  PTRTYPE_MASK_APT_GLOBAL_VAR   0x0E     /* Read only , optional Task Axis index */
#define  PTRTYPE_DBL_APT_TASK_VAR      0x0F     /* needs Task Axis index in addition    */
#define  PTRTYPE_MASK_APT_TASK_VAR     0x10     /* Read only , optional Task Axis index */

#define  DBL_TASK_VAR_LASTONE          0xFFFE   /* Special flag to indicate last allocated task variable */

         /* Parameters */
#define  PTRTYPE_DBL_AXIS_PARM         0x11     /* needs Axis index in addition */
#define  PTRTYPE_DBL_TASK_PARM         0x12     /* needs Task index in addition */
#define  PTRTYPE_DBL_FIBER_PARM        0x13     /* needs Task index in addition */
#define  PTRTYPE_DBL_GLOBAL_PARM       0x14
#define  PTRTYPE_DBL_VPP_PARM          0x15

         /* Virtual I/O */
// C#	CLASS=PtrType	REGION=Virtual IO
#define  PTRTYPE_BYTE_BI_VIO           0x16
#define  PTRTYPE_BYTE_BO_VIO           0x17
#define  PTRTYPE_WORD_RI_VIO           0x18
#define  PTRTYPE_WORD_RO_VIO           0x19
// C# END

         /* Drive I/O */
// C#	CLASS=PtrType	REGION=Drive IO
#define  PTRTYPE_BYTE_DI_IO            0x1A     /* needs Axis index in addition */
#define  PTRTYPE_BYTE_DO_IO            0x1B     /* needs Axis index in addition */
#define  PTRTYPE_WORD_WI_IO            0x1C     /* needs Axis index in addition */
#define  PTRTYPE_WORD_WO_IO            0x1D     /* needs Axis index in addition */
#define  PTRTYPE_DBL_AI_IO             0x1E     /* needs Axis index in addition */
#define  PTRTYPE_DBL_AO_IO             0x1F     /* needs Axis index in addition */
// C# END

         /* Ethernet I/O */
// C#	CLASS=PtrType	REGION=Ethernet IO
#define  PTRTYPE_BYTE_EDI_IO           0x20     /* needs Axis index in addition */
#define  PTRTYPE_BYTE_EDO_IO           0x21     /* needs Axis index in addition */
#define  PTRTYPE_WORD_ERI_IO           0x22     /* needs Axis index in addition */
#define  PTRTYPE_WORD_ERO_IO           0x23     /* needs Axis index in addition */
#define  PTRTYPE_WORD_EPI_IO           0x24     /* needs Axis index in addition */
#define  PTRTYPE_WORD_EPO_IO           0x25     /* needs Axis index in addition */
#define  PTRTYPE_WORD_ECI_IO           0x26     /* needs Axis index in addition */

#define  PTRTYPE_WORD_ERIS_IO          0x27     /* needs Axis index in addition */
#define  PTRTYPE_WORD_EROS_IO          0x28     /* needs Axis index in addition */
#define  PTRTYPE_WORD_ERIF_IO          0x29     /* needs Axis index in addition */
#define  PTRTYPE_WORD_EROF_IO          0x2A     /* needs Axis index in addition */
#define  PTRTYPE_WORD_ERID_IO          0x2B     /* needs Axis index in addition */
#define  PTRTYPE_WORD_EROD_IO          0x2C     /* needs Axis index in addition */

#define  PTRTYPE_WORD_EPIS_IO          0x2D     /* needs Axis index in addition */
#define  PTRTYPE_WORD_EPOS_IO          0x2E     /* needs Axis index in addition */
#define  PTRTYPE_WORD_EPIF_IO          0x2F     /* needs Axis index in addition */
#define  PTRTYPE_WORD_EPOF_IO          0x30     /* needs Axis index in addition */
#define  PTRTYPE_WORD_EPID_IO          0x31     /* needs Axis index in addition */
#define  PTRTYPE_WORD_EPOD_IO          0x32     /* needs Axis index in addition */
// C# END

#define  PTRTYPE_BYTE_PORT_IO          0x33     /* needs Axis index in addition */
#define  PTRTYPE_DBL_STAT_ITEM         0x34     /* needs "Axis" index in addition */

         /* Virtual I/O */
// C#	CLASS=PtrType	REGION=Virtual IO
#define  PTRTYPE_WORD_RIS_VIO          0x40
#define  PTRTYPE_WORD_ROS_VIO          0x41
#define  PTRTYPE_WORD_RIF_VIO          0x42
#define  PTRTYPE_WORD_ROF_VIO          0x43
#define  PTRTYPE_WORD_RID_VIO          0x44
#define  PTRTYPE_WORD_ROD_VIO          0x45
/* WARNING ! PTRTYPE_xxxx constants are limited to 256, because they are stored in a byte (PTR_DATA.bType) */
// C# END

/* Additional index types */
/* WARNING ! PTRINDEXTYPE_xxxx constants are limited to 256, because they are stored in a byte (PTR_DATA.Index.Const.bType) */

#define  PTRINDEXTYPE_MAX_BITS        4       // upper this many bits reserved for other use
#define  PTRINDEXTYPE_NULL            0x0
#define  PTRINDEXTYPE_PHYSAXIS        0x1
#define  PTRINDEXTYPE_TASKAXIS        0x2
#define  PTRINDEXTYPE_CSPARM          0x3
#define  PTRINDEXTYPE_MAX             (1<<PTRINDEXTYPE_MAX_BITS)    // PTRINDEXTYPE_xxxx can not be this value or larger then this !
#define  PTRINDEXMASK_VAR             0xF000

/* Math operand */
#define MATHDBLTYPE_VALUE            1

/* Math operations */
// C#	CLASS=MathOperators		REGION=Operation
#define MATHDBLTYPE_ADD              2
#define MATHDBLTYPE_SUBTRACT         3
#define MATHDBLTYPE_MULTIPLY         4
#define MATHDBLTYPE_DIVIDE           5
#define MATHDBLTYPE_POWER            6
#define MATHDBLTYPE_MOD              7
#define MATHDBLTYPE_LOG_AND          8
#define MATHDBLTYPE_LOG_OR           9
#define MATHDBLTYPE_LOG_NAND         10
#define MATHDBLTYPE_LOG_NOR          11
#define MATHDBLTYPE_BIT_AND          12
#define MATHDBLTYPE_BIT_OR           13
#define MATHDBLTYPE_BIT_XOR          14
#define MATHDBLTYPE_BIT_SHIFT_LEFT   15
#define MATHDBLTYPE_BIT_SHIFT_RIGHT  16
#define MATHDBLTYPE_BIT_NAND         17
#define MATHDBLTYPE_BIT_NOR          18
#define MATHDBLTYPE_BIT_NXOR         19
// C# END

// C#	CLASS=MathOperators		REGION=Comparision
#define CONDTYPE_EQUAL               24
#define CONDTYPE_NOT_EQUAL           25
#define CONDTYPE_GREATER             26
#define CONDTYPE_GREATER_EQUAL       27
#define CONDTYPE_LESS                28
#define CONDTYPE_LESS_EQUAL          29
// C# END

#define MATHDBLTYPE_LOG              30
#define MATHDBLTYPE_ATAN2            31
#define MATHDBLTYPEMAX_TWO_OPERANDS  32
/* Numbers up to 31 are reserved for other two operand floating point operations */

#define MATHDBLTYPE_NEGATE           32
#define MATHDBLTYPE_ABSOLUTE         33
#define MATHDBLTYPE_INTEGER          34
#define MATHDBLTYPE_FRACTION         35
#define MATHDBLTYPE_SQRT             36
#define MATHDBLTYPE_EXPONENTIAL      37
#define MATHDBLTYPE_SIN              38
#define MATHDBLTYPE_COS              39
#define MATHDBLTYPE_TAN              40
#define MATHDBLTYPE_ASIN             41
#define MATHDBLTYPE_ACOS             42
#define MATHDBLTYPE_ATAN             43
#define MATHDBLTYPE_BIT_NOT          44
#define MATHDBLTYPE_LOG_NOT          45
#define MATHDBLTYPE_IS_AVAIL         46
#define MATHDBLTYPE_IS_MOVING        47
#define MATHDBLTYPE_IS_INPOS         48
#define MATHDBLTYPE_PLUSONE          49
#define MATHDBLTYPE_MINONE           50
#define MATHDBLTYPE_RAND             51
#define MATHDBLTYPEMAX_ONE_OPERANDS  64
/* Numbers up to 63 are reserved for other one operand floating point operations */

/* String operations */
#define MATHSTRTYPE_LEN             100
#define MATHSTRTYPE_FIND            101
#define MATHSTRTYPE_CHAR            102
#define MATHSTRTYPE_CMP             103
#define MATHSTRTYPE_MID             104
#define MATHSTRTYPE_S2A             105
#define MATHSTRTYPE_S2D             106
#define MATHSTRTYPE_D2S             107
#define MATHSTRTYPE_UPR             108
#define MATHSTRTYPE_LWR             109


/************************************************/
// See AerErrxxxx()
// Severity levels MUS T be in ascending order, and lowest one MUST be zero.
/* Severity levels ... */

// C#	CLASS=CNCProgram		REGION=Compiler Error Messages
#define AERERR_TYPE_MSG    0
#define AERERR_TYPE_WARN   1
#define AERERR_TYPE_ERROR  2
#define AERERR_TYPE_NONE   3
// C# END

/************************************************/
// See AerCompilerxxxx()  Compiler mode masks
//  1. Use AERCMPLR_DEFAULT for fastest possible compile. The "compile" actually consists of two parts, "Compiling into object code"
//     and "downloading the object code". By default, the compiler writes the object code into
//     a ".OGM" file (same path the original source file is). Then on subsequent recompiles, if no
//     source code that went into the compile has changed since writing the .OGM file,
//     it only does the download of the OGM. Downloading the OGM is about ten times faster then compiling into
//     an OGM. But the .OGM file uses about 10 times the disk space as the source code.
//     Also, by default, if it can use the OGM, it does not read in the souce code. (that also takes some time)
//     Therefore, calls to AerCompilerGetLineText() to see source code, will fail.
//     ALSO the above behavior is followed regardless of wether the program has compile errors or not.
//  2. Use AERCMPLR_NO_READ_OBJ and AERCMPLR_NO_WRITE_OBJ toegather to not read/write OGMS
//       (better use both toegather because then after first OGM creation, the source will never be used, even if it changes)
//  3. Use AERCMPLR_SRC_WITH_OBJ to force it to read source anyway, this takes more time but allows you to call
//        AerCompilerGetLineText()
//  4. AERCMPLR_NO_USE_SRC can be used to force it to look at the OGM always (no out-of date-check)
//  5. AERCMPLR_DVARS_ONLY only reads in the text, and compiles up to the line after the last "dvar"
//         (so AerCompilerGetLineText() and  AerCompilerGetProgVar() can be called)
//
#define AERCMPLR_MODE_DEFAULT       0x0    // Uses object files, only recompile from source when necessary, don't make listing
#define AERCMPLR_MODE_PREPROC_ONLY  0x1    // Only run preprocessor
#define AERCMPLR_MODE_NO_READ_OBJ   0x2    // Always recompile it from source, even if object availiable
#define AERCMPLR_MODE_NO_USE_SRC    0x4    // Must read object, never use source (source better exist)
#define AERCMPLR_MODE_SRC_WITH_OBJ  0x10   // Forces us to read source file, even if object is there and uptodate
                                      // (need this if calling AerCompilerGetLineText() later)
#define AERCMPLR_MODE_NO_WRITE_OBJ  0x20   // Never writes objects
#define AERCMPLR_MODE_DVARS_ONLY    0x40   // Only compiles variable declarations (so to use AerCompilerGetProgVar*() functions) on it
#define AERCMPLR_MODE_FAST          0x80   // Limited error information
#define AERCMPLR_MODE_NO_VERS_CHECK 0x100
#define AERCMPLR_MODE_IS_STICKY     0x200

// Note: the below are deprecated as of 2.18 (01/09/07) and should not be used
#define AERCMPLR_DEFAULT       0x0    // Uses object files, only recompile from source when necessary, don't make listing
#define AERCMPLR_PREPROC_ONLY  0x1    // Only run preprocessor
#define AERCMPLR_NO_READ_OBJ   0x2    // Always recompile it from source, even if object availiable
#define AERCMPLR_NO_USE_SRC    0x4    // Must read object, never use source (source better exist)
#define AERCMPLR_SRC_WITH_OBJ  0x10   // Forces us to read source file, even if object is there and uptodate
                                      // (need this if calling AerCompilerGetLineText() later)
#define AERCMPLR_NO_WRITE_OBJ  0x20   // Never writes objects
#define AERCMPLR_DVARS_ONLY    0x40   // Only compiles variable declarations (so to use AerCompilerGetProgVar*() functions) on it
#define AERCMPLR_FAST          0x80   // Limited error information
#define AERCMPLR_NO_VERSION_CHECK 0x100

/************************************************/
/* For CODETYPE_RETURN */
#define RETURNTYPE_NULL          0
#define RETURNTYPE_START         1
#define RETURNTYPE_INTERRUPT     2
#define RETURNTYPE_END           3
#define RETURNTYPE_OFFSET        4
#define RETURNTYPE_MAX          15     // no more because of below mask
#define RETURNTYPE_MODE_BITS   0xF

/************************************************/
// See AerToolxxxx()
#define AERTOOL_F_ENGLISH        0x0000001   /* units are english */
#define AERTOOL_F_INUSE          0x0000002   /* tool can be used */
#define AERTOOL_F_FORCE_UNITS    0x0000004   /* when tool is activated */
                                             /*   it must be in proper units */
#define AERTOOL_F_USER_DIAMETER  0x0008000   /* Used by Interface/Display only */
                                             /* Values (Radius/Wear) are Diameter */
#define AERTOOL_F_VALID          0x8000000   /* Internal - the tool has been loaded */

/************************************************/
/* Other */

#define AER_PROG_MSG_SIZE   20

#define MAX_INT_EVENTS        10

#define MAX_AXISCAM_TABLES                   100   // MAX_AXES*2

/* /////////////////////////////////////////////////////////////////////////////// */
/*
     Camming
*/
/* Define status bit returned by AerGetCamTableStatus() */
// C#	CLASS=Camming		REGION=Status
#define AERCAM_STAT_NOT_ALLOCATED   0x00        // cam table not allocated
#define AERCAM_STAT_ALLOCATED       0x01        // cam table allocated
#define AERCAM_STAT_COEFFS_BUSY     0x02        // coefficients being calculated
#define AERCAM_STAT_COEFFS_DONE     0x04        // cam coefficients done and ready
#define AERCAM_STAT_IN_USE          0x08        // cam table has an axis synched to it
// C# END

/* AerCamTableGetMode()/AerCamTableSetMode() */
// C#	CLASS=Camming		REGION=Mode
#define AERCAM_MODE_OFF       0
#define AERCAM_MODE_RELATIVE  1
#define AERCAM_MODE_ABSOLUTE  2
#define AERCAM_MODE_VELOCITY  3
#define AERCAM_MODE_MAX       4
// C# END

/* AerCamTableGetPoint()/AerCamTableSetPoint() */
// C#	CLASS=Camming		REGION=Interpolation Type
#define AERCAM_POINT_LINEAR   0
#define AERCAM_POINT_CUBIC    1
#define AERCAM_POINT_MAX      2
// C# END

/* AerCamTableConfigMaster() */
// C#	CLASS=MasterSlave		REGION=Command
#define AER_MASTER_SLAVE_CMD_POSFDBK   0
#define AER_MASTER_SLAVE_CMD_POSCMD    1
#define AER_MASTER_SLAVE_CMD_EXTPOS    2
#define AER_MASTER_SLAVE_CMD_MAX       3
// C# END

#define MAX_AXISCAL_TABLES                   100   // MAX_AXES*2
#define MAX_2D_AXISCAL_TABLES                10
#define MAX_NUMBER_AXISCAL_TABLES_PER_AXIS   8

#define AXISCAL_INVALID_MATERIAL_TEMP       -1000
#define AXISCAL_INVALID_EXPAND_COEFF        -1000

#define AXISCAL_EXPAND_COEFF                 0
#define AXISCAL_CURRENT_EXPAND_COEFF         1
#define AXISCAL_REFERENCE_TEMP               2
#define AXISCAL_CURRENT_MATERIAL_TEMP        3

#define AXISCAL_2D_MODE_OFF                        0
#define AXISCAL_2D_MODE_ON                         1
#define AXISCAL_2D_MODE_ON_IN_TABLE                2

#define AXISCAL_2D_DO_NOT_CORRECT_POS        0
#define AXISCAL_2D_CORRECT_POS               1

// Definitions for probe status
#define AER_PROBESTATUS_ARMED       0x01  /* Probe is armed */
#define AER_PROBESTATUS_VALIDPOS    0x02  /* Position is valid */
#define AER_PROBESTATUS_INPUT       0x04  /* Has been configured */
#define AER_PROBESTATUS_HIGHSPEED   0x08  /* Using High speed position latch */

// Definitions for the AerQueueProbeSetInput() and AerQueueProbeSetMode() functions
// These are set to the same values previousy defined in aerparam.pgm for the original CNC Probe command
// The values follow the appropiate bit settings for the A3200 Input spec.
#define QUEUE_PROBE_INPUT_DRIVE_DIGITAL                  0x1000000
#define QUEUE_PROBE_INPUT_ETHERNET_DIGITAL               0x3000000
#define QUEUE_PROBE_INPUT_VIRTUAL_BINARY                 0x6000000
#define QUEUE_PROBE_INPUT_DRIVE_LATCH1                   0xC000000
#define QUEUE_PROBE_INPUT_DRIVE_LATCH2                   0xD000000
#define QUEUE_PROBE_SW_MODE                              0
#define QUEUE_PROBE_HW_MODE                              1

// Constants for the dwSlewDimension parameter of the AerQueueSlew function
#define QUEUE_SLEW_2D                     0
#define QUEUE_SLEW_3D                     1

// Constants for dwCmdMode parameter of the AerQueuePsoControl() function
// C#	CLASS=PSO		REGION=PSO CONTROL
#define PSO_CONTROL_OFF                   0
#define PSO_CONTROL_ARM                   1
#define PSO_CONTROL_FIRE                  2
#define PSO_CONTROL_FIRE_CONTINUOUS       3
#define PSO_CONTROL_OUTPUT                4
#define PSO_CONTROL_RESET                 5
#define PSO_CONTROL_WINDOW                6
#define PSO_CONTROL_ON                    7
// C# END

// Constants for dwCmdMode parameter of the AerQueuePsoDistance() function
// C#	CLASS=PSO		REGION=PSO DISTANCE
#define PSO_DISTANCE_OFF                  0
#define PSO_DISTANCE_FIXED_UNITS          1
#define PSO_DISTANCE_FIXED_STEPS          2
#define PSO_DISTANCE_ARRAY                3
// C# END

// Constants for dwCmdMode parameter of the AerQueuePsoOutput() function
// C#	CLASS=PSO		REGION=PSO OUTPUT
#define PSO_OUTPUT_PULSE                        0
#define PSO_OUTPUT_PULSE_BITMASK                1
#define PSO_OUTPUT_PULSE_WINDOWMASK             2
#define PSO_OUTPUT_PULSE_WINDOWMASK_HARD        3
#define PSO_OUTPUT_PULSE_WINDOWMASK_EDGE        4
#define PSO_OUTPUT_PULSE_WINDOWMASK_HARD_EDGE   5
#define PSO_OUTPUT_TOGGLE                       6
#define PSO_OUTPUT_BIT_MAP                      7
#define PSO_OUTPUT_WINDOW                       8
#define PSO_OUTPUT_COMBINE                      9
#define PSO_OUTPUT_CONTROL                      10
// C# END

// Constants for dwCmdMode parameter of the AerQueuePsoWindow() function
// C#	CLASS=PSO		REGION=PSO WINDOW
#define PSO_WINDOW_ON                     0
#define PSO_WINDOW_OFF                    1
#define PSO_WINDOW_RANGE_UNITS            2
#define PSO_WINDOW_RANGE_STEPS            3
#define PSO_WINDOW_LOAD_UNITS             4
#define PSO_WINDOW_LOAD_STEPS             5
#define PSO_WINDOW_RANGE_ARRAY            6
#define PSO_WINDOW_RANGE_ARRAY_EDGE       7
#define PSO_WINDOW_INPUT                  8
#define PSO_WINDOW_RESET                  9
#define PSO_WINDOW_CONTROL                10
// C# END

// Constants for dwCmdMode parameter of the AerQueuePsoTrack() function
// C#	CLASS=PSO		REGION=PSO TRACK
#define PSO_TRACK_INPUT_1D                0
#define PSO_TRACK_INPUT_2D                1
#define PSO_TRACK_INPUT_3D                2
#define PSO_TRACK_SCALE_1D                3
#define PSO_TRACK_SCALE_2D                4
#define PSO_TRACK_SCALE_3D                5
#define PSO_TRACK_RESET                   6
#define PSO_TRACK_DIRECTION               7
// C# END

// Constants for dwCmdMode parameter of the AerQueueArray() function
// C#	CLASS=PSO		REGION=ARRAY
#define ARRAY_WRITE_ABSOLUTE_UNITS        0
#define ARRAY_WRITE_ABSOLUTE_STEPS        1
#define ARRAY_WRITE_INCREMENTAL_UNITS     2
#define ARRAY_WRITE_INCREMENTAL_STEPS     3
#define ARRAY_READ_UNITS                  4
#define ARRAY_READ_STEPS                  5
// C# END

// Constants for dwCmdMode parameter of the AerQueueDataAcq() function
// C#	CLASS=PSO		REGION=DATA ACQ
#define DATA_ACQ_OFF                      0
#define DATA_ACQ_ARRAY                    1
#define DATA_ACQ_INPUT                    2
#define DATA_ACQ_TRIGGER                  3
// C# END

#define MAX_STOP_WATCHES      10

// Constants for the Analog Command
#define ANALOG_TRACK_CMD_POS		            0        // Tracks the command position of the axes
#define ANALOG_TRACK_FDBK_POS		            1        // Tracks the feedback position of the axes
#define ANALOG_TRACK_CMD_VEL		            2        // Tracks the command velocity of the axes
#define ANALOG_TRACK_FDBK_VEL		            3        // Tracks the feedback velocity of the axes

// Constants for AerSysIsResetDone() Error states
// cant change these !
#define RESET_ERROR    -1 // when reset done, but fatal error in firewire commun (must reset it again)
#define RESET_NOTRESET  0 // while commun reset not done yet, or still going on (no library communications going on)
#define RESET_RUNNING   1 // when commun reset done (drives reset, default parameters downloaded, firewire commun OK),
#define RESET_DONE_AX   2 // when all axis parameters in file downloaded
#define RESET_DONE_ALL  3 // when all parameters in file downloaded

// Constants for AerSysIsSystemInitialized
// C#	CLASS=Init  		REGION=Initialization State
#define SYSTEM_INITIALIZED       0
#define SYSTEM_NOT_INITIALIZED   1
// C# END

// Constants for AerSysInit() Error states
// C#	CLASS=Init  		REGION=Initialization Error State
#define INIT_ERROR_STATE_NO_ERROR            0
#define INIT_ERROR_STATE_GENERIC_ERROR       1
#define INIT_ERROR_STATE_START_SMC           2
#define INIT_ERROR_STATE_RESET_SMC           3
#define INIT_ERROR_STATE_GET_SMC_DATA        4
#define INIT_ERROR_STATE_HARDWARE_COMPAT     5
#define INIT_ERROR_STATE_AXIS_PARM           6
#define INIT_ERROR_STATE_AXIS_CAL_1D         7
#define INIT_ERROR_STATE_GAIN_CAL            8
#define INIT_ERROR_STATE_TASK_PARM           9
#define INIT_ERROR_STATE_FIBER_PARM         10
#define INIT_ERROR_STATE_VPP_PARM           11
#define INIT_ERROR_STATE_AXIS_CAL_2D        12
#define INIT_ERROR_STATE_GLOB_PARM          13
#define INIT_ERROR_STATE_TASK_INIT          14
#define INIT_ERROR_STATE_PROGRAM_AUTOMATION 15
#define INIT_ERROR_STATE_TOOL_TABLE         16
// C# END

// Constants for event creation
// C#	CLASS=Event_ID  		REGION=Event ID constants
#define AER_EVENT_UNKNOWN_EVENT     0x00010000
//#define AER_EVENT_IRQ2_TIMER        0x00010001
#define AER_EVENT_TASK_FAULT        0x00010002
#define AER_EVENT_AXIS_FAULT        0x00010003
//#define AER_EVENT_SERIAL            0x00010004
//#define AER_EVENT_VIRTIO_UPDATE     0x00010005
#define AER_EVENT_TASK_CALLBACK     0x00010006
#define AER_EVENT_JOYSTICK          0x00010007
#define AER_EVENT_DRV_TIMEOUT       0x00010008
#define AER_EVENT_END_G1_ACCEL      0x00010009
#define AER_EVENT_START_G1_DECEL    0x0001000A
#define AER_EVENT_ENET_WDOG         0x0001000B

#define AER_EVENT_NONE           0xFFFFFFFF
// C# END

// Drive info word constants for AerStatusGetDriveInfoWord()
#define DRIVE_INFO_HARDWARE_TYPE    0
#define DRIVE_INFO_STICKY_BITS      1
#define DRIVE_INFO_AMP_PEAK_RATING  2
#define DRIVE_INFO_MAJOR_VERSION    3
#define DRIVE_INFO_MINOR_VERSION    4
#define DRIVE_INFO_BUILD_VERSION    5
#define DRIVE_INFO_FPGA_VERSION     6
#define DRIVE_INFO_MXH_VERSION      7

// C#	CLASS=Status  		REGION=Drive Info Blocks
#define DRIVE_INFO_BLOCK_0			0
#define DRIVE_INFO_BLOCK_1			1
#define DRIVE_INFO_BLOCK_2			2
#define DRIVE_INFO_BLOCK_3			3
#define DRIVE_INFO_BLOCK_4			4
#define DRIVE_INFO_BLOCK_5			5
#define NUM_DRIVE_INFO_BLOCKS		6
// C# END

// Constants for Global Stripchart Trigger Modes (see AERSTRIP.H)
// C#	CLASS=StripGlobal  		REGION=Trigger Modes
#define STRIPGLOBAL_MODE_TIME                   0         // normal, starts when triggered
#define STRIPGLOBAL_MODE_TIME_DELAYED           1         // sends COMMAND_DATA_COLLECT_TRIG to all drives, starts after a fixed delay, so as to coordinate with the COMMAND_DATA_COLLECT_TRIG
#define STRIPGLOBAL_MODE_QUEUE_DELAYED          2         // sends COMMAND_DATA_COLLECT_TRIG to all drives, starts after a fixed delay, so as to coordinate with the COMMAND_DATA_COLLECT_TRIG
#define STRIPGLOBAL_MODE_QUEUE                  3
#define STRIPGLOBAL_MODE_QUEUE_HOLD             4
#define STRIPGLOBAL_MODE_EVENT_COUNT            5
#define STRIPGLOBAL_MODE_EVENT_QUEUE            6
#define STRIPGLOBAL_MODE_IO                     7
#define STRIPGLOBAL_MODE_IO_QUEUE               8
// C# END

// Constants for Global Stripchart Status  (see AERSTRIP.H)
// C#	CLASS=StripGlobal  		REGION=Status constants
#define STRIPGLOBAL_STATUS_ALLOCATED         0x00001  // buffer to receieve data is allocated on SMC
#define STRIPGLOBAL_STATUS_ARMED             0x00002  // not used ?
#define STRIPGLOBAL_STATUS_TRIGGERED         0x00004  // now dumping data into SMC buffer
#define STRIPGLOBAL_STATUS_DONE              0x00008  // done collecting data
#define STRIPGLOBAL_STATUS_OVERFLOW          0x00010  // overflow of buffer in queue mode
#define STRIPGLOBAL_STATUS_FR_MODE           0x00020
#define STRIPGLOBAL_STATUS_HOLD              0x00040  // in queue mode, but "held"
#define STRIPGLOBAL_STATUS_TABLE_MODE        0x00080
#define STRIPGLOBAL_STATUS_TRIG_DELAYED      0x00100  // plot start was delayed (may not have starrted yet)
#define STRIPGLOBAL_STATUS_QUEUE_MODE        0x00200
#define STRIPGLOBAL_STATUS_ABORTED           0x00400
#define STRIPGLOBAL_STATUS_EVENT_MODE        0x00800
#define STRIPGLOBAL_STATUS_UPLOADING         0x01000
#define STRIPGLOBAL_STATUS_IO_MODE           0x02000  // triggers when an input bit edge is seen
#define STRIPGLOBAL_STATUS_IO_QUEUE_MODE     0x04000
#define STRIPGLOBAL_STATUS_TRIG_DELAYED_Q    0x08000  // plot start was delayed (may not have started yet)
// C# END

// Constants for Global Stripchart Input Edges (see AERSTRIP.H)
#define STRIPGLOBAL_INPUT_RISING_EDGE        0
#define STRIPGLOBAL_INPUT_FALLING_EDGE       1

/* Global strip status definitions (see AERSTRIP.H) */
#define BSTRIPGLOBAL_STATUS_ALLOCATED         0
#define BSTRIPGLOBAL_STATUS_ARMED             1
#define BSTRIPGLOBAL_STATUS_TRIGGERED         2
#define BSTRIPGLOBAL_STATUS_DONE              3
#define BSTRIPGLOBAL_STATUS_OVERFLOW          4
#define BSTRIPGLOBAL_STATUS_FR_MODE           5
#define BSTRIPGLOBAL_STATUS_HOLD              6
#define BSTRIPGLOBAL_STATUS_TABLE_MODE        7
#define BSTRIPGLOBAL_STATUS_TRIG_DELAYED      8
#define BSTRIPGLOBAL_STATUS_QUEUE_MODE        9
#define BSTRIPGLOBAL_STATUS_ABORTED           10
#define BSTRIPGLOBAL_STATUS_EVENT_MODE        11
#define BSTRIPGLOBAL_STATUS_UPLOADING         12
#define BSTRIPGLOBAL_STATUS_IO                13
#define BSTRIPGLOBAL_STATUS_IO_QUEUE          14
#define BSTRIPGLOBAL_STATUS_TRIG_DELAYED_Q    15
#define BSTRIPGLOBAL_STATUS_MAX               16

/* Defines for GetQueueDecimate  wType (see AERSTRIP.H) */
#define STRIPGLOBAL_GET_RECENT				 0x0001
#define STRIPGLOBAL_GET_OLDEST               0x0000

/* Defines for SetIOPosLatch   (see AERSTRIP.H) */
#define STRIP_IOPOSLATCH_DISABLE			 0
#define STRIP_IOPOSLATCH_ONESHOT			 1
#define STRIP_IOPOSLATCH_CONTINUOUS			 2
#define STRIP_IOPOSLATCH_AXISIO				 0
#define STRIP_IOPOSLATCH_VIRTUALIO			 1
#define STRIP_IOPOSLATCH_BITCCW				 0
#define STRIP_IOPOSLATCH_BITCW				 1
#define STRIP_IOPOSLATCH_BITHOME			    2
#define STRIP_IOPOSLATCH_BITENCFLT			 3
#define STRIP_IOPOSLATCH_BITFLT			    4
#define STRIP_IOPOSLATCH_BITHALLA			 5
#define STRIP_IOPOSLATCH_BITHALLB			 6
#define STRIP_IOPOSLATCH_BITHALLC 			 7

/* Defines for RT Data collection */
// C#	CLASS=StripGlobal  		REGION=Real-Time Data Buffer constants
#define STRIPGLOBAL_RTD_BUFFER_1             0
#define STRIPGLOBAL_RTD_BUFFER_2             1
#define STRIPGLOBAL_RTD_BUFFER_3             2
#define STRIPGLOBAL_RTD_BUFFER_4             3
#define STRIPGLOBAL_RTD_BUFFER_5             4
#define STRIPGLOBAL_RTD_BUFFER_6             5
#define STRIPGLOBAL_RTD_BUFFER_7             6
#define STRIPGLOBAL_RTD_BUFFER_8             7
#define STRIPGLOBAL_RTD_MAX_BUFFERS          8
// C# END

// C#	CLASS=StripGlobal  		REGION=Real-Time Data Rate constants
#define STRIPGLOBAL_RTD_RATE_OFF          0x00      // not using "high spped rate" feature (see AerStripGlobalSetRTDataCollectRate() )
#define STRIPGLOBAL_RTD_RATE_1KHZ         0x01
#define STRIPGLOBAL_RTD_RATE_2KHZ         0x02
#define STRIPGLOBAL_RTD_RATE_4KHZ         0x04
#define STRIPGLOBAL_RTD_RATE_8KHZ         0x08
// C# END

/*
#define RTD_SAMPLE_OFF                       0
#define RTD_SAMPLE_POS_CMD                   1
#define RTD_SAMPLE_POS_FDBK                  2
#define RTD_SAMPLE_ID_CMD_FDBK               3
#define RTD_SAMPLE_IQ_CMD_FDBK               4
#define RTD_SAMPLE_CURRENT_FDBK_A_B          5
#define RTD_SAMPLE_ANALOG_ENC_SIN_COS        6
#define RTD_SAMPLE_AD_1_2                    7
#define RTD_SAMPLE_LOOP_TRANS_BEFORE         8
#define RTD_SAMPLE_LOOP_TRANS_AFTER          9
#define RTD_SAMPLE_MAX                      10
*/


// C#	CLASS=StripGlobal  		REGION=Real-Time Data Type constants
/* Defines for AerStripGlobalSetOptionalDataItem() (see AERSTRIP.H) */
//#define OPTIONAL_DATA_TYPE_SHORT   0    // for specifing OPTIONAL_DATA_CODES_ quantities
#define OPTIONAL_DATA_TYPE_LONG    0    // for specifing OPTIONAL_DATA_CODEL_ quantities
// NOTE: OPTIONAL_DATA_TYPE_EXTPOS is maintained for back compatibility ONLY (from 2.13 on).
// It is replaced by DATABIT_OPTIONAL_DATA* constants and DATAOPT CNC statement.
#define OPTIONAL_DATA_TYPE_EXTPOS  1    // for specifing EXTPOS_OPTCODE quantities (changes ext pos fdbck to something else)
// C# END


//#define OPTIONAL_DATA_NUM_MAX_SHORT 3
//#define OPTIONAL_DATA_NUM_MAX_LONG  2   // was 2 when only using std GStrip Axis packet
#define OPTIONAL_DATA_NUM_MAX_LONG  8     // now 8 when only using opt GStrip Axis packet
#define OPTIONAL_DATA_NUM_NONE (WORD)-1   // this is a EXTPOS_OPTCODE quantity used to signal strip to turn off optional data collection for all axes

// C#	CLASS=OptionalData	REGION=Codes
// All of these codes are used only within the SMC - not sent to the drives (see getDriveCodeForOptionalDataCode())
// These first codes are things that can be collected < 1 khtz from drive
//
#define OPTIONAL_DATA_CODE_NONE             0x00
#define OPTIONAL_DATA_CODEL_POSCMD_1MSEC    0x01
#define OPTIONAL_DATA_CODEL_POSFBK_1MSEC    0x02
#define OPTIONAL_DATA_CODES_ID_CMD          0x03
#define OPTIONAL_DATA_CODES_ID_FBK          0x04
#define OPTIONAL_DATA_CODES_IQ_CMD          0x05
#define OPTIONAL_DATA_CODES_IQ_FBK          0x06
#define OPTIONAL_DATA_CODES_PHASEA_IFBK     0x07
#define OPTIONAL_DATA_CODES_PHASEB_IFBK     0x08
#define OPTIONAL_DATA_CODES_ENCODER_SIN_AD  0x09
#define OPTIONAL_DATA_CODES_ENCODER_COS_AD  0x0A
#define OPTIONAL_DATA_CODES_AD_1            0x0B
#define OPTIONAL_DATA_CODES_AD_2            0x0C
#define OPTIONAL_DATA_CODEL_LT_CHANNEL1     0x0D
#define OPTIONAL_DATA_CODEL_LT_CHANNEL2     0x0E
#define OPTIONAL_DATA_CODEL_ZHEIGHT_DIFF    0x0F
#define OPTIONAL_DATA_CODEL_ZHEIGHT_SUM     0x10
#define OPTIONAL_DATA_CODEL_ZHEIGHT         0x11
#define OPTIONAL_DATA_CODEL_ZHEIGHT_LPF     0x12
#define OPTIONAL_DATA_CODES_AD_1_OUT        0x13
#define OPTIONAL_DATA_CODES_AD_2_OUT        0x14
#define OPTIONAL_DATA_CODES_DIGITALIO       0x15  // Not implemented
#define OPTIONAL_DATA_CODEL_TEST            0x16
#define OPTIONAL_DATA_CODEL_DSPMEM_INT      0x17  // WARNING: the *DSPMEM* optional data collection codes
#define OPTIONAL_DATA_CODEL_DSPMEM_FLOAT    0x18  // hijack one of the other buffer locations and therefore
#define OPTIONAL_DATA_CODEL_DSPMEM_DOUBLE   0x19  // should only be used for 1kHz collection
#define OPTIONAL_DATA_CODEL_PSO_STATUS      0x1A
#define OPTIONAL_DATA_CODEL_DRIVE_TIMER     0x1B
#define OPTIONAL_DATA_DRIVE_MAX             0x1C  // this must be updated, if we add any optional codes above
// C# END

// All of these codes are used only within the SMC - not sent to the drives
// Optional Data is normally axis based. But some of the below are task based, in which case,
// retrieving axis data actually gets task data as follows: (X-task1,Y-task 2,Z-task3,U-task4,all other axes undefined)

#define OPTIONAL_DATA_CODEL_CNCLINE         0x20      // do we need a CAMLINE too???
#define OPTIONAL_DATA_CODEL_VME_AXIS1       0x21
#define OPTIONAL_DATA_CODEL_VME_AXIS2       0x22
#define OPTIONAL_DATA_CODEL_VME_AXIS3       0x23
#define OPTIONAL_DATA_CODEL_VME_AXIS4       0x24
#define OPTIONAL_DATA_CODEL_PROFFLAGS       0x25
#define OPTIONAL_DATA_CODEL_PROFTRG         0x26
#define OPTIONAL_DATA_CODEL_INPUT           0x27
#define OPTIONAL_DATA_CODEL_OUTPUT          0x28
#define OPTIONAL_DATA_CODEL_DRIVESTAT       0x29       // upper byte is mask to and with (0 means no anding)
#define OPTIONAL_DATA_CODEL_AXISSTAT        0x2A       // upper byte is mask to and with (0 means no anding)
#define OPTIONAL_DATA_CODEL_RAWACC          0x2B
#define OPTIONAL_DATA_CODEL_VBIN_INPUT      0x2C
#define OPTIONAL_DATA_CODEL_VBIN_OUTPUT     0x2D
#define OPTIONAL_DATA_CODEL_VREG_INPUT      0x2E
#define OPTIONAL_DATA_CODEL_VREG_OUTPUT     0x2F
#define OPTIONAL_DATA_CODEL_DRIVEFAULT      0x30
#define OPTIONAL_DATA_CODEL_POSCAL          0x31      // adjusment to POS cmd due to: 1d cal + 2d cal + backlash
#define OPTIONAL_DATA_CODEL_CLOCK           0x32      // always msec

// The data below is actually part of the MISC DATA - the lower nibble is used to select which
// data is returned from the GetCurrentCommandPointdMiscCounts function (ANDed with OPTDATAPROF_MAX)
#define OPTDATAPROF_BASE       0x40   // internal use constants only
#define OPTDATAPROF_STARTANG    0x0   // internal use constants only
#define OPTDATAPROF_ENDANG      0x1   // internal use constants only
#define OPTDATAPROF_POS         0x2   // internal use constants only
#define OPTDATAPROF_VEL         0x3   // internal use constants only
#define OPTDATAPROF_TOT_DIST    0x4   // internal use constants only
#define OPTDATAPROF_PERC_DONE   0x5   // internal use constants only
#define OPTDATAPROF_RAD         0x6   // internal use constants only
#define OPTDATAPROF_RADERR      0x7   // internal use constants only
#define OPTDATAPROF_MAX         0xF   // internal use constants only  WARNING ! CANNOT INCREASE OR ELSE USER WHO USE "QCOMMAND READ DATA"
                                      // will be screwed if they dont declare their misc array large enough to cover increase !

#define OPTIONAL_DATA_CODEL_PROFSTARTANG    (OPTDATAPROF_BASE+OPTDATAPROF_STARTANG)   // counts
#define OPTIONAL_DATA_CODEL_PROFENDANG      (OPTDATAPROF_BASE+OPTDATAPROF_ENDANG)     // counts
#define OPTIONAL_DATA_CODEL_PROFPOS         (OPTDATAPROF_BASE+OPTDATAPROF_POS)        // counts
#define OPTIONAL_DATA_CODEL_PROFVEL         (OPTDATAPROF_BASE+OPTDATAPROF_VEL)        // counts/msec
#define OPTIONAL_DATA_CODEL_PROFTOT_DIST    (OPTDATAPROF_BASE+OPTDATAPROF_TOT_DIST)   // counts
#define OPTIONAL_DATA_CODEL_PROFPERC_DONE   (OPTDATAPROF_BASE+OPTDATAPROF_PERC_DONE)  // unitless
#define OPTIONAL_DATA_CODEL_PROFRADIUS      (OPTDATAPROF_BASE+OPTDATAPROF_RAD)        // counts
#define OPTIONAL_DATA_CODEL_PROFRADERR      (OPTDATAPROF_BASE+OPTDATAPROF_RADERR)     // counts

// These are available through other means, but to make the STATUS command work we need to reserve these constants
// These were available through parameters and/or stripchart
#define OPTIONAL_DATA_CODED_PositionUnits        0x70
#define OPTIONAL_DATA_CODED_PositionCmdUnits     0x71
#define OPTIONAL_DATA_CODED_PositionCnts         0x72
#define OPTIONAL_DATA_CODED_PositionCmdCnts      0x73
#define OPTIONAL_DATA_CODED_PositionProgCmdCnts  0x74
#define OPTIONAL_DATA_CODED_PositionExtUnit      0x75
#define OPTIONAL_DATA_CODED_PositionExtCnts      0x76
#define OPTIONAL_DATA_CODED_PositionErrUnits     0x77
#define OPTIONAL_DATA_CODED_PositionErrCnts      0x78
#define OPTIONAL_DATA_CODED_CurrentErr           0x79
// These were avaliable through parameters only
#define OPTIONAL_DATA_CODED_VelocityCntsSecAvg   0x7A
#define OPTIONAL_DATA_CODED_CurrentAvg           0x7B
// These were avaliable through strip chart only
#define OPTIONAL_DATA_CODED_CurrentCmd           0x7C
#define OPTIONAL_DATA_CODED_Current              0x7D
#define OPTIONAL_DATA_CODED_Misc                 0x7E     // SMC data, upper byte is a subcode indicating what is returned
#define OPTIONAL_DATA_CODED_GlobalVar            0x7F     // upper byte is var number, axis number is ignored
#define OPTIONAL_DATA_CODED_AxisParm             0x80     // upper byte is prm number
#define OPTIONAL_DATA_CODED_TaskParm             0x81     // upper byte is prm number, axis number is really task number
#define OPTIONAL_DATA_CODED_ProgVar              0x82     // upper byte is prm number, axis number is really task number
#define OPTIONAL_DATA_CODED_TaskStat             0x83     // upper byte status number, axis number is really task number
#define OPTIONAL_DATA_CODED_PeakCurrent          0x84
#define OPTIONAL_DATA_CODED_TaskFault            0x85     // axis number is really task number
#define OPTIONAL_DATA_CODED_TaskWarning          0x86     // axis number is really task number
#define OPTIONAL_DATA_CODED_Backlash             0x87
#define OPTIONAL_DATA_CODED_HomeState            0x88     // a HOMESTATUSMASK_ value
#define OPTIONAL_DATA_CODED_PosCal2D             0x89     // position cal (2D contribution only)
#define OPTIONAL_DATA_CODED_VelocityCntsSec      0x8A     // instantaonous (as opposed to averaged)
#define OPTIONAL_DATA_CODED_VelocityCmdCntsSec   0x8B     // instantaonous (as opposed to averaged)
#define OPTIONAL_DATA_CODED_RawFire              0x8C     // items from the firewire record sent to the drive
#define OPTIONAL_DATA_CODED_Normalcy             0x8D
#define OPTIONAL_DATA_CODED_TotalMoveTime        0x8E     // milliseconds
#define OPTIONAL_DATA_CODED_GlobalEnetStatus     0x8F     // Global Ethernet (thru RT-TCP/IP) status
#define OPTIONAL_DATA_CODED_RemoteServerStatus   0x90     // Global Ethernet (thru RT-TCP/IP) status
#define OPTIONAL_DATA_CODED_RelativeSettleTime   0x91     // milliseconds
#define OPTIONAL_DATA_CODED_PositionEncoder      0x92     // counts (no transforms, G92, backlash or cal subtracted out, what Nstat shows)
#define OPTIONAL_DATA_CODED_JerkCommand          0x93     // counts/msec*msec*msec
#define OPTIONAL_DATA_CODED_PositionProgCmdUnits 0x94
#define OPTIONAL_DATA_CODED_GantryOffset         0x95
#define OPTIONAL_DATA_CODED_SoftwareHome         0x96     // current G92 value

#define OPTIONAL_DATA_CODED_LAST_VALID           0x97

#define OPTIONAL_DATA_CODE_MAX              0xff

// OPTIONAL_DATA_CODED_Misc optional address sub codes
#define OPTIONAL_DATA_CODED_Misc_SUBCODE_PHASE             0     // reserved for gathering phase (FIRMWARE/TEST/PHASE1.TST)
#define OPTIONAL_DATA_CODED_Misc_SUBCODE_FAST_TASK         1     // reserved for gathering fast task data
#define OPTIONAL_DATA_CODED_Misc_SUBCODE_LIBTEST_HAMMER  100     // reserved for libtest hammering (FIRMWARE/TEST/LIBTEST)

// NOTE: These are maintained for back compatibility ONLY (from 2.13 on).
// They are replaced by DATABIT_OPTIONAL_DATA* constants and DATAOPT CNC statement.
//   External position is normally axis based. But some of the be,ow are task absed, in which case,
//   retriving axis data actually gets task data as follows: (X-task1,Y-task 2,Z-task3,U-task4,all other axes undefined)
#define EXTPOS_OPTCODE_CNCLINE         0x51   // per TASK
#define EXTPOS_OPTCODE_PROFFLAGS       0x52   // per TASK
#define EXTPOS_OPTCODE_PROFTRG         0x53
#define EXTPOS_OPTCODE_INPUT           0x54   // binary drive
#define EXTPOS_OPTCODE_OUTPUT          0x55   // binary drive
#define EXTPOS_OPTCODE_DRIVESTAT       0x56   // unitless
#define EXTPOS_OPTCODE_AXISSTAT        0x57   // unitless
#define EXTPOS_OPTCODE_RAWACC          0x58
#define EXTPOS_OPTCODE_VBIN_INPUT      0x59   // upper byte is starting number for the I/O
#define EXTPOS_OPTCODE_VBIN_OUTPUT     0x5A   // upper byte is starting number for the I/O
#define EXTPOS_OPTCODE_VREG_INPUT      0x5B   // upper byte is starting number for the I/O
#define EXTPOS_OPTCODE_VREG_OUTPUT     0x5C   // upper byte is starting number for the I/O
#define EXTPOS_OPTCODE_DRIVEFAULT      0x5D   // unitless
#define EXTPOS_OPTCODE_PROFSTARTANG    0x70   // task-based  "scaled" radians
#define EXTPOS_OPTCODE_PROFENDANG      0x71   // task-based  "scaled" radians
#define EXTPOS_OPTCODE_PROFPOS         0x72   // task-based  "scaled" inches
#define EXTPOS_OPTCODE_PROFVEL         0x73   // task-based  "scaled" inches
#define EXTPOS_OPTCODE_PROFTOT_DIST    0x74   // NOT USED??? task-based inches
#define EXTPOS_OPTCODE_PROFPERC_DONE   0x75   // NOT USED??? unitless
#define EXTPOS_OPTCODE_AVG_CUR_FBCK    0x76   //
#define EXTPOS_OPTCODE_MAX             0xFF   // upper byte reserved for I/O address number, if any


// NService Constants
#define NSERVICE_NAME			      TEXT("NService SMC Control")
#define NSERVICE_START_SMC_EVENT    TEXT("NServiceStartSMC")
#define NSERVICE_STOP_SMC_EVENT     TEXT("NServiceStopSMC")
#define NSERVICE_RUNNING_EVENT      TEXT("NServiceRunning")
#define NSERVICE_SLEEP_INTERVAL     1000

//
// these opcodes used in _aerMoveAxis()
//
#define AERMOVE_ABSOLUTE      MOTION_ABSOLUTE_SUBCODE
#define AERMOVE_HOME          MOTION_HOME_SUBCODE
#define AERMOVE_INCREMENTAL   MOTION_INCREMENTAL_SUBCODE
#define AERMOVE_FREERUN       MOTION_FREERUN_SUBCODE
#define AERMOVE_HALT          MOTION_HALT_SUBCODE
#define AERMOVE_ABORT         MOTION_ABORT_SUBCODE
#define AERMOVE_OSCILLATE     MOTION_OSCILLATE_SUBCODE
#define AERMOVE_ALIGN         MOTION_ALIGN_SUBCODE
#define AERMOVE_FEEDHOLD      MOTION_FEEDHOLD_SUBCODE
#define AERMOVE_FEEDRELEASE   MOTION_FEEDRELEASE_SUBCODE
#define AERMOVE_ENABLE        MOTION_ENABLE_SUBCODE
#define AERMOVE_DISABLE       MOTION_DISABLE_SUBCODE
#define AERMOVE_BRAKE_ENABLE  MOTION_BRAKE_ENABLE_SUBCODE
#define AERMOVE_BRAKE_DISABLE MOTION_BRAKE_DISABLE_SUBCODE
#define AERMOVE_SET_POS_CMD   MOTION_SET_POS_CMD_SUBCODE
#define AERMOVE_HALT_TASK_JOG MOTION_HALT_TASK_JOG_SUBCODE

// these values are used to get values (AerRegGetPath())
// WARNING: YOU MUST ADD/CHANGE pszKeyReg[] and pszKeyIni[] arrays in AERREG.C to get them to work in C or VB
// WARNING: YOU CANNOT CHANGE EXISTING ONES OF THESE (If the registry ID for the A3200 Param File changes, it will change FILEWRITEINI and FILEREADINI)
//
// C#	CLASS=RegistryID	REGION=File Identifies
#define AERREGID_InstallDir                0   // C:\A3200 (created by setup)
#define AERREGID_A3200IniFile              1   // A32.INI
#define AERREGID_A3200ParamFile            2   // A32Param.INI (A3200 parameter file)
#define AERREGID_A3200MMIPosFile           3   // A32POS.SS  (A3200 positions display file)
#define AERREGID_DefaultTeachFile          4   // MMITEACH.PGM
#define AERREGID_Cal1DFile                 5   // 1D Calibration File
#define AERREGID_Cal2DFile                 6   // 2D Calibration File
#define AERREGID_ProgramDir                7   // <installDir>\Programs
#define AERREGID_RTX_InstallDir            8   // where VenturCom RTX was installed
#define AERREGID_RTX_VersionString         9   // VenturCom RTX version number (string)
#define AERREGID_RT_TCP_IP_VersionString  10
#define AERREGID_Max                      11
// C# END

#define AERREG_SMC_OPTION_GLOBAL_ENET         0x00000001  // Enable / Disable Global Ethernet IO
#define AERREG_SMC_OPTION_ASYNC_COMM_ONLY     0x00000002  // Used by the new Nload

#define FIBERSTART_HILLCLIMB    0
#define FIBERSTART_SPIRAL_ROUGH 1
#define FIBERSTART_SPIRAL_FINE  2
#define FIBERSTART_FASTALIGN	  3
#define FIBERSTART_GEOCENTER    4
#define FIBERSTART_CENTROID	  5

#define FIBERSTART_SUBCODE_NONE	  0   // used with Centroid 1D and FastAlign 2D Only
#define FIBERSTART_SUBCODE_2D	     1   // used with Centroid Only
#define FIBERSTART_SUBCODE_3D	     2   // used with Centroid and FastAlign Only
#define FIBERSTART_SUBCODE_4D	     3   // used with FastAlign Only
#define FIBERSTART_SUBCODE_5D	     4   // used with FastAlign Only
#define FIBERSTART_SUBCODE_6D	     5   // used with FastAlign Only

#define AER_VME_16_BIT_ACCESS      0
#define AER_VME_32_BIT_ACCESS      1
#define AER_VME_8_BIT_ACCESS       2

#define AER_VME_BUF_NO_AUTO_INC     0x10 // this constant to be OR'ed with the 3 above constants for the VME_write/read_buf fn's

#define AERMOVEWAIT_CHECK_ONCE      0xFFFFFFFF

// Amplifier/drive type (used in auto tuning)
#define AER_AMP_TYPE_NDRIVEHP_PWM    0x01      // of course all NdriveHP amps are PWM
#define AER_AMP_TYPE_NDRIVEHL_LINEAR 0x02      // of course all NdriveHL amps are Linear
#define AER_AMP_TYPE_NPAQ_PWM        0x03
#define AER_AMP_TYPE_NPAQ_LINEAR     0x04
#define AER_AMP_TYPE_NDRIVECP_PWM    0x05
#define AER_AMP_TYPE_NDRIVEMP_PWM    0x06
#define AER_AMP_TYPE_NDRIVECL_LINEAR 0x07

// used by AerProgramRun()
// C#	CLASS=CNCProgram	REGION=Program Compile/Run Constants
#define AERPROG_RUN_FILE         0x0     // Run a .pgm (Use .ogm file if present)
#define AERPROG_RUN_FILE_CMPL    0x1     // Run a .pgm forcing a compilation
#define AERPROG_RUN_TEXT         0x2     // Run a text cmd. string
#define PROGRAM_FLAG_HIDDEN          0x1
// C# END

// Used by program automation
#define AUTOPROG_DRIVER_NAME_C "AUTOPROG"          // prefix that AerCompilerAutoRunEx2() uses to name "programAutomation driver files"
// Should be able to do the below but *#!?*#* VB won't let me. So its hardcoded in VB
//#define AUTOPROG_DRIVER_NAME_VB "MMIAUTO"          // prefix that VB uses (Nview/NScope) to name "programAutomation driver files"
//
// These must match the text as shown in Nview/Nview.rc: "Auto Run identifiers"
//
// C#	CLASS=ProgramAutomation REGION=Type
#define AUTOPROG_TYPE_INCLUDE          0
#define AUTOPROG_TYPE_DOWNLOAD_ONLY    1
#define AUTOPROG_TYPE_RUN_SILENT       2
#define AUTOPROG_TYPE_RUN              3
#define AUTOPROG_TYPE_RUN_IMMEDIATE    4
#define AUTOPROG_TYPE_LOAD             5
#define AUTOPROG_TYPE_RUN_HMI_SHUTDOWN 6
#define AUTOPROG_TYPE_RUN_PC_SHUTDOWN  7
// C# END

// Used by program automation
// C#	CLASS=ProgramAutomation REGION=System
#define AUTOPROG_SYSTEM_SYSFILE        -1
#define AUTOPROG_SYSTEM_USERFILE       0
// C# END

// Display Flags used to determine the MSGxxx Command
// C#	CLASS=DisplayFlags   REGION=MsgCommands
#define DF_LIST                              0x40000000     // Display text is added to MMI List area
#define DF_MSGBOX                            0x20000000     // Display message box
#define DF_INPUT                             0x10000000     // Display user input box
#define DF_LAMP                              0x08000000     // Lamp display
#define DF_MSGMENU                           0x04000000     // Menu Display
#define DF_MSGTASK                           0x02000000     // Task Display
#define DF_MANIO                             0x01000000     // Change ManIO Key
#define DF_LIST_CLEAR                        0x00800000     // Clear the List window
#define DF_LIST_SHOW                         0x00400000     // Show the List window
#define DF_LIST_HIDE                         0x00200000     // Hide the List window
#define DF_PROGRAM                           0x00100000     // Display/Load Program
#define DF_FILE_SELECT                       0x00080000     // Select a file
// C# END

// Applies to MSGBOX Buttons
// C#	CLASS=DisplayFlags   REGION=MsgBoxButtons
#define DF_MSGBOX_BUTTONS_OKONLY             0              // OK Button
#define DF_MSGBOX_BUTTONS_OKCANCEL           1              // OK/Cancel Button
#define DF_MSGBOX_BUTTONS_ABORTRETRYIGNORE   2              // Abort/Retry/Ignore Button
#define DF_MSGBOX_BUTTONS_YESNOCANCEL        3              // Yes/No/Cancel Button
#define DF_MSGBOX_BUTTONS_YESNO              4              // Yes/No Button
#define DF_MSGBOX_BUTTONS_RETRYCANCEL        5              // Retry/Cancel Button
// C# END

// Applies to MSGBOX Icons
// C#	CLASS=DisplayFlags   REGION=MsgBoxIcon
#define DF_MSGBOX_ICON_CRITICAL              16
#define DF_MSGBOX_ICON_QUESTION              32
#define DF_MSGBOX_ICON_EXCLAMATION           48
#define DF_MSGBOX_ICON_INFORMATION           64
// C# END

// Applies to MSGBOX Default Buttons
// C#	CLASS=DisplayFlags   REGION=MsgBoxDefaultButton
#define DF_MSGBOX_DEFAULT_BUTTON_1           0
#define DF_MSGBOX_DEFAULT_BUTTON_2           256
#define DF_MSGBOX_DEFAULT_BUTTON_3           512
// C# END

// Applies to MSGBOX Result
// C#	CLASS=DisplayFlags   REGION=MsgBoxResult
#define DF_MSGBOX_OK_BUTTON                  1
#define DF_MSGBOX_CANCEL_BUTTON              2
#define DF_MSGBOX_ABORT_BUTTON               3
#define DF_MSGBOX_RETRY_BUTTON               4
#define DF_MSGBOX_IGNORE_BUTTON              5
#define DF_MSGBOX_YES_BUTTON                 6
#define DF_MSGBOX_NO_BUTTON                  7
// C# END

// Applies to MSGINPUT Type
// C#	CLASS=DisplayFlags   REGION=MsgInputType
#define DF_MSGINPUT_INTEGER                  0x00800000     // Input Value is an integer
#define DF_MSGINPUT_STRING                   0x00400000     // Input Value is a string
// C# END

// Applies to MSGMENU commands
// C#	CLASS=DisplayFlags   REGION=MsgMenu
#define DF_MSGMENU_ADD                       0x04800000     // Add an item to a Menu   (DF_MSGMENU + 0x00800000)
#define DF_MSGMENU_SHOW                      0x04400000     // Show the Menu           (DF_MSGMENU + 0x00400000)
#define DF_MSGMENU_REMOVE                    0x04200000     // Clear items the Menu    (DF_MSGMENU + 0x00200000)
// C# END

// Applies to MSGPROGRAM commands
// C#	CLASS=DisplayFlags   REGION=MsgProgram
#define DF_MSGPROGRAM_ABORT            0x0010
#define DF_MSGPROGRAM_RESET            0x0020
#define DF_MSGPROGRAM_EXECUTE          0x0040
#define DF_MSGPROGRAM_FORCE_EXECUTE    0x0070         // (DF_PROGRAM_ABORT + DF_PROGRAM_RESET + DF_PROGRAM_EXECUTE)
// C# END

// Applies to MSGFILESELECT commands
// C#	CLASS=DisplayFlags   REGION=MsgFileSelect
#define DF_MSGFILESELECT_OPEN    0
#define DF_MSGFILESELECT_SAVE    1
// C# END

#endif
