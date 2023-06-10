
/*
   Packets used in CNC commands executed by firmware
      (see CMDCODES.H for LIBRARY command codes)
      They are availiable to user so user can compile manually.

   This header file is used by the 960 code, library, and all Win32 applications.
   (including those made by customers)

   NOTE: Structures must be aligned on a 2 byte boundary,
   NOTE: Each element must be aligned on an even boundary as well.
   NOTE: Most structures have a "P" name that can be used as a pointer
      example: typedef AXISPOINT *PAXISPOINT;
    Also many have a "PC" name, that can be used to designate read-only pointers
      example: typedef const AXISPOINT *const PAXISPOINT;

   NOTATION: "xxxx" is a wildcard, and "=" in comment indicates that the element must be one of those. Example:
      DWORD    dwType;              // =AXISTYPE_xxxx
    means that dwType must be one of the constants starting with "AXISTYPE_"

*/

#ifndef __AER_960_H__
#define __AER_960_H__

#include "AerCode.h"    /* for error codes */
#include "AerVirtIO.h"  /* virtual I/O data structures */
#include "AerRtEnet.h"  /* for Global Ethernet I/O communication error structure */


/**************************************************************/
/*  AerVer stuff */

///* Returned by by AerVerGetCpuStatus() */
//#define AER_CPUSTAT_BOOT_EXECUTE 1
//#define AER_CPUSTAT_IMG_EXECUTE  2
//#define AER_CPUSTAT_FATAL        3
/*
Version data structure  (returned by AerVerGet*())
*/
typedef struct tagAER_VERSION
{
   WORD  wUnidex;    /* AER_UNIDEX_xxxx */
   WORD  wMajor;     /* Major Version Number */
   WORD  wMinor;     /* Minor Version Number */
   WORD  wBuild;     /* Internal Build Number */
} AER_VERSION;
typedef AER_VERSION   *PAER_VERSION;


/**************************************************************/
/* AerServerGetData() */

#define ULXMITSHUTOUT_DEADAXIS      (ULONG)-1    // No data in the 1394 Xmit record (but a record was still sent)
#define ULXMITSHUTOUT_VERYDEADAXIS  (ULONG)-2    // No 1394 Xmit record was sent
#define ULXMITSHUTOUT_SMC_DEAD      (ULONG)-3

typedef struct tagAERSOFT_COMMERRORS
{
   // Firewire errors (OHCI1394_IntEvent register mask bits)
   ULONG ulEvent_cycleLost;
   ULONG ulEvent_cycleInconsistent;
   ULONG ulEvent_cycleTooLong;
   ULONG ulEvent_unrecoverableError;
   ULONG ulEvent_postedWriteErr;
   ULONG ulEvent_lockRespErr;

   // Problems with data in the recieve packet
   ULONG ulBadChannelNumsTot;
   //ULONG ulBadChannelNums[MAX_AXES];
   ULONG ulBadRecvLengthsTot;
   //ULONG ulBadRecvLengths[MAX_AXES];
   ULONG ulEvent_busResets;
   ULONG ulSpare[MAX_AXES-1];

   // Recieve time stamp problems
   ULONG ulRcvNTimeStampsLate[MAX_AXES];
   ULONG ulRcvNTimeStampsEarly[MAX_AXES];
   ULONG ulRcvNTimeStampsUnchanged[MAX_AXES];
   USHORT usRcvTimeStampShortest[MAX_AXES];     // units of 1/8 msec
   USHORT usRcvTimeStampLongest[MAX_AXES];      // units of 1/8 msec

   ULONG ulNumRecordsRcv[MAX_AXES];             // number of records received from drive
   ULONG ulBadEtherNetAddresses;                // number of invalid ethernet addresses recieved
   ULONG ulXmitShutOuts[MAX_AXES];       // a positive number representing number of times a record was sent twice OR a ULXMITSHUTOUT* constant
   // Other communication errors/timeouts
//   ULONG ulStarvationsOnRcv;           // recieve firewire queue was full, when we tried to load a new point into it, just sent to us over firewire
//   ULONG ulStarvationsOnXmit;          // xmit firewire queue was empty, when we tried to unload a new point from it, to send over firewire
//   ULONG ulPollThreadTimeouts;         // The polling firewire thread (PCI_1394.C/InterruptThread()) was not called in a timely enough fashion
//   ULONG ulMissesonRcv;                // we did not "digest" a point in the recieve firewire queue, before a new one arrived
   // Xmit generic problems
   ULONG ulTxTimeouts;                 // ASWMAIN.C did not receive event from PCI_1394.C/IsoTxRoutine() to load new point in timely enough fashion
   ULONG ulRxTimeouts;                 // ASWMAIN.C did not receive event from PCI_1394.C/IsoRxRoutine() to unload new point in timely enough fashion
   ULONG ulTxInterruptMiss;            // We detected this many missed interrupt sequences on TX
   ULONG ulRxInterruptMiss;            // We detected this many missed interrupt sequences on RX
   ULONG ulTxInterruptOverflow;        // Num of times Tx queue could not accomadate for this TX miss sequence
   ULONG ulRxInterruptOverflow;        // Num of times RX queue could not accomadate for this RX miss sequence
   USHORT usRxInterruptMax;            // Max numOf interrupts miss in a miss sequence on RX seen
   USHORT usTxInterruptMax;            // Max numOf interrupts miss in a miss sequence on TX seen

   // Polling loop data
   ULONG  ulPollLoopHits;
   USHORT usPollLoopMax;               // in 100 nanosec units
   USHORT usPollLoopMin;               // in 100 nanosec units
   USHORT usPollLoopDeltaMax;          // in 100 nanosec units
   USHORT usPollLoopDeltaMin;          // in 100 nanosec units
   USHORT usNumContextProgs;

   // Thermal data: bit#0-data valid (only if 1 are remaining bits are filled in), bit#1-currently
   // doing "pull back", bit#2- have done "pullback", bit#3-"On Demand" clock modulation is currently operating
   // bit#4->6 - Modulation duty cycle if "On Demand" active.
   //
   DWORD dwThermalDataMask;

   // Exceptions thrown
   ULONG n1394InterruptExceptions;
   ULONG nForeverLoopExceptions;
   ULONG nLibraryExceptions;
   ULONG nTaskExceptions[MAX_TASKS+1];  // the MAX_TASKS slot is for generic engine problems not related to a particular task

   // Misc
   ULONG nQueueStomps[MAX_TASKS+1];       // Delays due to ndrive communications pipeline choking
   ULONG ulnStarvations[MAX_TASKS+1];   // task starvations (MAX_TASKS+1 is for library)
   ULONG ulMemData[2];
} AERSOFT_COMMERRORS;
typedef AERSOFT_COMMERRORS *PAERSOFT_COMMERRORS;

typedef struct tagTIMER_VALUE
{
   DWORD       dwAvg;        /* timing value (avg)   (10 micorsec units) */
   DWORD       dwWorst;      /* timing value (worst) (10 microsec units) */
   DWORD       dwBest;       /* timing value (best)  (10 microsec units) */
} TIMER_VALUE;
typedef TIMER_VALUE *PTIMER_VALUE;

//
// Returned by AerSysServerGetData
#define MAX_NUM_OF_NODES    MAX_AXES  // Arbitrary ? Maximum number of nodes (drives+repeaters+thirdparties)
                                      // that can be connected to network, where Nservo*/npaq counts as one node
typedef struct tagAERSOFT_SERVERDATA
{
   DWORD			   dwServerIsRunning;     /* ! if a server is running */
   long				lServerIntNSeen;       /* Number of interrupts (scans) seen */
   TIMER_VALUE		Time1;                 /* timing values */
   TIMER_VALUE		Time2;                 /* timing values */
   AERERR_CODE		dwServerError;         /* Error in server operation (not library command errors) */
   AERERR_CODE		dwServerErrorStage;    /* Server State (a BYISRESSETING_ constant) */
   AERERR_CODE		dwServerErrorLast;     /* Indicates GetLastError() call */
   AERERR_CODE		dwServerErrorAxis;     /* Extra error data */
   AERERR_CODE		dwServerWarn;          /* Warn in server operation (not library command errors) */
   AERSOFT_COMMERRORS ServerCommErrors;    /* Firewire type errors */
   DWORD			   dwVerison;             /* SMC version */
   LONG			   lNDriversAttached;        /* number of device drivers that are open to me */
   USHORT			usIDOf[MAX_NUM_OF_NODES];     /* a DRIVE_HARDWARE_* constant */
   USHORT			usChanOf[MAX_NUM_OF_NODES];
   ULONG			   ulRcvHits[MAX_AXES];       // number of records received from drive
   ULONG			   ulSpare[MAX_AXES];
   ENETErrorData	ENETErrorDat ;        // Global Ethernet I/O error status
} AERSOFT_SERVERDATA;
typedef AERSOFT_SERVERDATA *PAERSOFT_SERVERDATA;

/**************************************************************/
/* BreakPoint definitions */

#define AER_BP_TOGGLE   (DWORD) -1
#define AER_BP_OFF      0
#define AER_BP_ON       1

/**************************************************************/
/* Variable information
*/
/* String type */
typedef struct tagAERSTRING128
{
   CHAR  sz[MAX_AERSTRING128_LEN];
} AERSTRING128;
typedef AERSTRING128 *PAERSTRING128;

/* Mask type (32 bits only) */
typedef struct tagMASK_DATA
{
   DWORD    dwMask;        /* Bit specific mask */
   WORD     wNumber;       /* Number of bits set in mask */
} MASK_DATA;
typedef MASK_DATA *PMASK_DATA;
typedef const MASK_DATA *const PCMASK_DATA;

/* Axis Point type
   - task axes reference
   - stored values are in ENGLISH
*/
typedef struct tagAXISPOINT
{
   AXISMASK       mTaskAxis;              /* AXISINDEX_ */
   DOUBLE         fdValue[MAX_AXES];
} AXISPOINT;
typedef AXISPOINT *PAXISPOINT;
typedef const AXISPOINT *const PCAXISPOINT;

/* Call Stack Parameter type
   - call stack index reference
*/
typedef struct tagCSPARM
{
   CSPARMMASK  mCSParm1;                  /* CSPARMINDEX_ */
   CSPARMMASK  mCSParm2;                  /* CSPARMINDEX_ */
   DOUBLE      fdValue[MAX_CSPARMS];
} CSPARM;
typedef CSPARM *PCSPARM;
typedef const CSPARM *const PCCSPARM;

/**************************************************************/
/* Program information
*/

#define  MAX_PROGRAMS         100
#define  MAX_PROG_NAME_LEN    32
#define  MAX_PROG_LABEL_LEN   32

/* Program handle - unique name for each program
*/
typedef struct tagAER_PROG_HANDLE
{
   CHAR  szName[MAX_PROG_NAME_LEN];
} AER_PROG_HANDLE;
typedef AER_PROG_HANDLE  *PAER_PROG_HANDLE;

/* Program label - unique name for each label
*/
typedef struct tagAER_PROG_LABEL
{
   CHAR     szName[MAX_PROG_LABEL_LEN];   /* Label name */
} AER_PROG_LABEL;
typedef AER_PROG_LABEL  *PAER_PROG_LABEL;
typedef struct tagAER_PROG_LABEL_INFO
{
   AER_PROG_LABEL Label;         /* Label */
   DWORD          dwLine960;     /* Label line number */
} AER_PROG_LABEL_INFO;
typedef AER_PROG_LABEL_INFO   *PAER_PROG_LABEL_INFO;

#define MAX_FILE_INFO_LEN  256           // should be _MAX_PATH ?
typedef struct tagAER_PROG_FILE_INFO
{
   CHAR     szFile[MAX_FILE_INFO_LEN];   /* path and drive of filename   */
   DWORD    dwDate;                      /* File Date */
   DWORD    dwTime;                      /* File Time (time_t) */
} AER_PROG_FILE_INFO;
typedef AER_PROG_FILE_INFO *PAER_PROG_FILE_INFO;

/* Program header
   contains compile, allocation, loading information
*/
typedef struct tagAER_PROG_HEADER
{
   DWORD    dwNumLines960; /* Number of program code lines */
   DWORD    dwSizeBytes;   /* Size in bytes for entire program code section */
   DWORD    dwNumLabels;   /* Number of program labels   */
   DWORD    dwNumDoubles;  /* Number of double variables */
   DWORD    dwNumTempDoubles;  /* Number of double variables that are temporary */
   DWORD    dwNumStrings;  /* Number of string variables */
   AER_PROG_FILE_INFO tFileInfo;   /* File Info */
} AER_PROG_HEADER;
typedef AER_PROG_HEADER  *PAER_PROG_HEADER;
#define  PROGTYPE_NORMAL            0
#define  PROGTYPE_QUEUE             1
#define  PROGTYPE_QUEUE_NO_FREE     2
//#define  PROGTYPE_NORMAL_STICKY     3
//#define  PROGTYPE_QUEUE_NO_FREE_STICKY  4

/* Program Status
*/
typedef union tagAER_PROG_STATUS
{
   DWORD    dwStatuss;
   struct
   {
      unsigned TaskAssociated:MAX_TASKS;  /* Mask of tasks that are associated with this program         */
      unsigned MemoryAllocated:1;         /* AerProgramAllocate has been called for this program         */
      unsigned LabelFilled:1;             /* All labels that have been allocated, have been downloaded   */
      unsigned Queue:1;                   /* True if a queued program                                    */
      unsigned nCodeFilled:1;             /* All code lines that has been allocated, has been downloaded */
      unsigned qBufferStarted:1;          /* A queue line has been downloaded since start                */
      unsigned qBufferFull:1;             /* Queue is currently full                                     */
      unsigned qBufferEmpty:1;            /* Queue is currently empty                                    */
      unsigned bSpare:1;                  /* Sticky? */
      unsigned TaskActive:MAX_TASKS;      /* Mask of tasks that are active with this program             */
   } Bit;
} AER_PROG_STATUS;
typedef AER_PROG_STATUS *PAER_PROG_STATUS;

/**************************************************************/
/* Status information (program, axis, task) MMI polls for these items constantly
*/
typedef struct tagAER_PROG_INFO
{
   AER_PROG_STATUS   Status;        /* Program status information    */
   DWORD             dwNumLines960; /* Number of program code lines  */
   DWORD             dwNumDoubles;  /* Number of double variables    */
   DWORD             dwNumTempDoubles;  /* Number of double variables that are temporary */
   DWORD             dwNumStrings;  /* Number of string variables    */
   DWORD             dwNumLabels;   /* Number of program labels      */
   struct
   {
      DWORD          dwCurrentLine960;    /* Current 960 line number */
      DWORD          dwCurrentLineUser;   /* Current user line number */
      DWORD          dwCurrentQueueLines; /* open slots availiable (used only for queues) */
      DWORD          dwSpare;             /* spare */
   } Task[MAX_TASKS];
} AER_PROG_INFO;
typedef AER_PROG_INFO  *PAER_PROG_INFO;

typedef struct tagAER_AXIS_DATA_IO
{
   DOUBLE AnalogIn[ANALOGS_IN_EACH_DRIVE];    // double representation of a 16 bit word
   WORD   BinaryIn;                           // one word, though only the lower DIG_BITS_IN_EACH_DRIVE are actually used
   WORD   BinaryOut;                          // one word, though only the lower DIG_BITS_IN_EACH_DRIVE are actually used
   WORD   ExpBinaryIn;                        // one word
   WORD   ExpBinaryOut;                       // one word
} AER_AXIS_DATA_IO;
typedef AER_AXIS_DATA_IO   *PAER_AXIS_DATA_IO;
//
typedef struct tagAER_AXIS_DATAD
{
   DOUBLE   dPos;           /* Position feedback (with calibration, homeoffset), reflects PositionCnts Axis Parameter   (counts)  */
   DOUBLE   dPosCmd;        /* Position command (with calibration, homeoffset), reflects PositionCmdCnts Axis Parameter (counts) */
   DOUBLE   dPosCal;        /* Calibration value added to position command  (counts)  */
   DOUBLE   dPosCam;        /* Cam value added to position command  (counts)  */
   DOUBLE   dPosSecondary;  /* Position (without calibration), from secondary encoder feedback (counts)  */

   DOUBLE   dAvgVelCnts;    /* Average velocity (without calibration)       (cnts/msec) */
   DOUBLE   dPosProgCmd;    /* Position command             (user-prog-dist-units) */
   DOUBLE   dFixtureOffset; /* Active Fixture offset value  (user-prog-dist-units) */

   DOUBLE   dPosEncoder;    /* Position feedback (without calibration, homeoffset), reflects encoder position read from drive */
   DOUBLE   dTargetPos;     /* (user-prog-dist-units) only for G0,G1,G2,G3 */

   DOUBLE   dRollOver;      /* Rollover distance, if not 0.0, user-program units rollover to zero at this value. (user-virt-dist-units) */
   DOUBLE   dPosFactor;     /* Position conversion factor (multiply by to convert:
                                          (counts --> user-virt-dist-units)) */
   DOUBLE   dAvgVel;        /* Average velocity (without calibration), takes into account G70/G71,G75,G76  (user-virt-dist-units/user-prog-time-units) */

   DWORD    dwFaultStatus;  /* Reflects Axis parameter - Fault */
   DWORD    dwDriveStatus;  /* Reflects Axis parameter - DriveStatus */
   DWORD    dwAxisStatus;   /* Reflects Axis parameter - AxisStatus */

   WORD     wCurrent;       /* Feedback current */
   WORD     wCurrentCommand;/* Commanded current */

   BYTE     byType;         /* =AXISTYPE_xxxx */
   BYTE     byNumDecimals;  /* Number of Decimals (NumDecimalsxxx axis parameter value, where xxx based on dwType of axis, ad G70/G71 mode) */
   BYTE     byUnits;        /* Whether owning task is in Metric(0->G71) or English(1->G70) or Counts(2->G72) */
   BYTE     byMinutes;      /* Whether owning task is in per/min(G75) or not (G76) */
   LONG     lGantryOffset;  /* Gantry offset (if any) in cnts */
   WORD     wSpare;

   AER_AXIS_DATA_IO IOData;

   DOUBLE   dGantryOffset;
   DWORD    dMoreSpare[6];

} AER_AXIS_DATAD;
typedef AER_AXIS_DATAD   *PAER_AXIS_DATAD;
//
// Note: Suppose programA calls ProgramB which calls programC, which is now running.
//       Then all "current program" data below refers to:
//         programA if TaskMode1.RunOverMode is TRUE
//         programC if TaskMode1.RunOverMode is FALSE
//
typedef struct tagAER_TASK_DATA
{
   AER_TASK_STATUS   Status;              /* Reflects Task Parms - Status1, Status2, Status3 */
   AER_TASK_MODE     Mode;                /* Reflects Task Parm - Mode1 */
   DWORD             dwCallStackDepth;    /* Current program callstack depth (0 if no prog running) */
   DWORD             dwCurrentProgNumber; /* Current program running (returns 0 if no program running) */
   DWORD             dwCurrentLine960;    /* Current program line as 960 knows it (zero based) */
   DWORD             dwCurrentLineUser;   /* Current program line as User knows it (normally one based) */
   DWORD             dwCurrentPriorityLevel; /* Current program priority (PRIORITYLEVEL_xxx constant) */
   AERERR_CODE       Fault;               /* Reflects Task Parm - TaskFault */
   AERERR_CODE       Warning;             /* Reflects Task Parm - TaskWarning */
   DWORD             dwMoreSpare[16];
   //WORD              wProgNumberSticky;
   //WORD              wMoreSpare[31];
} AER_TASK_DATA;
typedef AER_TASK_DATA *PAER_TASK_DATA;
typedef const AER_TASK_DATA *const PCAER_TASK_DATA;

#define  PRIORITYLEVEL_NORMAL    0
#define  PRIORITYLEVEL_MEDIUM    1
#define  PRIORITYLEVEL_FAULT     2
#define  PRIORITYLEVEL_HIGHEST   3
#define  MAX_PRIORITYLEVEL       4

/**************************************************************/
/* Pointer Type Definitions
*/

/* Pointer structure */

//
// NOTE : Upper 2 bits of bType in the "Const" structure below is reserved. If any bits in it set, the code assummes that the
//        index is instead a string variable (see "wVarNum") (had to do it this goofy way, because the PTR_DATA structure size
//        could not be increased, when  string variable option was added.

#define PTRNUMSPECIAL (DWORD)-1  // special element number (PTRTYPE_STR_GLOBAL_VAR type only) for system error message

typedef struct tagPTR_DATA     // 10 bytes
{
   BYTE  bType;   /* =PTRTYPE_xxxx */
   BYTE  bTask;   /* to force a specific task (=BYTE_NULL for default task) */
   WORD  wNum;    /* element number */

   /* Array offset information (PTRTYPE_NULL) */
   struct
   {
      BYTE  bType;      /* =PTRTYPE_NULL or  =PTRTYPE_GLOBAL_VAR or =PTRTYPE_TASK_VAR or =PTRTYPE_PROGRAM_VAR */
      BYTE  bTask;      /* to force a specific task (=BYTE_NULL for default task) */
      WORD  wNum;       /* element number */
   } ArrayOffset;

   /* Additional index information */
   struct
   {
      /*
      Axis index information (TASKAXISINDEX or PHYSAXISINDEX)

              PTRTYPE_DBL_AXIS_PARM         (required)
              PTRTYPE_DBL_POSITION          (required)
              PTRTYPE_DBL_PRESET            (required)

      Task axis index information (TASKAXISINDEX)
              This information is required for all task axis index PTRTYPEs.

              PTRTYPE_DBL_APT_GLOBAL_VAR    (required)
              PTRTYPE_MASK_APT_GLOBAL_VAR   (optional)
              PTRTYPE_DBL_APT_TASK_VAR      (required)
              PTRTYPE_MASK_APT_TASK_VAR     (optional)

      Call stack parameter index information (CSPARMINDEX)

              PTRTYPE_DBL_CSPARM_VAR        (required)
              PTRTYPE_MASK_CSPARM_VAR       (optional)
      */
      union
      {
         struct         // if index is a constant
         {
            BYTE  bIndex;
            BYTE  bType;         /* =PTRINDEXTYPE_xxxx */
         } Const;
         //
         //  wVarNum allowed only for PTRTYPE_DBL_AXIS_PARM
         //                           PTRTYPE_BYTE_Dxxxx, PTRTYPE_WORD_Wxxxx,  PTRTYPE_BYTE_EDxxxx, PTRTYPE_WORD_Exxxx,
         //                           PTRTYPE_DBL_Axxxx
         //                           PTRTYPE_DBL_CSPARM_VAR, PTRTYPE_MASK_CSPARM_VAR
         WORD wVarNum;  // if index is a string/double variable (upper PTRINDEXTYPE_MAX_BITS of this reserved for the variable type !)

      };
   } Index;

} PTR_DATA;
typedef PTR_DATA *PPTR_DATA;
typedef const PTR_DATA *const PCPTR_DATA;

/* Mask type (64 bits) */
//
//
// NOTE : Upper byte of wNumberr in the "Const" is reserved. If any bits in it set, the code assummes that the
//        MASK_DATA64 is a string variable (see Ptrrr structure) (had to do it this goofy way, because the MASK_DATA64 structure size
//        could not be increased, when the string variable option was added.

#define MASK_DATATYPE_VAR 0xFF00
typedef struct tagMASK_DATA64
{
   union
   {
      struct  // if axismask is a constant
      {
         WORD     wNumberr;       /* Number of bits set in mask (MUST be first element in this structure) */
         DWORD    dwMask1;        /* lower 32 axes */
         DWORD    dwMask2;        /* upper 32 axes/prms */
      } Const;
              // or if axismask is a string variable
      PTR_DATA Ptrrr;             /* use when wType=DATATYPE_POINTER (must set Ptr.bType=PTRTYPE_STR_xxx or PTRTYPE_NULL)  */
   };
} MASK_DATA64;
typedef MASK_DATA64 *PMASK_DATA64;
typedef const MASK_DATA64 *const PCMASK_DATA64;


/**************************************************************/
/* Data Type Definitions
*/

/* Data type defines */
#define DATATYPE_NULL      0
#define DATATYPE_LITERAL   1
#define DATATYPE_POINTER   2
#define DATATYPE_MASK      3

/* DWORD data structure */
typedef struct tagDWORD_DATA    // 12 bytes
{
   WORD        wType;      /* =DATATYPE_xxxx (but cannot be DATATYPE_MASK) */
   union
   {
      DWORD    dwValue;    /* use when wType=DATATYPE_LITERAL */
      PTR_DATA Ptr;        /* use when wType=DATATYPE_POINTER (must set Ptr.bType=PTRTYPE_DBL_xxxx)  */
   } Data;
} DWORD_DATA;
typedef DWORD_DATA *PDWORD_DATA;
typedef const DWORD_DATA *const PCDWORD_DATA;

/* Double data structure */
typedef struct tagDOUBLE_DATA
{
   WORD        wType;      /* =DATATYPE_xxxx */
   union
   {
      DOUBLE    fdValue;    /* use when wType=DATATYPE_LITERAL */
      PTR_DATA  Ptr;        /* use when wType=DATATYPE_POINTER (must set Ptr.bType=PTRTYPE_DBL_xxxx) */
      MASK_DATA64 mMask64;  /* use when wType=DATATYPE_MASK */
   } Datad;
} DOUBLE_DATA;
typedef DOUBLE_DATA *PDOUBLE_DATA;
typedef const DOUBLE_DATA *const PCDOUBLE_DATA;

/* String data structure */
typedef struct tagSTRING128_DATA
{
   WORD        wType;      /* =DATATYPE_xxxx (but cannot be DATATYPE_MASK) */
   union
   {
      AERSTRING128   String128;  /* use when wType=DATATYPE_LITERAL */
      PTR_DATA       Ptr;        /* use when wType=DATATYPE_POINTER (must set Ptr.bType=PTRTYPE_STR_xxxx) */
   } Data;
} STRING128_DATA;
typedef STRING128_DATA *PSTRING128_DATA;
typedef const STRING128_DATA *const PCSTRING128_DATA;

/*
Complex data structures
*/

/* Math double data structure (a math operation stack element) */
typedef struct tagMATHDBL_DATA
{
   WORD           wType;      /* =MATHDBLTYPE_VALUE or =MATHDBLTYPE_xxxx */
   DOUBLE_DATA    Double;     /* used when wType=MATHDBLTYPE_VALUE only */
} MATHDBL_DATA;
typedef MATHDBL_DATA *PMATHDBL_DATA;
typedef const MATHDBL_DATA *const PCMATHDBL_DATA;


/* Double to String data structure */
typedef struct tagDBL2STR_DATA
{
   WORD           wCount;     /* Number of decimal places (DWORD_NULL for default) */
   DOUBLE_DATA    Double;
} DBL2STR_DATA;
typedef DBL2STR_DATA *PDBL2STR_DATA;

/* Math string data structure */
typedef struct tagMATHSTR_DATA
{
   WORD              wType;      /* =MATHSTRTYPE_STRING or =MATHSTRTYPE_DBL2STR or =MATHSTRTYPE_xxxx */
   union
   {
      DBL2STR_DATA   DblStr;     /* used when wType=MATHSTRTYPE_DBL2STR only */
      STRING128_DATA String128;  /* used when wType=MATHSTRTYPE_STRING only */
   } Data;
} MATHSTR_DATA;
typedef MATHSTR_DATA *PMATHSTR_DATA;
typedef const MATHSTR_DATA *const PCMATHSTR_DATA;
#define MATHSTRTYPE_STRING          1
#define MATHSTRTYPE_DBL2STR         2

/* Conditional data structure */
typedef struct tagCOND_DATA
{
   WORD           wType;         /* =CONDTYPE_xxxx */
   DOUBLE_DATA    Data1;
   DOUBLE_DATA    Data2;
} COND_DATA;
typedef COND_DATA *PCOND_DATA;
typedef const COND_DATA *const PCCOND_DATA;

/**************************************************************/
/*
Variable length list Definitions
   These structures must be the last item in the code packet.
*/

/* DOUBLE data list */
#define MAX_AXISDOUBLE_LIST       32
typedef struct tagAXISDOUBLE_LIST
{
   MASK_DATA64 Mask;
   DOUBLE_DATA mElement[MAX_AXISDOUBLE_LIST];
} AXISDOUBLE_LIST;
typedef AXISDOUBLE_LIST *PAXISDOUBLE_LIST;
typedef const AXISDOUBLE_LIST *const PCAXISDOUBLE_LIST;

#define MAX_AXES_PVT MAX_AXISDOUBLE_LIST
#define MAX_AXES_G1G0  MAX_AXISDOUBLE_LIST

/* Axis Point data structure (MUST BE THE SAME AS THE CSPARM_DATA STRUCTURE)*/
typedef struct tagAXISPOINT_DATA
{
   WORD              wType;   /* =DATATYPE_ */
   union
   {
      AXISDOUBLE_LIST   List;    /* use when wType=DATATYPE_LITERAL (its a APT constant) */
      PTR_DATA          Ptr;     /* use when wType=DATATYPE_POINTER (its a APT variable) (must set Ptr.bType=PTRTYPE_APT_xxxx) */
      STRING128_DATA    Str;     /* use when wType=DATATYPE_MASK (its a string) */
   } Data;
} AXISPOINT_DATA;
typedef AXISPOINT_DATA *PAXISPOINT_DATA;
typedef const AXISPOINT_DATA *const PCAXISPOINT_DATA;

/* Call Stack Parameter data structure (MUST BE THE SAME AS THE AXISPOINT_DATA STRUCTURE) */
typedef struct tagCSPARM_DATA
{
   WORD              wType;   /* =DATATYPE_ (but cannot be DATATYPE_MASK) */
   union
   {
      AXISDOUBLE_LIST   List;     /* use when wType=DATATYPE_LITERAL */
      PTR_DATA          Ptr;      /* use when wType=DATATYPE_POINTER (must set Ptr.bType=PTRTYPE_CSP_xxxx) */
   } Data;
} CSPARM_DATA;
typedef CSPARM_DATA *PCSPARM_DATA;
typedef const CSPARM_DATA *const PCCSPARM_DATA;



/* Math double data list */

#define MAX_STRING_FUNCT_LISTS     3
#define MAX_STRING_FUNCT_LISTD     2
typedef struct tagSTRING_FUNCT     /* string function */
{
   WORD            wSNum;
   WORD            wDNum;
   MATHSTR_DATA    ElementS[MAX_STRING_FUNCT_LISTS];
   MATHDBL_DATA    ElementD[MAX_STRING_FUNCT_LISTD];
} STRING_FUNCT;
typedef STRING_FUNCT *PSTRINGFUNCT;


/* WARNING ! STACK_TYPE_xxxx constants are limited to 256, because they are stored in a byte */
#define STACK_TYPE_NONE      0   // must be zero
#define STACK_TYPE_STACK     1
#define STACK_TYPE_STR_FUNCT 2
//#define STACK_TYPE_AXISMASK  3
#define STACK_TYPE_APTVAR    4

#define MAX_MATHDBL_LIST      20        // CANNOT EXCEED 255  (unsigned char limit)
typedef struct tagMATHDBL_LIST
{
   unsigned char csType;    /* STACK_TYPE_ const */
   unsigned char csSize;    /* num elements */
   union
   {
      MATHDBL_DATA Element[MAX_MATHDBL_LIST];  // used for STACK_TYPE_STACK only
      STRING_FUNCT StrVar;                     // used for STACK_TYPE_STR_FUNCT only
   } Data;
} MATHDBL_LIST;
typedef MATHDBL_LIST *PMATHDBL_LIST;
typedef const MATHDBL_LIST *const PCMATHDBL_LIST;

/* Math string data list */
#define MAX_MATHSTR_LIST      8        // CANNOT EXCEED 255 (unsigned char limit)
typedef struct tagMATHSTR_LIST
{
   unsigned char csType;    /* STACK_TYPE_ const */
   unsigned char csSize;    /* num elements */
   union
   {
      MATHSTR_DATA Element[MAX_MATHSTR_LIST];      /* use when csType=STACK_TYPE_STACK */
      STRING_FUNCT StrVar;                         /* use when csType=STACK_TYPE_STR_FUNCT */
      MASK_DATA64 mMask64;                         /* use when ?? */
      PTR_DATA DestPtr;                            /* use when csType=STACK_TYPE_APTVAR  (pointer must be PTRTYPE_DBL_APT_***) */
   } Data;
} MATHSTR_LIST;
typedef MATHSTR_LIST *PMATHSTR_LIST;
typedef const MATHSTR_LIST *const PCMATHSTR_LIST;




/**************************************************************/
/* Complex data/list Definitions
*/

/* Linear motion data structure */
#define MAX_RETRACE_AXES 5
typedef struct tagLINEAR_DATA
{
   WORD              wMType;         /* MOTIONTYPEn_ */
   DWORD             dwMode;         /* MOTIONMODEn_ */
   struct
   {
      DOUBLE            fdStartPosInch[MAX_RETRACE_AXES];
      DWORD             dwStartPhysAxes;  // mask of axes in StartPos
      DWORD             dwLastMoveMode;   /* MOTIONMODEn_ */
      DWORD             dwLastMoveType;   /* MOTIONTYPEn_ */
      DOUBLE            fdFeedRateFwordUserUnits; // F word in user units (what these are is dependent on G70/G71, G75/G76, G93/G94/G95)
      DOUBLE            fdFeedRateEwordUserUnits; // E word in user units (what these are is dependent on G75/G76, G93/G94/G95)
   } RetraceStuff;
   AXISPOINT_DATA    Target;        /* Must be last element */
} LINEAR_DATA;
typedef LINEAR_DATA *PLINEAR_DATA;
typedef const LINEAR_DATA *const PCLINEAR_DATA;

typedef struct tagSLICE_DATA
{
   AXISINDEX         iStepAxis;
   AXISINDEX         iScanAxis;
   DOUBLE_DATA       dStepJumpPos;
   DOUBLE_DATA       dScanJumpPos;
   DOUBLE_DATA       dStepEndPos;
   DOUBLE_DATA       dScanEndPos;
   DOUBLE_DATA       dVectorSpeed;
   DOUBLE_DATA       dStepAxisJumpSpeed;
   DOUBLE_DATA       dScanAxisJumpSpeed;
   DWORD_DATA        dwIntAccel;
   DWORD_DATA        dwIntDecel;
}SLICE_DATA;
typedef SLICE_DATA *PSLICE_DATA;
typedef const SLICE_DATA *const PCSLICE_DATA;

/* Motion types (linear) */
#define AER_MOTIONTYPE_DEFAULT         0x00    /* none */
#define AER_MOTIONTYPE_PT_TO_PT        0x02    /* G0   */
#define AER_MOTIONTYPE_INTERP_LINE     0x03    /* G1   */
#define AER_MOTIONTYPE_MAX             0x04
/* These used by firmware for retrace only, for tracking previous line types */
#define AER_MOTIONTYPE_CW              0x04    /* G2/G202   */
#define AER_MOTIONTYPE_CCW             0x08    /* G3/G203   */
#define AER_MOTIONTYPE_ELLIPS          0x10   /* G202/G203 */

/* Motion modes (INTERPOLATION/SPLINE only) */
#define MOTIONMODE1_CUTTER_LEADIN_LEFT    0x00000001
#define MOTIONMODE1_CUTTER_LEADIN_RIGHT   0x00000002
#define MOTIONMODE1_CUTTER_LEADOUT        0x00000004
#define MOTIONMODE1_MASK                  0x00000007

#define MOTIONMODE2_CONTINUOUS            0x00000010   /* G8 */
#define MOTIONMODE2_DECEL_TO_ZERO         0x00000020   /* G9 */
#define MOTIONMODE2_MASK                  0x00000030

#define MOTIONMODE7_IJK_ABSOLUTE          0x00000040   /* G118 */
#define MOTIONMODE7_MASK                  0x00000040

#define MOTIONMODE3_NORMALCY_ON_LEFT      0x00000100
#define MOTIONMODE3_NORMALCY_ON_RIGHT     0x00000200
#define MOTIONMODE3_NORMALCY_OFF          0x00000400
#define MOTIONMODE3_MASK                  0x00000700

#define MOTIONMODE4_SUPRESS_FIXTURE_OFF   0x00001000   /* G153 */
#define MOTIONMODE4_MASK                  0x00001000

//#define MOTIONMODE8_ASYNC                 0x00002000   /* G210 */
//#define MOTIONMODE8_ASYNC_FREE            0x00004000   /* G220 */
//#define MOTIONMODE8_MASK                  0x00006000

#define MOTIONMODE5_INT_ACCEL             0x00010000     /* Interrupt will happen first time accel phase bit comes on */
#define MOTIONMODE5_INT_DECEL             0x00020000     /* Interrupt will happen first time decel phase bit comes on */
#define MOTIONMODE5_MASK                  0x00030000

#define MOTIONMODE6_CUTTER_OFFSETS_ON     0x00100000
#define MOTIONMODE6_CUTTER_OFFSETS_ON_REV 0x00200000
#define MOTIONMODE6_CUTTER_OFFSETS_OFF    0x00400000
#define MOTIONMODE6_MASK                  0x00700000

//#define MOTIONMODE_ALLMODES (MOTIONMODE1_MASK | MOTIONMODE2_MASK | MOTIONMODE3_MASK | MOTIONMODE4_MASK | MOTIONMODE5_MASK | MOTIONMODE6_MASK | MOTIONMODE7_MASK | MOTIONMODE8_MASK)
#define MOTIONMODE_ALLMODES (MOTIONMODE1_MASK | MOTIONMODE2_MASK | MOTIONMODE3_MASK | MOTIONMODE4_MASK | MOTIONMODE5_MASK | MOTIONMODE6_MASK | MOTIONMODE7_MASK)
//#define MOTIONMODE_MODE_UNRESTRICTED (MOTIONMODE4_MASK | MOTIONMODE8_MASK)   // these OK for g0/g123 etc.
#define MOTIONMODE_MODE_UNRESTRICTED (MOTIONMODE4_MASK)   // these OK for g0/g123 etc.

/* Circular motion data structure */
typedef struct tagCIRCULAR_DATA
{
   /* Special data types to shrink structure size */
   //SHORT             sCCW;             /* 0 = void , + = CCW , - = CW */
   BYTE              byCCW;             /* 0 = void , 1 = CCW , 2 = CW */
   BYTE              byEllipse;
   DWORD             mTargetIndxPlusOne[2]; /* index (+1) of targets (optional) */
   DOUBLE_DATA       Target[2];        /* array of target values (ordered by ascending value of axis index) */
   WORD              mOffset;          /* mask of "center coords" (see OFFSETINDEX_ below) (minimum 1, maximum 2) */
   DOUBLE_DATA       Offset[2];        /* "center coords" (ordered by ascending value of OFFSETINDEX_) */
} CIRCULAR_DATA;
typedef CIRCULAR_DATA *PCIRCULAR_DATA;
typedef const CIRCULAR_DATA *const PCCIRCULAR_DATA;

// the below must be consective
#define OFFSETINDEX_I     0    // has to be zero
#define OFFSETINDEX_J     1
#define OFFSETINDEX_K     2
#define OFFSETINDEX_R     3
#define OFFSETINDEX_MAX   4
#define OFFSETMASK_I     0x1
#define OFFSETMASK_J     0x2
#define OFFSETMASK_K     0x4
#define OFFSETMASK_R     0x8
#define OFFSETMASK_FULL  0xF
/*
Call Back data structures
*/
// WARNING: this must match the similar constant defined in /ini/aerparam.pgm
#define CALLBACKTYPE_RESERVED     10000

#define ARG_TYPE_DWORD     0       /* pData points to a type DWORD */
#define ARG_TYPE_DOUBLE    1       /* pData points to a type DOUBLE */
//#define ARG_TYPE_STRING32  2       /* pData points to a type STRING32 */
#define ARG_TYPE_STRING128 3       /* pData points to a type STRING128 */
#define MAX_BLOB_SIZE 1040
/*
  Compiler passes down a CALLBACK_DATA to 960. 960, when executing a Callback
  translates this (resolving variables) into a CALLBACK_VALUE. This is passed back to front end
*/
typedef struct tagCALLBACK_DATA
{
   PTR_DATA       RetCode;               /* Must be first in this structure */
   DWORD          dwCallbackType;        /* =ACL_CALLBACK_xxxx */
   DWORD          dwArgs;                /* Number of callback arguments in below */
   BYTE           byBlob[MAX_BLOB_SIZE];
} CALLBACK_DATA;
typedef CALLBACK_DATA *PCALLBACK_DATA;
typedef const CALLBACK_DATA *const PCCALLBACK_DATA;

typedef struct tagCALLBACK_VALUE
{
   PTR_DATA       RetCode;               /* Must be first in this structure */
   DWORD          dwCallbackType;        /* =ACL_CALLBACK_xxxx */
   DWORD          dwArgs;                /* Number of "CALLBACK_BLOB, data" pairs in below */
   BYTE           byBlob[MAX_BLOB_SIZE]; /* Consists of CALLBACK_BLOB, data, CALLBACK_BLOB, data, etc. */
} CALLBACK_VALUE;
typedef CALLBACK_VALUE *PCALLBACK_VALUE;
typedef const CALLBACK_VALUE *const PCCALLBACK_VALUE;

typedef struct tagCALLBACK_BLOB
{
   WORD  wType;       /* =ARG_TYPE_xxxx */
   WORD  wSize;       /* size of data following this */
   void* pDataJunk;   /* this does not appear to be used */
} CALLBACK_BLOB;
typedef CALLBACK_BLOB   *PCALLBACK_BLOB;
typedef const CALLBACK_BLOB   *const PCCALLBACK_BLOB;

/*
Tool Table structures/definitions
*/
#define  MAX_TOOL_NAME_LEN    32

typedef struct tagAER_TOOL
{
   DWORD    mFlags;
   DOUBLE   dCutterRadius;
   DOUBLE   dCutterLength;
   DOUBLE   dCutterWear;
   DOUBLE   dOffsetX;
   DOUBLE   dOffsetY;
   DOUBLE   dFWord;            /* if non zero, specifies feed */
   DOUBLE   dSWord;            /* if non zero, specifies spindle speed */
   CHAR     szName[MAX_TOOL_NAME_LEN];
} AER_TOOL;
typedef AER_TOOL  *PAER_TOOL;
typedef const AER_TOOL  *const PCAER_TOOL;

typedef struct tagDIG_DRIVE_INFO
{
   WORD  wDriveHardware;     // see DRIVE_ID_ constants
   WORD  wDriveHardware2;    // least significant bit=1 if its an NDriveHL
   DWORD dwStickyBits;
   BYTE  byDriveSize;
   BYTE  byDriveVersion1;
   BYTE  byDriveVersion2;
   BYTE  byDriveVersion3;
   WORD  wFPGAVersion;
   WORD  wMXHVersion;
} DIG_DRIVE_INFO;
typedef DIG_DRIVE_INFO *PDIG_DRIVE_INFO;
typedef struct tagDIG_DRIVE_INFO2    // for generic return of drive data
{
   DWORD dwOne;
   DWORD dwTwo;
   DWORD dwThree;
   DWORD dwFour;
} DIG_DRIVE_INFO2;
typedef DIG_DRIVE_INFO2 *PDIG_DRIVE_INFO2;


/**************************************************************/
/*
Code Structure Definitions
*/

/*
Base code structure
*/
/* WARNING ! CODEATTR_xxxx constants are limited to 256, because they are stored in a byte */
#define CODEATTR_BLOCK_DELETE  0x0001
#define CODEATTR_BREAK_POINT   0x0002
#define CODEATTR_MOTION        0x0004
#define CODEATTR_BLOCK_DELETE2 0x0008
#define CODEATTR_CONVERT_UNITS 0x0010
#define CODEATTR_MASK          0x001F
typedef struct tagCODE_BASE
{
   BYTE     bType;         /* =CODETYPE_xxxx */
   BYTE     bAttr;         /* =CODEATTR_xxxx */
   WORD     wSize;         /* Complete size of valid code packet data */
   DWORD    dwLineUser;    /* Corresponding user line number */
} CODE_BASE;
typedef CODE_BASE *PCODE_BASE;

/* PSO code structure */
typedef struct tagCODE_PSO
{
   CODE_BASE         Base;         /* Must be first element */
   DWORD             dwSubCode;    /* */
   MASK_DATA64       TaskAxisMask64;
   DOUBLE_DATA       Start;        /* start index, or start value, or channel1, or total time, or fixed distance  */
   DOUBLE_DATA       End  ;        /* nelements,   or end   value, or channel2, or ontime */
   DWORD_DATA        Dimension;    /* dimension (used only for PSOWINDOW) or ncycles  */
   DWORD_DATA        ResetMask;    /* */
   PTR_DATA          Variab;       /* variable */

} CODE_PSO;
typedef CODE_PSO *PCODE_PSO;

/*
Near call/jump code structure
*/
typedef struct tagCODE_NEAR_CALL
{
   CODE_BASE            Base;    /* Must be first element */
   DWORD_DATA           Line960;
   CSPARM_DATA          Parm;    /* Must be last element */
} CODE_NEAR_CALL;
typedef CODE_NEAR_CALL *PCODE_NEAR_CALL;


/*
Far call/jump code structure
*/
typedef struct tagCODE_FAR_CALL
{
   CODE_BASE            Base;    /* Must be first element */
   STRING128_DATA       Program;
   STRING128_DATA       Label;
   CSPARM_DATA          Parm;    /* Must be last element */
} CODE_FAR_CALL;
typedef CODE_FAR_CALL *PCODE_FAR_CALL;
/*
 SetCannedFunction code structure
*/
typedef struct tagCODE_SET_CANNEDFUNCTION
{
   CODE_BASE            Base;    /* Must be first element */
   DWORD_DATA           IDNum;
   STRING128_DATA       Program;
   STRING128_DATA       Label;
   DWORD_DATA           Flags;
} CODE_SET_CANNEDFUNCTION;
typedef CODE_SET_CANNEDFUNCTION  *PCODE_SET_CANNEDFUNCTION;
/*
 ExeCannedFunction code structure
*/
typedef struct tagCODE_EXE_CANNEDFUNCTION
{
   CODE_BASE            Base;    /* Must be first element */
   DWORD_DATA           IDNum;
   DWORD_DATA           TaskNum;
   CSPARM_DATA          Parm;    /* Must be last element */
} CODE_EXE_CANNEDFUNCTION;
typedef CODE_EXE_CANNEDFUNCTION  *PCODE_EXE_CANNEDFUNCTION;

#define AER_CANNEDFUNCTION_FOLLOW_AUTOMODE  0x0001 /* follows Auto/Step Mode */
#define AER_CANNEDFUNCTION_INTERRUPT        0x0002 /* Interrupts return, follows ReturnType */

#define AER_STD_CANNEDFUNCTION_ID_BEGINOFBLOCK  -1 /* Beginning of Block */
#define AER_STD_CANNEDFUNCTION_ID_ENDOFBLOCK    -2 /* End of Block */
#define AER_STD_CANNEDFUNCTION_ID_PROGSTART     -3 /* Program Started  */
#define AER_STD_CANNEDFUNCTION_ID_PROGEND       -4 /* Program End */
#define AER_STD_CANNEDFUNCTION_ID_LAST          -4 /* last canned function id */
#define NUM_STD_CANNEDFUNCTIONS                 4

#define IsStdCannedFunction(dwID)   \
        (((LONG) dwID < 0) && ((LONG) dwID >= AER_STD_CANNEDFUNCTION_ID_LAST))
#define StdCannedFunctionIDToArrayID(dwID)  labs( (LONG) dwID )

typedef struct tagCANNEDFUNCTION_DATA
{
   DWORD             dwID;
   AER_PROG_HANDLE   Handle;
   AER_PROG_LABEL    Label;
   DWORD             mFlags;
   DWORD             bActive;
} CANNEDFUNCTION_DATA;
typedef CANNEDFUNCTION_DATA *PCANNEDFUNCTION_DATA;

typedef struct tagCODE_PROBE_SET
{
   CODE_BASE      Base;          /* Must be first element */
   DWORD_DATA     CommandType;   // 0 for PROBE MODE, 1 for PROBE INPUT
   DWORD_DATA     Mode;          // used with PROBE MODE command - indicate SW or HW mode
   DWORD_DATA     Level;         // used with PROBE MODE command - indicates active level of input
   DWORD_DATA     InputType;     // used with PROBE INPUT command - specifies type of input, drive, Ethernet etc..
   DWORD_DATA     BitNum;        // used with PROBE INPUT command - input bit number
   MASK_DATA64    m64Drive;      // used with PROBE INPUT command - drive number - translate from an
                                 // axis name on the command line
   PTR_DATA       Dest;          // used with PROBE MODE command - points to variable array - string PTRTYPE_

}  CODE_PROBE_SET;
typedef CODE_PROBE_SET *PCODE_PROBE_SET;

/*
Double assignment code structure
*/
typedef struct tagCODE_ASSIGN_DBL
{
   CODE_BASE      Base;    /* Must be first element */
   PTR_DATA       DestPtr; /* LHS (must be numeric PTRTYPE_) */
   MATHDBL_LIST   Stack;   /* RHS (must be last element in structure) */
} CODE_ASSIGN_DBL;
typedef CODE_ASSIGN_DBL *PCODE_ASSIGN_DBL;

typedef struct tagCODE_ASSIGN_DBL2
{
   CODE_BASE      Base;    /* Must be first element */
   PTR_DATA       DestPtr; /* LHS (numeric PTRTYPE_) */
   DOUBLE         fdOldValue;      /* added for retrace */
   MATHDBL_LIST   Stack;   /* RHS (must be last element in structure) */
} CODE_ASSIGN_DBL2;
typedef CODE_ASSIGN_DBL2 *PCODE_ASSIGN_DBL2;


/*
String assignment code structure
*/
typedef struct tagCODE_ASSIGN_STR
{
   CODE_BASE      Base;    /* Must be first element */
   PTR_DATA       DestPtr; /* LHS (string PTRTYPE_) */
   MATHSTR_LIST   Stack;   /* RHS (must be last element in structure) */
} CODE_ASSIGN_STR;
typedef CODE_ASSIGN_STR *PCODE_ASSIGN_STR;


/*
Axis Point assignment code structure
*/
typedef struct tagCODE_ASSIGN_APT
{
   CODE_BASE         Base;    /* Must be first element */
   PTR_DATA          DestPtr; /* LHS (Axis Point PTRTYPE_) */
   AXISPOINT_DATA    Point;   /* RHS (must be last element in structure) */
} CODE_ASSIGN_APT;
typedef CODE_ASSIGN_APT *PCODE_ASSIGN_APT;


/*
DWORD data code structure
*/
typedef struct tagCODE_DWORD1
{
   CODE_BASE      Base;    /* Must be first element */
   DWORD_DATA     Dword;
} CODE_DWORD1;
typedef CODE_DWORD1 *PCODE_DWORD1;

/*
Double data code structure
*/
typedef struct tagCODE_DOUBLE1
{
   CODE_BASE      Base;    /* Must be first element */
   DOUBLE_DATA    Double;
} CODE_DOUBLE1;
typedef CODE_DOUBLE1 *PCODE_DOUBLE1;


/*
MASK code structure
*/
typedef struct tagCODE_MASK
{
   CODE_BASE      Base;    /* Must be first element */
   MASK_DATA64    TaskAxisMask64;     /* TASKAXISINDEX_ */
} CODE_MASK;
typedef CODE_MASK *PCODE_MASK;

/*
MASK code structure (plus a DWORD const or var)
*/
typedef struct tagCODE_MASKPLUS
{
   CODE_BASE      Base;    /* Must be first element */
   MASK_DATA64    TaskAxisMask64;     /* TASKAXISINDEX_ */
   DWORD_DATA     Param;   /* extra data */
} CODE_MASKPLUS;
typedef CODE_MASKPLUS *PCODE_MASKPLUS;
/*
MASK code structure (plus a DOUBLE const or var)
*/
typedef struct tagCODE_MASKPLUS2
{
   CODE_BASE      Base;    /* Must be first element */
   MASK_DATA64    TaskAxisMask64;     /* TASKAXISINDEX_ */
   DOUBLE_DATA    Param;   /* extra data */
} CODE_MASKPLUS2;
typedef CODE_MASKPLUS2 *PCODE_MASKPLUS2;


/*
Axis DOUBLE data list code structure
*/
typedef struct tagCODE_AXISDOUBLE_LIST
{
   CODE_BASE         Base;    /* Must be first element */
   AXISDOUBLE_LIST   List;    /* Must be last element */
} CODE_AXISDOUBLE_LIST;
typedef CODE_AXISDOUBLE_LIST *PCODE_AXISDOUBLE_LIST;
typedef const CODE_AXISDOUBLE_LIST *const PCCODE_AXISDOUBLE_LIST;


/*
Axis Point data list code structure
*/
typedef struct tagCODE_AXISPOINT
{
   CODE_BASE      Base;    /* Must be first element */
   AXISPOINT_DATA Point;   /* Must be last element */
} CODE_AXISPOINT;
typedef CODE_AXISPOINT *PCODE_AXISPOINT;
typedef const CODE_AXISPOINT *const PCCODE_AXISPOINT;


/*
Conditional jump code structure
*/
typedef struct tagCODE_COND_JUMP
{
   CODE_BASE         Base;       /* Must be first element */
   COND_DATA         Cond;
   DWORD_DATA        Line960;
} CODE_COND_JUMP;
typedef CODE_COND_JUMP *PCODE_COND_JUMP;


/*
Conditional wait code structure
*/
typedef struct tagCODE_COND_WAIT
{
   CODE_BASE         Base;       /* Must be first element */
   COND_DATA         Cond;
   DOUBLE_DATA       TimeOut;       /* milliseconds timeout */
   DWORD_DATA        TimeOutAction; /* 0 throws task fault, other means put timeout erroro in ErrCode parameter */
   CODE_ASSIGN_DBL   PreAssignDbl;
} CODE_COND_WAIT;
typedef CODE_COND_WAIT *PCODE_COND_WAIT;


/*
Setting Mode code structure
*/
typedef struct tagCODE_MODE
{
   CODE_BASE         Base;       /* Must be first element */
   AER_TASK_MODE     ModeMask;
   AER_TASK_MODE     ModeValue;
   DWORD             dwOldMode;  /* added for retrace */
} CODE_MODE;
typedef CODE_MODE *PCODE_MODE;


/*
Asyncronous Motion code structure
*/
typedef struct tagCODE_ASYNC_MOTION
{
   CODE_BASE         Base;    /* Must be first element */
   MASK_DATA64       TaskAxisMask64;
   WORD              wType;   /* ASYNCTYPE_ */
   DOUBLE_DATA       jData1;     // normally distance (direction if a freerun)
   DOUBLE_DATA       jData2;     // normally speed (sign is normally ignored)
   DOUBLE_DATA       jData3;     // number of cycles if oscillate, "special instruction" if not
} CODE_ASYNC_MOTION;
typedef CODE_ASYNC_MOTION *PCODE_ASYNC_MOTION;

#define ASYNCTYPE_MOVETO              0x000   /* limited by MaxFeedRate */
#define ASYNCTYPE_HOME                0x001   /* for the full home cycle, see below for home cycle parts only */
#define ASYNCTYPE_INDEX               0x002   /* limited by MaxFeedRate */
#define ASYNCTYPE_START               0x003   /* limited by MaxFeedRate */
#define ASYNCTYPE_INFEED              0x004
//#define ASYNCTYPE_QINDEX              5
//#define ASYNCTYPE_QMOVETO             6
//#define ASYNCTYPE_QHOME        7   /* MACHPARM_HomeType = 3 */
//#define ASYNCTYPE_ALTHOME      8   /* MACHPARM_HomeType = 1 */
//#define ASYNCTYPE_NOHOME       9   /* MACHPARM_HomeType = 2 */
#define ASYNCTYPE_HOME_LIM_MOV_IN     0x007   /* limited by MaxFeedRate */
#define ASYNCTYPE_HOME_LIM_MOV_OUT    0x008   /* limited by MaxFeedRate */
#define ASYNCTYPE_HOME_MARKER_MOV     0x009   /* limited by MaxFeedRate */
#define ASYNCTYPE_OSCILLATE           0x00A
//#define ASYNCTYPE_ALIGN               0x00B
//#define ASYNCTYPE_VHOME       12
#define ASYNCTYPE_HALT                0x064   // why so high ?


/*
Spindle motion code structure
*/
typedef struct tagCODE_SPINDLE
{
   CODE_BASE         Base;       /* Must be first element */
   SPINDLEINDEX      iSpindle;
   WORD              wType;      /* SPINDLETYPE_ */
} CODE_SPINDLE;
typedef CODE_SPINDLE *PCODE_SPINDLE;
#define SPINDLETYPE_ON_CW              1
#define SPINDLETYPE_ON_CCW             2
#define SPINDLETYPE_OFF                3
#define SPINDLETYPE_REORIENT           4
#define SPINDLETYPE_ON_CW_ASYNC        5
#define SPINDLETYPE_ON_CCW_ASYNC       6

/*
Motion code structure
*/
typedef struct tagCODE_MOTION
{
   CODE_BASE         Base;       /* Must be first element */
   void*             pProfilingData;
   LINEAR_DATA       Linear;     /* Must be last element */
} CODE_MOTION;
typedef CODE_MOTION *PCODE_MOTION;

/*
Slice Motion code structure
*/
typedef struct tagCODE_SLICE
{
   CODE_BASE         Base;       /* Must be first element */
   void*             pProfilingData;   // May not need this
   SLICE_DATA        SliceData;
} CODE_SLICE;
typedef CODE_SLICE *PCODE_SLICE;

/*
Circular Motion code structure
*/
#define COORD_SYSTEM_1     1
#define COORD_SYSTEM_2     2
#define MAX_COORD_SYSTEM   2
typedef struct tagCODE_MOTIONCIR
{
   CODE_BASE         Base;       /* Must be first element */
   CIRCULAR_DATA     Circular[MAX_COORD_SYSTEM];
   void*             pProfilingData;
   LINEAR_DATA       Linear;     /* Must be last element */
} CODE_MOTIONCIR;
typedef CODE_MOTIONCIR *PCODE_MOTIONCIR;

typedef struct tagCODE_MOTIONELLIPSE
{
   CODE_BASE         Base;       /* Must be first element */
   CIRCULAR_DATA     Circular[1];
   DOUBLE_DATA       EllipseP;
   DOUBLE_DATA       EllipseQ;
   DOUBLE_DATA       EllipseR;
   void*             pProfilingData;
   LINEAR_DATA       Linear;     /* Must be last element */
} CODE_MOTIONELLIPSE;
typedef CODE_MOTIONELLIPSE *PCODE_MOTIONELLIPSE;


/*
Call Back code structures
*/
typedef struct tagCODE_CALLBACK
{
   CODE_BASE      Base;    /* Must be first element */
   CALLBACK_DATA  Data;
} CODE_CALLBACK;
typedef CODE_CALLBACK *PCODE_CALLBACK;


/*
Slew code structure
*/
typedef struct tagCODE_SLEW
{
   CODE_BASE         Base;          /* Must be first element */
   DWORD_DATA        AxisPair;          /* port number  */

   // the following 3 members can be removed when the Compiler is changed to support
   // the new slew syntax
   //DWORD_DATA        Port;          /* port number  */
   //TASKAXISINDEX     iTaskAxis[2];
   //DOUBLE_DATA       FeedRate[2];
} CODE_SLEW;
typedef CODE_SLEW *PCODE_SLEW;

/*
Hand Wheel code structure
*/
typedef struct tagCODE_HANDWHEEL
{
   CODE_BASE         Base;       /* Must be first element */
   DWORD_DATA        Mode;       /* 1 is ON     0 is OFF  */
   MASK_DATA64       TaskAxisMask64;
   DOUBLE_DATA       Distance;
} CODE_HANDWHEEL;
typedef CODE_HANDWHEEL *PCODE_HANDWHEEL;

//typedef struct tagCODE_TRACK
//{
//   CODE_BASE         Base;
//   TASKAXISINDEX     iTaskAxis;
//   DOUBLE_DATA       MasterStart;
//   DOUBLE_DATA       MasterAccel;
//   DOUBLE_DATA       SlaveAccel;
//} CODE_TRACK;
//typedef CODE_TRACK *PCODE_TRACK;

typedef struct tagCODE_CFGMASTER
{
   CODE_BASE         Base;
   MASK_DATA64       TaskAxisMask64;
   MASK_DATA64       MasterAxisMask64;
   DWORD_DATA        Type;
} CODE_CFGMASTER;
typedef CODE_CFGMASTER *PCODE_CFGMASTER;

/*
Mset code structure
*/
typedef struct tagCODE_MSET
{
   CODE_BASE         Base;       /* Must be first element */
   MASK_DATA64       TaskAxisMask64;
   DOUBLE_DATA       dTorq;
   DWORD_DATA        dwAngle;    /* channel number  */
} CODE_MSET;
typedef CODE_MSET *PCODE_MSET;


/*
Auto Focus code structure
*/
//typedef struct tagCODE_AFCO
//{
//   CODE_BASE         Base;       /* Must be first element */
//   DWORD_DATA        Channel;    /* channel number  */
//   TASKAXISINDEX     iTaskAxis;
//   DOUBLE_DATA       FeedRate;
//   DOUBLE_DATA       DeadBand;
//   DOUBLE_DATA       Offset;
//   DOUBLE_DATA       AntiDive;
//} CODE_AFCO;
//typedef CODE_AFCO *PCODE_AFCO;


/*
  code structure
*/
typedef struct tagCODE_SOMECOMMAND2
{
   CODE_BASE         Base;       /* Must be first element */
   MASK_DATA64       TaskAxisMask64;
   DWORD_DATA        Table;
   DWORD_DATA        Mode;
   DOUBLE_DATA       DoubleData2;
   PTR_DATA          ReturnVar;
   MASK_DATA64       TaskAxisMask64Two;
   DOUBLE_DATA       DoubleData3;
   DOUBLE_DATA       DoubleData4;
   DOUBLE_DATA       DoubleData5;
   DOUBLE_DATA       DoubleData6;
   DOUBLE_DATA       DoubleData7;
   DOUBLE_DATA       DoubleData8;
   DWORD_DATA        Flags;
} CODE_SOMECOMMAND2;
typedef CODE_SOMECOMMAND2 *PCODE_SOMECOMMAND2;


/*
Pendant code structure
*/
//typedef struct tagCODE_PENDANT
//{
//   CODE_BASE         Base;    /* Must be first element */
//   WORD              wType;   /* PENDANTTYPE_ */
//   DWORD_DATA        Port;
//   union
//   {
//      struct
//      {
//         DWORD_DATA     Line;
//         STRING32_DATA  Text;
//      } SetText;
//      struct
//      {
//         DWORD_DATA     AnalogChannel;
//         PTR_DATA       DestPtr;
//      } GetAnalog;
//      struct
//      {
//         PTR_DATA       DestPtr;
//      } GetKey;
//      struct
//      {
//         DWORD_DATA     AnalogChannel;
//         TASKAXISINDEX  iTaskAxis;
//         DOUBLE_DATA    FeedRate;
//      } EnableJog;
//      struct
//      {
//         TASKAXISINDEX  iTaskAxis;
//      } DisableJog;
//   } Data;
//} CODE_PENDANT;
//typedef CODE_PENDANT *PCODE_PENDANT;
//#define PENDANTTYPE_ENABLE          1
//#define PENDANTTYPE_DISABLE         2
//#define PENDANTTYPE_SET_TEXT        3
//#define PENDANTTYPE_GET_ANALOG      4
//#define PENDANTTYPE_GET_KEY         5
//#define PENDANTTYPE_ENABLE_JOG      6
//#define PENDANTTYPE_DISABLE_JOG     7

typedef struct tagCODE_MEM
{  /* Drive memory reads/writes */
   CODE_BASE         Base;       /* Must be first element */
   MASK_DATA64       TaskAxisMask64;
   DWORD_DATA        MemType;
   DWORD_DATA        Address;
   DOUBLE_DATA       Value;
} CODE_MEM;
typedef CODE_MEM *PCODE_MEM;

/*
Monitor code structure
*/
typedef struct tagCODE_MONITOR
{
   CODE_BASE         Base;       /* Must be first element */
   COND_DATA         Cond;
   PTR_DATA          DestPtr;    /* numeric PTRTYPE_ */
   DOUBLE_DATA       DestOnValue;
   DOUBLE_DATA       DestOffValue;
   DOUBLE_DATA       DestMode;
   CODE_ASSIGN_DBL   PreAssignDbl;
} CODE_MONITOR;
typedef CODE_MONITOR *PCODE_MONITOR;

/*
On "condition" gosub code structure
*/
typedef struct tagCODE_ONGOSUB
{
   CODE_BASE         Base;       /* Must be first element */
   COND_DATA         Cond;
   DWORD_DATA        PriorityLevel;
   STRING128_DATA    Program;
   STRING128_DATA    Label;
   CODE_ASSIGN_DBL   PreAssignDbl;
} CODE_ONGOSUB;
typedef CODE_ONGOSUB *PCODE_ONGOSUB;

typedef struct tagCODE_PVT
{
   CODE_BASE         Base;
   MASK_DATA         mTaskAxis;
   DWORD_DATA        Duration;   /* msec */
   DOUBLE_DATA       dAxDistance[MAX_AXES_PVT];   /* user units */
   DOUBLE_DATA       dAxVelocity[MAX_AXES_PVT];   /* user units / user time unit */
} CODE_PVT;
typedef CODE_PVT *PCODE_PVT;

typedef struct tagCODE_SOMECOMMAND
{
   CODE_BASE         Base;
   DWORD_DATA        DWord1;
   DOUBLE_DATA       DoubleData1;
   DOUBLE_DATA       DoubleData2;
   PTR_DATA          ReturnVar;
} CODE_SOMECOMMAND;
typedef CODE_SOMECOMMAND *PCODE_SOMECOMMAND;


#define HILLCLIMB    1
#define SPROUGH      2
#define SPFINE       3
#define FASTALIGN2D  4
#define FASTALIGN3D  5
#define FASTALIGN4D  6
#define FASTALIGN5D  7
#define FASTALIGN6D  8
#define CENTROID     9
#define GEOCENTER    10

typedef struct tagCODE_FIBERSTART
{
   CODE_BASE         Base;
   DWORD_DATA        NRoutineNumber;
   DWORD_DATA        NRoutineNumber2;
   PTR_DATA          dVarCentroidReport;
} CODE_FIBERSTART;
typedef CODE_FIBERSTART *PCODE_FIBERSTART;

#define VPP_OFF               0
#define VPP_ON                1
typedef struct tagCODE_VPP
{
   CODE_BASE         Base;
   DWORD_DATA        VppMode;
} CODE_VPP;
typedef CODE_VPP *PCODE_VPP;

typedef struct tagCODE_DRIVECMD
{
   CODE_BASE         Base;
   PTR_DATA          PtrLHS;
   MASK_DATA         mTaskAxisRHS;
   DOUBLE_DATA       ParamRHS[MCOMMAND_NUMBER_PARAMS_PER_COMMAND+2];
} CODE_DRIVECMD;
typedef CODE_DRIVECMD *PCODE_DRIVECMD;


typedef struct tagCODE_ASSIGN_Q_GEN
{
   CODE_BASE         Base;       /* Must be first element */
   TASKINDEX         iTask;
   MASK_DATA         Mask;
   PTR_DATA          PtrUniqueID;
   PTR_DATA          PtrPosDest;    // cnts
   PTR_DATA          PtrVelDest;    // cnts/msec
   PTR_DATA          PtrTimeDest;   // msec
   PTR_DATA          PtrTargDest;   // cnts
   PTR_DATA          PtrMiscDest;
   PTR_DATA          PtrAccelDest;  // cnts/msec/msec
} CODE_ASSIGN_Q_GEN;
typedef CODE_ASSIGN_Q_GEN *PCODE_ASSIGN_Q_GEN;

typedef struct tagCODE_PCI_COMMAND
{
   CODE_BASE         Base;       /* Must be first element */
   DWORD             subFunctionCode;    // 0=open, 1=close, 2=readByte, 3=writeByte, 4=readWord, 5=writeWord
   PTR_DATA          ReturnVariable;
   DWORD_DATA        Argument1;
   DWORD_DATA        Argument2;
   DWORD_DATA        Argument3;
   DWORD_DATA        Argument4;
   DWORD_DATA        Argument5;
} CODE_PCI_COMMAND;
typedef CODE_PCI_COMMAND *PCODE_PCI_COMMAND;

typedef struct tagCODE_VME_COMMAND
{
   CODE_BASE         Base;                /* Must be first element */
   DWORD             subFunctionCode;     // VME_READ_REG=read, VME_WRITE_REG=write,
   PTR_DATA          ReturnVariable;
   DWORD_DATA        Argument1;           // address
   DWORD_DATA        Argument2;           // value (for write only)
   DWORD_DATA        Argument3;           // value for 'mode' - this is an optional arg for read and write
} CODE_VME_COMMAND;
typedef CODE_VME_COMMAND *PCODE_VME_COMMAND;

// these also used for the PULSE command and ENCODER OUTPUT command
#define ANALOG_SUBCODE_OFF        0
#define ANALOG_SUBCODE_ON         1
#define ANALOG_SUBCODE_SET        2
#define ANALOG_SUBCODE_SET_UNITS  3
#define ANALOG_SUBCODE_SET_TIME   4
#define ANALOG_SUBCODE_OFF_OUT    5
#define ANALOG_SUBCODE_TRACK      6
#define ANALOG_SUBCODE_ON_PWM     7

typedef struct tagCODE_ANALOG_COMMAND
{
   CODE_BASE         Base;       /* Must be first element */
   DWORD             subFunctionCode;   // an ANALOG_SUBCODE_* constant
   MASK_DATA         mTaskAxisTrack;    // input, for pulse commands
   DWORD_DATA        TrackMode;         // or time, for "time" subcode
   DOUBLE_DATA       VelPos;
   MASK_DATA         mTaskAxisOutput;   // output, for pulse commands
   DWORD_DATA        DACNum;
   DOUBLE_DATA       DACMinV;
   DOUBLE_DATA       DACMaxV;
   DOUBLE_DATA       OffValue;          // voltage for "off (voltage)" subcode
   DOUBLE_DATA       OffsetV;
} CODE_ANALOG_COMMAND;
typedef CODE_ANALOG_COMMAND *PCODE_ANALOG_COMMAND;

#define SUBCODE_FOCUS_ONOFF  0
#define SUBCODE_FOCUS_TARGET 1
#define SUBCODE_FOCUS_GAIN   2
#define SUBCODE_FOCUS_LIMIT  3
typedef struct tagCODE_AUTOFOCUS
{
   CODE_BASE         Base;              /* Must be first element */
   DWORD             subFunctionCode;   // SUBCODE_FOCUS_* constant
   MASK_DATA64       mAxis;
   DOUBLE_DATA       Argument1;
   DOUBLE_DATA       Argument2;
   DOUBLE_DATA       Argument3;
   DOUBLE_DATA       Argument4;
} CODE_AUTOFOCUS;
typedef CODE_AUTOFOCUS *PCODE_AUTOFOCUS;



/**************************************************************/
/* Code Packet / Type Definitions
   NOTE: When adding a new CNC command, besides compiler you need also to change:
      AERSYS/AERCNC.C   So AERDEBUG can show a program dump of command (instead of "???")
      960/AERPROG.C     Aer960ProgramExecuteCode(); to execute command on 960
      960/AERPROG.C     Aer960ProgramIsLegalImmediateModeCommand() all legal immediatre mode commands must be spelled out
      960/AERPROG.C     Aer960ProgramExecuteCode(); to tell retrace what to do with it
      960/AERLOOK.C     Each of the two lookahead routinmes needs to know what to do with the command
      ACL/ACLPROGC.CPP  But only if its a new M or G code
*/

/* Code type defines */
/* WARNING ! CODETYPE_xxxx constants are limited to 256, because they are stored in a byte (CODE_BASE.bType) */
#define CODETYPE_NULL                 0    /* CODE_BASE            */
#define CODETYPE_END                  1    /* CODE_BASE            */
#define CODETYPE_RETURN               2    /* CODE_DWORD1          */
#define CODETYPE_PUSH_MODES           3    /* CODE_BASE            */
#define CODETYPE_POP_MODES            4    /* CODE_BASE            */
#define CODETYPE_NEAR_JUMP            5    /* CODE_NEAR_CALL       */
#define CODETYPE_FAR_JUMP             6    /* CODE_FAR_CALL        */
#define CODETYPE_NEAR_CALL            7    /* CODE_NEAR_CALL       */
#define CODETYPE_FAR_CALL             8    /* CODE_FAR_CALL        */
#define CODETYPE_ASSIGN_DBL           9    /* CODE_ASSIGN_DBL      */
#define CODETYPE_ASSIGN_STR          10    /* CODE_ASSIGN_STR      */
#define CODETYPE_ASSIGN_APT          11    /* CODE_ASSIGN_APT      */
#define CODETYPE_DWELL               12    /* CODE_DOUBLE1         */
#define CODETYPE_PSO                 13    /* CODE_PSO             */
#define CODETYPE_COND_JUMP           14    /* CODE_COND_JUMP       */
#define CODETYPE_COND_WAIT           15    /* CODE_CODE_WAIT       */
#define CODETYPE_SET_OFFSET_PRESET   16    /* CODE_AXISPOINT       */
#define CODETYPE_SET_OFFSET_FIXTURE  17    /* CODE_AXISPOINT       */
#define CODETYPE_CLR_OFFSET_PRESET   18    /* CODE_MASK            */
#define CODETYPE_CLR_OFFSET_FIXTURE  19    /* CODE_MASK            */
#define CODETYPE_SET_MODE            20    /* CODE_MODE            */
#define CODETYPE_ASYNC_MOTION        21    /* CODE_ASYNC_MOTION    */
#define CODETYPE_SPINDLE             22    /* CODE_SPINDLE         */
#define CODETYPE_MOTION              23    /* CODE_MOTION (linear) */
#define CODETYPE_MOTIONCIR           24    /* CODE_MOTIONCIR       */
#define CODETYPE_HOME                25    /* CODE_MASK            */
#define CODETYPE_REWIND              26    /* CODE_BASE            */
#define CODETYPE_RESTART             27    /* CODE_BASE            */
#define CODETYPE_CALLBACK            28    /* CODE_CALLBACK        */
#define CODETYPE_STOP                29    /* CODE_BASE            */
#define CODETYPE_OPTIONAL_STOP       30    /* CODE_BASE            */
#define CODETYPE_SLEW                31    /* CODE_SLEW            */
#define CODETYPE_HANDWHEEL           32    /* CODE_HANDWHEEL       */
#define CODETYPE_AFCO                33    /* CODE_AFCO            */
#define CODETYPE_SYNC                34    /* CODE_SOMECOMMAND2    */
#define CODETYPE_PENDANT             35    /* CODE_PENDANT         */
#define CODETYPE_SET_SAFEZONE_MIN    36    /* CODE_AXISPOINT       */
#define CODETYPE_SET_SAFEZONE_MAX    37    /* CODE_AXISPOINT       */
#define CODETYPE_ENABLE_SAFEZONE     38    /* CODE_MASK            */
#define CODETYPE_DISABLE_SAFEZONE    39    /* CODE_MASK            */
#define CODETYPE_ENABLE_REVERSAL     40    /* CODE_AXISPOINT       */
#define CODETYPE_DISABLE_REVERSAL    41    /* CODE_MASK            */
//#define CODETYPE_ENABLE_ERROR_MAP    42    /* CODE_MASK            */
//#define CODETYPE_DISABLE_ERROR_MAP   43    /* CODE_MASK            */
//#define CODETYPE_ENABLE_FBK_MODE     44    /* CODE_MASK            */
//#define CODETYPE_DISABLE_FBK_MODE    45    /* CODE_BASE            */
#define CODETYPE_SET_MIRROR_AXES     46    /* CODE_MASK            */
#define CODETYPE_MONITOR             47    /* CODE_MONITOR         */
#define CODETYPE_ONGOSUB             48    /* CODE_ONGOSUB         */
#define CODETYPE_PROBE_SETUP         49    /* CODE_PROBE_SET       */
#define CODETYPE_PROBE_EXECUTE       50    /* CODE_MASK            */
#define CODETYPE_SET_CORNER_ROUND    51    /* CODE_DOUBLE1         */
#define CODETYPE_CLR_CORNER_ROUND    52    /* CODE_BASE            */
//#define CODETYPE_TRACK               53    /* CODE_TRACK           */
#define CODETYPE_CFGMASTER           54    /* CODE_CFGMASTER       */
#define CODETYPE_SET_OFFSET_FIXTURE2 55   /* CODE_AXISPOINT       */
#define CODETYPE_SET_OFFSET_FIXTURE3 56   /* CODE_AXISPOINT       */
#define CODETYPE_SET_OFFSET_FIXTURE4 57   /* CODE_AXISPOINT       */
#define CODETYPE_SET_OFFSET_FIXTURE5 58   /* CODE_AXISPOINT       */
#define CODETYPE_SET_OFFSET_FIXTURE6 59   /* CODE_AXISPOINT       */
#define CODETYPE_LOCK_MACHINE        60   /* CODE_BASE            */
#define CODETYPE_UNLOCK_MACHINE      61   /* CODE_BASE            */
#define CODETYPE_SET_CANNEDFUNCTION  62   /* CODE_BASE            */
#define CODETYPE_EXE_CANNEDFUNCTION  63   /* CODE_BASE            */
#define CODETYPE_MSET                64   /* CODE_MSET            */
#define CODETYPE_SET_SCALING         65   /* CODE_DOUBLE1         */
#define CODETYPE_CLR_SCALING         66   /* CODE_BASE            */
#define CODETYPE_PVT                 67   /* CODE_BASE            */
#define CODETYPE_FIRE                68   /* CODE_BASE            */
#define CODETYPE_SETDRIVE            69   /* CODE_MASKPLUS        */
#define CODETYPE_FAULTACK            70   /* CODE_MASKPLUS        */
#define CODETYPE_SETPOSCMD           71   /* CODE_MASKPLUS2       */
#define CODETYPE_FIBERSTART          72   /* CODE_FIBERSTART      */
#define CODETYPE_VPP                 73   /* CODE_VPP             */
#define CODETYPE_MC                  74   /* CODE_MSET            */

#define CODETYPE_SERVER              75   /* CODE_SOMECOMMAND     */
//#define CODETYPE_MEMREAD             74   /* CODE_MEM             */
//#define CODETYPE_MEMWRITE            75   /* CODE_MEM             */

#define CODETYPE_BRAKE               76   /* CODE_MASKPLUS        */
#define CODETYPE_DRIVE_COMMAND       77   /* CODE_DRIVECMD        */
#define CODETYPE_DRIVE_COMMANDF      78   /* CODE_DRIVECMD        */
#define CODETYPE_GET_Q_GEN           79   /* CODE_ASSIGN_Q_GEN    */
#define CODETYPE_SET_Q_GEN           80   /* CODE_ASSIGN_Q_GEN    */
#define CODETYPE_SETCALPARM          81   /* CODE_SOMECOMMAND     */
#define CODETYPE_GETCALPARM          82   /* CODE_SOMECOMMAND     */

#define CODETYPE_PCI_COMMAND         83   /* CODE_PCI_COMMAND     */
#define CODETYPE_SLEW3D              84   /* CODE_SLEW            */
#define CODETYPE_RELEASE_SLICE       85   /* CODE_BASE            */
#define CODETYPE_ANALOG_COMMAND      86   /* CODE_ANALOG_COMMAND  */
#define CODETYPE_MOTIONELLIPSE       87   /* CODE_MOTIONELLIPSE   */
#define CODETYPE_AUTOFOCUS           88   /* CODE_AUTOFOCUS       */

//#define CODETYPE_SET_CORNER_ROUND2   89   /* CODE_DOUBLE1         */
//#define CODETYPE_CLR_CORNER_ROUND2   90   /* CODE_BASE            */

#define CODETYPE_PULSE_COMMAND       91   /* CODE_ANALOG_COMMAND  */
#define CODETYPE_RESET_Q_GEN         92   /* CODE_ASSIGN_Q_GEN    */
#define CODETYPE_PORTDIR             93   /* CODE_SOMECOMMAND2    */

#define CODETYPE_WRITEFIRE           94   /* CODE_SOMECOMMAND2    */
#define CODETYPE_PVTU                95   /* CODE_BASE            */

#define CODETYPE_READFIRE            96   /* CODE_SOMECOMMAND2    */
#define CODETYPE_RETRACE_0           97   /* CODE_BASE            */
#define CODETYPE_SLICE               98   /* CODE_SLICE           */

#define CODETYPE_VME_COMMAND         99   /* CODE_VME_COMMAND     */

#define CODETYPE_LOADCAMTABLE2      100   /* CODE_SOMECOMMAND2    */
#define CODETYPE_DRIVEINFO          101   /* CODE_SOMECOMMAND2    */
#define CODETYPE_CLAMPI             102   /* CODE_SOMECOMMAND2    */
#define CODETYPE_ENCODERTRACK       103   /* CODE_ANALOG_COMMAND  */

#define CODETYPE_GRAB_SLICE         104   /* CODE_BASE            */
#define CODETYPE_FREECAM            105   /* CODE_SOMECOMMAND     */
#define CODETYPE_RAMP_SET           106   /* CODE_SOMECOMMAND2    */
#define CODETYPE_DRIVE_COMMANDD     107   /* CODE_DRIVECMD        */
#define CODETYPE_DRIVE_COMMANDD2    108   /* CODE_DRIVECMD        */
#define CODETYPE_DRIVE_COMMANDDL    109   /* CODE_DRIVECMD        */
#define CODETYPE_DSPMEM_WRITEF      110   /* CODE_DRIVECMD        */

#define CODETYPE_RAMP_SET_TIMEVALS  111   /* CODE_AXISDOUBLE_LIST */
#define CODETYPE_RAMP_SET_RATEVALS  112   /* CODE_AXISDOUBLE_LIST */

#define CODETYPE_BLOCKMOTION        113   /* CODE_SOMECOMMAND2    */
#define CODETYPE_SETPOSFBK          114   /* CODE_MASKPLUS2       */
#define CODETYPE_TEST_COMMAND       115   /* CODE_DRIVECMD        */
#define CODETYPE_SETEXTCLOCKFREQ    116   /* CODE_SOMECOMMAND2    */
#define CODETYPE_SETGAINS           117   /* CODE_SOMECOMMAND2    */
/* WARNING ! CODETYPE_xxxx constants are limited to 256, because they are stored in a byte (CODE_BASE.bType) */
#define MAX_CODETYPE                118

// subcodes for CODETYPE_TEST_COMMAND
#define CODETYPE_TEST_SUBCODE_JOGSIMPLE 1

/* Code packet union */
typedef union tagCODE_PACKET
{
   CODE_BASE               Base;
   CODE_NEAR_CALL          NearCall;
   CODE_FAR_CALL           FarCall;
   CODE_ASSIGN_DBL2        AssignDbl2;
   CODE_ASSIGN_STR         AssignStr;
   CODE_ASSIGN_APT         AssignAPt;
   CODE_DOUBLE1            Double1;
   CODE_DWORD1             Dword1;
   CODE_PSO                Pso;
   CODE_MASK               Mask64;
   CODE_MASKPLUS           MaskPlus;
   CODE_MASKPLUS2          MaskPlus2;
   CODE_AXISDOUBLE_LIST    AxisDoubleList;
   CODE_COND_JUMP          CondJump;
   CODE_COND_WAIT          CondWait;
   CODE_AXISPOINT          AxisPoint;        // StatmntHasOneAxisPointParam()
   CODE_MODE               Mode;
   CODE_ASYNC_MOTION       ASyncMotion;
   CODE_SPINDLE            Spindle;
   CODE_MOTION             Motion;
   CODE_MOTIONCIR          MotionCir;        // circlular data
   CODE_MOTIONELLIPSE      MotionEllipse;    // elliptical data
   CODE_CALLBACK           CallBack;
   CODE_SLEW               Slew;
   CODE_HANDWHEEL          HandWheel;
//   CODE_AFCO               Afco;
   CODE_SOMECOMMAND2       SomeCommand2;
//   CODE_PENDANT            Pendant;
   CODE_MONITOR            Monitor;
   CODE_ONGOSUB            OnGosub;
   CODE_PROBE_SET          ProbeSetup;
   CODE_CFGMASTER          ConfigMaster;
//   CODE_TRACK              Track;
   CODE_SET_CANNEDFUNCTION SetCannedFunction;
   CODE_EXE_CANNEDFUNCTION ExeCannedFunction;
   CODE_PVT                Pvt;
   CODE_SOMECOMMAND        SomeCommand;
   CODE_FIBERSTART         FiberStart;
   CODE_MSET               MSet;
   CODE_VPP                Vpp;
   CODE_DRIVECMD           DriveCmd;
   CODE_ASSIGN_Q_GEN       QGenCmd;
   CODE_PCI_COMMAND        PCICommand;
   CODE_ANALOG_COMMAND     AnalogCommand;
   CODE_AUTOFOCUS          AutoFocus;
   CODE_SLICE              SliceCommand;
   CODE_VME_COMMAND        VMECommand;
} CODE_PACKET;
typedef CODE_PACKET *PCODE_PACKET;

/* /////////////////////////////////////////////////////////////////////////////// */
/*
   IO
*/

/*
Taken to include\AerVirtIO.h for global access
(used by NRtEnet rtss project file)
typedef struct tagBINARY_DATA
{
   WORD        Word[MAX_VIRT_BINARY_WORDS];
} BINARY_DATA;
typedef BINARY_DATA *PBINARY_DATA;

typedef struct tagREGISTER_DATA
{
   WORD        Word[MAX_VIRT_REGISTERS];
} REGISTER_DATA;
typedef REGISTER_DATA *PREGISTER_DATA;

*/


//typedef union tagLONG_DATA
//{
//   DWORD        Long[MAX_VIRT_REGISTERS];
//} LONG_DATA;
//typedef LONG_DATA *PLONG_DATA;

typedef struct tagANALOG_DATA
{
   DOUBLE      Double[MAX_AXES][ANALOGS_IN_EACH_DRIVE];    // double representation of a 16 bit word
} ANALOG_DATA;
typedef ANALOG_DATA *PANALOG_DATA;

typedef struct tagDRIVE_DATA
{
   DWORD        DWord[MAX_AXES];                           // one DWord per drive, though only the lower DIG_BITS_IN_EACH_DRIVE are actually used
} DRIVE_DATA;
typedef DRIVE_DATA *PDRIVE_DATA;

typedef struct tagETHER_BINARY_DATA
{
   BOOL        Bool[MAX_AXES][ETHER_BIN_EACH_DRIVE];
}ETHER_BINARY_DATA;
typedef ETHER_BINARY_DATA *PETHER_BINARY_DATA;

typedef struct tagETHER_REGISTER_DATA
{
   WORD        Word[MAX_AXES][ETHER_REG_EACH_DRIVE];
} ETHER_REGISTER_DATA;
typedef ETHER_REGISTER_DATA *PETHER_REGISTER_DATA;

///**************************************************************/
///*  structure size and ordering is the same as OS/2 network version although
//    the virtual update functionality is different
//*/
//typedef struct tagVIRTIO_DATA
//{
//
//   BINARY_DATA       BinaryOutput;
//   BINARY_DATA       BinaryOutputStatus;
//   BINARY_DATA       BinaryInput;
//   REGISTER_DATA     RegisterInput;
//   REGISTER_DATA     ProcessRegisterInput;
//   REGISTER_DATA     RegisterOutput;
//   REGISTER_DATA     RegisterOutputStatus;
//   REGISTER_DATA     ProcessRegisterOutput;
//
////   LONG_DATA         LongOutput;
////   LONG_DATA         LongInput;
////   WORD           notUsed_SemaphoreDD;
////   WORD           notUsed_Semaphore960;
////   DWORD          notUsed_ScanFlag;
////   WORD           notUsed_BinaryOutputUpdateMap[2];
////   WORD           notUsed_BinaryInputUpdateMap[2];
////   WORD           notUsed_RegisterOutputUpdateMap[8];
////   WORD           notUsed_RegisterInputUpdateMap[8];
////   WORD           notUsed_CommandActive;
////   WORD           notUsed_CommandDone;
////   WORD           notUsed_CommandError;
////   WORD           notUsed_CommandState;
////   WORD           notUsed_CommandOpcode;
////   WORD           notUsed_RegisterLength;
////   BYTE           notUsed_Pad[4];
////   WORD           notUsed_Command[32];
////   WORD           notUsed_CommandData[128];
//} VIRTIO_DATA;
//typedef VIRTIO_DATA *PVIRTIO_DATA;
//


/**************************************************************/
/* Monitor Type Definitions
*/
#define AER_MONITOR_ACTIVE_OFF       0
#define AER_MONITOR_ACTIVE_BROKEN    1   /* Monitor was defined, but could not be evaluated */
#define AER_MONITOR_ACTIVE_ON        2   /* Monitor defined, and working */
#define AER_MONITOR_ACTIVE_ON_SIMPLE 3   /* Simplified Monitor defined, and working (these are for higher speed) */

#define AER_MONITOR_MODE_LEVEL         0
#define AER_MONITOR_MODE_EDGE          1
#define AER_MONITOR_MODE_LATCH         2

typedef struct tagMONITOR_DATA
{
   DWORD       bActive;   /* Must be one of AER_MONITOR_ACTIVE... */
   DWORD       dwCond;
   DWORD       dwPrevCond;
   COND_DATA   Cond;
   PTR_DATA    DestPtr;
   DOUBLE_DATA DestOnValue;
   DOUBLE_DATA DestOffValue;
   DOUBLE_DATA DestMode;  /* Must be one of AER_MONITOR_MODE... */
   CODE_ASSIGN_DBL   PreAssignDbl;
} MONITOR_DATA;
typedef MONITOR_DATA *PMONITOR_DATA;

typedef struct tagONGOSUB_DATA
{
   DWORD             bActive;       /* Must be one of AER_MONITOR_ACTIVE... */
   DWORD             dwCond;
   COND_DATA         Cond;
   DWORD             dwPriorityLevel;
   AER_PROG_HANDLE   Handle;
   AER_PROG_LABEL    Label;
   CODE_ASSIGN_DBL   PreAssignDbl;
} ONGOSUB_DATA;
typedef ONGOSUB_DATA *PONGOSUB_DATA;

/*** DataCenter Calculate Defines ****/
//#define AERDC_CALC_POSPROGCMD 0x0001
//#define AERDC_CALC_TARGET     0x0002
//#define AERDC_CALC_FIXTURE    0x0004
//#define AERDC_CALC_POSFACTOR  0x0008

typedef struct tagMEASUREMENT_ENTRY
{
   double dValue;
   LONG   lCNCLine;
} MEASUREMENT_ENTRY;
typedef MEASUREMENT_ENTRY *PMEASUREMENT_ENTRY;


typedef struct tagAER_MEASUREMENT_PACKET
{
   double duration;       /* Seconds */
   double length;         /* Inches */
   DWORD  nHits;
} AER_MEASUREMENT_PACKET;
typedef AER_MEASUREMENT_PACKET *PAER_MEASUREMENT_PACKET;

typedef struct tagAER_MEASUREMENT_SUMMARY
{
   /* These items apply to whole program */
   AER_MEASUREMENT_PACKET cut;
   AER_MEASUREMENT_PACKET motion;
   AER_MEASUREMENT_PACKET total;
   DWORD  numMoves;
   DWORD  numSlices;
   DWORD  numMsecsToMakeSlices;     /* Milliseconds */
   DWORD  numMsecsToRunProgram;     /* Milliseconds */

   /* These items refer to a particular line */
   MEASUREMENT_ENTRY durationMin;   /* milliseconds */
   MEASUREMENT_ENTRY durationMax;   /* milliseconds */
   MEASUREMENT_ENTRY lengthMin;     /* inches       */
   MEASUREMENT_ENTRY lengthMax;     /* inches       */
   MEASUREMENT_ENTRY radiusMin;     /* inches       */
   MEASUREMENT_ENTRY sliceMin;      /* milliseconds */
   MEASUREMENT_ENTRY sliceMax;      /* milliseconds */
   MEASUREMENT_ENTRY profVelMax;    /* inch/sec     */
   MEASUREMENT_ENTRY profAccMax;    /* inch/sec/sec */
   MEASUREMENT_ENTRY cheatDistMax;  /* inch         */
   MEASUREMENT_ENTRY cheatDistAvg;  /* inch         */
   MEASUREMENT_ENTRY truncTimeMax;  /* inch         */
   MEASUREMENT_ENTRY truncTimeAvg;  /* inch         */
   MEASUREMENT_ENTRY truncTimeAvgAbs;  /* inch         */

} AER_MEASUREMENT_SUMMARY;
typedef AER_MEASUREMENT_SUMMARY *PAER_MEASUREMENT_SUMMARY;

/*** AerShape defines */
#define MAX_SHAPER_SIZE 32          /* Max # of Input Shaping coefficients      */
												/*    32 coefficients is the largest number */
												/*    we have ever used in an application   */
typedef struct tagTPACKET_AERSHAPER
{
   long numPulses;    /* Must be < MAX_SHAPER_SIZE */
   long DelayTimes[MAX_SHAPER_SIZE];
   long ShaperCoeffs[MAX_SHAPER_SIZE];
} TPACKET_AERSHAPER;
typedef TPACKET_AERSHAPER *PTPACKET_AERSHAPER;

/* /////////////////////////////////////////////////////////////////////////////// */
/*
     Axis Calibration
*/
typedef struct tagAER_AXISCAL_PACKET
{
    DWORD      dwTable;
    AXISINDEX  iMasterAxis;
} AER_AXISCAL_PACKET;
typedef AER_AXISCAL_PACKET *PAER_AXISCAL_PACKET;

typedef struct tagAER_AXISCAL_STATUS_PACKET
{
   DWORD       dwSize;
   DWORD       dwStatus;
   AXISINDEX   iMasterAxis;
   AXISINDEX   iCorrectedAxis;
   DOUBLE      dCalFudgeOnValue;      // Offset added to computed cal value when a table is active (normally zero)
   DOUBLE      dCalFudgeOffValue;     // Computed Cal Value when a table is not active (normally zero)
} AER_AXISCAL_STATUS_PACKET;
typedef AER_AXISCAL_STATUS_PACKET *PAER_AXISCAL_STATUS_PACKET;

typedef struct tagAER_GAINCAL_PACKET
{
    DWORD      dwTable;
    AXISINDEX  iMasterAxis;
} AER_GAINCAL_PACKET;
typedef AER_GAINCAL_PACKET *PAER_GAINCAL_PACKET;

typedef struct tagAER_GAINCAL_STATUS_PACKET
{
   DWORD       dwSize;
   DWORD       dwStatus;
   AXISINDEX   iMasterAxis;
   AXISINDEX   iCorrectedAxis;
} AER_GAINCAL_STATUS_PACKET;
typedef AER_GAINCAL_STATUS_PACKET *PAER_GAINCAL_STATUS_PACKET;

typedef struct tagAER_CAM_SETPOINT
{
   DWORD    dwType;     // AERCAM_POINT_xxx constant
   DOUBLE   dMasterPos; // Master position (counts)
   DOUBLE   dSlavePos;  // Slave position (counts)
} AER_CAM_SETPOINT;
typedef AER_CAM_SETPOINT   *PAER_CAM_SETPOINT;

typedef struct tagAER_CAM_GETPOINT
{
	DWORD    dwType;        // AERCAM_POINT_xxx constant
	DOUBLE   dMasterPos;    // Master position (counts)
	DOUBLE   dSlavePos;     // Slave position (counts)
	DOUBLE   dCoeffA;       // A coefficient
	DOUBLE   dCoeffB;       // B coefficient
   DOUBLE   dCoeffC;       // C coefficient
   DOUBLE   dCoeffD;       // D coefficient
} AER_CAM_GETPOINT;
typedef AER_CAM_GETPOINT   *PAER_CAM_GETPOINT;

typedef struct tagAER_CAM_STATUS_PACKET
{
   DWORD  dwSize;
   DWORD   dwStatus;
} AER_CAM_STATUS_PACKET;
typedef AER_CAM_STATUS_PACKET *PAER_CAM_STATUS_PACKET;

#endif
