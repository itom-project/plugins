/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __960PSO_H__
#define __960PSO_H__

/******************************* PSOC *************************************/

/* PSOC type 1,2 code structure */
typedef struct tagCODE_PSOC12
{
   DWORD_DATA  Input;
   DWORD_DATA  Level;
} CODE_PSOC12;
typedef CODE_PSOC12 *PCODE_PSOC12;

/* PSOC type 3 code structure */
typedef struct tagCODE_PSOC3
{
   DWORD_DATA  InputMask;
   DWORD_DATA  InputLevel;
   DWORD_DATA  OutputMask;
   DWORD_DATA  OutputLevel;
} CODE_PSOC3;
typedef CODE_PSOC3 *PCODE_PSOC3;

/* PSOC type 4 code structure */
typedef struct tagCODE_PSOC4
{
   DWORD_DATA  Inputs;
   DWORD_DATA  Outputs;
} CODE_PSOC4;
typedef CODE_PSOC4 *PCODE_PSOC4;

/******************************* PSOD *************************************/

/* PSOD type 0  and type 8 code structure */
typedef struct tagCODE_PSOD0
{
   DWORD_DATA  Distance;
} CODE_PSOD0;
typedef CODE_PSOD0 *PCODE_PSOD0;

/* PSOD type 1,2 code structure */
typedef struct tagCODE_PSOD12
{
   PTR_DATA    Pointer;
   DWORD_DATA  Count;
} CODE_PSOD12;
typedef CODE_PSOD12 *PCODE_PSOD12;


typedef struct tagCODE_PSOD7
{
   DWORD_DATA  Points;
   DWORD_DATA  Distance;
} CODE_PSOD7;
typedef CODE_PSOD7 *PCODE_PSOD7;

/******************************* PSOF *************************************/

/* PSOF type 2 code structure */
typedef struct tagCODE_PSOF2
{
   DWORD_DATA  Count;
} CODE_PSOF2;
typedef CODE_PSOF2 *PCODE_PSOF2;

/* PSOF type 3 code structure */
typedef struct tagCODE_PSOF3
{
   DWORD_DATA  Mask;
} CODE_PSOF3;
typedef CODE_PSOF3 *PCODE_PSOF3;

/* PSOF type 4,5 code structure */
typedef struct tagCODE_PSOF45
{
   DWORD_DATA  Distance;
   DWORD_DATA  Mask;
} CODE_PSOF45;
typedef CODE_PSOF45 *PCODE_PSOF45;


/******************************* PSOM *************************************/

/* PSOM type 0 code structure */
typedef struct tagCODE_PSOM0
{
   PTR_DATA    Pointer;
   DWORD_DATA  Count;
} CODE_PSOM0;
typedef CODE_PSOM0 *PCODE_PSOM0;


/******************************* PSOP *************************************/

/* PSOP type 0,4 code structure */
typedef struct tagCODE_PSOP04
{
   DWORD_DATA  Width;
} CODE_PSOP04;
typedef CODE_PSOP04 *PCODE_PSOP04;

/* PSOP type 1 code structure */
typedef struct tagCODE_PSOP1
{
   DWORD_DATA  Lead;
   DWORD_DATA  Width;
   DWORD_DATA  Trail;
} CODE_PSOP1;
typedef CODE_PSOP1 *PCODE_PSOP1;

/* PSOP type 2 code structure */
typedef struct tagCODE_PSOP2
{
   DWORD_DATA  Lead;
   DWORD_DATA  Width;
   DWORD_DATA  Trail;
   DWORD_DATA  Ramp;
   DWORD_DATA  Interval;
} CODE_PSOP2;
typedef CODE_PSOP2 *PCODE_PSOP2;

/* PSOP type 3 code structure */
typedef struct tagCODE_PSOP3
{
   PTR_DATA    Pointer;
   DWORD_DATA  Count;
} CODE_PSOP3;
typedef CODE_PSOP3 *PCODE_PSOP3;


/******************************* PSOT *************************************/

/* PSOT type 0 code structure */
typedef struct tagCODE_PSOT0
{
   DWORD_DATA  OutputMask;
   DWORD_DATA  OutputLevel;
} CODE_PSOT0;
typedef CODE_PSOT0 *PCODE_PSOT0;

/* PSOT type 1 code structure */
typedef struct tagCODE_PSOT1
{
   DWORD_DATA  OutputMask;
} CODE_PSOT1;
typedef CODE_PSOT1 *PCODE_PSOT1;

/* PSOT type 2,3 code structure */
typedef struct tagCODE_PSOT23
{
   DWORD_DATA  Count;
   struct
   {
      DWORD_DATA  Channel;
      DOUBLE_DATA Voltage;
   } Output[4];
} CODE_PSOT23;
typedef CODE_PSOT23 *PCODE_PSOT23;

/* PSOT type 4,5,6,7 code structure */
typedef struct tagCODE_PSOT4567
{
   DWORD_DATA  Count;
   struct
   {
      DWORD_DATA  Channel;
      DOUBLE_DATA ZeroVoltage;
      DOUBLE_DATA TargetVoltage;
      DWORD_DATA  TargetVelocity;
   } Output[4];
} CODE_PSOT4567;
typedef CODE_PSOT4567 *PCODE_PSOT4567;

/* PSOT type 8 code structure */
typedef struct tagCODE_PSOT8
{
   DWORD_DATA  Count;
   struct
   {
      DWORD_DATA  Channel;
      DWORD_DATA  ZeroFrequency;
      DWORD_DATA  TargetFrequency;
      DWORD_DATA  TargetVelocity;
   } Output[4];
} CODE_PSOT8;
typedef CODE_PSOT8 *PCODE_PSOT8;

/******************************* PSOS *************************************/

/* PSOS type 0/1 code structure */
typedef struct tagCODE_PSOS01
{
   DWORD_DATA  Enable;     /* 0 for disable, 1 for enable */
} CODE_PSOS01;
typedef CODE_PSOS01 *PCODE_PSOS01;

/* PSOS type 2 code structure */
typedef struct tagCODE_PSOS2
{
   DWORD_DATA   Channel;
   DOUBLE_DATA  Scale;
} CODE_PSOS2;
typedef CODE_PSOS01 *PCODE_PSOS2;

/******************************* PSO **************************************/

/* PSO Data structure */
typedef union tagPSO_DATA
{
   CODE_PSOC12       C1;   /* PSOC data */
   CODE_PSOC12       C2;
   CODE_PSOC3        C3;
   CODE_PSOC4        C4;

   CODE_PSOD0        D0;   /* PSOD data */
   CODE_PSOD12       D1;
   CODE_PSOD12       D2;
   CODE_PSOD7        D7;

   CODE_PSOF2        F2;   /* PSOF data */
   CODE_PSOF3        F3;
   CODE_PSOF45       F4;
   CODE_PSOF45       F5;

   CODE_PSOM0        M0;   /* PSOP data */

   CODE_PSOP04       P0;   /* PSOP data */
   CODE_PSOP1        P1;
   CODE_PSOP2        P2;
   CODE_PSOP3        P3;
   CODE_PSOP04       P4;

			   /* PSOR data (none) */

   CODE_PSOT0        T0;   /* PSOT data */
   CODE_PSOT1        T1;
   CODE_PSOT23       T2;
   CODE_PSOT23       T3;
   CODE_PSOT4567     T4;
   CODE_PSOT4567     T5;
   CODE_PSOT4567     T6;
   CODE_PSOT4567     T7;
   CODE_PSOT8        T8;

   CODE_PSOS01       S0;
   CODE_PSOS2        S2;

} PSO_DATA;
typedef PSO_DATA *PPSO_DATA;


/* PSO code structure */
typedef struct tagCODE_PSO
{
   CODE_BASE         Base;    /* Must be first element */
   DWORD             dwType;  /* PSOTYPE_ | subtype */

   PSO_DATA          Data;
} CODE_PSO;
typedef CODE_PSO *PCODE_PSO;
#define PSOTYPE_MASK 0xF0
#define PSOTYPE_C    0x10
#define PSOTYPE_D    0x20
#define PSOTYPE_F    0x30
#define PSOTYPE_M    0x40
#define PSOTYPE_P    0x50
#define PSOTYPE_R    0x60
#define PSOTYPE_T    0x70
#define PSOTYPE_S    0x80

#endif
