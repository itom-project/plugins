////////////////////////////////////////////////////////////////////////////
// Fileame : aerio.h
// Purpose : linkage between virtual I/O and ethernet I/O required the
//           data type VIRTIO_DATA to be declared in an "h" file so
//           the type could be used in both places.
//Author   : Ratin
////////////////////////////////////////////////////////////////////////////

#ifndef __AERVIRTIO_H_INCLUDED
#define __AERVIRTIO_H_INCLUDED

#include "AerType.h"
#include "aercdef.h"
#include "AertDef.h"




/**************************************************************/
/*  structure size and ordering is the same as OS/2 network version although
    the virtual update functionality is different
*/

typedef struct tagCFG_WORDS
{
   WORD Word[20] ;
} CFG_WORDS ;


/////* Binary/Registor I/O
//#define MAX_VIRT_BINARY_BITS        1024
//#define MAX_VIRT_BINARY_WORDS       (MAX_VIRT_BINARY_BITS/16) /* 64 */
//#define MAX_VIRT_REGISTERS          896

//#define MAX_DRIVE_WORDS             MAX_AXES         // one for each drive
//#define MAX_ETHER_IO_READ           8

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


typedef struct tagVIRTIO_DATA
{

   BINARY_DATA       BinaryOutput;           //32 bytes
   BINARY_DATA       BinaryInput;            //32 bytes

   REGISTER_DATA     RegisterInput;          //896 word inputs

   REGISTER_DATA     RegisterOutput;         //896 word outputs

   CFG_WORDS         CONFIG_WORDS ;          //20 words for holding config info

//I/O start locations

   WORD               InetGlobalInputBitsStart  ;
   WORD               InetGlobalOutputBitsStart ;
   WORD               InetGlobalOutputBitStatusStart;
   WORD               InetGlobalInputWordsStart ;
   WORD               InetGlobalOutputWordsStart;
   WORD               InetGlobalOutputWordStatusStart;
   WORD               InetGlobalInputProcessStart ;
   WORD               InetGlobalOutputProcessStart;
   WORD               InetGlobalConfigStart;

//start of ethernet registers I/O

//   LONG_DATA      LongOutput;
//   LONG_DATA      LongInput;
//   WORD           notUsed_SemaphoreDD;
//   WORD           notUsed_Semaphore960;
//   DWORD          notUsed_ScanFlag;
//   WORD           notUsed_BinaryOutputUpdateMap[2];
//   WORD           notUsed_BinaryInputUpdateMap[2];
//   WORD           notUsed_RegisterOutputUpdateMap[8];
//   WORD           notUsed_RegisterInputUpdateMap[8];
//   WORD           notUsed_CommandActive;
//   WORD           notUsed_CommandDone;
//   WORD           notUsed_CommandError;
//   WORD           notUsed_CommandState;
//   WORD           notUsed_CommandOpcode;
//   WORD           notUsed_RegisterLength;
//   BYTE           notUsed_Pad[4];
//   WORD           notUsed_Command[32];
//   WORD           notUsed_CommandData[128];
} VIRTIO_DATA;
typedef VIRTIO_DATA *PVIRTIO_DATA;

#endif
