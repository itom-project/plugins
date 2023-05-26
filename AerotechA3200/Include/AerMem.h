/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_MEM_H__
#define __AER_MEM_H__

//#include "aermacro.h"

//#define AER_MEMSPACE_SHORT       0
//#define AER_MEMSPACE_STD         1     // standard
//#define AER_MEMSPACE_EXT         2     // extended
//#define AER_MEMSPACE_PORT        99    // address is port
//#define AER_MEMSPACE_ATWINDOW2   100   // use the 2nd ATWindow
//#define AER_MEMSPACE_VIRTIO      101   // use the virtio space if mapped

typedef struct tagAER_UTIL_FREEMEM_PACKET
{
   DWORD dwTotal;
   DWORD dwLargest;
} AER_UTIL_FREEMEM_PACKET;
typedef AER_UTIL_FREEMEM_PACKET  *PAER_UTIL_FREEMEM_PACKET;

//typedef struct tagAER_MEMBYTE_PACKET
//{
//   DWORD dwAddr;
//   BYTE  byData;
//} AER_MEMBYTE_PACKET;
//typedef AER_MEMBYTE_PACKET *PAER_MEMBYTE_PACKET;
//
//typedef struct tagAER_MEMWORD_PACKET
//{
//   DWORD dwAddr;
//   WORD  wData;
//} AER_MEMWORD_PACKET;
//typedef AER_MEMWORD_PACKET *PAER_MEMWORD_PACKET;
//
//typedef struct tagAER_MEMDWORD_PACKET
//{
//   DWORD dwAddr;
//   DWORD dwData;
//} AER_MEMDWORD_PACKET;
//typedef AER_MEMDWORD_PACKET *PAER_MEMDWORD_PACKET;

//#define AER_MEMMULTI_SIZE  128
//typedef struct tagAER_MEMMULTI_PACKET
//{
//   DWORD dwAddr;
//   WORD  wLen;
//   BYTE  byData[AER_MEMMULTI_SIZE];
//} AER_MEMMULTI_PACKET;

#ifdef __cplusplus
extern "C" {
#endif

//AERERR_CODE AER_DLLENTRY AerMemCheck( HAERCTRL hAerCtrl, PDWORD pdwSize );
AERERR_CODE AER_DLLENTRY AerSysMemGetFree( HAERCTRL hAerCtrl, PDWORD pdwSpare, PDWORD pdwLargest );
AERERR_CODE AER_DLLENTRY AerSysMemGetUsed( HAERCTRL hAerCtrl, PDWORD pdwLocal, PDWORD pdwShared );


//AERERR_CODE AER_DLLENTRY aerMemReadByte( HAERCTRL hAerCtrl, DWORD dwAddr, PBYTE pbyData );
//AERERR_CODE AER_DLLENTRY aerMemReadWord( HAERCTRL hAerCtrl, DWORD dwAddr, PWORD pwData );
//AERERR_CODE AER_DLLENTRY aerMemReadDWord( HAERCTRL hAerCtrl, DWORD dwAddr, PDWORD pdwData );
//AERERR_CODE AER_DLLENTRY aerMemReadQuadWord( HAERCTRL hAerCtrl, DWORD dwAddr, PVOID pvData );
//AERERR_CODE AER_DLLENTRY aerMemReadMulti( HAERCTRL hAerCtrl, DWORD dwAddr, PVOID pvData, WORD wLen );
//AERERR_CODE AER_DLLENTRY aerMemWriteByte( HAERCTRL hAerCtrl, DWORD dwAddr, BYTE byData );
//AERERR_CODE AER_DLLENTRY aerMemWriteWord( HAERCTRL hAerCtrl, DWORD dwAddr, WORD wData );
//AERERR_CODE AER_DLLENTRY aerMemWriteDWord( HAERCTRL hAerCtrl, DWORD dwAddr, DWORD dwData );
//AERERR_CODE AER_DLLENTRY aerMemWriteMulti( HAERCTRL hAerCtrl, DWORD dwAddr, PVOID pvData, WORD wLen );
//
//AERERR_CODE AER_DLLENTRY aerMemDirectWriteEx( HAERCTRL hAerCtrl,
//                                              DWORD dwAddrSpace, DWORD dwAddr,
//                                              PVOID pvData, DWORD dwSize,
//                                              BOOL bSwap );
//AERERR_CODE AER_DLLENTRY aerMemDirectWriteByteEx( HAERCTRL hAerCtrl,
//                                                  DWORD dwAddrSpace,
//                                                  DWORD dwAddr, BYTE byData,
//                                                  BOOL bSwap );
//AERERR_CODE AER_DLLENTRY aerMemDirectWriteWordEx( HAERCTRL hAerCtrl,
//                                                  DWORD dwAddrSpace, DWORD dwAddr,
//                                                  WORD wData, BOOL bSwap );
//AERERR_CODE AER_DLLENTRY aerMemDirectWriteDWordEx( HAERCTRL hAerCtrl,
//                                                   DWORD dwAddrSpace, DWORD dwAddr,
//						   DWORD dwData, BOOL bSwap );
//AERERR_CODE AER_DLLENTRY aerMemDirectReadByteEx( HAERCTRL hAerCtrl,
//						  DWORD dwAddrSpace,
//						  DWORD dwAddr, PBYTE pbyData,
//						  BOOL bSwap );
//AERERR_CODE AER_DLLENTRY aerMemDirectReadWordEx( HAERCTRL hAerCtrl,
//						  DWORD dwAddrSpace,
//						  DWORD dwAddr,
//						  PWORD pwData, BOOL bSwap );
//AERERR_CODE AER_DLLENTRY aerMemDirectReadDWordEx( HAERCTRL hAerCtrl,
//						   DWORD dwAddrSpace,
//						   DWORD dwAddr,
//						   PDWORD pdwData, BOOL bSwap );
//
//#define aerMemDirectWrite( hAerCtrl, dwAddr, pvData, dwSize )  \
//   aerMemDirectWriteEx( hAerCtrl, AER_MEMSPACE_STD, dwAddr, pvData, dwSize, FALSE )
//#define aerMemDirectWriteByte( hAerCtrl, dwAddr, byData ) \
//   aerMemDirectWriteByteEx( hAerCtrl, AER_MEMSPACE_STD, dwAddr, byData, FALSE )
//#define aerMemDirectWriteWord( hAerCtrl, dwAddr, wData ) \
//   aerMemDirectWriteWordEx( hAerCtrl, AER_MEMSPACE_STD, dwAddr, wData, FALSE )
//#define aerMemDirectWriteDWord( hAerCtrl, dwAddr, dwData) \
//   aerMemDirectWriteDWordEx( hAerCtrl,AER_MEMSPACE_STD, dwAddr, dwData, FALSE )
//
//AERERR_CODE AER_DLLENTRY aerMemDirectReadEx( HAERCTRL hAerCtrl,
//                                             DWORD dwAddrSpace, DWORD dwAddr,
//					     PVOID pvData, DWORD dwSize, BOOL bSwap );
//
//
//#define aerMemDirectRead( hAerCtrl, dwAddr, pvData, dwSize )   \
//   aerMemDirectReadEx( hAerCtrl, AER_MEMSPACE_STD, dwAddr, pvData, dwSize, FALSE )
//#define aerMemDirectReadByte( hAerCtrl, dwAddr, pbyData )   \
//   aerMemDirectRead( hAerCtrl, dwAddr, pbyData, sizeof(BYTE) )
//#define aerMemDirectReadWord( hAerCtrl, dwAddr, pwData )   \
//   aerMemDirectRead( hAerCtrl, dwAddr, pwData, sizeof(WORD) )
//#define aerMemDirectReadDWord( hAerCtrl, dwAddr, pdwData )   \
//   aerMemDirectRead( hAerCtrl, dwAddr, pdwData, sizeof(DWORD) )
//
//#define aerMemPortWriteByte( hAerCtrl, wPort, byData ) \
//   aerMemDirectWriteByteEx( hAerCtrl, AER_MEMSPACE_PORT, wPort, byData, FALSE )
//#define aerMemPortReadByte( hAerCtrl, wPort, pbyData )   \
//   aerMemDirectReadEx( hAerCtrl, AER_MEMSPACE_PORT, wPort, pbyData, sizeof(BYTE), FALSE )
//
#ifdef __cplusplus
}
#endif

#endif
// __AER_MEM_H__
