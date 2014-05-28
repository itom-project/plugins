/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_UTIL_H__
#define __AER_UTIL_H__

//typedef struct tagAER_UTIL_REF_PACKET
//{
//   DWORD dwParm1;
//   DWORD dwParm2;
//} AER_UTIL_REF_PACKET;

#ifdef __cplusplus
extern "C" {
#endif

//AERERR_CODE AER_DLLENTRY  AerUtilReference( HAERCTRL hAerCtrl, AXISINDEX iAxis,
//                                            DWORD dwParm1, DWORD dwParm2 );
//AERERR_CODE AER_DLLENTRY  AerUtilReferenceMaster( HAERCTRL hAerCtrl, AXISINDEX iAxis,
//                                                  DWORD dwParm1, DWORD dwParm2 );
//AERERR_CODE AER_DLLENTRY  AerUtilMotorSet( HAERCTRL hAerCtrl, AXISINDEX iAxis, WORD wSet );
//
//AERERR_CODE AER_DLLENTRY AerUtilAlignMaster( HAERCTRL hAerCtrl, AXISINDEX iAxis );
//AERERR_CODE AER_DLLENTRY AerUtilHitWatchdogTimer( HAERCTRL hAerCtrl, AXISINDEX iAxis );

double AER_DLLENTRY AerAtoDbl(LPTSTR iv_pString, DWORD* ierror, BOOL bCheck);
DWORD AER_DLLENTRY AerNextWord(LPTSTR iv_pString, ULONG indxStart, ULONG* pdwIndexWordStart);
BOOL AER_DLLENTRY AerIsTerminator(LPTSTR iv_pString, ULONG indx);

AERERR_CODE AER_DLLENTRY aerErrGetMessage0( HAERCTRL hAerCtrl, WORD wSubCode, WORD wMsg, PSZ pszMsg );
//AERERR_CODE aerErrSetUserFault0( HAERCTRL hAerCtrl, AXISINDEX iAxis );

DWORD AER_DLLENTRY AerUserModeLogIn();
DWORD AER_DLLENTRY aerIsReadOnlyAccessible(LPCTSTR pszFileName);

#ifdef __cplusplus
}
#endif

#endif
// __AER_UTIL_H__
