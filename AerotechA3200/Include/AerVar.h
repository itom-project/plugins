/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_VAR_H__
#define __AER_VAR_H__


#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY aerPtrDblGetCount( HAERCTRL hAerCtrl, PPTR_DATA pPtr, PDWORD pdwCount );
/*
   One variable at a time functions
*/
AERERR_CODE AER_DLLENTRY AerVarGlobalSetDouble( HAERCTRL hAerCtrl, DWORD dwNum, DOUBLE fdValue);
AERERR_CODE AER_DLLENTRY AerVarGlobalGetDouble( HAERCTRL hAerCtrl, DWORD dwNum, PDOUBLE pfdValue);

AERERR_CODE AER_DLLENTRY AerVarTaskSetDouble( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwNum, DOUBLE fdValue);
AERERR_CODE AER_DLLENTRY AerVarTaskGetDouble( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwNum, PDOUBLE pfdValue);

AERERR_CODE AER_DLLENTRY AerVarProgramSetDouble( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwNum, DOUBLE fdValue);
AERERR_CODE AER_DLLENTRY AerVarProgramGetDouble( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwNum, PDOUBLE pfdValue);

AERERR_CODE AER_DLLENTRY AerVarStackSetDouble( HAERCTRL hAerCtrl, TASKINDEX iTask, CSPARMINDEX iCSParm, DOUBLE fdValue);
AERERR_CODE AER_DLLENTRY AerVarStackGetDouble( HAERCTRL hAerCtrl, TASKINDEX iTask, CSPARMINDEX iCSParm, PDOUBLE pfdValue);

AERERR_CODE AER_DLLENTRY AerVarGlobalSetAPt( HAERCTRL hAerCtrl, DWORD dwNum, AXISMASK axisMask, PAIDOUBLE pfdValues);
AERERR_CODE AER_DLLENTRY AerVarGlobalGetAPt( HAERCTRL hAerCtrl, DWORD dwNum, PAXISMASK pAxisMask, PAODOUBLE pfdValues);

AERERR_CODE AER_DLLENTRY AerVarGlobalSetString( HAERCTRL hAerCtrl, DWORD dwNum, LPSTR pszString);
AERERR_CODE AER_DLLENTRY AerVarGlobalGetString( HAERCTRL hAerCtrl, DWORD dwNum, LPSTR pszString);

AERERR_CODE AER_DLLENTRY AerVarTaskSetString( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwNum, LPSTR pszString);
AERERR_CODE AER_DLLENTRY AerVarTaskGetString( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwNum, LPSTR pszString);

AERERR_CODE AER_DLLENTRY AerVarProgramSetString( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwNum, LPSTR pszString);
AERERR_CODE AER_DLLENTRY AerVarProgramGetString( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwNum, LPSTR pszString);

/*
    Array functions
*/
AERERR_CODE AER_DLLENTRY AerVarGlobalSetDoubleArray( HAERCTRL hAerCtrl, DWORD dwStart,
                                                     PAIDOUBLE pdArray, DWORD dwNumVars );
AERERR_CODE AER_DLLENTRY AerVarGlobalGetDoubleArray( HAERCTRL hAerCtrl, DWORD dwStart,
                                                     PAODOUBLE pdArray, DWORD dwNumVars );

AERERR_CODE AER_DLLENTRY AerVarTaskSetDoubleArray( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwStart,
                                                   PAIDOUBLE pdArray, DWORD dwNumVars );
AERERR_CODE AER_DLLENTRY AerVarTaskGetDoubleArray( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwStart,
                                                   PAODOUBLE pdArray, DWORD dwNumVars );

AERERR_CODE AER_DLLENTRY AerVarProgramSetDoubleArray( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwStart,
                                                      PAIDOUBLE pdArray, DWORD dwNumVars );
AERERR_CODE AER_DLLENTRY AerVarProgramGetDoubleArray( HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwStart,
                                                      PAODOUBLE pdArray, DWORD dwNumVars );

//AERERR_CODE AER_DLLENTRY AerVarAnalogInGetDouble( HAERCTRL hAerCtrl,
//                                                  ANALOGINDEX iAnalog,
//                                                  PDOUBLE pfdValue );
//AERERR_CODE AER_DLLENTRY AerVarAnalogOutSetDouble( HAERCTRL hAerCtrl,
//                                                   ANALOGINDEX iAnalog,
//                                                   DOUBLE fdValue );
/*
    low level functions
*/
AERERR_CODE AER_DLLENTRY aerPtrDblSetPtr( PPTR_DATA pPtr, DWORD dwPtrType,
                                          DWORD dwNum, DWORD dwIndex1,
                                          DWORD dwIndex2 );
AERERR_CODE AER_DLLENTRY aerPtrDblSetValue( HAERCTRL hAerCtrl, PPTR_DATA pPtr,
                                            DOUBLE fdValue );
AERERR_CODE AER_DLLENTRY aerPtrDblGetValue( HAERCTRL hAerCtrl, PPTR_DATA pPtr, PDOUBLE pfdValue );

AERERR_CODE AER_DLLENTRY aerPtrStrSetValue ( HAERCTRL hAerCtrl, PPTR_DATA pPtr, PAERSTRING128 pString128 );
AERERR_CODE AER_DLLENTRY aerPtrStrSetValue0( HAERCTRL hAerCtrl, PPTR_DATA pPtr, LPCTSTR pString );

AERERR_CODE AER_DLLENTRY aerPtrStrGetValue ( HAERCTRL hAerCtrl, PPTR_DATA pPtr, PAERSTRING128 pString128 );

AERERR_CODE AER_DLLENTRY aerPtrAPtSetValue( HAERCTRL hAerCtrl, PPTR_DATA pPtr, PAXISPOINT pPoint);
AERERR_CODE AER_DLLENTRY aerPtrAPtGetValue( HAERCTRL hAerCtrl, PPTR_DATA pPtr, PAXISPOINT pPoint);

AERERR_CODE AER_DLLENTRY aerPtrCSPSetValue( HAERCTRL hAerCtrl, PPTR_DATA pPtr, PCSPARM pCSParm);
AERERR_CODE AER_DLLENTRY aerPtrCSPGetValue( HAERCTRL hAerCtrl, PPTR_DATA pPtr, PCSPARM pCSParm);

#ifdef __cplusplus
}
#endif

#endif
// __AER_VAR_H__
