/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_VIRT_H__
#define __AER_VIRT_H__

#ifdef __cplusplus
extern "C" {
#endif


//AERERR_CODE AER_DLLENTRY AerVirtGetBinaryIO( HAERCTRL hAerCtrl,
//                                             PAERVIRT_BINARY_DATA pInputs,
//                                             PAERVIRT_BINARY_DATA pOutputs );
//AERERR_CODE AER_DLLENTRY AerVirtSetBinaryIO( HAERCTRL hAerCtrl,
//                                             PAERVIRT_BINARY_DATA pInputs,
//                                             PAERVIRT_BINARY_DATA pOutputs );
//AERERR_CODE AER_DLLENTRY AerVirtGetRegisterIO( HAERCTRL hAerCtrl,
//                                               PAERVIRT_REGISTER_DATA pInputs,
//                                               PAERVIRT_REGISTER_DATA pOutputs );
//AERERR_CODE AER_DLLENTRY AerVirtSetRegisterIO( HAERCTRL hAerCtrl,
//                                               PAERVIRT_REGISTER_DATA pInputs,
//                                               PAERVIRT_REGISTER_DATA pOutputs );

AERERR_CODE AER_DLLENTRY AerIOVirtGetBinaryInput( HAERCTRL hAerCtrl, DWORD dwNum, PBOOL pbValue);
AERERR_CODE AER_DLLENTRY AerIOVirtGetBinaryOutput( HAERCTRL hAerCtrl, DWORD dwNum, PBOOL pbValue);
AERERR_CODE AER_DLLENTRY AerIOVirtGetRegisterInput( HAERCTRL hAerCtrl, DWORD dwNum, PWORD pwValue);
AERERR_CODE AER_DLLENTRY AerIOVirtGetRegisterOutput( HAERCTRL hAerCtrl, DWORD dwNum, PWORD pwValue);

AERERR_CODE AER_DLLENTRY AerIOVirtSetBinaryInput( HAERCTRL hAerCtrl, DWORD dwNum, BOOL bValue);
AERERR_CODE AER_DLLENTRY AerIOVirtSetBinaryOutput( HAERCTRL hAerCtrl, DWORD dwNum, BOOL bValue);
AERERR_CODE AER_DLLENTRY AerIOVirtSetRegisterInput( HAERCTRL hAerCtrl, DWORD dwNum, WORD wValue);
AERERR_CODE AER_DLLENTRY AerIOVirtSetRegisterOutput( HAERCTRL hAerCtrl, DWORD dwNum, WORD wValue);

//AERERR_CODE AER_DLLENTRY AerIOVirtGetBinaryInputByteEx( HAERCTRL hAerCtrl, DWORD dwByte, PBYTE pbyData );
//AERERR_CODE AER_DLLENTRY AerIOVirtSetBinaryInputByteEx( HAERCTRL hAerCtrl, DWORD dwByte, BYTE byData );
//AERERR_CODE AER_DLLENTRY AerIOVirtGetBinaryOutputByteEx( HAERCTRL hAerCtrl, DWORD dwByte, PBYTE pbyData );
//AERERR_CODE AER_DLLENTRY AerIOVirtSetBinaryOutputByteEx( HAERCTRL hAerCtrl, DWORD dwByte, BYTE byData );

//AERERR_CODE AER_DLLENTRY AerIOVirtGetBinaryInputWordEx( HAERCTRL hAerCtrl, DWORD dwWord, PWORD pwData );
//AERERR_CODE AER_DLLENTRY AerIOVirtSetBinaryInputWordEx( HAERCTRL hAerCtrl, DWORD dwWord, WORD wData );
//AERERR_CODE AER_DLLENTRY AerIOVirtGetBinaryOutputWordEx( HAERCTRL hAerCtrl, DWORD dwWord, PWORD pwData );
//AERERR_CODE AER_DLLENTRY AerIOVirtSetBinaryOutputWordEx( HAERCTRL hAerCtrl, DWORD dwWord, WORD wData );
//
//AERERR_CODE AER_DLLENTRY AerIOVirtGetBinaryInputDWordEx( HAERCTRL hAerCtrl, DWORD dwDWord, PDWORD pdwData );
//AERERR_CODE AER_DLLENTRY AerIOVirtSetBinaryInputDWordEx( HAERCTRL hAerCtrl, DWORD dwDWord, DWORD dwData );
//AERERR_CODE AER_DLLENTRY AerIOVirtGetBinaryOutputDWordEx( HAERCTRL hAerCtrl, DWORD dwDWord, PDWORD pdwData );
//AERERR_CODE AER_DLLENTRY AerIOVirtSetBinaryOutputDWordEx( HAERCTRL hAerCtrl, DWORD dwDWord, DWORD dwData );
//
//AERERR_CODE AER_DLLENTRY AerIOVirtGetRegisterInputEx( HAERCTRL hAerCtrl, DWORD dwReg, PWORD pwData );
//AERERR_CODE AER_DLLENTRY AerIOVirtSetRegisterInputEx( HAERCTRL hAerCtrl, DWORD dwReg, WORD wData );
//AERERR_CODE AER_DLLENTRY AerIOVirtGetRegisterOutputEx( HAERCTRL hAerCtrl, DWORD dwReg, PWORD pwData );
//AERERR_CODE AER_DLLENTRY AerIOVirtSetRegisterOutputEx( HAERCTRL hAerCtrl, DWORD dwReg, WORD wData );
////
AERERR_CODE AER_DLLENTRY AerIOVirtGetBinaryInputWordArray( HAERCTRL hAerCtrl, DWORD dwStartWord,
                                                         PAOWORD pwArray, DWORD dwNumWords );
AERERR_CODE AER_DLLENTRY AerIOVirtSetBinaryInputWordArray( HAERCTRL hAerCtrl, DWORD dwStartWord,
                                                         PAIWORD pwArray, DWORD dwNumWords );
AERERR_CODE AER_DLLENTRY AerIOVirtGetBinaryOutputWordArray( HAERCTRL hAerCtrl, DWORD dwStartWord,
                                                          PAOWORD pwArray, DWORD dwNumWords );
AERERR_CODE AER_DLLENTRY AerIOVirtSetBinaryOutputWordArray( HAERCTRL hAerCtrl, DWORD dwStartWord,
                                                          PAIWORD pwArray, DWORD dwNumWords );
AERERR_CODE AER_DLLENTRY AerIOVirtGetRegisterInputArray( HAERCTRL hAerCtrl, DWORD dwStartReg,
                                                       PAOWORD pwArray, DWORD dwNumReg );
AERERR_CODE AER_DLLENTRY AerIOVirtSetRegisterInputArray( HAERCTRL hAerCtrl, DWORD dwStartReg,
                                                       PAIWORD pwArray, DWORD dwNumReg );
AERERR_CODE AER_DLLENTRY AerIOVirtGetRegisterOutputArray( HAERCTRL hAerCtrl, DWORD dwStartReg,
                                                        PAOWORD pwArray, DWORD dwNumReg );
AERERR_CODE AER_DLLENTRY AerIOVirtSetRegisterOutputArray( HAERCTRL hAerCtrl, DWORD dwStartReg,
                                                        PAIWORD pwArray, DWORD dwNumReg );

AERERR_CODE AER_DLLENTRY AerIOGetAnalogInput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PDOUBLE pdValue );
AERERR_CODE AER_DLLENTRY AerIOSetAnalogOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, DOUBLE dValue );
AERERR_CODE AER_DLLENTRY AerIOGetAnalogOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PDOUBLE pdValue );
AERERR_CODE AER_DLLENTRY AerIOGetDriveInput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PBOOL pbValue );
AERERR_CODE AER_DLLENTRY AerIOSetDriveOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, BOOL bValue );
AERERR_CODE AER_DLLENTRY AerIOGetDriveOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PBOOL pbValue );
AERERR_CODE AER_DLLENTRY AerIOGetDriveInputWord( HAERCTRL hAerCtrl, AXISINDEX iAxis, PWORD pwValue );
AERERR_CODE AER_DLLENTRY AerIOSetDriveOutputWord( HAERCTRL hAerCtrl, AXISINDEX iAxis, WORD wValue );
AERERR_CODE AER_DLLENTRY AerIOGetDriveOutputWord( HAERCTRL hAerCtrl, AXISINDEX iAxis, PWORD pwValue );

AERERR_CODE AER_DLLENTRY AerIOGetDriveInputDWord( HAERCTRL hAerCtrl, AXISINDEX iAxis, PDWORD pdwValue);
AERERR_CODE AER_DLLENTRY AerIOSetDriveOutputDWord( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwValue );
AERERR_CODE AER_DLLENTRY AerIOGetDriveOutputDWord( HAERCTRL hAerCtrl, AXISINDEX iAxis, PDWORD pdwValue);

AERERR_CODE AER_DLLENTRY AerIOSetAux( HAERCTRL hAerCtrl, AXISINDEX iAxis );

AERERR_CODE AER_DLLENTRY AerIOGetEtherBinInput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PBOOL pwValue );
AERERR_CODE AER_DLLENTRY AerIOSetEtherBinOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, BOOL wValue );
AERERR_CODE AER_DLLENTRY AerIOGetEtherBinOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PBOOL pwValue );
AERERR_CODE AER_DLLENTRY AerIOGetEtherRegInput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PWORD pwValue );
AERERR_CODE AER_DLLENTRY AerIOGetEtherRegInputF( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PFLOAT pfValue );
AERERR_CODE AER_DLLENTRY AerIOGetEtherRegInputD( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PDOUBLE pdValue );
AERERR_CODE AER_DLLENTRY AerIOSetEtherRegOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, WORD wValue );
AERERR_CODE AER_DLLENTRY AerIOGetEtherRegOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PWORD pwValue );
AERERR_CODE AER_DLLENTRY AerIOSetEtherRegOutputF( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, FLOAT fValue );
AERERR_CODE AER_DLLENTRY AerIOGetEtherRegOutputF( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PFLOAT pfValue );
AERERR_CODE AER_DLLENTRY AerIOSetEtherRegOutputD( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, DOUBLE dValue );
AERERR_CODE AER_DLLENTRY AerIOGetEtherRegOutputD( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PDOUBLE pdValue );
AERERR_CODE AER_DLLENTRY AerIOGetEtherPrcInput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PWORD pwValue );
AERERR_CODE AER_DLLENTRY AerIOSetEtherPrcOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, WORD wValue );
AERERR_CODE AER_DLLENTRY AerIOGetEtherPrcOutput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PWORD pwValue );
AERERR_CODE AER_DLLENTRY AerIOGetEtherCfgInput( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwNum, PWORD pwValue );

AERERR_CODE AER_DLLENTRY AerIOSetAnalogOutputMult( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwChangeMask, DOUBLE dValues[ANALOGS_IN_EACH_DRIVE] );
AERERR_CODE AER_DLLENTRY AerIOSetEtherBinOutputMult( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwChangeMask, DWORD dwChangeValues, WORD wStartIndx );
AERERR_CODE AER_DLLENTRY AerIOGetEtherBinOutputMult( HAERCTRL hAerCtrl, AXISINDEX iAxis, WORD wStartIndx, WORD wNumDWords, PAODWORD pdwDwords);
AERERR_CODE AER_DLLENTRY AerIOGetEtherBinInputMult( HAERCTRL hAerCtrl, AXISINDEX iAxis, WORD wStartIndx, WORD wNumDWords, PAODWORD pdwDwords);


AERERR_CODE AER_DLLENTRY AerIOModeEncoderSigOutHS(HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD iMode, ULONG uAxisSel1, ULONG uAxisSel2, ULONG uAxisSel3);

#ifdef __cplusplus
}
#endif

#endif
// __AER_VIRT_H__
