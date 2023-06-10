
#ifndef __AERSERV_H__
#define __AERSERV_H__

BOOL AER_DLLENTRY        AerSysServerIsOpen(void);
//BOOL AER_DLLENTRY        AerSysServerIsOpen2(PDWORD pdwResetStatus);
AERERR_CODE AER_DLLENTRY AerSysServerStart(void);
AERERR_CODE AER_DLLENTRY AerSysServerStartAsyncOnly(void);

AERERR_CODE AER_DLLENTRY AerSysServerGetData(HAERCTRL hAerCtrl, PAERSOFT_SERVERDATA pServerData);
AERERR_CODE AER_DLLENTRY AerSysServerGetDataEx(HAERCTRL hAerCtrl, PAERERR_CODE dwErr, PAERERR_CODE dwWarn);
AERERR_CODE AER_DLLENTRY AerSysServerGetDataTiming(HAERCTRL hAerCtrl, PDOUBLE pdTimeFirewire, PDOUBLE pdTimeTask);
AERERR_CODE AER_DLLENTRY AerSysServerResetData(HAERCTRL hAerCtrl);
AERERR_CODE AER_DLLENTRY AerSysServerSetErrorLogMode(HAERCTRL hAerCtrl, DWORD dwErrLogMode);
AERERR_CODE AER_DLLENTRY AerSysServerGetLastError(HAERCTRL hAerCtrl,  PDWORD pdwErrorDeath, PDWORD pdwErrorComotose );
AERERR_CODE AER_DLLENTRY AerSysServerGeneral(HAERCTRL hAerCtrl, WORD wSubCode, DWORD dwData);

#endif
