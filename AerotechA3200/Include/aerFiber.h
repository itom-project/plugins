/*
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_FIBER_H__
#define __AER_FIBER_H__


AERERR_CODE AER_DLLENTRY AerFiberAlignIsActive(HAERCTRL hAerCtrl, TASKINDEX iTask, PBOOL pbIsRunning,
                                               PWORD pwAlignID);

AERERR_CODE AER_DLLENTRY AerFiberGetErrorCode(HAERCTRL hAerCtrl, TASKINDEX iTask, WORD wAlignID,
                                              PAERERR_CODE pdwErrorCode);

AERERR_CODE AER_DLLENTRY AerFiberGetPowerValue(HAERCTRL hAerCtrl, TASKINDEX iTask, WORD wAlignID,
                                               PDOUBLE pfdPowerValue, PDOUBLE pfdMaxPowerValue);

AERERR_CODE AER_DLLENTRY AerFiberGetSavedDataStatus(HAERCTRL hAerCtrl, TASKINDEX iTask, PBOOL pbDataValid,
                                                    PWORD pwAlignID, PWORD pwMaxPoints, PWORD pwNumPoints);

AERERR_CODE AER_DLLENTRY AerFiberGetSavedData( HAERCTRL hAerCtrl, TASKINDEX iTask, WORD wColumn,
                                               WORD wNumPoints, PAODOUBLE pfdData);



#endif
