/*
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_VME_H__
#define __AER_VME_H__

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY VME_read_reg(HAERCTRL hAerCtrl, LONG lMode, LONG lAddress, PLONG plValue);
AERERR_CODE AER_DLLENTRY VME_write_reg(HAERCTRL hAerCtrl, LONG lMode, LONG lAddress, LONG lValue);
AERERR_CODE AER_DLLENTRY VME_read_buffer(HAERCTRL hAerCtrl, LONG lMode, LONG lAddress, PAOLONG plBuffer, LONG lSize);
AERERR_CODE AER_DLLENTRY VME_write_buffer(HAERCTRL hAerCtrl, LONG lMode, LONG lAddress, PAILONG plBuffer, LONG lSize);
AERERR_CODE AER_DLLENTRY VME_write_byte_blocks(HAERCTRL hAerCtrl, LONG lAddress, PAILONG plBuffer, LONG lNumBytes, LONG lDelay);
long AER_DLLENTRY VME_Get_Cycles_Per_Microsecond();


#ifdef __cplusplus
}
#endif

#endif
