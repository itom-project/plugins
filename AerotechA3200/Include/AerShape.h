#ifndef __AER_SHAPE_H__
#define __AER_SHAPE_H__

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerShaperInitialize( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                              ULONG numPulses, PAIDWORD pdwDelayTimes,
                                              PAIDWORD pdwShaperCoeffs );
AERERR_CODE AER_DLLENTRY AerShaperActivate( HAERCTRL hAerCtrl, AXISINDEX iAxis );
AERERR_CODE AER_DLLENTRY AerShaperDeactivate( HAERCTRL hAerCtrl, AXISINDEX iAxis );
AERERR_CODE AER_DLLENTRY AerShaperTest( HAERCTRL hAerCtrl );

#ifdef __cplusplus
}
#endif

#endif
