/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AERAT_H__
#define __AERAT_H__

#define AT_SINGULAR     0x0001
#define AT_ZEROJKT      0x0002
#define AT_GAINNEGATIVE 0x0004
#define AT_GAINSINVALID 0x0008

// Internal defines
#define AT_ERRORMARGIN (double)0.05
#define AT_SKIPSTART    30  // This factor is the starting point for the Regression matrix
                            // this allows skipping the first few points that are affected
                            // by the startup and filter transients

#define PI (double)3.14159265358979
#define PI2 (double)(PI * 2.0)

#define Tsamp 0.00005 // DSP sample time; based on 40MHz / 2048
// Time Constant of the Feedback Low-Pass Filter (used in PI Loop Gain Calculations)
#define RfCf 0.0000022

#define NORMAL 0

// Encoder Feedback Tuning
#define EFT_TUNING_BAND (double) 30.0
#define EFT_PHASE_TUNING_BAND (double) 0.01
#define TINY 1.0e-20
#define OFFSET 32767.0
#define GAIN 30000.0


AERERR_CODE AER_DLLENTRY AerTuneAutoTuneCalcGains (HAERCTRL hAerCtrl,
                                                   PAILONG plInputVel,
                                                   PAISHORT psInputTorq,
                                                   DWORD dwMode,
                                                   DWORD dwSampleTimeMSec,
                                                   DWORD dwNumberOfSamples,
                                                   DWORD dwAmpPeakRating,
                                                   DOUBLE dServoFreq,
                                                   DOUBLE dCenterFreq,
                                                   DOUBLE dZeta,
                                                   DOUBLE dAlpha,
                                                   DOUBLE dPhaseM,
                                                   PDOUBLE pdKpos,
                                                   PDOUBLE pdKi,
                                                   PDOUBLE pdKp,
                                                   PDOUBLE pdVff,
                                                   PDOUBLE pdAff,
                                                   PDOUBLE pdKv,
                                                   PDOUBLE pdJ_Kt,
                                                   PDOUBLE pdB_Kt,
								                           PDOUBLE pdFc_Kt,
                                                   PDOUBLE pdR2,
                                                   PWORD pwInfo );

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// TSL Pre 2.09 Auto Tune code
//
//AERERR_CODE AER_DLLENTRY AerTuneAutoTuneCalcGains (HAERCTRL hAerCtrl,
//                                                   PLONG plInputVel,
//                                                   PSHORT psInputTorq,
//                                                   DWORD dwMode,
//                                                   DWORD dwSampleTime,
//                                                   DWORD dwNumSamples,
//                                                   DWORD dwAmpPeakRating,
//                                                   DOUBLE dServoFreq,
//                                                   DOUBLE dCenterFreq,
//                                                   DOUBLE dZeta,
//                                                   DOUBLE dAlpha,
//                                                   DOUBLE dPhaseM,
//                                                   PDOUBLE pdKpos,
//                                                   PDOUBLE pdKi,
//                                                   PDOUBLE pdKp,
//                                                   PDOUBLE pdVff,
//                                                   PDOUBLE pdAff,
//                                                   PDOUBLE pdKf,
//                                                   PDOUBLE pdJ_Kt,
//                                                   PDOUBLE pdB_Kt,
//                                                   PWORD pwInfo );
////////////////////////////////////////////////////////////////////////////////////////////////////

AERERR_CODE AER_DLLENTRY AerTuneFFT(PAIFLOAT data, WORD n, WORD isign);

AERERR_CODE AER_DLLENTRY AerTuneFFT2(PAIDOUBLE data, WORD n, WORD isign);

AERERR_CODE AER_DLLENTRY AerTuneSetupPIGains( DOUBLE dMotorR,
											             DOUBLE dMotorL,
											             DOUBLE dCrossOverFreq,
											             DOUBLE dBusVoltage,
											             DOUBLE dAmpImax,
											             DOUBLE dPhaseMargin,
											             LONG lAmpType,
											             DOUBLE dSampDelay,
											             PDOUBLE pdIGainK,
											             PDOUBLE pdIGainKp,
											             PDOUBLE pdIGainKi,
                                              PDOUBLE pdActualCrossOverFreq );

AERERR_CODE AER_DLLENTRY AerTuneNotchFilterCalculate( DOUBLE SampleFreq,
											                     DOUBLE CenterFreq,
											                     DOUBLE Width,
											                     DOUBLE Depth,
                                                      PDOUBLE pN_0,
											                     PDOUBLE pN_1,
											                     PDOUBLE pN_2,
											                     PDOUBLE pD_1,
											                     PDOUBLE pD_2 );

AERERR_CODE AER_DLLENTRY AerTuneNotchFilterBackCalculate( DOUBLE SampleFreq,
                                                          DOUBLE N_0,
											                         DOUBLE N_1,
											                         DOUBLE N_2,
											                         DOUBLE D_1,
											                         DOUBLE D_2,
											                         PDOUBLE pCenterFreq,
											                         PDOUBLE pWidth,
											                         PDOUBLE pDepth );

AERERR_CODE AER_DLLENTRY AerTuneLowPassFilterCalculate( DOUBLE SampleFreq,
											                       DOUBLE CutoffFreq,
                                                        PDOUBLE pN_0,
											                       PDOUBLE pN_1,
											                       PDOUBLE pN_2,
											                       PDOUBLE pD_1,
											                       PDOUBLE pD_2 );


AERERR_CODE AER_DLLENTRY AerTuneLowpassFilterBackCalculate( DOUBLE SampleFreq,
                                                            DOUBLE N_0,
											                           DOUBLE N_1,
											                           DOUBLE N_2,
											                           DOUBLE D_1,
											                           DOUBLE D_2,
											                           PDOUBLE pCutoffFreq );

AERERR_CODE AER_DLLENTRY AerTuneLeadLagFilterCalculate( DOUBLE SampleFreq,
											         DOUBLE MaxPhaseFreq,
											         DOUBLE PhaseAngle,
                                                     PDOUBLE pN_0,
											         PDOUBLE pN_1,
											         PDOUBLE pN_2,
											         PDOUBLE pD_1,
											         PDOUBLE pD_2 );

AERERR_CODE AER_DLLENTRY AerTuneLeadLagFilterBackCalculate( DOUBLE SampleFreq,
                                                            DOUBLE N_0,
                                                            DOUBLE N_1,
                                                            DOUBLE N_2,
                                                            DOUBLE D_1,
                                                            DOUBLE D_2,
                                                            PDOUBLE pMaxPhaseFreq,
                                                            PDOUBLE pPhaseAngle );

AERERR_CODE AER_DLLENTRY AerTuneResonantFilterCalculate( DOUBLE SampleFreq,
                                                         DOUBLE CenterFreq,
                                                         DOUBLE Width,
                                                         DOUBLE Gain,
                                                         PDOUBLE pN_0,
                                                         PDOUBLE pN_1,
                                                         PDOUBLE pN_2,
                                                         PDOUBLE pD_1,
                                                         PDOUBLE pD_2 );

AERERR_CODE AER_DLLENTRY AerTuneResonantFilterBackCalculate(DOUBLE SampleFreq,
                                                            DOUBLE N_0,
                                                            DOUBLE N_1,
                                                            DOUBLE N_2,
                                                            DOUBLE D_1,
                                                            DOUBLE D_2,
                                                            PDOUBLE pCenterFreq,
                                                            PDOUBLE pWidth,
                                                            PDOUBLE pGain );

AERERR_CODE AER_DLLENTRY AerTuneEFTClear ();

AERERR_CODE AER_DLLENTRY AerTuneEFTCalcGains (PAIDOUBLE Sine,
                                              PAIDOUBLE Cosine,
                                              LONG NumberSamples,
                                              LONG OldSineGain,
                                              LONG OldSineOffset,
                                              LONG OldCosineGain,
                                              LONG OldCosineOffset,
                                              LONG OldPhase,
                                              PLONG plNewSineGain,
                                              PLONG plNewSineOffset,
                                              PLONG plNewCosineGain,
                                              PLONG plNewCosineOffset,
                                              PLONG plNewPhase,
	                                           PDOUBLE pdGX,
                                              PDOUBLE pdX0,
                                              PDOUBLE pdPH,
	                                           PDOUBLE pdGY,
                                              PDOUBLE pdY0);

AERERR_CODE AER_DLLENTRY AerTuneEFTCalcGainsQuick (PDOUBLE Sine,
                                                   PDOUBLE Cosine,
                                                   DWORD NumberSamples,
                                                   DWORD OldSineGain,
                                                   DWORD OldSineOffset,
                                                   DWORD OldCosineGain,
                                                   DWORD OldCosineOffset,
                                                   DWORD OldPhase,
                                                   PDWORD pdwNewSineGain,
                                                   PDWORD pdwNewSineOffset,
                                                   PDWORD pdwNewCosineGain,
                                                   PDWORD pdwNewCosineOffset,
                                                   PDWORD pdwNewPhase,
	                                                PDOUBLE pdGX,
                                                   PDOUBLE pdX0,
                                                   PDOUBLE pdPH,
	                                                PDOUBLE pdGY,
                                                   PDOUBLE pdY0);

AERERR_CODE AER_DLLENTRY AerTuneEFTCalcEllipseCoord (PAIDOUBLE Sine,
                                                     PAIDOUBLE Cosine,
                                                     LONG NumberSamples,
                                                     PDOUBLE pdGX,
                                                     PDOUBLE pdX0,
                                                     PDOUBLE pdPH,
                                                     PDOUBLE pdGY,
                                                     PDOUBLE pdY0);

AERERR_CODE AER_DLLENTRY AerTuneDigitalLoopTransmission( HAERCTRL    hAerCtrl,
                                                         AXISINDEX   iAxis,
                                                         DWORD       dwMode,
                                                         DOUBLE      fdAmplitude,
                                                         DOUBLE      fdFrequency,
														               DWORD       dwType);

AERERR_CODE AER_DLLENTRY AerTuneSpectrum( PDOUBLE pInput,
										            PDOUBLE pOutput,
										            DWORD dwNumSamples,
										            DWORD dwNsects,
										            PDOUBLE pMag,
										            PDOUBLE pPhase);

AERERR_CODE AER_DLLENTRY AerTuneFFTbase2(PAIDOUBLE data,
                                         DWORD dwNumSamples);

AERERR_CODE AER_DLLENTRY AerTuneRandNormalDist(DWORD dwNumSamples,
                                               PDOUBLE dRandData);


AERERR_CODE AER_DLLENTRY AerTuneGetLoopTransMagPhaseOffset( PAIDOUBLE pddata,
                                                            DWORD   dwNumSamples,
                                                            DOUBLE  dFrequency,
                                                            DOUBLE  dSampleTime,
                                                            PDOUBLE pdMag,
                                                            PDOUBLE pdPhase,
                                                            PDOUBLE pdOffset);

AERERR_CODE AER_DLLENTRY AerTuneSineLoopTransmissionInit( HAERCTRL hAerCtrl,
                                                          AXISINDEX iAxis,
                                                          DWORD  dwType );


AERERR_CODE AER_DLLENTRY AerTuneSineLoopTransmission(HAERCTRL hAerCtrl,
                                                     AXISINDEX iAxis,
                                                     DWORD  dwType,
                                                     DOUBLE dAmplitude,
                                                     DOUBLE dFrequency,
                                                     DWORD dwNumSineWavesCapture);

AERERR_CODE AER_DLLENTRY AerTuneSineLoopTransGetData(HAERCTRL hAerCtrl,
                                                     AXISINDEX iAxis,
                                                     DWORD  dwType,
                                                     DOUBLE dFrequency,
                                                     BOOL bWaitForData,
                                                     PBOOL pbCompleted,
                                                     PDOUBLE pdMagnitude,
                                                     PDOUBLE pdPhase,
                                                     PBOOL pbDataValid);


AERERR_CODE AER_DLLENTRY AerTuneSineLoopTransmissionHalt( HAERCTRL hAerCtrl,
                                                          AXISINDEX iAxis,
                                                          DWORD  dwType );

#endif
// __AERAT_H__
