/// \file EnsembleDataCollectionInfo.h
/// \brief Contains the defines for datacollection item numbers.
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_DATACOLLECTION_INFO_H__
#define __Ensemble_DATACOLLECTION_INFO_H__

typedef enum {
	/// \brief Optional signal. This item returns the encoder sine in A/D counts. No extra argument.
	DATASIGNAL_EncoderSine = 26,
	/// \brief Optional signal. This item returns the encoder cosine in A/D counts. No extra argument.
	DATASIGNAL_EncoderCosine = 27,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_LoopTransmissionBefore = 28,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_LoopTransmissionAfter = 29,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_DriveMemoryInt32 = 30,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_DriveMemoryFloat = 31,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_DriveMemoryDouble = 32,
	/// \brief Optional signal. This item returns the amplifier temperature of the unit. No extra argument.
	DATASIGNAL_AmplifierTemperature = 33,
	/// \brief Optional signal. This item returns the value of analog input 2. No extra argument.
	DATASIGNAL_AnalogInput2 = 34,
	/// \brief Optional signal. This item returns the value of analog input 3. No extra argument.
	DATASIGNAL_AnalogInput3 = 35,
	/// \brief Optional signal. This item returns the value of analog output 2. No extra argument.
	DATASIGNAL_AnalogOutput2 = 36,
	/// \brief Optional signal. This item returns the value of analog output 3. No extra argument.
	DATASIGNAL_AnalogOutput3 = 37,
	/// \brief Optional signal. This item returns the PSO status. Extra argument: Mask.
	DATASIGNAL_PSOStatus = 38,
	/// \brief Optional signal. This item returns the value of PSO counter 1 in counts. No extra argument.
	DATASIGNAL_PSOCounter1 = 39,
	/// \brief Optional signal. This item returns the value of PSO counter 2 in counts. No extra argument.
	DATASIGNAL_PSOCounter2 = 40,
	/// \brief Optional signal. This item returns the value of PSO counter 3 in counts. No extra argument.
	DATASIGNAL_PSOCounter3 = 41,
	/// \brief Optional signal. This item returns the value of the PSO window 1 counter in counts. No extra argument.
	DATASIGNAL_PSOWindow1 = 42,
	/// \brief Optional signal. This item returns the value of the PSO window 2 counter in counts. No extra argument.
	DATASIGNAL_PSOWindow2 = 43,
	/// \brief Optional signal. This item returns the number of data acquisition samples that have been acquired. No extra argument.
	DATASIGNAL_DataAcquisitionSamples = 44,
	/// \brief Optional signal. This item returns the phase A current feedback. No extra argument.
	DATASIGNAL_PhaseACurrentFeedback = 45,
	/// \brief Optional signal. This item returns the phase B current feedback. No extra argument.
	DATASIGNAL_PhaseBCurrentFeedback = 46,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_DriveMemoryInt8 = 47,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_DriveMemoryInt16 = 48,
	/// \brief Optional signal. This item returns the 1D + 2D position calibration contributions in counts. No extra argument.
	DATASIGNAL_PositionCalibrationAll = 49,
	/// \brief Optional signal. This item returns the value of the first resolver channel in counts. No extra argument.
	DATASIGNAL_ResolverChannel1 = 50,
	/// \brief Optional signal. This item returns the value of the second resolver channel in counts. No extra argument.
	DATASIGNAL_ResolverChannel2 = 51,
	/// \brief Optional signal. This item returns the value of the EnDat absolute position in counts. No extra argument.
	DATASIGNAL_EnDatAbsolutePosition = 52,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_DriveTimer = 53,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_PhaseAVoltageCommand = 54,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_PhaseBVoltageCommand = 55,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_PhaseCVoltageCommand = 56,
	/// \brief Optional signal. This item returns the peak current rating of the amplifier. No extra argument.
	DATASIGNAL_AmplifierPeakCurrent = 57,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_FPGAVersion = 58,
	/// \brief Optional signal. This item returns an ID number identifying the type of drive (e.g., CP, HPe, Epaq etc). No extra argument.
	DATASIGNAL_DriveTypeID = 59,
	/// \brief Optional signal. This item returns the current array index for window 1 when using the PSOWINDOW RANGE ARRAY command. No extra argument.
	DATASIGNAL_PSOWindow1ArrayIndex = 60,
	/// \brief Optional signal. This item returns the current array index for window 2 when using the PSOWINDOW RANGE ARRAY command. No extra argument.
	DATASIGNAL_PSOWindow2ArrayIndex = 61,
	/// \brief Optional signal. This item returns the current array index for the PSODISTANCE ARRAY command. No extra argument.
	DATASIGNAL_PSODistanceArrayIndex = 62,
	/// \brief Optional signal. This item returns the current array index for the PSOOUTPUT BIT MAP or PSOOUTPUT PULSE BIT MASK commands. No extra argument.
	DATASIGNAL_PSOBitArrayIndex = 63,
	/// \brief Optional signal. This item returns the absolute encoder angle for the optional Encoder Multiplier. No extra argument.
	DATASIGNAL_MXAbsolutePosition = 64,
	/// \brief Optional signal. This item returns the servo update rate in kilohertz. No extra argument.
	DATASIGNAL_ServoUpdateRate = 65,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_FirmwareVersionMajor = 66,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_FirmwareVersionMinor = 67,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_FirmwareVersionPatch = 68,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_FirmwareVersionBuild = 69,
	/// \brief Optional signal. This item returns the maximum value of the timer returned by the drive. No extra argument.
	DATASIGNAL_DriveTimerMax = 70,
	/// \brief Optional signal. This item returns the distance in counts that was traveled while searching for the marker. No extra argument.
	DATASIGNAL_MarkerSearchDistance = 71,
	/// \brief Optional signal. This item returns the position at which the marker was detected during the home cycle. No extra argument.
	DATASIGNAL_LatchedMarkerPosition = 72,
	/// \brief Optional signal. This is an internal signal, use only if directed by Aerotech.
	DATASIGNAL_EthernetDebuggingInformation = 73,
	/// \brief Optional signal. This item returns the value of the Resolute absolute position in counts. No extra argument.
	DATASIGNAL_ResoluteAbsolutePosition = 74,
	/// \brief Axis signal. This item returns the position feedback. No extra argument.
	DATASIGNAL_PositionFeedback = 0,
	/// \brief Axis signal. This item returns the position command. No extra argument.
	DATASIGNAL_PositionCommand = 1,
	/// \brief Axis signal. This item returns the position error. No extra argument.
	DATASIGNAL_PositionError = 2,
	/// \brief Axis signal. This item returns the velocity feedback. No extra argument.
	DATASIGNAL_VelocityFeedback = 3,
	/// \brief Axis signal. This item returns the velocity command. No extra argument.
	DATASIGNAL_VelocityCommand = 4,
	/// \brief Axis signal. This item returns the velocity error. No extra argument.
	DATASIGNAL_VelocityError = 5,
	/// \brief Axis signal. This item returns the value of the acceleration command. No extra argument.
	DATASIGNAL_AccelerationCommand = 6,
	/// \brief Axis signal. This item returns the current feedback. No extra argument.
	DATASIGNAL_CurrentFeedback = 7,
	/// \brief Axis signal. This item returns the current command. No extra argument.
	DATASIGNAL_CurrentCommand = 8,
	/// \brief Axis signal. This item returns the current error. No extra argument.
	DATASIGNAL_CurrentError = 9,
	/// \brief Axis signal. This item returns the value of analog input 0. No extra argument.
	DATASIGNAL_AnalogInput0 = 10,
	/// \brief Axis signal. This item returns the value of analog input 1. No extra argument.
	DATASIGNAL_AnalogInput1 = 11,
	/// \brief Axis signal. This item returns the value of analog output 0. No extra argument.
	DATASIGNAL_AnalogOutput0 = 12,
	/// \brief Axis signal. This item returns the value of analog output 1. No extra argument.
	DATASIGNAL_AnalogOutput1 = 13,
	/// \brief Axis signal. This item returns the position feedback auxiliary position. No extra argument.
	DATASIGNAL_PositionFeedbackAuxiliary = 14,
	/// \brief Axis signal. This item returns the value of the digital input word that is read from the drive. No extra argument.
	DATASIGNAL_DigitalInput0 = 15,
	/// \brief Axis signal. This item returns the value of the digital input word that is read from the drive. No extra argument.
	DATASIGNAL_DigitalInput1 = 16,
	/// \brief Axis signal. This item returns the value of the digital input word that is read from the drive. No extra argument.
	DATASIGNAL_DigitalInput2 = 17,
	/// \brief Axis signal. This item returns the value of the digital output word that is read from the drive. No extra argument.
	DATASIGNAL_DigitalOutput0 = 18,
	/// \brief Axis signal. This item returns the value of the digital output word that is read from the drive. No extra argument.
	DATASIGNAL_DigitalOutput1 = 19,
	/// \brief Axis signal. This item returns the value of the digital output word that is read from the drive. No extra argument.
	DATASIGNAL_DigitalOutput2 = 20,
	/// \brief Axis signal. This item returns the value of the axis fault word. Extra argument: Mask.
	DATASIGNAL_AxisFault = 22,
	/// \brief Axis signal. This item returns the value of the axis status word. Extra argument: Mask.
	DATASIGNAL_AxisStatus = 23,
	/// \brief Task signal. This item returns the value of the program counter. No extra argument.
	DATASIGNAL_ProgramCounter = 21,
} DATASIGNAL;

#endif // __Ensemble_DATACOLLECTION_INFO_H__
