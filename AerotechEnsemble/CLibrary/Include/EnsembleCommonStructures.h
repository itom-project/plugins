/// \file EnsembleCommonStructures.h
/// \brief Contains some common structures and enumerations.
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_COMMON_STRUCTURES_H__
#define __Ensemble_COMMON_STRUCTURES_H__

/// \brief Represents the servo update rates
typedef enum
{
	/// \brief 1 kHz rate (1 msec update time)
	SERVORATEPARAMETER_OnekHz = 0,
	/// \brief 2 kHz rate (0.5 msec update time)
	SERVORATEPARAMETER_TwokHz = 1,
	/// \brief 4 kHz rate (0.25 msec update time)
	SERVORATEPARAMETER_FourkHz = 2,
	/// \brief 5 kHz rate (0.2 msec update time)
	SERVORATEPARAMETER_FivekHz = 3,
	/// \brief 10 kHz rate (0.1 msec update time)
	SERVORATEPARAMETER_TenkHz = 4,
	/// \brief 20 kHz rate (0.05 msec update time)
	SERVORATEPARAMETER_TwentykHz = 5,
}	SERVORATEPARAMETER;

/// \brief This value represents information about the state of this task.
typedef enum
{
	/// \brief This task was disabled by the TaskExecutionSetup parameter.
	TASKSTATE_Inactive = 0,
	/// \brief No program is associated or running.
	TASKSTATE_Idle = 1,
	/// \brief A program is associated but not running.
	TASKSTATE_ProgramReady = 2,
	/// \brief A program is associated and running.
	TASKSTATE_ProgramRunning = 3,
	/// \brief A program is associated, was run, and was paused.
	TASKSTATE_ProgramPaused = 4,
	/// \brief A program is associated, was run, and completed.
	TASKSTATE_ProgramComplete = 5,
	/// \brief A task error occurred on this task.
	TASKSTATE_Error = 6,
	/// \brief A task-based plugin is running in this task.
	TASKSTATE_RunningPlugin = 10,
}	TASKSTATE;

/// \brief Represents plane status
typedef enum
{
	/// \brief The plane is generating motion on one or more axes.
	PLANESTATUS_MotionActive = (1u << 0),
	/// \brief The plane is generating a contoured motion on one or more axes.
	PLANESTATUS_VelocityProfilingActive = (1u << 1),
	/// \brief The plane is accelerating to the commanded velocity.
	PLANESTATUS_AccelerationPhaseActive = (1u << 2),
	/// \brief The plane is decelerating to the commanded velocity.
	PLANESTATUS_DecelerationPhaseActive = (1u << 3),
	/// \brief The plane is decelerating to zero velocity when commanded to abort.
	PLANESTATUS_MotionAborting = (1u << 4),
	/// \brief The plane is waiting for the START command to begin motion.
	PLANESTATUS_HoldModeActive = (1u << 5),
}	PLANESTATUS;
typedef enum
{
	/// \brief The plane is generating motion on one or more axes.
	PLANESTATUSBITS_MotionActiveBit = 0,
	/// \brief The plane is generating a contoured motion on one or more axes.
	PLANESTATUSBITS_VelocityProfilingActiveBit = 1,
	/// \brief The plane is accelerating to the commanded velocity.
	PLANESTATUSBITS_AccelerationPhaseActiveBit = 2,
	/// \brief The plane is decelerating to the commanded velocity.
	PLANESTATUSBITS_DecelerationPhaseActiveBit = 3,
	/// \brief The plane is decelerating to zero velocity when commanded to abort.
	PLANESTATUSBITS_MotionAbortingBit = 4,
	/// \brief The plane is waiting for the START command to begin motion.
	PLANESTATUSBITS_HoldModeActiveBit = 5,
}	PLANESTATUSBITS;

/// \brief Represents the debug flags on the controller
typedef enum
{
	/// \brief
	DEBUGFLAGS_PrintStringCallbackPending = (1u << 0),
	/// \brief
	DEBUGFLAGS_CollectionTriggered = (1u << 1),
	/// \brief
	DEBUGFLAGS_CollectionDone = (1u << 2),
	/// \brief
	DEBUGFLAGS_InputBoxCallbackPending = (1u << 3),
}	DEBUGFLAGS;
typedef enum
{
	/// \brief
	DEBUGFLAGSBITS_PrintStringCallbackPendingBit = 0,
	/// \brief
	DEBUGFLAGSBITS_CollectionTriggeredBit = 1,
	/// \brief
	DEBUGFLAGSBITS_CollectionDoneBit = 2,
	/// \brief
	DEBUGFLAGSBITS_InputBoxCallbackPendingBit = 3,
}	DEBUGFLAGSBITS;

/// \brief Represents an axis status
typedef enum
{
	/// \brief The axis is enabled.
	AXISSTATUS_Enabled = (1u << 0),
	/// \brief The axis is homed.
	AXISSTATUS_Homed = (1u << 1),
	/// \brief The axis is considered to be in position as configured by the InPositionDistance and InPositionTime parameters.
	AXISSTATUS_InPosition = (1u << 2),
	/// \brief The axis is performing drive generated motion.
	AXISSTATUS_MoveActive = (1u << 3),
	/// \brief The axis is accelerating.
	AXISSTATUS_AccelerationPhase = (1u << 4),
	/// \brief The axis is decelerating.
	AXISSTATUS_DecelerationPhase = (1u << 5),
	/// \brief Position capture is armed on the axis and waiting for a trigger signal.
	AXISSTATUS_PositionCaptureActive = (1u << 6),
	/// \brief The motor output is being clamped to the value specified by the MaxCurrentClamp parameter.
	AXISSTATUS_CurrentClamp = (1u << 7),
	/// \brief This represents the state of the dedicated brake output.
	AXISSTATUS_BrakeOutput = (1u << 8),
	/// \brief Indicates the direction of the active (or last) motion.
	AXISSTATUS_MotionIsCw = (1u << 9),
	/// \brief
	AXISSTATUS_MasterSlaveControl = (1u << 10),
	/// \brief The correction table for this axis is currently being applied.
	AXISSTATUS_CalibrationActive = (1u << 11),
	/// \brief A calibration file contains a calibration table that corrects this axis. The state of this bit is not affected by the CALENABLE or CALDISABLE commands.
	AXISSTATUS_CalibrationEnabled = (1u << 12),
	/// \brief The axis is currently performing motion under control of the JOYSTICK command.
	AXISSTATUS_JoystickControl = (1u << 13),
	/// \brief The axis is currently performing motion as part of the home cycle.
	AXISSTATUS_Homing = (1u << 14),
	/// \brief The axis position lost synchronization with the master and is ignoring profiled position.
	AXISSTATUS_MasterMotionSuppressed = (1u << 15),
	/// \brief This gantry is actively part of a gantry pair.
	AXISSTATUS_GantryModeActive = (1u << 16),
	/// \brief This axis is a gantry master in a gantry pair.
	AXISSTATUS_GantryMasterActive = (1u << 17),
	/// \brief This axis is operating under control of the AUTOFOCUS loop.
	AXISSTATUS_AutofocusActive = (1u << 18),
	/// \brief The filter defined by the Command Shaping Parameters is complete.
	AXISSTATUS_CommandShapingFilterDone = (1u << 19),
	/// \brief The axis is considered to be in position as configured by the InPosition2Distance and InPosition2Time parameters.
	AXISSTATUS_InPosition2 = (1u << 20),
	/// \brief This represents the state of the CW end of travel limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	AXISSTATUS_CwEndOfTravelLimitInput = (1u << 22),
	/// \brief This represents the state of the CCW end of travel limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	AXISSTATUS_CcwEndOfTravelLimitInput = (1u << 23),
	/// \brief This represents the state of the home limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	AXISSTATUS_HomeLimitInput = (1u << 24),
	/// \brief This represents the state of the marker input.
	AXISSTATUS_MarkerInput = (1u << 25),
	/// \brief This represents the state of the Hall-effect sensor A input.
	AXISSTATUS_HallAInput = (1u << 26),
	/// \brief This represents the state of the Hall-effect sensor B input.
	AXISSTATUS_HallBInput = (1u << 27),
	/// \brief This represents the state of the Hall-effect sensor C input.
	AXISSTATUS_HallCInput = (1u << 28),
	/// \brief An error condition is present on the Sine encoder input of the position feedback device.
	AXISSTATUS_SineEncoderError = (1u << 29),
	/// \brief An error condition is present on the Cosine encoder input of the position feedback device.
	AXISSTATUS_CosineEncoderError = (1u << 30),
	/// \brief This represents the state of the emergency stop sense input.
	AXISSTATUS_EmergencyStopInput = (1u << 31),
}	AXISSTATUS;
typedef enum
{
	/// \brief The axis is enabled.
	AXISSTATUSBITS_EnabledBit = 0,
	/// \brief The axis is homed.
	AXISSTATUSBITS_HomedBit = 1,
	/// \brief The axis is considered to be in position as configured by the InPositionDistance and InPositionTime parameters.
	AXISSTATUSBITS_InPositionBit = 2,
	/// \brief The axis is performing drive generated motion.
	AXISSTATUSBITS_MoveActiveBit = 3,
	/// \brief The axis is accelerating.
	AXISSTATUSBITS_AccelerationPhaseBit = 4,
	/// \brief The axis is decelerating.
	AXISSTATUSBITS_DecelerationPhaseBit = 5,
	/// \brief Position capture is armed on the axis and waiting for a trigger signal.
	AXISSTATUSBITS_PositionCaptureActiveBit = 6,
	/// \brief The motor output is being clamped to the value specified by the MaxCurrentClamp parameter.
	AXISSTATUSBITS_CurrentClampBit = 7,
	/// \brief This represents the state of the dedicated brake output.
	AXISSTATUSBITS_BrakeOutputBit = 8,
	/// \brief Indicates the direction of the active (or last) motion.
	AXISSTATUSBITS_MotionIsCwBit = 9,
	/// \brief
	AXISSTATUSBITS_MasterSlaveControlBit = 10,
	/// \brief The correction table for this axis is currently being applied.
	AXISSTATUSBITS_CalibrationActiveBit = 11,
	/// \brief A calibration file contains a calibration table that corrects this axis. The state of this bit is not affected by the CALENABLE or CALDISABLE commands.
	AXISSTATUSBITS_CalibrationEnabledBit = 12,
	/// \brief The axis is currently performing motion under control of the JOYSTICK command.
	AXISSTATUSBITS_JoystickControlBit = 13,
	/// \brief The axis is currently performing motion as part of the home cycle.
	AXISSTATUSBITS_HomingBit = 14,
	/// \brief The axis position lost synchronization with the master and is ignoring profiled position.
	AXISSTATUSBITS_MasterMotionSuppressedBit = 15,
	/// \brief This gantry is actively part of a gantry pair.
	AXISSTATUSBITS_GantryModeActiveBit = 16,
	/// \brief This axis is a gantry master in a gantry pair.
	AXISSTATUSBITS_GantryMasterActiveBit = 17,
	/// \brief This axis is operating under control of the AUTOFOCUS loop.
	AXISSTATUSBITS_AutofocusActiveBit = 18,
	/// \brief The filter defined by the Command Shaping Parameters is complete.
	AXISSTATUSBITS_CommandShapingFilterDoneBit = 19,
	/// \brief The axis is considered to be in position as configured by the InPosition2Distance and InPosition2Time parameters.
	AXISSTATUSBITS_InPosition2Bit = 20,
	/// \brief This represents the state of the CW end of travel limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	AXISSTATUSBITS_CwEndOfTravelLimitInputBit = 22,
	/// \brief This represents the state of the CCW end of travel limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	AXISSTATUSBITS_CcwEndOfTravelLimitInputBit = 23,
	/// \brief This represents the state of the home limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	AXISSTATUSBITS_HomeLimitInputBit = 24,
	/// \brief This represents the state of the marker input.
	AXISSTATUSBITS_MarkerInputBit = 25,
	/// \brief This represents the state of the Hall-effect sensor A input.
	AXISSTATUSBITS_HallAInputBit = 26,
	/// \brief This represents the state of the Hall-effect sensor B input.
	AXISSTATUSBITS_HallBInputBit = 27,
	/// \brief This represents the state of the Hall-effect sensor C input.
	AXISSTATUSBITS_HallCInputBit = 28,
	/// \brief An error condition is present on the Sine encoder input of the position feedback device.
	AXISSTATUSBITS_SineEncoderErrorBit = 29,
	/// \brief An error condition is present on the Cosine encoder input of the position feedback device.
	AXISSTATUSBITS_CosineEncoderErrorBit = 30,
	/// \brief This represents the state of the emergency stop sense input.
	AXISSTATUSBITS_EmergencyStopInputBit = 31,
}	AXISSTATUSBITS;

/// \brief Represents the faults of an axis
typedef enum
{
	/// \brief The absolute value of the difference between the position command and the position feedback exceeded the threshold specified by the PositionErrorThreshold parameter.
	AXISFAULT_PositionErrorFault = (1u << 0),
	/// \brief The average motor current exceeded the threshold specified by the AverageCurrentThreshold and AverageCurrentTime parameters.
	AXISFAULT_OverCurrentFault = (1u << 1),
	/// \brief The axis encountered the clockwise (positive) end-of-travel limit switch.
	AXISFAULT_CwEndOfTravelLimitFault = (1u << 2),
	/// \brief The axis encountered the counter-clockwise (negative) end-of-travel limit switch.
	AXISFAULT_CcwEndOfTravelLimitFault = (1u << 3),
	/// \brief The axis was commanded to move beyond the position specified by the SoftwareLimitHigh parameter.
	AXISFAULT_CwSoftwareLimitFault = (1u << 4),
	/// \brief The axis was commanded to move beyond the position specified by the SoftwareLimitLow parameter.
	AXISFAULT_CcwSoftwareLimitFault = (1u << 5),
	/// \brief The amplifier for this axis exceeded its maximum current rating or experienced an internal error.
	AXISFAULT_AmplifierFault = (1u << 6),
	/// \brief The drive detected a problem with the feedback device specified by the PositionFeedbackType and PositionFeedbackChannel parameters.
	AXISFAULT_PositionFeedbackFault = (1u << 7),
	/// \brief The drive detected a problem with the feedback device specified by the VelocityFeedbackType and VelocityFeedbackChannel parameters.
	AXISFAULT_VelocityFeedbackFault = (1u << 8),
	/// \brief The drive detected an invalid state (all high or all low) for the Hall-effect sensor inputs on this axis.
	AXISFAULT_HallSensorFault = (1u << 9),
	/// \brief The commanded velocity is more than the velocity command threshold. Before the axis is homed, this threshold is specified by the VelocityCommandThresholdBeforeHome parameter. After the axis is homed, this threshold is specified by the VelocityCommandThreshold parameter.
	AXISFAULT_MaxVelocityCommandFault = (1u << 10),
	/// \brief The emergency stop sense input, specified by the ESTOPFaultInput parameter, was triggered.
	AXISFAULT_EmergencyStopFault = (1u << 11),
	/// \brief The absolute value of the difference between the velocity command and the velocity feedback exceeded the threshold specified by the VelocityErrorThreshold parameter.
	AXISFAULT_VelocityErrorFault = (1u << 12),
	/// \brief The external fault input, specified by the ExternalFaultAnalogInput or ExternalFaultDigitalInput parameters, was triggered.
	AXISFAULT_ExternalFault = (1u << 15),
	/// \brief The motor thermistor input was triggered, which indicates that the motor exceeded its maximum recommended operating temperature.
	AXISFAULT_MotorTemperatureFault = (1u << 17),
	/// \brief The amplifier exceeded its maximum recommended operating temperature.
	AXISFAULT_AmplifierTemperatureFault = (1u << 18),
	/// \brief The encoder fault input on the motor feedback connector was triggered.
	AXISFAULT_EncoderFault = (1u << 19),
	/// \brief One or more of the drives on the network lost communications with the controller.
	AXISFAULT_CommunicationLostFault = (1u << 20),
	/// \brief The difference between the position feedback and the scaled (adjusted by GainKv) velocity feedback exceeds the threshold specified by the PositionErrorThreshold parameter.
	AXISFAULT_FeedbackScalingFault = (1u << 23),
	/// \brief The distance that the axis moved while searching for the marker exceeded the threshold specified by the MarkerSearchThreshold parameter.
	AXISFAULT_MarkerSearchFault = (1u << 24),
}	AXISFAULT;
typedef enum
{
	/// \brief The absolute value of the difference between the position command and the position feedback exceeded the threshold specified by the PositionErrorThreshold parameter.
	AXISFAULTBITS_PositionErrorFaultBit = 0,
	/// \brief The average motor current exceeded the threshold specified by the AverageCurrentThreshold and AverageCurrentTime parameters.
	AXISFAULTBITS_OverCurrentFaultBit = 1,
	/// \brief The axis encountered the clockwise (positive) end-of-travel limit switch.
	AXISFAULTBITS_CwEndOfTravelLimitFaultBit = 2,
	/// \brief The axis encountered the counter-clockwise (negative) end-of-travel limit switch.
	AXISFAULTBITS_CcwEndOfTravelLimitFaultBit = 3,
	/// \brief The axis was commanded to move beyond the position specified by the SoftwareLimitHigh parameter.
	AXISFAULTBITS_CwSoftwareLimitFaultBit = 4,
	/// \brief The axis was commanded to move beyond the position specified by the SoftwareLimitLow parameter.
	AXISFAULTBITS_CcwSoftwareLimitFaultBit = 5,
	/// \brief The amplifier for this axis exceeded its maximum current rating or experienced an internal error.
	AXISFAULTBITS_AmplifierFaultBit = 6,
	/// \brief The drive detected a problem with the feedback device specified by the PositionFeedbackType and PositionFeedbackChannel parameters.
	AXISFAULTBITS_PositionFeedbackFaultBit = 7,
	/// \brief The drive detected a problem with the feedback device specified by the VelocityFeedbackType and VelocityFeedbackChannel parameters.
	AXISFAULTBITS_VelocityFeedbackFaultBit = 8,
	/// \brief The drive detected an invalid state (all high or all low) for the Hall-effect sensor inputs on this axis.
	AXISFAULTBITS_HallSensorFaultBit = 9,
	/// \brief The commanded velocity is more than the velocity command threshold. Before the axis is homed, this threshold is specified by the VelocityCommandThresholdBeforeHome parameter. After the axis is homed, this threshold is specified by the VelocityCommandThreshold parameter.
	AXISFAULTBITS_MaxVelocityCommandFaultBit = 10,
	/// \brief The emergency stop sense input, specified by the ESTOPFaultInput parameter, was triggered.
	AXISFAULTBITS_EmergencyStopFaultBit = 11,
	/// \brief The absolute value of the difference between the velocity command and the velocity feedback exceeded the threshold specified by the VelocityErrorThreshold parameter.
	AXISFAULTBITS_VelocityErrorFaultBit = 12,
	/// \brief The external fault input, specified by the ExternalFaultAnalogInput or ExternalFaultDigitalInput parameters, was triggered.
	AXISFAULTBITS_ExternalFaultBit = 15,
	/// \brief The motor thermistor input was triggered, which indicates that the motor exceeded its maximum recommended operating temperature.
	AXISFAULTBITS_MotorTemperatureFaultBit = 17,
	/// \brief The amplifier exceeded its maximum recommended operating temperature.
	AXISFAULTBITS_AmplifierTemperatureFaultBit = 18,
	/// \brief The encoder fault input on the motor feedback connector was triggered.
	AXISFAULTBITS_EncoderFaultBit = 19,
	/// \brief One or more of the drives on the network lost communications with the controller.
	AXISFAULTBITS_CommunicationLostFaultBit = 20,
	/// \brief The difference between the position feedback and the scaled (adjusted by GainKv) velocity feedback exceeds the threshold specified by the PositionErrorThreshold parameter.
	AXISFAULTBITS_FeedbackScalingFaultBit = 23,
	/// \brief The distance that the axis moved while searching for the marker exceeded the threshold specified by the MarkerSearchThreshold parameter.
	AXISFAULTBITS_MarkerSearchFaultBit = 24,
}	AXISFAULTBITS;

/// \brief Specifies the status flags of data collection
typedef enum
{
	/// \brief Data collection was triggered
	DATACOLLECTIONFLAGS_Triggered = (1u << 2),
	/// \brief Data collection is done
	DATACOLLECTIONFLAGS_Done = (1u << 3),
	/// \brief Data collection buffer overflowed
	DATACOLLECTIONFLAGS_Overflow = (1u << 5),
	/// \brief Data collection was started by a SCOPETRIG
	DATACOLLECTIONFLAGS_IsScopeTrigInitiated = (1u << 6),
}	DATACOLLECTIONFLAGS;
typedef enum
{
	/// \brief Data collection was triggered
	DATACOLLECTIONFLAGSBITS_TriggeredBit = 2,
	/// \brief Data collection is done
	DATACOLLECTIONFLAGSBITS_DoneBit = 3,
	/// \brief Data collection buffer overflowed
	DATACOLLECTIONFLAGSBITS_OverflowBit = 5,
	/// \brief Data collection was started by a SCOPETRIG
	DATACOLLECTIONFLAGSBITS_IsScopeTrigInitiatedBit = 6,
}	DATACOLLECTIONFLAGSBITS;

/// \brief Specifies which status item to collect
typedef enum
{
	/// \brief Position command
	STATUSITEM_PositionCommand = 0,
	/// \brief Position feedback
	STATUSITEM_PositionFeedback = 1,
	/// \brief Position feedback auxiliary
	STATUSITEM_PositionFeedbackAuxiliary = 2,
	/// \brief Axis status
	STATUSITEM_AxisStatus = 3,
	/// \brief Axis fault
	STATUSITEM_AxisFault = 4,
	/// \brief Analog input #0
	STATUSITEM_AnalogInput0 = 5,
	/// \brief Analog input #1
	STATUSITEM_AnalogInput1 = 6,
	/// \brief Analog output #0
	STATUSITEM_AnalogOutput0 = 7,
	/// \brief Analog output #1
	STATUSITEM_AnalogOutput1 = 8,
	/// \brief Digital input #0
	STATUSITEM_DigitalInput0 = 9,
	/// \brief Digital input #1
	STATUSITEM_DigitalInput1 = 10,
	/// \brief Digital input #2
	STATUSITEM_DigitalInput2 = 11,
	/// \brief Digital output #0
	STATUSITEM_DigitalOutput0 = 12,
	/// \brief Digital output #1
	STATUSITEM_DigitalOutput1 = 13,
	/// \brief Digital output #2
	STATUSITEM_DigitalOutput2 = 14,
	/// \brief Current command
	STATUSITEM_CurrentCommand = 15,
	/// \brief Current feedback
	STATUSITEM_CurrentFeedback = 16,
	/// \brief Amplifier temperature
	STATUSITEM_AmplifierTemperature = 17,
	/// \brief Auxiliary flags
	STATUSITEM_DebugFlags = 18,
	/// \brief Program position command
	STATUSITEM_ProgramPositionCommand = 19,
	/// \brief Program count of library task
	STATUSITEM_ProgramCountTaskLibrary = 20,
	/// \brief Program count of task #1
	STATUSITEM_ProgramCountTask1 = 21,
	/// \brief Program count of task #2
	STATUSITEM_ProgramCountTask2 = 22,
	/// \brief Program count of task #3
	STATUSITEM_ProgramCountTask3 = 23,
	/// \brief Program count of task #4
	STATUSITEM_ProgramCountTask4 = 24,
	/// \brief Time of the packet retrieval in milliseconds
	STATUSITEM_PacketTime = 25,
	/// \brief Program position feedback
	STATUSITEM_ProgramPositionFeedback = 26,
	/// \brief Absolute feedback
	STATUSITEM_AbsoluteFeedback = 27,
	/// \brief Status of plane #0
	STATUSITEM_PlaneStatus0 = 28,
	/// \brief Status of plane #1
	STATUSITEM_PlaneStatus1 = 29,
	/// \brief Status of plane #2, Epaq only
	STATUSITEM_PlaneStatus2 = 30,
	/// \brief Status of plane #3, Epaq only
	STATUSITEM_PlaneStatus3 = 31,
	/// \brief State of task #1
	STATUSITEM_TaskState1 = 33,
	/// \brief State of task #2
	STATUSITEM_TaskState2 = 34,
	/// \brief State of task #3
	STATUSITEM_TaskState3 = 35,
	/// \brief State of task #4
	STATUSITEM_TaskState4 = 36,
	/// \brief Analog input #2
	STATUSITEM_AnalogInput2 = 38,
	/// \brief Analog input #3
	STATUSITEM_AnalogInput3 = 39,
	/// \brief Analog output #2
	STATUSITEM_AnalogOutput2 = 40,
	/// \brief Analog output #3
	STATUSITEM_AnalogOutput3 = 41,
	/// \brief Velocity Command
	STATUSITEM_VelocityCommand = 42,
	/// \brief Velocity Feedback
	STATUSITEM_VelocityFeedback = 43,
	/// \brief Acceleration Command
	STATUSITEM_AccelerationCommand = 44,
	/// \brief Velocity Error
	STATUSITEM_VelocityError = 45,
	/// \brief Position Error
	STATUSITEM_PositionError = 46,
	/// \brief Current Error
	STATUSITEM_CurrentError = 47,
	/// \brief Acceleration Feedback
	STATUSITEM_AccelerationFeedback = 48,
	/// \brief Acceleration Error
	STATUSITEM_AccelerationError = 49,
}	STATUSITEM;

/// \brief The type of loop transmission disturbance to use
typedef enum
{
	/// \brief Turn off loop transmission
	LOOPTRANSMISSIONMODE_Off = 0,
	/// \brief Uses a sinusoid disturbance
	LOOPTRANSMISSIONMODE_Sinusoid = 1,
	/// \brief Uses a sinusoid disturbance and excites both axes of a gantry
	LOOPTRANSMISSIONMODE_SinusoidGantry = 2,
}	LOOPTRANSMISSIONMODE;

/// \brief The loop transmission type to use
typedef enum
{
	/// \brief Open Loop
	LOOPTRANSMISSIONTYPE_OpenLoop = 0,
	/// \brief Closed Loop
	LOOPTRANSMISSIONTYPE_ClosedLoop = 1,
	/// \brief Current Loop
	LOOPTRANSMISSIONTYPE_CurrentLoop = 2,
	/// \brief AF Open Loop
	LOOPTRANSMISSIONTYPE_AFOpenLoop = 3,
	/// \brief AF Closed Loop
	LOOPTRANSMISSIONTYPE_AFClosedLoop = 4,
}	LOOPTRANSMISSIONTYPE;

/// \brief Represents the OnOff mode in AeroBasic
typedef enum
{
	/// \brief Off or 0 is issued
	ONOFF_Off = 0,
	/// \brief On or 1 is issued
	ONOFF_On = 1,
}	ONOFF;

/// \brief Represents the PSO Encoder in AeroBasic
typedef enum
{
	/// \brief Primary PSO encoder
	PSOENCODER_Primary = 1,
	/// \brief Auxiliary PSO encoder
	PSOENCODER_Auxiliary = 2,
	/// \brief MXH PSO encoder
	PSOENCODER_MXH = 3,
	/// \brief SSI Net Port 1 PSO encoder
	PSOENCODER_SSINetPort1 = 5,
	/// \brief SSI Net Port 2 PSO encoder
	PSOENCODER_SSINetPort2 = 6,
}	PSOENCODER;

/// \brief Represents the PSO mode in AeroBasic
typedef enum
{
	/// \brief Reset PSO
	PSOMODE_Reset = 0,
	/// \brief Turn off PSO
	PSOMODE_Off = 1,
	/// \brief Arm PSO
	PSOMODE_Arm = 2,
	/// \brief Fire PSO
	PSOMODE_Fire = 3,
	/// \brief Turn on PSO
	PSOMODE_On = 4,
	/// \brief Fire Continuous
	PSOMODE_FireContinuous = 5,
}	PSOMODE;

/// \brief The ethernet status information
typedef enum
{
	/// \brief Whether there is data to be transmitted
	ETHERNETSTATUS_DataInTransmitter = (1u << 0),
	/// \brief Whether there is data to be processed
	ETHERNETSTATUS_DataInReceiver = (1u << 1),
}	ETHERNETSTATUS;
typedef enum
{
	/// \brief Whether there is data to be transmitted
	ETHERNETSTATUSBITS_DataInTransmitterBit = 0,
	/// \brief Whether there is data to be processed
	ETHERNETSTATUSBITS_DataInReceiverBit = 1,
}	ETHERNETSTATUSBITS;

/// \brief Represents the semaphores in AeroBasic
typedef enum
{
	/// \brief Modbus registers semaphore
	SEMAPHORES_ModbusRegisters = 0,
	/// \brief Global integers semaphore
	SEMAPHORES_GlobalIntegers = 1,
	/// \brief Global doubles semaphore
	SEMAPHORES_GlobalDoubles = 2,
}	SEMAPHORES;

/// \brief The wait option for waiting for a move to be completed
typedef enum
{
	/// \brief Wait in position
	WAITOPTION_InPosition = 0,
	/// \brief Wait for move done
	WAITOPTION_MoveDone = 1,
}	WAITOPTION;

/// \brief Represents the wait mode in AeroBasic
typedef enum
{
	/// \brief Do not wait for motion to be done
	WAITTYPE_NoWait = 0,
	/// \brief Wait for motion to be done
	WAITTYPE_MoveDone = 1,
	/// \brief Wait for axis to be in position
	WAITTYPE_InPos = 2,
}	WAITTYPE;

/// \brief The Mode types that can be retrieved by using the GETMODE immediate command.
typedef enum
{
	/// \brief Returns the Motion Mode type (absolute/incremental).
	MODETYPE_MotionMode = 0,
	/// \brief Returns the Wait Mode type (in position / move done / no wait).
	MODETYPE_WaitMode = 1,
	/// \brief Returns the Ramp Mode type (linear / SCurve).
	MODETYPE_RampMode = 2,
	/// \brief Returns the Velocity Mode type (profiled / not profiled).
	MODETYPE_VelocityMode = 3,
	/// \brief Returns the SCurve value.
	MODETYPE_ScurveValue = 4,
	/// \brief Returns the Timescale value.
	MODETYPE_TimeScaleValue = 5,
	/// \brief Returns the default velocity value.
	MODETYPE_DefaultVelocityValue = 6,
	/// \brief Returns the acceleration rate value.
	MODETYPE_AccelRateValue = 7,
	/// \brief Returns the acceleration time value.
	MODETYPE_AccelTimeValue = 8,
	/// \brief Returns the acceleration distance value.
	MODETYPE_AccelDistValue = 9,
	/// \brief Returns the deceleration rate value.
	MODETYPE_DecelRateValue = 10,
	/// \brief Returns the deceleration time value.
	MODETYPE_DecelTimeValue = 11,
	/// \brief Returns the deceleration distance value.
	MODETYPE_DecelDistValue = 12,
	/// \brief Returns the current plane number for the task.
	MODETYPE_Plane = 14,
}	MODETYPE;

/// \brief The type of error that occured during compilation
typedef enum
{
	/// \brief The syntax does not follow AeroBasic
	COMPILERERRORTYPE_Syntax = 0,
	/// \brief Specific testing detected an error (ex: unknown register set)
	COMPILERERRORTYPE_Process = 1,
	/// \brief The operating system blocked an operation (ex: file access restriction)
	COMPILERERRORTYPE_System = 2,
	/// \brief An unrecoverable error occured (ex: out of memory)
	COMPILERERRORTYPE_Unrecoverable = 3,
	/// \brief The message given is a warning
	COMPILERERRORTYPE_Warning = 4,
}	COMPILERERRORTYPE;

/// \brief
typedef enum
{
	/// \brief
	DAYOFWEEK_Sunday = 0,
	/// \brief
	DAYOFWEEK_Monday = 1,
	/// \brief
	DAYOFWEEK_Tuesday = 2,
	/// \brief
	DAYOFWEEK_Wednesday = 3,
	/// \brief
	DAYOFWEEK_Thursday = 4,
	/// \brief
	DAYOFWEEK_Friday = 5,
	/// \brief
	DAYOFWEEK_Saturday = 6,
}	DAYOFWEEK;

/// \brief If this is different from , corrective measures should be taken.
typedef enum
{
	/// \brief There are no mismatches between the axes
	AXISMISMATCH_None = 0,
	/// \brief The version of firmware is different between the axes, update the firmware
	AXISMISMATCH_FirmwareVersion = 1,
	/// \brief The axes numbers are different from expected, please setup the axes that are expected
	AXISMISMATCH_AxisMask = 2,
}	AXISMISMATCH;

/// \brief This is an enumeration of the Register types used when calling the Register immediate command functions found in .
typedef enum
{
	/// \brief Global Integers
	REGISTERTYPE_GlobalIntegers = 0,
	/// \brief Global Doubles
	REGISTERTYPE_GlobalDoubles = 1,
	/// \brief Conversion Registers
	REGISTERTYPE_ConversionRegisters = 2,
	/// \brief Modbus Master Input Registers
	REGISTERTYPE_ModbusMasterInputWords = 3,
	/// \brief Modbus Master Output Registers
	REGISTERTYPE_ModbusMasterOutputWords = 4,
	/// \brief Modbus Master Input Bit Registers
	REGISTERTYPE_ModbusMasterInputBits = 5,
	/// \brief Modbus Master Output Bit Registers
	REGISTERTYPE_ModbusMasterOutputBits = 6,
	/// \brief Modbus Master Status Registers
	REGISTERTYPE_ModbusMasterStatusWords = 7,
	/// \brief Modbus Master Status Bit Registers
	REGISTERTYPE_ModbusMasterStatusBits = 8,
	/// \brief Modbus Master Virtual Input Registers
	REGISTERTYPE_ModbusMasterVirtualInputs = 9,
	/// \brief Modbus Master Virtual Output Registers
	REGISTERTYPE_ModbusMasterVirtualOutputs = 10,
	/// \brief Modbus Slave Input Registers
	REGISTERTYPE_ModbusSlaveInputWords = 11,
	/// \brief Modbus Slave Output Registers
	REGISTERTYPE_ModbusSlaveOutputWords = 12,
	/// \brief Modbus Slave Input Bit Registers
	REGISTERTYPE_ModbusSlaveInputBits = 13,
	/// \brief Modbus Slave Output Bit Registers
	REGISTERTYPE_ModbusSlaveOutputBits = 14,
}	REGISTERTYPE;

/// \brief The type of the component
typedef enum
{
	/// \brief Compact pulse width modulation
	COMPONENTTYPE_CP = 32773,
	/// \brief Micro pulse width modulation
	COMPONENTTYPE_MP = 32774,
	/// \brief Ensemble control board
	COMPONENTTYPE_Control = 32775,
	/// \brief Compact Linear
	COMPONENTTYPE_CL = 32776,
	/// \brief High performance pulse width modulation enhanced
	COMPONENTTYPE_HPE = 32777,
	/// \brief High performance linear enhanced
	COMPONENTTYPE_HLE = 32778,
	/// \brief
	COMPONENTTYPE_ML = 32779,
	/// \brief
	COMPONENTTYPE_PMT = 32780,
	/// \brief Ensemble Lab controller
	COMPONENTTYPE_Lab = 32781,
}	COMPONENTTYPE;

/// \brief The communication type used to communicate with a controller
typedef enum
{
	/// \brief Communications are over ethernet
	COMMUNICATIONTYPE_Ethernet = 2,
	/// \brief Communications are over USB
	COMMUNICATIONTYPE_Usb = 3,
}	COMMUNICATIONTYPE;

/// \brief Specifies the build result kind
typedef enum
{
	/// \brief The build result describes a warning that occurred during the compilation
	BUILDRESULTKIND_Warning = 0,
	/// \brief The build result describes an error that occurred during the compilation
	BUILDRESULTKIND_Error = 1,
}	BUILDRESULTKIND;

/// \brief Represents the ramp mode in AeroBasic
typedef enum
{
	/// \brief Distance-based acceleration and deceleration
	RAMPMODE_Dist = 0,
	/// \brief Rate-based acceleration and deceleration
	RAMPMODE_Rate = 1,
	/// \brief Time-based acceleration and deceleration
	RAMPMODE_Time = 2,
}	RAMPMODE;

#endif // __Ensemble_COMMON_STRUCTURES_H__
