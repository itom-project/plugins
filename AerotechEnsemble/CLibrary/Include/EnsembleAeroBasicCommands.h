/// \file EnsembleAeroBasicCommands.h
/// \brief Contains immediate commands
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_AEROBASIC_COMMANDS_H__
#define __Ensemble_AEROBASIC_COMMANDS_H__

#include "EnsembleCommonTypes.h"
#include "EnsembleCommonStructures.h"
#ifdef __cplusplus
extern "C" {
#endif

/// \defgroup root Generic Commands

/// \defgroup motion Motion Commands

/// \defgroup motsetup Motion Setup Commands

/// \brief Specifies the ramp mode calculation type to use.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP MODE_Command.html">RAMP MODE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] Mode The ramp mode to use.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampModeAxis
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampModeAxis(EnsembleHandle handle, AXISMASK axisMask, RAMPMODE Mode);

/// \brief Specifies distance-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP DIST_Command.html">RAMP DIST</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] Value The acceleration and deceleration distance.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampDistAxis
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampDistAxis(EnsembleHandle handle, AXISMASK axisMask, DOUBLE* Value);

/// \brief Specifies rate-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP RATE_Command.html">RAMP RATE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] Value The acceleration and deceleration rate.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampRateAxis
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampRateAxis(EnsembleHandle handle, AXISMASK axisMask, DOUBLE* Value);

/// \brief Specifies time-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP TIME_Command.html">RAMP TIME</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] Value The acceleration and deceleration time.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampTimeAxis
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampTimeAxis(EnsembleHandle handle, AXISMASK axisMask, DOUBLE* Value);

/// \brief Reconciles the position of the axes in the list on the plane to servo position.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RECONCILE_Command.html">RECONCILE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupReconcile
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupReconcile(EnsembleHandle handle, AXISMASK axisMask);


/// \brief Disables the axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DISABLE_Command.html">DISABLE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionDisable
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionDisable(EnsembleHandle handle, AXISMASK axisMask);

/// \brief Enables the axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/ENABLE_Command.html">ENABLE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionEnable
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionEnable(EnsembleHandle handle, AXISMASK axisMask);

/// \brief Acknowledges and clears the fault on axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/FAULTACK_Command.html">FAULTACK</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionFaultAck
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionFaultAck(EnsembleHandle handle, AXISMASK axisMask);

/// \brief Freeruns the axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/FREERUN_Command.html">FREERUN</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] Speed The speed at which to run the axes.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionFreeRun
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionFreeRun(EnsembleHandle handle, AXISMASK axisMask, DOUBLE* Speed);

/// \brief Freeruns the axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/FREERUN_Command.html">FREERUN</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionFreeRunStop
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionFreeRunStop(EnsembleHandle handle, AXISMASK axisMask);

/// \brief Homes the axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/HOME_Command.html">HOME</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionHome
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionHome(EnsembleHandle handle, AXISMASK axisMask);

/// \brief Executes a linear move on axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/LINEAR_Command.html">LINEAR</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] Distance The distance to move an axis.
/// \param[in] CoordinatedSpeed The vectorial speed at which to move an axis.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionLinear
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionLinear(EnsembleHandle handle, AXISMASK axisMask, DOUBLE* Distance, DOUBLE CoordinatedSpeed);

/// \brief Executes an incremental move on axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/MOVEINC_Command.html">MOVEINC</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] Distance The distance to move an axis, in user units.
/// \param[in] Speed The speed at which to move an axis, in user units.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionMoveInc
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionMoveInc(EnsembleHandle handle, AXISMASK axisMask, DOUBLE* Distance, DOUBLE* Speed);

/// \brief Executes an absolute move on axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/MOVEABS_Command.html">MOVEABS</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] Distance The location to move an axis to, in user units.
/// \param[in] Speed The speed at which to move an axis, in user units.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionMoveAbs
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionMoveAbs(EnsembleHandle handle, AXISMASK axisMask, DOUBLE* Distance, DOUBLE* Speed);

/// \brief Sets motion blocking to On or OFF.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/BLOCKMOTION_Command.html">BLOCKMOTION</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] OnOff Sets motion blocking to On or OFF.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionBlockMotion
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionBlockMotion(EnsembleHandle handle, AXISMASK axisMask, ONOFF OnOff);

/// \brief Turns on or turns off autofocus.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/AUTOFOCUS_Command.html">AUTOFOCUS</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] OnOff Turns on or turns off autofocus.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionAutoFocus
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionAutoFocus(EnsembleHandle handle, AXISMASK axisMask, ONOFF OnOff);


/// \defgroup io IO Commands

/// \brief Controls the brake output of axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/BRAKE_Command.html">BRAKE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup io
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] axisMask The mask of axes to execute the command on
/// \param[in] OnOff Sets the brake to On or Off.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleIOBrake
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleIOBrake(EnsembleHandle handle, AXISMASK axisMask, ONOFF OnOff);



/// \defgroup root Generic Commands

/// \defgroup motion Motion Commands

/// \defgroup motsetup Motion Setup Commands

/// \brief Sets the current plane of motion.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PLANE_Command.html">PLANE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] PlaneNumber The plane to use.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupPlane
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupPlane(EnsembleHandle handle, DWORD PlaneNumber);

/// \brief Specifies the SCurve value to use.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/SCURVE_Command.html">SCURVE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Value The value by which to SCurve.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupScurve
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupScurve(EnsembleHandle handle, DOUBLE Value);

/// \brief Sets motion commands to be in absolute mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/ABS_Command.html">ABS</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupAbsolute
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupAbsolute(EnsembleHandle handle);

/// \brief Sets motion commands to be in incremental mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/INC_Command.html">INC</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupIncremental
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupIncremental(EnsembleHandle handle);

/// \brief Specifies the ramp mode calculation type to use.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP MODE_Command.html">RAMP MODE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Mode The ramp mode to use.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampModeCoordinated
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampModeCoordinated(EnsembleHandle handle, RAMPMODE Mode);

/// \brief Specifies distance-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP DIST_Command.html">RAMP DIST</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Value The acceleration and deceleration distance.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampDistCoordinated
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampDistCoordinated(EnsembleHandle handle, DOUBLE Value);

/// \brief Specifies distance-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP DIST_Command.html">RAMP DIST</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Value The acceleration and deceleration distance.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampDistCoordinatedAccel
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampDistCoordinatedAccel(EnsembleHandle handle, DOUBLE Value);

/// \brief Specifies distance-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP DIST_Command.html">RAMP DIST</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Value The acceleration and deceleration distance.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampDistCoordinatedDecel
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampDistCoordinatedDecel(EnsembleHandle handle, DOUBLE Value);

/// \brief Specifies rate-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP RATE_Command.html">RAMP RATE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Value The acceleration and deceleration rate.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampRateCoordinated
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampRateCoordinated(EnsembleHandle handle, DOUBLE Value);

/// \brief Specifies rate-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP RATE_Command.html">RAMP RATE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Value The acceleration and deceleration rate.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampRateCoordinatedAccel
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampRateCoordinatedAccel(EnsembleHandle handle, DOUBLE Value);

/// \brief Specifies rate-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP RATE_Command.html">RAMP RATE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Value The acceleration and deceleration rate.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampRateCoordinatedDecel
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampRateCoordinatedDecel(EnsembleHandle handle, DOUBLE Value);

/// \brief Specifies time-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP TIME_Command.html">RAMP TIME</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Value The acceleration and deceleration time.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampTimeCoordinated
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampTimeCoordinated(EnsembleHandle handle, DOUBLE Value);

/// \brief Specifies time-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP TIME_Command.html">RAMP TIME</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Value The acceleration and deceleration time.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampTimeCoordinatedAccel
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampTimeCoordinatedAccel(EnsembleHandle handle, DOUBLE Value);

/// \brief Specifies time-based acceleration and deceleration.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/RAMP TIME_Command.html">RAMP TIME</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Value The acceleration and deceleration time.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupRampTimeCoordinatedDecel
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupRampTimeCoordinatedDecel(EnsembleHandle handle, DOUBLE Value);

/// \brief Sets an arbitrary position value, in encoder counts, in external position register.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/SETEXTPOS_Command.html">SETEXTPOS</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Value The value to set in external position register.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupSetExtPos
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupSetExtPos(EnsembleHandle handle, AXISINDEX Axis, DOUBLE Value);

/// \brief Sets or clears an arbitrary program offset position.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/POSOFFSET_Command.html">POSOFFSET</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Value The value at which to set the parameter.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupPosOffsetSet
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupPosOffsetSet(EnsembleHandle handle, AXISINDEX Axis, DOUBLE Value);

/// \brief Sets or clears an arbitrary program offset position.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/POSOFFSET_Command.html">POSOFFSET</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupPosOffsetClear
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupPosOffsetClear(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Sets or clears the scale factor for an axis.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/SCALEFACTOR_Command.html">SCALEFACTOR</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Value The scale factor for the axis.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupScaleFactorSet
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupScaleFactorSet(EnsembleHandle handle, AXISINDEX Axis, DOUBLE Value);

/// \brief Sets or clears the scale factor for an axis.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/SCALEFACTOR_Command.html">SCALEFACTOR</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupScaleFactorClear
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupScaleFactorClear(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Configures the position data capture.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/POSCAP_Function.html">POSCAP</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[out] returnValue The captured position.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupPosCapStatus
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupPosCapStatus(EnsembleHandle handle, AXISINDEX Axis, DOUBLE* returnValue);

/// \brief Configures the position data capture.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/POSCAP_Function.html">POSCAP</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] SourceNumber The source of the position data to be captured.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupPosCapSetSource
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupPosCapSetSource(EnsembleHandle handle, AXISINDEX Axis, DWORD SourceNumber);

/// \brief Configures the position data capture.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/POSCAP_Function.html">POSCAP</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] TriggerNumber The user interrupt signal to use as the trigger for the position capture hardware.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupPosCapSetTrigger
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupPosCapSetTrigger(EnsembleHandle handle, AXISINDEX Axis, DWORD TriggerNumber);

/// \brief Configures the position data capture.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/POSCAP_Function.html">POSCAP</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupPosCapSetArm
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupPosCapSetArm(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Specifies the time scale to use.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/TIMESCALE_Command.html">TIMESCALE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motsetup
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Percentage The percentage timescaling, 1 - 200.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionSetupTimeScale
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionSetupTimeScale(EnsembleHandle handle, DOUBLE Percentage);


/// \brief Executes a clockwise circular move on axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/CW_Command.html">CW</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis1 The first axis on which to do clockwise circular motion.
/// \param[in] Axis1End The end point of the movement on the first specified axis.
/// \param[in] Axis2 The second axis on which to do clockwise circular motion.
/// \param[in] Axis2End The end point of the movement on the second specified axis.
/// \param[in] Radius The radius of the circle to use.
/// \param[in] CoordinatedSpeed The speed of the coordinated circular motion.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionCWRadius
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionCWRadius(EnsembleHandle handle, AXISINDEX Axis1, DOUBLE Axis1End, AXISINDEX Axis2, DOUBLE Axis2End, DOUBLE Radius, DOUBLE CoordinatedSpeed);

/// \brief Executes a clockwise circular move on axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/CW_Command.html">CW</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis1 The first axis on which to do clockwise circular motion.
/// \param[in] Axis1End The end point of the movement on the first specified axis.
/// \param[in] Axis2 The second axis on which to do clockwise circular motion.
/// \param[in] Axis2End The end point of the movement on the second specified axis.
/// \param[in] Axis1Center The relative offset of the center point from the starting position of the first axis.
/// \param[in] Axis2Center The relative offset of the center point from the starting position of the second axis.
/// \param[in] CoordinatedSpeed The speed of the coordinated circular motion.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionCWCenter
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionCWCenter(EnsembleHandle handle, AXISINDEX Axis1, DOUBLE Axis1End, AXISINDEX Axis2, DOUBLE Axis2End, DOUBLE Axis1Center, DOUBLE Axis2Center, DOUBLE CoordinatedSpeed);

/// \brief Executes a counterclockwise circular move on axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/CCW_Command.html">CCW</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis1 The first axis on which to do counterclockwise circular motion.
/// \param[in] Axis1End The end point of the movement on the first specified axis.
/// \param[in] Axis2 The second axis on which to do counterclockwise circular motion.
/// \param[in] Axis2End The end point of the movement on the second specified axis.
/// \param[in] Radius The radius of the circle to use.
/// \param[in] CoordinatedSpeed The speed of the coordinated circular motion.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionCCWRadius
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionCCWRadius(EnsembleHandle handle, AXISINDEX Axis1, DOUBLE Axis1End, AXISINDEX Axis2, DOUBLE Axis2End, DOUBLE Radius, DOUBLE CoordinatedSpeed);

/// \brief Executes a counterclockwise circular move on axes.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/CCW_Command.html">CCW</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis1 The first axis on which to do counterclockwise circular motion.
/// \param[in] Axis1End The end point of the movement on the first specified axis.
/// \param[in] Axis2 The second axis on which to do counterclockwise circular motion.
/// \param[in] Axis2End The end point of the movement on the second specified axis.
/// \param[in] Axis1Center The relative offset of the center point from the starting position of the first axis.
/// \param[in] Axis2Center The relative offset of the center point from the starting position of the second axis.
/// \param[in] CoordinatedSpeed The speed of the coordinated circular motion.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionCCWCenter
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionCCWCenter(EnsembleHandle handle, AXISINDEX Axis1, DOUBLE Axis1End, AXISINDEX Axis2, DOUBLE Axis2End, DOUBLE Axis1Center, DOUBLE Axis2Center, DOUBLE CoordinatedSpeed);

/// \brief Halts the vector motion queue and prevents motion from starting.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/HALT_Command.html">HALT</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionHalt
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionHalt(EnsembleHandle handle);

/// \brief Starts execution of the vector motion queue.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/START_Command.html">START</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionStart
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionStart(EnsembleHandle handle);

/// \brief Sets the mode of wait of a task.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/WAIT MODE_Command.html">WAIT MODE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup motion
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Type The wait mode type to set. Possible modes are NOWAIT, MOVEDONE, and INPOS.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleMotionWaitMode
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleMotionWaitMode(EnsembleHandle handle, WAITTYPE Type);


/// \defgroup reg Register Commands

/// \brief Locks a specified semaphore.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/LOCK_Command.html">LOCK</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup reg
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Semaphore The semaphore to lock.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleRegisterLock
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleRegisterLock(EnsembleHandle handle, SEMAPHORES Semaphore);

/// \brief Unlocks a specified semaphore.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/UNLOCK_Command.html">UNLOCK</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup reg
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] RegSet The semaphore to unlock.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleRegisterUnLock
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleRegisterUnLock(EnsembleHandle handle, SEMAPHORES RegSet);

/// \brief Provides access to the global integer variable (register) set.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/IGLOBAL_Function.html">IGLOBAL</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup reg
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] RegNumber The variable (register) number in the set.
/// \param[in] Value The value to store.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleRegisterIntegerGlobalWrite
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleRegisterIntegerGlobalWrite(EnsembleHandle handle, DWORD RegNumber, DWORD Value);

/// \brief Provides access to the global integer variable (register) set.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/IGLOBAL_Function.html">IGLOBAL</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup reg
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] RegNumber The variable (register) number in the set.
/// \param[out] returnValue The stored value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleRegisterIntegerGlobalRead
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleRegisterIntegerGlobalRead(EnsembleHandle handle, DWORD RegNumber, DWORD* returnValue);

/// \brief Provides access to the global double variable (register) set.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DGLOBAL_Function.html">DGLOBAL</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup reg
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] RegNumber The variable (register) number in the set.
/// \param[in] Value The value to store.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleRegisterDoubleGlobalWrite
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleRegisterDoubleGlobalWrite(EnsembleHandle handle, DWORD RegNumber, DOUBLE Value);

/// \brief Provides access to the global double variable (register) set.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DGLOBAL_Function.html">DGLOBAL</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup reg
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] RegNumber The variable (register) number in the set.
/// \param[out] returnValue The stored value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleRegisterDoubleGlobalRead
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleRegisterDoubleGlobalRead(EnsembleHandle handle, DWORD RegNumber, DOUBLE* returnValue);


/// \defgroup io IO Commands

/// \brief Reads the analog input value.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/AIN_Function.html">AIN</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup io
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Channel The analog channel to get the value of.
/// \param[out] returnValue The value of the analog input.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleIOAnalogInput
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleIOAnalogInput(EnsembleHandle handle, AXISINDEX Axis, DWORD Channel, DOUBLE* returnValue);

/// \brief Sets the value of the analog output.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/AOUT_Command.html">AOUT</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup io
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Channel The analog channel to set the value of.
/// \param[in] ChannelCount The number of elements in the Channel array
/// \param[in] Value The value of the analog output.
/// \param[in] ValueCount The number of elements in the Value array
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleIOAnalogOutput
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleIOAnalogOutput(EnsembleHandle handle, AXISINDEX Axis, DWORD* Channel, DWORD ChannelCount, DOUBLE* Value, DWORD ValueCount);

/// \brief Reads the digital input value.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DIN_Function.html">DIN</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup io
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Port The port from which to read the value.
/// \param[out] returnValue The value of the digital input.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleIODigitalInput
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleIODigitalInput(EnsembleHandle handle, AXISINDEX Axis, DWORD Port, DWORD* returnValue);

/// \brief Sets the digital output.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DOUT_Command.html">DOUT</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup io
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Port The port on which to set the value.
/// \param[in] Value The value to set the port to.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleIODigitalOutputEntire
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleIODigitalOutputEntire(EnsembleHandle handle, AXISINDEX Axis, DWORD Port, DWORD Value);

/// \brief Sets the digital output.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DOUT_Command.html">DOUT</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup io
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Port The port on which to set the value.
/// \param[in] Bits The bits to set.
/// \param[in] BitsCount The number of elements in the Bits array
/// \param[in] Values The values to set the bits to.
/// \param[in] ValuesCount The number of elements in the Values array
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleIODigitalOutputByBits
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleIODigitalOutputByBits(EnsembleHandle handle, AXISINDEX Axis, DWORD Port, DWORD* Bits, DWORD BitsCount, DWORD* Values, DWORD ValuesCount);


/// \defgroup status Status Commands

/// \brief Gets the position feedback latched when the marker signal occurred during a home.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PMRKLATCH_Function.html">PMRKLATCH</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup status
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[out] returnValue The latched marker position.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleStatusPositionMarkerLatched
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleStatusPositionMarkerLatched(EnsembleHandle handle, AXISINDEX Axis, DOUBLE* returnValue);

/// \brief Gets the Ethernet status.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/ETHERSTATUS_Function.html">ETHERSTATUS</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup status
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[out] returnValue The status of the Ethernet code.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleStatusEtherStatus
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleStatusEtherStatus(EnsembleHandle handle, ETHERNETSTATUS* returnValue);

/// \brief Gets the setting of one of the modal variables.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/GETMODE_Function.html">GETMODE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup status
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] ModeType The type of information that is requested.
/// \param[out] returnValue The requested value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleStatusGetMode
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleStatusGetMode(EnsembleHandle handle, MODETYPE ModeType, DOUBLE* returnValue);


/// \defgroup tuning Tuning Commands

/// \brief Sets all the servo control loop gains.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/SETGAIN_Command.html">SETGAIN</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup tuning
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] GainKp The GainKp value.
/// \param[in] GainKi The GainKi value.
/// \param[in] GainKpos The GainKpos value.
/// \param[in] GainAff The GainAff value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleTuningSetGain
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleTuningSetGain(EnsembleHandle handle, AXISINDEX Axis, DOUBLE GainKp, DOUBLE GainKi, DOUBLE GainKpos, DOUBLE GainAff);

/// \brief Sends a direct current command to the servo control loop.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/MCOMM_Command.html">MCOMM</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup tuning
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Current The current to output, in amperes.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleTuningMComm
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleTuningMComm(EnsembleHandle handle, AXISINDEX Axis, DOUBLE Current);

/// \brief Generates an open-loop current command.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/MSET_Command.html">MSET</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup tuning
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Current The current to output, in amperes.
/// \param[in] Angle The electrical angle.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleTuningMSet
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleTuningMSet(EnsembleHandle handle, AXISINDEX Axis, DOUBLE Current, DOUBLE Angle);

/// \brief Generates sinusoidal oscillation on an axis.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/OSCILLATE_Command.html">OSCILLATE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup tuning
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Distance The distance to move the axis.
/// \param[in] Frequency The frequency at which to move the axis.
/// \param[in] Cycles The number of cycles to complete.
/// \param[in] NumFreqs The number of frequencies to execute (multiples of ).
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleTuningOscillate
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleTuningOscillate(EnsembleHandle handle, AXISINDEX Axis, DOUBLE Distance, DOUBLE Frequency, DWORD Cycles, DWORD NumFreqs);

/// \brief Initiates loop transmission mode.

///
/// \ingroup tuning
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Mode Sets the loop transmission mode to On or Off.
/// \param[in] Amplitude Sets the maximum loop output as a percentage.
/// \param[in] Frequency The frequency of the disturbance generated, in Hertz.
/// \param[in] Type The type of loop transmission to run.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleTuningLoopTrans
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleTuningLoopTrans(EnsembleHandle handle, AXISINDEX Axis, LOOPTRANSMISSIONMODE Mode, DOUBLE Amplitude, DOUBLE Frequency, LOOPTRANSMISSIONTYPE Type);


/// \defgroup dataacq DataAcquisition Commands

/// \brief Enables data collection.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DATAACQ ON_Command.html">DATAACQ ON</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup dataacq
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] NumberOfSamples The number of samples to be collected. After the specified number of samples are collected, data acquisition turns off automatically. On multi-axis drives, multiple samples can be collected at each trigger event, so this argument must specify the number of data samples to store, not the number of trigger events.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleDataAcquisitionArraySetup
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleDataAcquisitionArraySetup(EnsembleHandle handle, AXISINDEX Axis, DWORD NumberOfSamples);

/// \brief Transfers drive array values into the specified controller array variables.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DATAACQ READ_Command.html">DATAACQ READ</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup dataacq
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] VariableStart The starting location of IGLOBAL where captured data will be read into.
/// \param[in] NumberOfSamples The number of samples to be read back.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleDataAcquisitionArrayRead
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleDataAcquisitionArrayRead(EnsembleHandle handle, AXISINDEX Axis, DWORD VariableStart, DWORD NumberOfSamples);

/// \brief Specifies the data element collected when a trigger occurs.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DATAACQ INPUT_Command.html">DATAACQ INPUT</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup dataacq
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] SourceSignal The source signal to be collected.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleDataAcquisitionInput
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleDataAcquisitionInput(EnsembleHandle handle, AXISINDEX Axis, DWORD SourceSignal);

/// \brief Specifies which signal will be monitored to collect data.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DATAACQ TRIGGER_Command.html">DATAACQ TRIGGER</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup dataacq
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] TriggerSignal The signal to be triggered on.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleDataAcquisitionTrigger
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleDataAcquisitionTrigger(EnsembleHandle handle, AXISINDEX Axis, DWORD TriggerSignal);

/// \brief Turns off data acquisition. All previously specified DATAACQ command configurations are cleared and must be re-specified if required.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/DATAACQ OFF_Command.html">DATAACQ OFF</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup dataacq
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleDataAcquisitionOff
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleDataAcquisitionOff(EnsembleHandle handle, AXISINDEX Axis);


/// \defgroup pso PSO Commands

/// \brief Gets the PSO status information.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOSTATUS_Function.html">PSOSTATUS</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[out] returnValue The PSO status bits.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOStatus
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOStatus(EnsembleHandle handle, AXISINDEX Axis, DWORD* returnValue);

/// \brief Sends array mode distances into the PSO array.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOARRAY_Command.html">PSOARRAY</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] StartIndex The index on which to start.
/// \param[in] NumberOfPoints The number of points to send.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOArrayDistance
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOArrayDistance(EnsembleHandle handle, AXISINDEX Axis, DWORD StartIndex, DWORD NumberOfPoints);

/// \brief Sends array mode distances into the PSO array.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOARRAY_Command.html">PSOARRAY</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] StartIndex The index on which to start.
/// \param[in] NumberOfPoints The number of points to send.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOArrayLaser
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOArrayLaser(EnsembleHandle handle, AXISINDEX Axis, DWORD StartIndex, DWORD NumberOfPoints);

/// \brief Sends array mode distances into the PSO array.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOARRAY_Command.html">PSOARRAY</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] StartIndex The index on which to start.
/// \param[in] NumberOfPoints The number of points to send.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOArrayWindow1
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOArrayWindow1(EnsembleHandle handle, AXISINDEX Axis, DWORD StartIndex, DWORD NumberOfPoints);

/// \brief Sends array mode distances into the PSO array.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOARRAY_Command.html">PSOARRAY</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] StartIndex The index on which to start.
/// \param[in] NumberOfPoints The number of points to send.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOArrayWindow2
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOArrayWindow2(EnsembleHandle handle, AXISINDEX Axis, DWORD StartIndex, DWORD NumberOfPoints);

/// \brief Enables and disables the PSO hardware.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOCONTROL_Command.html">PSOCONTROL</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Mode The mode of operation of the PSO hardware.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOControl
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOControl(EnsembleHandle handle, AXISINDEX Axis, PSOMODE Mode);

/// \brief Sets the distance to travel between firing events.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSODISTANCE_Command.html">PSODISTANCE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSODistanceArray
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSODistanceArray(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Sets the distance to travel between firing events.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSODISTANCE_Command.html">PSODISTANCE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] FireDistance The distance an axis must travel before a firing event triggers the pulse generator.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSODistanceFixed
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSODistanceFixed(EnsembleHandle handle, AXISINDEX Axis, DOUBLE FireDistance);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT PULSE_Command.html">PSOOUTPUT PULSE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputPulse
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputPulse(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT PULSE BIT MASK_Command.html">PSOOUTPUT PULSE BIT MASK</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputPulseBitMask
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputPulseBitMask(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT TOGGLE_Command.html">PSOOUTPUT TOGGLE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputToggle
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputToggle(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT WINDOW_Command.html">PSOOUTPUT WINDOW</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputWindow
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputWindow(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT PULSE WINDOW MASK_Command.html">PSOOUTPUT PULSE WINDOW MASK</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputPulseWindowMaskHard
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputPulseWindowMaskHard(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT PULSE WINDOW MASK_Command.html">PSOOUTPUT PULSE WINDOW MASK</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] EdgeMode Specifies the pulse output behavior. This argument is required if the EDGE keyword is used.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputPulseWindowMaskEdgeMode
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputPulseWindowMaskEdgeMode(EnsembleHandle handle, AXISINDEX Axis, DWORD EdgeMode);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT PULSE WINDOW BIT MASK_Command.html">PSOOUTPUT PULSE WINDOW BIT MASK</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] EdgeMode Specifies the pulse output behavior. This argument is required if the EDGE keyword is used.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputPulseWindowBitMask
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputPulseWindowBitMask(EnsembleHandle handle, AXISINDEX Axis, DWORD EdgeMode);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT CONTROL_Command.html">PSOOUTPUT CONTROL</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Mode Used on all drives to select the mode of PSO output.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputControl
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputControl(EnsembleHandle handle, AXISINDEX Axis, DWORD Mode);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT BIT MAP_Command.html">PSOOUTPUT BIT MAP</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputBitMap
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputBitMap(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT BIT MAP_Command.html">PSOOUTPUT BIT MAP</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Mode Specifies the mode of operation for the BIT MAP functionality. If this argument is omitted or set to 0, only the most-significant bit (bit 31) of each array value is used. If set to 1, all 32 bits of each array value is used.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputBitMapMode
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputBitMapMode(EnsembleHandle handle, AXISINDEX Axis, DWORD Mode);

/// \brief Sets the PSO output mode.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOOUTPUT PULSE EXTSYNC_Command.html">PSOOUTPUT PULSE EXTSYNC</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOOutputPulseExtSync
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOOutputPulseExtSync(EnsembleHandle handle, AXISINDEX Axis);

/// \brief Configures the pulse sequence used for PSO.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOPULSE_Command.html">PSOPULSE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] TotalTime The total time of generated pulse in microseconds.
/// \param[in] OnTime The time of the cycle when the PSO output is in the ON state.
/// \param[in] NumCycles The number of pulses to generate in a single pulse event.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOPulseCyclesOnly
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOPulseCyclesOnly(EnsembleHandle handle, AXISINDEX Axis, DOUBLE TotalTime, DOUBLE OnTime, DOUBLE NumCycles);

/// \brief Configures the pulse sequence used for PSO.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOPULSE_Command.html">PSOPULSE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] TotalTime The total time of generated pulse in microseconds.
/// \param[in] OnTime The time of the cycle when the PSO output is in the ON state.
/// \param[in] DelayTime The quantity of time to delay between a fire event and the laser output.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOPulseDelayOnly
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOPulseDelayOnly(EnsembleHandle handle, AXISINDEX Axis, DOUBLE TotalTime, DOUBLE OnTime, DOUBLE DelayTime);

/// \brief Configures the pulse sequence used for PSO.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOPULSE_Command.html">PSOPULSE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] TotalTime The total time of generated pulse in microseconds.
/// \param[in] OnTime The time of the cycle when the PSO output is in the ON state.
/// \param[in] NumCycles The number of pulses to generate in a single pulse event.
/// \param[in] DelayTime The quantity of time to delay between a fire event and the laser output.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOPulseCyclesAndDelay
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOPulseCyclesAndDelay(EnsembleHandle handle, AXISINDEX Axis, DOUBLE TotalTime, DOUBLE OnTime, DOUBLE NumCycles, DOUBLE DelayTime);

/// \brief Configures the PSO distance tracking counters.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOTRACK INPUT_Command.html">PSOTRACK INPUT</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Source1 The encoder to use as the source for the first axis of tracking (use for single-axis).
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOTrackInput
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOTrackInput(EnsembleHandle handle, AXISINDEX Axis, PSOENCODER Source1);

/// \brief Configures the PSO distance tracking counters.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOTRACK INPUT_Command.html">PSOTRACK INPUT</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Source1 The encoder to use as the source for the first axis of tracking (use for single-axis).
/// \param[in] Source2 Ensemble HLe and HPe only. The encoder to use as the source for the second axis of tracking (DO NOT use for single-axis).
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOTrackInputInput2
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOTrackInputInput2(EnsembleHandle handle, AXISINDEX Axis, PSOENCODER Source1, PSOENCODER Source2);

/// \brief Configures the PSO distance tracking counters.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOTRACK INPUT_Command.html">PSOTRACK INPUT</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] Source1 The encoder to use as the source for the first axis of tracking (use for single-axis).
/// \param[in] Source2 Ensemble HLe and HPe only. The encoder to use as the source for the second axis of tracking (DO NOT use for single-axis).
/// \param[in] Source3 Ensemble HLe and HPe only. The encoder to use as the source for the third axis of tracking (DO NOT use for single-axis).
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOTrackInputInput2Input3
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOTrackInputInput2Input3(EnsembleHandle handle, AXISINDEX Axis, PSOENCODER Source1, PSOENCODER Source2, PSOENCODER Source3);

/// \brief Configures the PSO distance tracking counters.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOTRACK RESET_Command.html">PSOTRACK RESET</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] RBitMask The mask of possible conditions that can hold the tracking counter in reset.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOTrackReset
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOTrackReset(EnsembleHandle handle, AXISINDEX Axis, DWORD RBitMask);

/// \brief Configures the PSO distance tracking counters.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOTRACK SCALE_Command.html">PSOTRACK SCALE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] PreScale1 The divisor applied to the encoder input to the first PSO channel.
/// \param[in] PreScale2 The divisor applied to the encoder input to the second PSO channel.
/// \param[in] PreScale3 The divisor applied to the encoder input to the third PSO channel.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOTrackScale
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOTrackScale(EnsembleHandle handle, AXISINDEX Axis, DWORD PreScale1, DWORD PreScale2, DWORD PreScale3);

/// \brief Configures the PSO distance tracking counters.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOTRACK DIRECTION_Command.html">PSOTRACK DIRECTION</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] DBitMask The mask of possible directions to be suppressed when tracking position.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOTrackDirection
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOTrackDirection(EnsembleHandle handle, AXISINDEX Axis, DWORD DBitMask);

/// \brief Enables the PSO Window Hardware.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOWINDOW ON_Command.html">PSOWINDOW ON</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] WindowNumber The window to configure. The Ensemble CL, CP, MP, Epaq MR with MP controllers, and Lab must be set to 1. All other controllers can be set to 1 or 2.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOWindowOn
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOWindowOn(EnsembleHandle handle, AXISINDEX Axis, DWORD WindowNumber);

/// \brief Enables the PSO Window Hardware.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOWINDOW ON_Command.html">PSOWINDOW ON</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] WindowNumber The window to configure. The Ensemble CL, CP, MP, Epaq MR with MP controllers, and Lab must be set to 1. All other controllers can be set to 1 or 2.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOWindowOnInvert
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOWindowOnInvert(EnsembleHandle handle, AXISINDEX Axis, DWORD WindowNumber);

/// \brief Disables the PSO Window Hardware.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOWINDOW OFF_Command.html">PSOWINDOW OFF</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] WindowNumber The window to configure. The Ensemble CL, CP, MP, Epaq MR with MP controllers, and Lab must be set to 1. All other controllers can be set to 1 or 2.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOWindowOff
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOWindowOff(EnsembleHandle handle, AXISINDEX Axis, DWORD WindowNumber);

/// \brief Configures which encoder channel is connected to each window.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOWINDOW INPUT_Command.html">PSOWINDOW INPUT</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] WindowNumber The window to configure. The Ensemble CL, CP, MP, Epaq MR with MP controllers, and Lab must be set to 1. All other controllers can be set to 1 or 2.
/// \param[in] Source The encoder source to use.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOWindowInput
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOWindowInput(EnsembleHandle handle, AXISINDEX Axis, DWORD WindowNumber, PSOENCODER Source);

/// \brief Configures which encoder channel is connected to each window.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOWINDOW INPUT_Command.html">PSOWINDOW INPUT</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] WindowNumber The window to configure. The Ensemble CL, CP, MP, Epaq MR with MP controllers, and Lab must be set to 1. All other controllers can be set to 1 or 2.
/// \param[in] Source The encoder source to use.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOWindowInputInvert
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOWindowInputInvert(EnsembleHandle handle, AXISINDEX Axis, DWORD WindowNumber, PSOENCODER Source);

/// \brief Resets the window counter to 0 based on the encoder marker signal.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOWINDOW RESET_Command.html">PSOWINDOW RESET</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] WindowNumber The window to configure. The Ensemble CL, CP, MP, Epaq MR with MP controllers, and Lab must be set to 1. All other controllers can be set to 1 or 2.
/// \param[in] BitMask The mask of possible conditions that hold the tracking counter in reset.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOWindowReset
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOWindowReset(EnsembleHandle handle, AXISINDEX Axis, DWORD WindowNumber, DWORD BitMask);

/// \brief Loads the specified window counter with a value.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOWINDOW LOAD_Command.html">PSOWINDOW LOAD</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] WindowNumber The window to configure. The Ensemble CL, CP, MP, Epaq MR with MP controllers, and Lab must be set to 1. All other controllers can be set to 1 or 2.
/// \param[in] Value The value to load into the specified window.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOWindowLoad
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOWindowLoad(EnsembleHandle handle, AXISINDEX Axis, DWORD WindowNumber, DWORD Value);

/// \brief Specifies the array mode parameters for the specified PSO window.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOWINDOW RANGE ARRAY_Command.html">PSOWINDOW RANGE ARRAY</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] WindowNumber The window to configure. Set to 1 or 2.
/// \param[in] EdgeCode An optional argument that restricts updating to one encoder direction.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOWindowRangeArray
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOWindowRangeArray(EnsembleHandle handle, AXISINDEX Axis, DWORD WindowNumber, DOUBLE EdgeCode);

/// \brief Specifies the low and high comparison values for specified PSO window.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/PSOWINDOW RANGE_Command.html">PSOWINDOW RANGE</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup pso
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \param[in] Axis The axis on which to execute the command
/// \param[in] WindowNumber The window to configure. The Ensemble CL, CP, MP, Epaq MR with MP controllers, and Lab must be set to 1. All other controllers can be set to 1 or 2.
/// \param[in] LowValue The low position range for fixed window.
/// \param[in] HighValue The high position range for fixed window.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsemblePSOWindowRange
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsemblePSOWindowRange(EnsembleHandle handle, AXISINDEX Axis, DWORD WindowNumber, DOUBLE LowValue, DOUBLE HighValue);


/// \brief Acknowledges all axis faults and clears all task errors.

/// \htmlonly This command is the same as "<a href="mk:@MSITStore:Ensemble.chm::/Commands/ACKNOWLEDGEALL_Command.html">ACKNOWLEDGEALL</a>" in AeroBasic.
/// \endhtmlonly
///
/// \ingroup root
///
/// \param[in] handle The handle to the controller on which to execute the command.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
///
/// Example usage:
///
/// \dontinclude "CommandsDoc.h"
/// \skip EnsembleAcknowledgeAll
/// \until EnsembleDisconnect
BOOL DLLENTRYDECLARATION EnsembleAcknowledgeAll(EnsembleHandle handle);



#ifdef __cplusplus
}
#endif

#endif // __Ensemble_AEROBASIC_COMMANDS_H__
