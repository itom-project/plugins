/// \file EnsembleProgram.h
/// \brief Contains the functions to execute AeroBasic programs.
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_PROGRAM_H__
#define __Ensemble_PROGRAM_H__

#include "EnsembleCommonTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \defgroup programtask Program/Task Control Functions
/// @{

/// Runs an Aerobasic program.  The given Aerobasic file will be compiled,
/// sent to the drive, associated with the given task number, and then executed.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] taskId The Task ID
/// \param[in] filePath The path to the Aerobasic source file.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleProgramRun(EnsembleHandle handle, TASKID taskId, LPCSTR filePath);

/// Loads an Aerobasic program.  The given Aerobasic file will be compiled,
/// sent to the drive, and associated with the given task number.  The program is fully
/// loaded and ready to execute.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] taskId The Task ID
/// \param[in] filePath The path to the Aerobasic source file.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleProgramLoad(EnsembleHandle handle, TASKID taskId, LPCSTR filePath);

/// Start the execution of an Aerobasic program on a task.  The given task
/// is started, and the associated Aerobasic program is executed. An error
/// will occur if no program is associated with the task.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] taskId The Task ID
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleProgramStart(EnsembleHandle handle, TASKID taskId);

/// Stop the execution of an Aerobasic program on a task.  The given task
/// is stopped immediately, and all motion is aborted.  This function does
/// not block, so axis may still be aborting after it returns.  After the stop
/// completes, the program will be disassociated.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] taskId The Task ID
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleProgramStop(EnsembleHandle handle, TASKID taskId);

/// Pauses the execution of an Aerobasic program on a task.  The program on
/// the given task will complete the currently executing line, and pause
/// before starting the next line.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] taskId The Task ID
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleProgramPause(EnsembleHandle handle, TASKID taskId);

/// Steps into one line of an Aerobasic program on a task.  The program currently associated with
/// the given task will step into one line, and executes without regard to the
/// current execution mode.  If the program is currently executing, then this
/// command has no effect.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] taskId The Task ID
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleProgramStepInto(EnsembleHandle handle, TASKID taskId);

/// Steps over one line of an Aerobasic program on a task.  The program currently associated with
/// the given task will step over one line, and executes without regard to the
/// current execution mode.  If the program is currently executing, then this
/// command has no effect.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] taskId The Task ID
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleProgramStepOver(EnsembleHandle handle, TASKID taskId);

/// Steps out of a function that was stepped into in an Aerobasic program on a task. If the program currently
/// associated with the given task is within a function call, it will complete the execution of the function
/// up until after the function returns. Execution will then be paused on the line after the function call.
/// If already at the base level (main program function), execution will step over one line, just like  a call
/// to EnsembleProgramStepOver. If the program is currently executing, then this command has no effect.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] taskId The Task ID
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleProgramStepOut(EnsembleHandle handle, TASKID taskId);

/// Gets the state of the given task. This function retrieves the execution state
/// of the given task. Refer to the TASKSTATE defines in EnsembleCommonStructures.h.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] taskId The Task ID
/// \param[out] taskState The retrieved state of the task.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleProgramGetTaskState(EnsembleHandle handle, TASKID taskId, TASKSTATE* taskState);

/// @}

#ifdef __cplusplus
}
#endif

#endif // __Ensemble_PROGRAM_H__
