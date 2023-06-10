/// \file EnsembleVariable.h
/// \brief Contains the functions to get and set Aerobasic variables.
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_VARIABLE_H__
#define __Ensemble_VARIABLE_H__

#include "EnsembleCommonTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \defgroup variable Variable Functions
/// @{

/// \brief Retrieves the value of a global double variable
///
/// This function will retrieve the value of one of the globally allocated
/// system doubles from the Aerobasic programs.  The
/// number of variables available is defined by the GlobalDoubles system
/// parameter.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] variableNumber The zero-based index of the variable to get.
/// \param[out] valueOut A pointer to the double to contain the retrieved value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleVariableGetGlobalDouble(EnsembleHandle handle, DWORD variableNumber, DOUBLE* valueOut);

/// \brief Retrieves an array of global double variables
///
/// This function will retrieve the value of a set of consecutive globally allocated
/// system doubles from the Aerobasic programs.  The
/// number of variables available is defined by the GlobalDoubles system
/// parameter.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] startVariableNumber The zero-based index of the first variable to get.
/// \param[out] valuesOutArray A pointer to an array of doubles to contain the retrieved values.
/// This must be of size count or greater.
/// \param[in] count The number of variables to retrieve. (startVariableNumber + count) cannot
/// exceed the defined number of GlobalDoubles available.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleVariableGetGlobalDoubles(EnsembleHandle handle, DWORD startVariableNumber, DOUBLE* valuesOutArray, DWORD count);

///////////////////////////////////////////////////////////////////////////

/// \brief Sets the value of a global double variable
///
/// This function will set the value of one of the globally allocated
/// system doubles in the Aerobasic programs.  The
/// number of variables available is defined by the GlobalDoubles system
/// parameter.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] variableNumber The zero-based index of the variable to set.
/// \param[in] value The double value to set.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleVariableSetGlobalDouble(EnsembleHandle handle, DWORD variableNumber, DOUBLE value);

/// \brief Sets an array of global double variables
///
/// This function will set the values of a set of consecutive globally allocated
/// system doubles in the Aerobasic programs.  The
/// number of variables available is defined by the GlobalDoubles system
/// parameter.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] startVariableNumber The zero-based index of the first variable to set.
/// \param[in] valuesArray A pointer to an array of doubles that contain the values to set.
/// This must be of size count or greater.
/// \param[in] count The number of variables to set. (startVariableNumber + count) cannot
/// exceed the defined number of GlobalDoubles available.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleVariableSetGlobalDoubles(EnsembleHandle handle, DWORD startVariableNumber, DOUBLE* valuesArray, DWORD count);

/// \brief Retrieves the value of a global integer variable
///
/// This function will retrieve the value of one of the globally allocated
/// system integers from the Aerobasic programs.  The
/// number of variables available is defined by the GlobalIntegers system
/// parameter.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] variableNumber The zero-based index of the variable to get.
/// \param[out] valueOut A pointer to the integer to contain the retrieved value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleVariableGetGlobalInteger(EnsembleHandle handle, DWORD variableNumber, INT* valueOut);

/// \brief Retrieves an array of global integer variables
///
/// This function will retrieve the value of a set of consecutive globally allocated
/// system integers from the Aerobasic programs.  The
/// number of variables available is defined by the GlobalIntegers system
/// parameter.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] startVariableNumber The zero-based index of the first variable to get.
/// \param[out] valuesOutArray A pointer to an array of integers to contain the retrieved values.
/// This must be of size count or greater.
/// \param[in] count The number of variables to retrieve. (startVariableNumber + count) cannot
/// exceed the defined number of GlobalIntegers available.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleVariableGetGlobalIntegers(EnsembleHandle handle, DWORD startVariableNumber, INT* valuesOutArray, DWORD count);

///////////////////////////////////////////////////////////////////////////

/// \brief Sets the value of a global integer variable
///
/// This function will set the value of one of the globally allocated
/// system integers in the Aerobasic programs.  The
/// number of variables available is defined by the GlobalIntegers system
/// parameter.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] variableNumber The zero-based index of the variable to set.
/// \param[in] value The integer value to set.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleVariableSetGlobalInteger(EnsembleHandle handle, DWORD variableNumber, INT value);

/// \brief Sets an array of global integer variables
///
/// This function will set the values of a set of consecutive globally allocated
/// system integers in the Aerobasic programs.  The
/// number of variables available is defined by the GlobalIntegers system
/// parameter.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] startVariableNumber The zero-based index of the first variable to set.
/// \param[in] valuesArray A pointer to an array of integers that contain the values to set.
/// This must be of size count or greater.
/// \param[in] count The number of variables to set. (startVariableNumber + count) cannot
/// exceed the defined number of GlobalIntegers available.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleVariableSetGlobalIntegers(EnsembleHandle handle, DWORD startVariableNumber, INT* valuesArray, DWORD count);

/// @}

#ifdef __cplusplus
}
#endif

#endif // __Ensemble_STATUS_H__
