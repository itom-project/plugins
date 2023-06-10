/// \file EnsembleConnection.h
/// \brief Contains the functions to establish connections to Ensembles.
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_CONNECTION_H__
#define __Ensemble_CONNECTION_H__

#include "EnsembleCommonTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \defgroup connection Connection Functions
/// @{

/// \brief Connects to all Ensembles.
///
/// This function will connect to all the mapped Ensembles that are
/// available on the Network.
///
/// \param[out] handles The array of handles to the Ensembles
/// \param[out] handleCount The number of handles returned
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleConnect(EnsembleHandle** handles, DWORD* handleCount);

/// \brief Disconnects from all the Ensembles.
///
/// This function disconnects from all the Ensembles and frees resources
/// allocated for the different handles. This handle be the same handle that
/// was returned from EnsembleConnect.
///
/// \param[in] handles The handles to the Ensemble
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDisconnect(EnsembleHandle* handles);

/// \brief Resets the Ensemble.
///
/// This resets the Ensemble system.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] restartPrograms Set to TRUE to run program automation after reset.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleReset(EnsembleHandle handle, BOOL restartPrograms);

/// @}

#ifdef __cplusplus
}
#endif

#endif
