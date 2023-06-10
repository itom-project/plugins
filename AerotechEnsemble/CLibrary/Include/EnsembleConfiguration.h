/// \file EnsembleConfiguration.h
/// \brief Contains the function to configure the Ensemble.
///
/// Copyright (c) Aerotech, Inc. 2013-2013.
///

#ifndef __Ensemble_CONFIGURATION_H__
#define __Ensemble_CONFIGURATION_H__

#include "EnsembleCommonTypes.h"
#include "EnsembleCommonStructures.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \defgroup configuration Configuration Functions
/// @{

/// \brief The handle to the Ensemble configuration.
typedef void* EnsembleConfigurationHandle;

/// \brief Represents a profile entry in the map file.
typedef struct {
	char controllerName[32]; ///< The controller name.
	COMMUNICATIONTYPE communicationType; ///< The communication type.
	DWORD spare[16]; ///< Reserved for future expansion
} EnsembleProfileEntry;

/// \brief Opens the current configuration.
///
/// This function opens the current configuration file and stores it in memory.  The object can then be
/// used to configure different Ensemble settings.
/// \param[out] handle A pointer to a handle for the Ensemble Configuration object.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleConfigurationOpen(EnsembleConfigurationHandle* handle);

/// \brief Saves the configuration.
///
/// \param[in] handle A pointer to the Ensemble Configuration object to save.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleConfigurationSave(EnsembleConfigurationHandle handle);

/// \brief Closes a configuration, releasing any memory.
///
/// This function closes and cleans up the Ensemble Configuration object.  To save the configuration, call
/// EnsembleConfigurationSave().
/// \param[in] handle A pointer to the Ensemble Configuration object to close.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleConfigurationClose(EnsembleConfigurationHandle handle);

/// \brief Gets the count of entries in the network setup.
///
/// \param[in] handle A pointer to the Ensemble Configuration object to use.
/// \param[out] count The number of entries in the network setup.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleConfigurationNetworkSetupGetCount(EnsembleConfigurationHandle handle, DWORD* count);

/// \brief Sets an entry in the network setup.
///
/// \param[in] handle A pointer to the Ensemble Configuration object to use.
/// \param[in] index The index of the entry to set.
/// \param[in] entry The information to set.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleConfigurationNetworkSetupSet(EnsembleConfigurationHandle handle, DWORD index, EnsembleProfileEntry entry);

/// \brief Gets an entry in the network setup.
///
/// \param[in] handle A pointer to the Ensemble Configuration object to use.
/// \param[in] index The index of the entry to get.
/// \param[out] entry The entry information.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleConfigurationNetworkSetupGet(EnsembleConfigurationHandle handle, DWORD index, EnsembleProfileEntry* entry);

/// \brief Adds an entry to the network setup.
///
/// \param[in] handle A pointer to the Ensemble Configuration object to use.
/// \param[in] entry The entry information.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleConfigurationNetworkSetupAdd(EnsembleConfigurationHandle handle, EnsembleProfileEntry entry);

/// \brief Removes an entry from the network setup.
///
/// \param[in] handle A pointer to the Ensemble Configuration object to use.
/// \param[in] index The index of the entry to remove.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleConfigurationNetworkSetupRemove(EnsembleConfigurationHandle handle, DWORD index);

/// @}

#ifdef __cplusplus
}
#endif

#endif
