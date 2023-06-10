/// \file EnsembleInformation.h
/// \brief Contains the functions to get information about Ensembles.
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_INFORMATION_H__
#define __Ensemble_INFORMATION_H__

#include "EnsembleCommonTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \defgroup information Controller Information Functions
/// @{

/// The Version structure is used to hold information about the version of a particular component
typedef struct tagVersion {
	DWORD major; ///< The major version number
	DWORD minor; ///< The minor version number
	DWORD patch; ///< The patch number
	DWORD build; ///< The build number
	DWORD spare[4]; ///< Reserved for future expansion
} Version;

/// \brief Gets the name of a controller.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] size The size of the buffer for the name
/// \param[out] name The buffer for the name
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleInformationGetName(EnsembleHandle handle, ULONG size, LPSTR name);

/// \brief Gets the communication type of a controller.
///
/// \param[in] handle The handle to the Ensemble
/// \param[out] communicationType The communication type of the controller
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleInformationGetCommunicationType(EnsembleHandle handle, COMMUNICATIONTYPE* communicationType);

/// \brief Gets the available axes of a controller.
///
/// \param[in] handle The handle to the Ensemble
/// \param[out] axisMask Which axes exist on the controller
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleInformationGetAxisMask(EnsembleHandle handle, AXISMASK* axisMask);

/// \brief Retrieves the version information of the C library.
///
/// \param[out] pVersion A pointer to a Version struct to be populated with the version information of the C library.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleInformationGetLibraryVersion(Version* pVersion);

/// @}

#ifdef __cplusplus
}
#endif

#endif
