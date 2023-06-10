/// \file EnsembleDataCollection.h
/// \brief Contains the functions to handle data collection.
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_DATACOLLECTION_H__
#define __Ensemble_DATACOLLECTION_H__

#include "EnsembleCommonTypes.h"
#include "EnsembleDataCollectionInfo.h"

#ifdef __cplusplus
extern "C" {
#endif

/// \defgroup datacollection Data Collection Functions
/// @{

typedef struct {
		DATACOLLECTIONFLAGS     dataCollectionStatus;   ///< The status code, defined in EnsembleCommonStructures.h
		DWORD                   samplePointsAllocated;  ///< The number of sample points per signal requested
		INT                     samplePointsCollected;  ///< The number of sample points per signal collected
		DWORD                   spare[5];               ///< Reserved for future expansion
} EnsembleDataCollectionStatus;

/// \brief Creates a new data collection configuration.
///
/// This function creates a new data collection configuration object.
/// Initial collection period is 1 msec.
///
/// \param[in] handle The handle to the Ensemble
/// \param[out] DCCHandle The handle to an Ensemble Data Collection Configuration object
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionConfigCreate(EnsembleHandle handle, EnsembleDataCollectConfigHandle *DCCHandle);

/// \brief Frees the data collection configuration.
///
/// This function frees memory of a data collection configuration object.
///
/// \param[in] DCCHandle The handle to an Ensemble Data Collection Configuration object
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionConfigFree(EnsembleDataCollectConfigHandle DCCHandle);

/// \brief Sets the number of sample points to collect per signal.
///
/// This function sets number of points per signal to collect.
///
/// \param[in] DCCHandle The handle to an Ensemble Data Collection Configuration object
/// \param[in] numberOfPoints The number of sample points per signal to collect
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionConfigSetSamples(EnsembleDataCollectConfigHandle DCCHandle, DWORD numberOfPoints);

/// \brief Sets how frequently a sample point will be collect on each signal.
///
/// This function sets the time period between two consecutive sample points.
///
/// \param[in] DCCHandle The handle to an Ensemble Data Collection Configuration object
/// \param[in] collectionPeriod The time period between two samples in msec
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionConfigSetPeriod(EnsembleDataCollectConfigHandle DCCHandle, DOUBLE collectionPeriod);

/// \brief Adds a signal at the end of the configuration.
///
/// This function adds a signal to the end in the data collection configuration.
///
/// \param[in] DCCHandle The handle to an Ensemble Data Collection Configuration object.
/// \param[in] signal The name of the signal. Defined in EnsembleDataCollectionInfo.h as DATASIGNAL_xxxx
/// \param[in] itemIndex The index of axis, or task on which the signal is to be collected.  Set to 0 if unused.
/// \param[in] additionalInformation Other argument for the signal.  Set to 0 if unused.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionConfigAddSignal(EnsembleDataCollectConfigHandle DCCHandle, DATASIGNAL signal, WORD itemIndex, DWORD additionalInformation);

/// \brief Retrieves the number of signals added to a data collection configuration.
///
/// This function retrieves the number of signals added to a data collection configuration.
///
/// \param[in] DCCHandle The handle to an Ensemble Data Collection Configuration object.
/// \param[out] signalCount The number of signals currently added to the data collection configuration.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionConfigGetSignalCount(EnsembleDataCollectConfigHandle DCCHandle, WORD* signalCount);

/// \brief Removes all signals from configuration.
///
/// This function will remove all existing signals in the data collection configuration.
/// The configuration will become invalid.
///
/// \param[in] DCCHandle The handle to an Ensemble Data Collection Configuration object.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionConfigRemoveSignalAll(EnsembleDataCollectConfigHandle DCCHandle);

/// \brief Applies data collection configuration to the Ensemble.
///
/// This function will send a data collection configuration to the Ensemble.
///
/// \param[in] handle The handle to the Ensemble
/// \param[in] DCCHandle The handle to an Ensemble Data Collection Configuration object.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionConfigApply(EnsembleHandle handle, EnsembleDataCollectConfigHandle DCCHandle);

/// \brief Starts data collection immediately.
///
/// This function will send a data collection configuration to the Ensemble and start data collection immediately.
///
/// \param[in] handle The handle to the Ensemble.
/// \param[in] DCCHandle The handle to an Ensemble Data Collection Configuration object.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionStart(EnsembleHandle handle, EnsembleDataCollectConfigHandle DCCHandle);

/// \brief Aborts data collection.
///
/// This function will abort data collection.
///
/// \param[in] handle The handle to the Ensemble
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionStop(EnsembleHandle handle);

/// \brief Retrieves data collection status.
///
/// This function will retrieve data collection status.
///
/// \param[in] handle The handle to the Ensemble
/// \param[out] status The status code, point allocated and collected
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionGetStatus(EnsembleHandle handle, EnsembleDataCollectionStatus *status);

/// \brief Retrieves all sample points from the Ensemble.
///
/// This function will retrieve all the sample points from the Ensemble.
/// Caller must declare data[numberOfSignals][numberOfSamples] to match the number of signals and samples in the configuration
/// Use (DOUBLE *)data or &data[0][0] when passing data to this function.
///
/// \param[in] handle The handle to the Ensemble.
/// \param[in] DCCHandle The handle to an Ensemble Data Collection Configuration object.
/// \param[in] signalCount The number of signals currently in the data collection configuration. Must match what is in the configuration.
/// \param[in] sampleCount The number of sample points per signal to retrieve.
/// \param[out] data The retrieved sample point in format data[signal][sample].
/// \param[out] collectionPeriod The collection period that the data was actually collected at.  If NULL is passed, not returned.  In milliseconds.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleDataCollectionDataRetrieve(EnsembleHandle handle, EnsembleDataCollectConfigHandle DCCHandle, WORD signalCount, DWORD sampleCount, DOUBLE *data, DOUBLE* collectionPeriod);

/// @}

#ifdef __cplusplus
}
#endif

#endif
