/// @file EnsembleEnumNames.h
/// \brief Contains functions to retrieve the names of various enumeration values.
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_ENUM_NAMES__
#define __Ensemble_ENUM_NAMES__

#include "EnsembleCommonTypes.h"
#include "EnsembleCommonStructures.h"
/// @defgroup enumname Enumeration Name Functions

#ifdef __cplusplus
extern "C" {
#endif
/// \brief Retrieves the name of a SERVORATEPARAMETER value.
///
/// @ingroup enumname
///
/// @param[in] value The SERVORATEPARAMETER enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumServoRateParameterGetValueName(SERVORATEPARAMETER value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a TASKSTATE value.
///
/// @ingroup enumname
///
/// @param[in] value The TASKSTATE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumTaskStateGetValueName(TASKSTATE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a PLANESTATUS bit value.
///
/// @ingroup enumname
///
/// @param[in] bitValue The PLANESTATUS enumeration bit value for which to retrieve the string name. Note that this bit value must contain only one bit that is high.
/// @param[in] bitValueNameBuffer The output buffer that will contain the string name of the enumeration bit value.
/// @param[in] bitValueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration bit value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumPlaneStatusGetBitName(PLANESTATUS bitValue, LPSTR bitValueNameBuffer, DWORD bitValueNameBufferSize);

/// \brief Retrieves the name of a DEBUGFLAGS bit value.
///
/// @ingroup enumname
///
/// @param[in] bitValue The DEBUGFLAGS enumeration bit value for which to retrieve the string name. Note that this bit value must contain only one bit that is high.
/// @param[in] bitValueNameBuffer The output buffer that will contain the string name of the enumeration bit value.
/// @param[in] bitValueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration bit value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumDebugFlagsGetBitName(DEBUGFLAGS bitValue, LPSTR bitValueNameBuffer, DWORD bitValueNameBufferSize);

/// \brief Retrieves the name of a AXISSTATUS bit value.
///
/// @ingroup enumname
///
/// @param[in] bitValue The AXISSTATUS enumeration bit value for which to retrieve the string name. Note that this bit value must contain only one bit that is high.
/// @param[in] bitValueNameBuffer The output buffer that will contain the string name of the enumeration bit value.
/// @param[in] bitValueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration bit value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumAxisStatusGetBitName(AXISSTATUS bitValue, LPSTR bitValueNameBuffer, DWORD bitValueNameBufferSize);

/// \brief Retrieves the name of a AXISFAULT bit value.
///
/// @ingroup enumname
///
/// @param[in] bitValue The AXISFAULT enumeration bit value for which to retrieve the string name. Note that this bit value must contain only one bit that is high.
/// @param[in] bitValueNameBuffer The output buffer that will contain the string name of the enumeration bit value.
/// @param[in] bitValueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration bit value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumAxisFaultGetBitName(AXISFAULT bitValue, LPSTR bitValueNameBuffer, DWORD bitValueNameBufferSize);

/// \brief Retrieves the name of a DATACOLLECTIONFLAGS bit value.
///
/// @ingroup enumname
///
/// @param[in] bitValue The DATACOLLECTIONFLAGS enumeration bit value for which to retrieve the string name. Note that this bit value must contain only one bit that is high.
/// @param[in] bitValueNameBuffer The output buffer that will contain the string name of the enumeration bit value.
/// @param[in] bitValueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration bit value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumDataCollectionFlagsGetBitName(DATACOLLECTIONFLAGS bitValue, LPSTR bitValueNameBuffer, DWORD bitValueNameBufferSize);

/// \brief Retrieves the name of a STATUSITEM value.
///
/// @ingroup enumname
///
/// @param[in] value The STATUSITEM enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumStatusItemGetValueName(STATUSITEM value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a LOOPTRANSMISSIONMODE value.
///
/// @ingroup enumname
///
/// @param[in] value The LOOPTRANSMISSIONMODE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumLoopTransmissionModeGetValueName(LOOPTRANSMISSIONMODE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a LOOPTRANSMISSIONTYPE value.
///
/// @ingroup enumname
///
/// @param[in] value The LOOPTRANSMISSIONTYPE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumLoopTransmissionTypeGetValueName(LOOPTRANSMISSIONTYPE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a ONOFF value.
///
/// @ingroup enumname
///
/// @param[in] value The ONOFF enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumOnOffGetValueName(ONOFF value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a PSOENCODER value.
///
/// @ingroup enumname
///
/// @param[in] value The PSOENCODER enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumPsoEncoderGetValueName(PSOENCODER value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a PSOMODE value.
///
/// @ingroup enumname
///
/// @param[in] value The PSOMODE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumPsoModeGetValueName(PSOMODE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a ETHERNETSTATUS bit value.
///
/// @ingroup enumname
///
/// @param[in] bitValue The ETHERNETSTATUS enumeration bit value for which to retrieve the string name. Note that this bit value must contain only one bit that is high.
/// @param[in] bitValueNameBuffer The output buffer that will contain the string name of the enumeration bit value.
/// @param[in] bitValueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration bit value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumEthernetStatusGetBitName(ETHERNETSTATUS bitValue, LPSTR bitValueNameBuffer, DWORD bitValueNameBufferSize);

/// \brief Retrieves the name of a SEMAPHORES value.
///
/// @ingroup enumname
///
/// @param[in] value The SEMAPHORES enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumSemaphoresGetValueName(SEMAPHORES value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a WAITOPTION value.
///
/// @ingroup enumname
///
/// @param[in] value The WAITOPTION enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumWaitOptionGetValueName(WAITOPTION value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a WAITTYPE value.
///
/// @ingroup enumname
///
/// @param[in] value The WAITTYPE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumWaitTypeGetValueName(WAITTYPE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a MODETYPE value.
///
/// @ingroup enumname
///
/// @param[in] value The MODETYPE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumModeTypeGetValueName(MODETYPE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a COMPILERERRORTYPE value.
///
/// @ingroup enumname
///
/// @param[in] value The COMPILERERRORTYPE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumCompilerErrorTypeGetValueName(COMPILERERRORTYPE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a DAYOFWEEK value.
///
/// @ingroup enumname
///
/// @param[in] value The DAYOFWEEK enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumDayOfWeekGetValueName(DAYOFWEEK value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a AXISMISMATCH value.
///
/// @ingroup enumname
///
/// @param[in] value The AXISMISMATCH enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumAxisMismatchGetValueName(AXISMISMATCH value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a REGISTERTYPE value.
///
/// @ingroup enumname
///
/// @param[in] value The REGISTERTYPE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumRegisterTypeGetValueName(REGISTERTYPE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a COMPONENTTYPE value.
///
/// @ingroup enumname
///
/// @param[in] value The COMPONENTTYPE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumComponentTypeGetValueName(COMPONENTTYPE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a COMMUNICATIONTYPE value.
///
/// @ingroup enumname
///
/// @param[in] value The COMMUNICATIONTYPE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumCommunicationTypeGetValueName(COMMUNICATIONTYPE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a BUILDRESULTKIND value.
///
/// @ingroup enumname
///
/// @param[in] value The BUILDRESULTKIND enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumBuildResultKindGetValueName(BUILDRESULTKIND value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

/// \brief Retrieves the name of a RAMPMODE value.
///
/// @ingroup enumname
///
/// @param[in] value The RAMPMODE enumeration value for which to retrieve the string name.
/// @param[in] valueNameBuffer The output buffer that will contain the string name of the enumeration value.
/// @param[in] valueNameBufferSize The size, in bytes, of the output buffer that will contain the string name of the enumeration value.
/// \return TRUE on success, FALSE if an error occurred. Call EnsembleGetLastError() for more information.
BOOL DLLENTRYDECLARATION EnsembleEnumRampModeGetValueName(RAMPMODE value, LPSTR valueNameBuffer, DWORD valueNameBufferSize);

#ifdef __cplusplus
}
#endif
#endif // __Ensemble_ENUM_NAMES__
