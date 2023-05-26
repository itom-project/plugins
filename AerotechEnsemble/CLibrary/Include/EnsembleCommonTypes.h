/// \file EnsembleCommonTypes.h
/// \brief Contains some common types that are used throughout the library.
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_COMMONTYPES_H__
#define __Ensemble_COMMONTYPES_H__

#include <wtypes.h>
#include "EnsembleCommonStructures.h"

/// \brief The handle to the Ensemble.
///
/// This is used to communicate with an Ensemble.  You can
/// get one by calling EnsembleConnect().
typedef void* EnsembleHandle;

/// \brief The handle to the Ensemble Data Collection Configuration.
///
/// This is used to hold the Ensemble Data Collection Configuration.
/// See EnsembleDataCollectionConfigCreate() for initalization details.
typedef void* EnsembleDataCollectConfigHandle;

/// \brief All available tasks to run commands or programs on.
///
typedef enum
{
	TASKID_Library = 0,
	TASKID_01 = 1,
	TASKID_02 = 2,
	TASKID_03 = 3,
	TASKID_04 = 4,
	TASKID_Auxiliary = 5,
} TASKID;

/// \brief All available axis indexes.
typedef enum
{
	AXISINDEX_0 = 0,
	AXISINDEX_1 = 1,
	AXISINDEX_2 = 2,
	AXISINDEX_3 = 3,
	AXISINDEX_4 = 4,
	AXISINDEX_5 = 5,
	AXISINDEX_6 = 6,
	AXISINDEX_7 = 7,
	AXISINDEX_8 = 8,
	AXISINDEX_9 = 9,
} AXISINDEX;

/// \brief All available axis masks.
typedef enum
{
	/// \brief No axes selected.
	AXISMASK_None = 0,

	AXISMASK_0 = (1 << 0),
	AXISMASK_1 = (1 << 1),
	AXISMASK_2 = (1 << 2),
	AXISMASK_3 = (1 << 3),
	AXISMASK_4 = (1 << 4),
	AXISMASK_5 = (1 << 5),
	AXISMASK_6 = (1 << 6),
	AXISMASK_7 = (1 << 7),
	AXISMASK_8 = (1 << 8),
	AXISMASK_9 = (1 << 9),

	/// \brief Maximum number of axes selected.
	AXISMASK_All = 0x3ff,
} AXISMASK;

#ifdef __cplusplus

/// \brief Allows doing logical ORs (|) on AXISMASK enumerations
///
/// This is only necessary for C++ since it does additional type safety checks.
/// For C the | operator just works.
inline AXISMASK operator|(const AXISMASK &a, const AXISMASK &b) {
	return (AXISMASK)( (int)a | (int)b );
}

#endif

/// \brief Define for specifying that the function is to be exported.
///
/// This is necessary to mark functions as exported from this library
/// and allow other libraries and applications to call them.
///
/// Most users can safely ignore this.
#ifdef __Ensemble_C_LIBRARY_INTERNAL__
#define DLLENTRYDECLARATION      /*export uses *.def file*/ __stdcall
#else
#define DLLENTRYDECLARATION       __declspec(dllimport) __stdcall
#endif

/// \brief Define for marking functions as obsolete or deprecated.
#if defined(__GCCXML__) || defined(__GNUC__)
	#define DEPRECATED(reason) __declspec(deprecated)
#elif _MSC_VER >= 1400
	#define DEPRECATED(reason) __declspec(deprecated(reason))
#elif _MSC_VER >= 1300
	#define DEPRECATED(reason) __declspec(deprecated)
#else
	#define DEPRECATED(reason)
#endif

#endif // __ENSEMBLE_COMMONTYPES_H__
