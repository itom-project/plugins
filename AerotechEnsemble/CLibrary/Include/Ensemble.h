/// \file Ensemble.h
/// \brief Contains the includes of all the other files that allow access to the Ensemble C Library.
///
/// Include this file to get all the functionality of the C library.
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

/// \mainpage Ensemble C Library
/// \brief This is documentation for the Ensemble C Library.
///
/// The following is an example enabling an axis:
/// \dontinclude "Examples.chh"
/// \skip GlobalSimpleExample
/// \until }
/// \until }
///
/// Example programs can be found in <b>[InstallDir]\\Samples\\Cpp\\CLibrary\\</b> and <b>[InstallDir]\\Samples\\C\\CLibrary\\</b>.
///
/// The C Library files can be found in <b>[InstallDir]\\CLibrary</b>.\n
/// 1) <b>include</b> directory contains the header files.\n
/// 2) <b>lib</b> and <b>lib64</b> directories contain the 32-bit and 64-bit lib files.\n
/// 3) <b>bin</b> and <b>bin64</b> directories contain the 32-bit and 64-bit dll files.
///
/// All of the files found in <b>bin</b> and <b>bin64</b> are required for proper operation.  Do one of the following for the
/// application to load correctly:\n
/// 1) Add <b>[InstallDir]\\CLibrary\\bin\\</b> and <b>[InstallDir]\\CLibrary\\bin64\\</b> to the PATH.\n
/// 2) Copy all the the *.dll files from <b>[InstallDir]\\CLibrary\\bin\\</b> and <b>[InstallDir]\\CLibrary\\bin64\\</b> into the output directory.\n

#ifndef __Ensemble_H__
#define __Ensemble_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "EnsembleAeroBasicCommands.h"
#include "EnsembleCommonTypes.h"
#include "EnsembleCommonStructures.h"
#include "EnsembleErrorCodes.h"
#include "EnsembleConnection.h"
#include "EnsembleCommands.h"
#include "EnsembleError.h"
#include "EnsembleParameter.h"
#include "EnsembleParameterId.h"
#include "EnsembleProgram.h"
#include "EnsembleStatus.h"
#include "EnsembleVariable.h"
#include "EnsembleInformation.h"
#include "EnsembleEnumNames.h"
#include "EnsembleDataCollection.h"
#include "EnsembleConfiguration.h"

#ifdef __cplusplus
}
#endif

#endif // __Ensemble_H__
