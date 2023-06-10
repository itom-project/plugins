/// \file EnsembleErrorCodes.h
/// \brief Contains the error code enumeration.
///
/// These are the different errors that you can get by calling EnsembleGetLastError()
///
/// Copyright (c) Aerotech, Inc. 2010-2013.
///

#ifndef __Ensemble_ERROR_CODES_H__
#define __Ensemble_ERROR_CODES_H__

/// \brief The enumeration containing the error codes that can be returned from the Ensemble functions.
typedef enum {
	/// \brief No error
	ErrorCode_NoError = 0,
	/// \brief Error [RegistryOperation] registry
	ErrorCode_RegistryAccessError = 1,
	/// \brief Error reading installation directory
	ErrorCode_InvalidInstallationDirectory = 2,
	/// \brief Access to Flash Parameters is denied
	ErrorCode_AccessToFlashParametersDenied = 3,
	/// \brief Error during the [SemaphoreOperation] of a semaphore: Windows error : [WindowsError]
	ErrorCode_SemaphoreFailure = 4,
	/// \brief Current operation requires doing an Identify on the network first
	ErrorCode_IdentifyNotIssued = 5,
	/// \brief Too many controllers were detected for a connection to be made
	ErrorCode_TooManyControllersFound = 6,
	/// \brief No mapped controllers found on network
	ErrorCode_NoControllersFoundOnNetwork = 7,
	/// \brief Duplicate controller number [ControllerNumber] detected
	ErrorCode_FoundDuplicateControllerNumbers = 8,
	/// \brief [ControllerName] controller name has been detected more than once on the network
	ErrorCode_DuplicateControllerFound = 9,
	/// \brief Network connections are not permitted while the loader is running
	ErrorCode_LoaderIsCurrentlyRunning = 10,
	/// \brief Error initializing Ethernet device with IP Address [ControllerIPAddress]
	ErrorCode_EthernetConnectionFailure = 11,
	/// \brief Error initializing USB device with device ID [ControllerUSBID]
	ErrorCode_USBConnectionFailure = 12,
	/// \brief Controller not found on active channel
	ErrorCode_ControllerNotFound = 13,
	/// \brief Controller does not have an AeroNet interface
	ErrorCode_InterDrvNoComm = 14,
	/// \brief Attempt made to connect to a slave axis
	ErrorCode_InterDrvSlaveConnectionAttempt = 15,
	/// \brief Error [InitializationErrorCode] detected during system initialization
	ErrorCode_InterDrvCmdResetError = 16,
	/// \brief Controller [ControllerName] detected with incompatible firmware version [VersionNumber]
	ErrorCode_VersionIncompatible = 17,
	/// \brief A problem occurred with the USB network setup on [USBID]
	ErrorCode_USBSetupFailure = 18,
	/// \brief Error during broadcast communication to [USBID]
	ErrorCode_USBBroadcastFailure = 19,
	/// \brief Ethernet socket services error: [WindowsErrorCode]
	ErrorCode_EthernetSocketSetupFailure = 20,
	/// \brief Error during broadcast communication to [MACAddress]
	ErrorCode_EthernetBroadcastFailure = 21,
	/// \brief The value of [GivenValue] is out of range for the [EthernetSettingType] Ethernet setting
	ErrorCode_InvalidValueForEthernetSettings = 22,
	/// \brief Value of [NetworkSetupKeyword] is invalid for use in the current operation
	ErrorCode_InvalidNetworkSetupArgument = 23,
	/// \brief [NetworkSetupKeyword] is too long for use as a controller name
	ErrorCode_ControllerNameIsTooLong = 24,
	/// \brief The specified controller name is invalid.
	ErrorCode_ControllerNameInvalid = 25,
	/// \brief The name of controller is locked which prevents the name of the controller from being modified.
	ErrorCode_ControllerNameIsLocked = 26,
	/// \brief Controller name is already locked
	ErrorCode_ControllerNameLockFailed = 27,
	/// \brief This network setup operation requires running the firmware loader first
	ErrorCode_ControllerSetupVersionMismatch = 28,
	/// \brief [FileName] could not be opened
	ErrorCode_CannotOpenFile = 29,
	/// \brief [FileName] could not be created
	ErrorCode_CannotCreateFile = 30,
	/// \brief File extension must be one of the following: [ExtensionList]
	ErrorCode_InvalidFileExtension = 31,
	/// \brief [FileName] file is corrupt
	ErrorCode_CorruptFileFound = 32,
	/// \brief An embedded memory file is corrupt
	ErrorCode_CorruptMemoryFileFound = 33,
	/// \brief [FileName] contains an error on line [LineNumber]
	ErrorCode_InvalidProfileFormat = 34,
	/// \brief The format of the parameter file is invalid : [SpecificMessage]
	ErrorCode_InvalidParameterFileFormat = 35,
	/// \brief [FileName] contains an invalid :START2D line: Missing [ArgumentName] Argument
	ErrorCode_Start2DLine = 37,
	/// \brief [FileName] is not a valid calibration file (no :START or :START2D arguments found)
	ErrorCode_MissingStartArgument = 38,
	/// \brief [FileName] is missing a :END argument
	ErrorCode_MissingEndArgument = 39,
	/// \brief [FileName] is missing a SAMPLEDIST value
	ErrorCode_MissingSampleDistValue = 40,
	/// \brief [FileName] contains invalid column data
	ErrorCode_InvalidColumnData = 41,
	/// \brief [FileName] contains [KeywordName] which is an invalid calibration keyword
	ErrorCode_InvalidCalKeyword = 42,
	/// \brief [FileName] contains [KeywordName] where a numeric value was expected
	ErrorCode_InvalidCalValue = 43,
	/// \brief [FileName] contains both 1D and 2D formats
	ErrorCode_MixedCalFileFound = 44,
	/// \brief Number of points found in [FileName] does not match what was expected
	ErrorCode_IncorrectNumberOfPoints = 45,
	/// \brief No SLAVE value was specified in [FileName]
	ErrorCode_MissingSlaveValue = 46,
	/// \brief No MASTER value was specified in [FileName]
	ErrorCode_MissingMasterValue = 47,
	/// \brief Points specified in [FileName] are out of sequence
	ErrorCode_IncorrectPointsSequence = 48,
	/// \brief Invalid MasterUnit keyword detected in [FileName]
	ErrorCode_InvalidMasterUnit = 49,
	/// \brief [FileName] MASTER values cannot include zero and must be either positive ascending or negative descending
	ErrorCode_InvalidDirectionReversal = 50,
	/// \brief [FileName] contains point data before NumPoints keyword
	ErrorCode_PointDataBeforeNumPoints = 51,
	/// \brief [FileName] contains point data before MasterUnits keyword
	ErrorCode_PointDataBeforeMasterUnit = 52,
	/// \brief [FileName] contains point data before SlaveUnit keyword
	ErrorCode_PointDataBeforeSlaveUnit = 53,
	/// \brief [FileName] contains the SlaveUnit keyword before the MasterUnit keyword
	ErrorCode_SlaveUnitBeforeMasterUnit = 54,
	/// \brief [FileName] contains the SlaveUnit keyword before the NumPoints keyword
	ErrorCode_SlaveUnitBeforeNumPoints = 55,
	/// \brief [FileName] contains [KeywordName] which is an invalid cam keyword
	ErrorCode_InvalidCamKeyword = 56,
	/// \brief Communication to IP address [ControllerIPAddress] was out of order and has been reset
	ErrorCode_EthernetCommunicationReset = 59,
	/// \brief Communication to USB device ID [ControllerUSBID] was out of order and has been reset
	ErrorCode_USBCommunicationReset = 60,
	/// \brief Controller at IP address [ControllerIPAddress] is busy communicating to a PC at a different IP address
	ErrorCode_EthernetCommunicationBusy = 61,
	/// \brief The controller has been disconnected from the network, either because a communication error occurred or because the controller was explicitly disconnected.
	ErrorCode_ControllerIsDisconnected = 62,
	/// \brief Connection to controller number [ControllerNumber] was never established
	ErrorCode_InvalidControllerNumber = 63,
	/// \brief No motion axis detected on AeroNet interface
	ErrorCode_InterDrvNoAxis = 64,
	/// \brief Axis number [SystemAxisNumber] does not exist on AeroNet
	ErrorCode_InvalidAxisNumber = 65,
	/// \brief Parameter type is not specified
	ErrorCode_NoParameterTypeSpecified = 66,
	/// \brief [ParameterName] is not a valid parameter name
	ErrorCode_ParameterNotIdentified = 67,
	/// \brief Commit Complete: Number of parameters committed to controller does not match the PC
	ErrorCode_ParameterMismatch = 68,
	/// \brief Access to [ParameterName] is not allowed
	ErrorCode_ParameterAccessDenied = 69,
	/// \brief Value specified for [ParameterName] is less than the minimum allowable value
	ErrorCode_ParameterValueTooSmall = 70,
	/// \brief Value specified for [ParameterName] is larger than the maximum allowable value
	ErrorCode_ParameterValueTooLarge = 71,
	/// \brief Source specifications for [PartName] do not agree with the destination specifications
	ErrorCode_InvalidParameterInfo = 72,
	/// \brief Error detected while triggering the collection of task usage statistics
	ErrorCode_UsageStatisticsFailure = 73,
	/// \brief Current operation requires that a program be loaded on task [TaskNumber]
	ErrorCode_NoProgramLoaded = 74,
	/// \brief Failure occurred during heap memory allocation on controller
	ErrorCode_HeapAllocationFailure = 75,
	/// \brief Code section is too small to load [FileName] on task [TaskNumber]
	ErrorCode_CodeSectionIsTooSmall = 76,
	/// \brief Data section is too small to load [FileName] on task [TaskNumber]
	ErrorCode_DataSectionIsTooSmall = 77,
	/// \brief Command issued on task [TaskNumber] that is unable to execute the command
	ErrorCode_TaskNotRunningInMonitor = 78,
	/// \brief Compiler version does not match firmware's AeroBasic processor version
	ErrorCode_VersionMismatch = 79,
	/// \brief Controller [ControllerNumber] Task [TaskNumber] currently running a program
	ErrorCode_TaskNotReadyForImmediate = 80,
	/// \brief Controller [ControllerNumber] Task [TaskNumber] currently disabled
	ErrorCode_TaskNotActiveForImmediate = 81,
	/// \brief The program ended incorrectly
	ErrorCode_ProgramEndedIncorrectly = 82,
	/// \brief The following immediate command was aborted : [ImmediateCommand]
	ErrorCode_ImmediateCommandAborted = 83,
	/// \brief Symbol file loaded using OpenApplication does not match the symbols on the controller
	ErrorCode_InvalidSymbolsFileGiven = 84,
	/// \brief Program counter not found for line number [GivenProgramCounter]
	ErrorCode_ProgramCounterNotFound = 85,
	/// \brief Line number not found for program counter value [GivenProgramCounter]
	ErrorCode_LineNumberNotFound = 86,
	/// \brief [FileName] is not currently being debugged
	ErrorCode_ProgramFileNotFound = 87,
	/// \brief A task error was generated while stepping the program
	ErrorCode_TaskErrorFound = 88,
	/// \brief Run or step command issued to a program that has reached completion on task [TaskNumber]
	ErrorCode_ProgramReachedCompletion = 89,
	/// \brief Value of [ArgumentName] is invalid for use in the current variable lookup
	ErrorCode_InvalidVariableLookupArgument = 90,
	/// \brief Variable lookup argument [ArgumentName] is missing
	ErrorCode_MissingVariableLookupArgument = 91,
	/// \brief [GivenReference] could not be found as a [ReferenceType] reference
	ErrorCode_ReferenceTypeNotFound = 92,
	/// \brief Specified reference [GivenReference] is an array
	ErrorCode_ReferenceIsAnArray = 93,
	/// \brief Type of specified reference [GivenReference] is unknown
	ErrorCode_UnknownReferenceType = 94,
	/// \brief Data collection size exceeds system maximum of [SystemMaxNumberOfPoints] points
	ErrorCode_MaximumCollectionExceeded = 95,
	/// \brief Optional data number [OptionalDataNumber] is invalid
	ErrorCode_InvalidOptionalDataNumber = 96,
	/// \brief Cannot allocate or free collect memory while data collection is active
	ErrorCode_CollectionActive = 97,
	/// \brief Unable to allocate memory due to controller restrictions
	ErrorCode_MemoryAllocationRequestTooLarge = 98,
	/// \brief The attempted access of [NumberOfRegisters] [RegisterType] register(s), starting at index [StartingIndexToAccess], is out of bounds
	ErrorCode_RegisterAccessOutOfBounds = 99,
	/// \brief Insufficient free memory to send and store the file
	ErrorCode_InsufficientFileSystemMemory = 100,
	/// \brief Error occurred during file transfer
	ErrorCode_FileTransferFailure = 101,
	/// \brief Error occurred while retrieving the file system directory
	ErrorCode_FailureRetrievingFileSystemDirectory = 102,
	/// \brief Failure occurred during optimization of the controller file system
	ErrorCode_FileSystemOptimizationFailure = 103,
	/// \brief Error occurred while erasing file system
	ErrorCode_UnableToEraseFileSystem = 104,
	/// \brief File system option [FileSystemOption] is not valid
	ErrorCode_InvalidFileSystemOption = 105,
	/// \brief {0} does not exist in the controller file system
	ErrorCode_FileDoesNotExist = 106,
	/// \brief File already exists in the file system or on the PC
	ErrorCode_FileCurrentlyExists = 107,
	/// \brief File name [FileName] is too long to use for the controller file system
	ErrorCode_FileNameIsTooLarge = 108,
	/// \brief Not registered for [CallbackType] callback
	ErrorCode_CallbacksNotRegistered = 109,
	/// \brief Callback type [CallbackType] is invalid
	ErrorCode_InvalidCallbackType = 110,
	/// \brief Controller is already registered for the specified callback
	ErrorCode_UnableToRegisterCallback = 111,
	/// \brief Unable to release registered callback.
	ErrorCode_UnableToReleaseCallback = 112,
	/// \brief Command parameter [ParameterNumber], argument name [ArgumentName] is out of range
	ErrorCode_ExecuteCmdParameterOutOfRange = 113,
	/// \brief Command parameter [ParameterNumber] has no size specification
	ErrorCode_CommandParameterSizeNotFound = 114,
	/// \brief Command [CommandName] resulted in an error.
	ErrorCode_InvalidReturnValue = 115,
	/// \brief Task Error: [TaskErrorString]
	ErrorCode_ResultingTaskError = 116,
	/// \brief Command [CommandName] is an invalid command: [Details]
	ErrorCode_InvalidCommand = 117,
	/// \brief Password provided does not match what is currently stored on the controller
	ErrorCode_PasswordFoundOnControllerIsInvalid = 121,
	/// \brief Controller number [GivenControllerNumber] is out of bounds
	ErrorCode_ControllerNumberOutOfRange = 122,
	/// \brief Task number [GivenTaskNumber] is out of bounds or protected
	ErrorCode_TaskNumberOutOfRangeOrProtected = 123,
	/// \brief Error Loading Firmware: [SystemErrorMessage]
	ErrorCode_FirmwareLoadError = 124,
	/// \brief The given name of the axis is invalid.
	ErrorCode_AxisNameInvalid = 125,
	/// \brief Time slot reset error
	ErrorCode_TimeSlotResetError = 126,
	/// \brief [DirectiveType] directives must be integer bit masks
	ErrorCode_InvalidAerobasicDirective = 127,
	/// \brief Build errors found while compiling [FileName]
	ErrorCode_BuildErrorsFoundInCompiler = 128,
	/// \brief Assembler failure detected: [AssemblerError]
	ErrorCode_AssemblerSystemErrorDetected = 129,
	/// \brief Command line specified is empty
	ErrorCode_EmptyCommandLine = 130,
	/// \brief Command line specified contains an invalid command
	ErrorCode_InvalidCommandLine = 131,
	/// \brief Current operation requires calls to DriveReset and DriveBoot first
	ErrorCode_ControllerBootHasNotBeenIssued = 132,
	/// \brief DriveReset must be issued before DriveBoot
	ErrorCode_ControllerResetHasNotBeenIssued = 133,
	/// \brief End quote is missing from a string argument
	ErrorCode_MissingEndQuoteInStringArgument = 134,
	/// \brief Unexpected Console command argument [InvalidArgumentNumber]
	ErrorCode_UnexpectedCommandArgument = 135,
	/// \brief Invalid Console command argument [InvalidArgumentNumber], [Reason]
	ErrorCode_InvalidCommandArgument = 136,
	/// \brief [SystemErrorMessage]
	ErrorCode_ErrorProcessingStringElement = 137,
	/// \brief [ReturnType] is an invalid return type
	ErrorCode_InvalidReturnTypeSpecified = 138,
	/// \brief Error occurred during conversion of string value [StringValue] to type [IntrinsicType]
	ErrorCode_StringToTypeConversionError = 139,
	/// \brief Missing [ArgumentType] argument number [ArgumentNumber]
	ErrorCode_MissingConsoleArgument = 140,
	/// \brief Cannot read STRING axis parameters with the GET ALL opcode
	ErrorCode_InvalidCommandUsage = 141,
	/// \brief Parameter [ParameterName] is limited to 32 characters
	ErrorCode_StringParameterIsTooLong = 142,
	/// \brief Value of [ValueEntered] is invalid for the integer parameter [ParameterName]
	ErrorCode_InvalidIntegerValueEntered = 143,
	/// \brief A null pointer or invalid string length was passed into a function
	ErrorCode_InvalidPassedPointer = 144,
	/// \brief A controller is mapped on [CommunicationType] with that hardware interface disabled via [RegistryKeyName]
	ErrorCode_InterfaceNotEnabled = 145,
	/// \brief Invalid argument specified: [Description]
	ErrorCode_InvalidArgument = 146,
	/// \brief Cannot perform this action while file system optimization is in progress
	ErrorCode_OptimizationInProgress = 147,
	/// \brief Number of samples requested exceeds the number of samples allocated
	ErrorCode_BufferSizeTooLarge = 148,
	/// \brief Collection aborted during wait for completion
	ErrorCode_CollectionAborted = 149,
	/// \brief The time limit for enabling demo mode has expired.
	ErrorCode_DemoUnlockFailed = 150,
	/// \brief Out of Memory
	ErrorCode_OutOfMemory = 151,
	/// \brief An incorrect renewal code was provided.
	ErrorCode_DemoRenewFailed = 152,
	/// \brief A data collection configuration error occured: [Information]
	ErrorCode_DataCollectionConfiguration = 153,
	/// \brief The value of [ReceivedHeader] that is received from [ControllerID] is not the same as the value of [ExpectedHeader]
	ErrorCode_ReceiveHeaderMismatch = 162,
	/// \brief A timeout occurred while the Ethernet or USB device was waiting for a command response from [ControllerID]
	ErrorCode_ReceiveTimeout = 163,
	/// \brief The Ethernet or USB device tried to communicate to the [ControllerID] device and returned the following device error: [LastError]
	ErrorCode_PCDeviceFailure = 164,
	/// \brief A timeout occurred while the Ethernet or USB device was trying to send a command to [ControllerID]
	ErrorCode_SendTimeout = 165,
}	ErrorCode;

/// \brief The number of error codes.
#define ErrorCode_COUNT 166

#endif // __Ensemble_ERROR_CODES_H__
