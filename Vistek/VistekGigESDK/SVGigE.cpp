/*******************************************************************************
 * SVGigE       Dynamic loading of GigE camera access functions
 *******************************************************************************
 *
 * Version:     1.5.1 / November 2015
 *
 * Copyright:   SVS VISTEK GmbH
 *
 *******************************************************************************
 *
 * THIS FILE CONTAINS BAYER PATTERN CONVERSION FROM THE FOLLOWING SOURCE:
 *
 * 1394-Based Digital Camera Control Library
 * Bayer pattern decoding functions
 * Copyright (C) Damien Douxchamps <ddouxchamps@users.sf.net>
 *
 * Written by Damien Douxchamps and Frederic Devernay
 *******************************************************************************
 */

#include "windows.h"

#include "SVGigE.h"

// Define a name for the SVS GigE API DLL
#ifdef X64
  #define SVGigE_DLL   _T("SVGigE.x64.dll")
#else
  #define SVGigE_DLL   _T("SVGigE.dll")
#endif

/**
 *  Handle for the GigE DLL
 */
HINSTANCE GigEDLL = NULL;

/**
 *  Enumerate all GigE function calls
 */
typedef
  enum
  {
    // 0 - GigE DLL (implicitly called)
    Func_isVersionCompliantDLL,
    Func_isDriverAvailable,

    // 1 - Camera: Discovery and bookkeeping
    Func_CameraContainer_create,
    Func_CameraContainer_delete,
    Func_CameraContainer_discovery,
    Func_CameraContainer_getNumberOfCameras,
    Func_CameraContainer_getCamera,
    Func_CameraContainer_findCamera,
    // 2 - Camera: Connection
    Func_Camera_openConnection,
	Func_Camera_openConnectionEx,
    Func_Camera_closeConnection,
    Func_Camera_setIPAddress,
    Func_Camera_forceValidNetworkSettings,
    Func_Camera_restartIPConfiguration,

    // 3 - Camera: Information
    Func_Camera_getManufacturerName,
    Func_Camera_getModelName,
    Func_Camera_getDeviceVersion,
    Func_Camera_getManufacturerSpecificInformation,
    Func_Camera_getSerialNumber,
    Func_Camera_setUserDefinedName,
    Func_Camera_getUserDefinedName,
    Func_Camera_getMacAddress,
    Func_Camera_getIPAddress,
    Func_Camera_getSubnetMask,
    Func_Camera_getPixelClock,
    Func_Camera_isCameraFeature,
    Func_Camera_readXMLFile,
	Func_Camera_getSensorTemperature,

    // 4 - Stream: Channel creation and control
    Func_StreamingChannel_create,
    Func_StreamingChannel_createEx,
    Func_StreamingChannel_delete,

	Func_StreamingChannel_setReadoutTransfer,
    Func_StreamingChannel_getReadoutTransfer,

    // 5 - Stream: Channel statistics
    Func_StreamingChannel_getFrameLoss,
    Func_StreamingChannel_getActualFrameRate,
    Func_StreamingChannel_getActualDataRate,
    Func_StreamingChannel_getPeakDataRate,

    // 6 - Stream: Channel info
    Func_StreamingChannel_getPixelType,
	Func_StreamingChannel_getBufferData,
    Func_StreamingChannel_getBufferSize,
    Func_StreamingChannel_getImagePitch,
    Func_StreamingChannel_getImageSizeX,
    Func_StreamingChannel_getImageSizeY,

    // 7 - Stream: Transfer parameters
    Func_Camera_evaluateMaximalPacketSize,
    Func_Camera_setStreamingPacketSize,
    Func_Camera_setInterPacketDelay,
    Func_Camera_getInterPacketDelay,
    Func_Camera_setMulticastMode,
    Func_Camera_getMulticastMode,
    Func_Camera_getMulticastGroup,

    // 8 - Stream: Image access
    Func_Image_getDataPointer,
    Func_Image_getBufferIndex,
    Func_Image_getSignalType,
    Func_Image_getCamera,
    Func_Image_release,

    // 9 - Stream: Image conversion
    Func_Image_getImageRGB,
    Func_Image_getImageGray,
    Func_Image_getImage12bitAs8bit,
    Func_Image_getImage12bitAs16bit,
    Func_Image_getImage16bitAs8bit,

    // 10 - Stream: Image characteristics
    Func_Image_getPixelType,
    Func_Image_getImageSize,
    Func_Image_getPitch,
    Func_Image_getSizeX,
    Func_Image_getSizeY,

    // 11 - Stream: Image statistics
    Func_Image_getImageID,
    Func_Image_getTimestamp,
	Func_Image_getTransferTime,
    Func_Image_getPacketCount,
    Func_Image_getPacketResend,

    // 12 - Stream: Messaging channel
    Func_Stream_createEvent,
    Func_Stream_addMessageType,
    Func_Stream_removeMessageType,
    Func_Stream_isMessagePending,
    Func_Stream_registerEventCallback,
    Func_Stream_unregisterEventCallback,
    Func_Stream_getMessage,
    Func_Stream_getMessageData,
    Func_Stream_getMessageTimestamp,
    Func_Stream_releaseMessage,
    Func_Stream_flushMessages,
    Func_Stream_closeEvent,

    // 13 - Controlling camera: Frame rate
    Func_Camera_setFrameRate,
    Func_Camera_getFrameRate,
    Func_Camera_getFrameRateMin,
    Func_Camera_getFrameRateMax,
    Func_Camera_getFrameRateRange,
    Func_Camera_getFrameRateIncrement,

    // 14 - Controlling camera: Exposure
    Func_Camera_setExposureTime,
    Func_Camera_getExposureTime,
    Func_Camera_getExposureTimeMin,
    Func_Camera_getExposureTimeMax,
    Func_Camera_getExposureTimeRange,
    Func_Camera_getExposureTimeIncrement,
    Func_Camera_setExposureDelay,
    Func_Camera_getExposureDelay,
    Func_Camera_getExposureDelayMax,
    Func_Camera_getExposureDelayIncrement,

    // 15 - Controlling camera: Gain and offset
    Func_Camera_setGain,
    Func_Camera_getGain,
    Func_Camera_getGainMax,
    Func_Camera_getGainMaxExtended,
    Func_Camera_getGainIncrement,
    Func_Camera_setOffset,
    Func_Camera_getOffset,
    Func_Camera_getOffsetMax,

    // 16 - Controlling camera: Auto gain/exposure
    Func_Camera_setAutoGainEnabled,
    Func_Camera_getAutoGainEnabled,
    Func_Camera_setAutoGainBrightness,
    Func_Camera_getAutoGainBrightness,
    Func_Camera_setAutoGainDynamics,
    Func_Camera_getAutoGainDynamics,
    Func_Camera_setAutoGainLimits,
    Func_Camera_getAutoGainLimits,
    Func_Camera_setAutoExposureLimits,
    Func_Camera_getAutoExposureLimits,


    // 17 - Controlling camera: Acquisition trigger
    Func_Camera_setAcquisitionControl,
    Func_Camera_getAcquisitionControl,
    Func_Camera_setAcquisitionMode,
    Func_Camera_setAcquisitionModeAndStart,
    Func_Camera_getAcquisitionMode,
    Func_Camera_softwareTrigger,
    Func_Camera_softwareTriggerID,
    Func_Camera_softwareTriggerIDEnable,
    Func_Camera_setTriggerPolarity,
    Func_Camera_getTriggerPolarity,
	Func_Camera_setPivMode,
	Func_Camera_getPivMode,
	Func_Camera_setDebouncerDuration,
  	Func_Camera_getDebouncerDuration,
	Func_Camera_setPrescalerDevisor,
	Func_Camera_getPrescalerDevisor,
	Func_Camera_loadSequenceParameters,
	Func_Camera_startSequencer,


    // 18 - Controlling camera: Strobe
    Func_Camera_setStrobePolarity,
  	Func_Camera_setStrobePolarityExtended,
    Func_Camera_getStrobePolarity,
	Func_Camera_getStrobePolarityExtended,
    Func_Camera_setStrobePosition,
	Func_Camera_setStrobePositionExtended,
    Func_Camera_getStrobePosition,
	Func_Camera_getStrobePositionExtended,
    Func_Camera_getStrobePositionMax,
    Func_Camera_getStrobePositionIncrement,
    Func_Camera_setStrobeDuration,
    Func_Camera_setStrobeDurationExtended,
    Func_Camera_getStrobeDuration,
	Func_Camera_getStrobeDurationExtended,
    Func_Camera_getStrobeDurationMax,
    Func_Camera_getStrobeDurationIncrement,

    // 19 - Controlling camera: Tap balance
    Func_Camera_setTapConfiguration,
    Func_Camera_getTapConfiguration,
	Func_Camera_setTapConfigurationEx,
    Func_Camera_getTapConfigurationEx,
    Func_Camera_setAutoTapBalanceMode,
    Func_Camera_getAutoTapBalanceMode,
	Func_Camera_setTapGain,
    Func_Camera_getTapGain,


    // 20 - Controlling camera: Image parameter
    Func_Camera_getImagerWidth,
    Func_Camera_getImagerHeight,
    Func_Camera_getImageSize,
    Func_Camera_getPitch,
    Func_Camera_getSizeX,
    Func_Camera_getSizeY,
    Func_Camera_setBinningMode,
    Func_Camera_getBinningMode,
    Func_Camera_setAreaOfInterest,
    Func_Camera_getAreaOfInterest,
    Func_Camera_getAreaOfInterestRange,
    Func_Camera_getAreaOfInterestIncrement,
    Func_Camera_resetTimestampCounter,
    Func_Camera_getTimestampCounter,
    Func_Camera_getTimestampTickFrequency,
	Func_Camera_setFlippingMode,
	Func_Camera_getFlippingMode,
	Func_Camera_setShutterMode,
	Func_Camera_getShutterMode,

    // 21 - Controlling camera: Image appearance
    Func_Camera_getPixelType,
    Func_Camera_setPixelDepth,
    Func_Camera_getPixelDepth,
    Func_Camera_setWhiteBalance,
    Func_Camera_getWhiteBalance,
    Func_Camera_getWhiteBalanceMax,
    Func_Camera_setGammaCorrection,
    Func_Camera_setGammaCorrectionExt,
    Func_Camera_setLowpassFilter,
    Func_Camera_getLowpassFilter,
    Func_Camera_setLookupTableMode,
    Func_Camera_getLookupTableMode,
    Func_Camera_setLookupTable,
    Func_Camera_getLookupTable,
    Func_Camera_startImageCorrection,
    Func_Camera_isIdleImageCorrection,
    Func_Camera_setImageCorrection,
    Func_Camera_getImageCorrection,
	Func_Camera_setPixelsCorrectionMap,
	Func_Camera_getPixelsCorrectionMap,
	Func_Camera_setPixelsCorrectionControlEnabel,
	Func_Camera_getPixelsCorrectionControlEnabel,
	Func_Camera_setPixelsCorrectionControlMark,
	Func_Camera_getPixelsCorrectionControlMark,
	Func_Camera_setPixelsCorrectionMapOffset,
	Func_Camera_getPixelsCorrectionMapOffset,
	Func_Camera_getPixelsCorrectionMapSize,
	Func_Camera_getMaximalPixelsCorrectionMapSize,
	Func_Camera_setMapIndexCoordinate,
	Func_Camera_getMapIndexCoordinate,
	Func_Camera_deletePixelCoordinateFromMap,

    // 22 - Special control: IOMux configuration
    Func_Camera_getMaxIOMuxIN,
    Func_Camera_getMaxIOMuxOUT,
    Func_Camera_setIOAssignment,
    Func_Camera_getIOAssignment,

    // 23 - Special control: IO control
    Func_Camera_setIOMuxIN,
    Func_Camera_getIOMuxIN,
    Func_Camera_setIO,
    Func_Camera_getIO,
    Func_Camera_setAcqLEDOverride,
    Func_Camera_getAcqLEDOverride,
    Func_Camera_setLEDIntensity,
    Func_Camera_getLEDIntensity,

    // 24 - Special control: Serial communication
    Func_Camera_setUARTBuffer,
    Func_Camera_getUARTBuffer,
    Func_Camera_setUARTBaud,
    Func_Camera_getUARTBaud,

    // 25 - Special control: Direct register and memory access
    Func_Camera_setGigECameraRegister,
    Func_Camera_getGigECameraRegister,
    Func_Camera_writeGigECameraMemory,
    Func_Camera_readGigECameraMemory,
    Func_Camera_forceOpenConnection,

    // 26 - Special control: Persistent settings and recovery
    Func_Camera_writeEEPROM,
    Func_Camera_readEEPROM,
    Func_Camera_restoreFactoryDefaults,
	Func_Camera_loadSettingsFromXml,
    Func_Camera_SaveSettingsToXml,

    // 27 - General functions
    Func_SVGigE_estimateWhiteBalance,
    Func_SVGigE_estimateWhiteBalanceExtended,
    Func_SVGigE_writeImageToBitmapFile,
    Func_SVGigE_installFilterDriver,
    Func_SVGigE_uninstallFilterDriver,

    // 28 - Diagnostics
    Func_Error_getMessage,
    Func_Camera_registerForLogMessages,

    // 29 - Special control: Lens control
    Func_Camera_isLensAvailable,
    Func_Camera_getLensName,

    Func_Camera_setLensFocalLenght,
    Func_Camera_getLensFocalLenght,
    Func_Camera_getLensFocalLenghtMin,
    Func_Camera_getLensFocalLenghtMax,

	Func_Camera_setLensFocusUnit,
    Func_Camera_getLensFocusUnit,
    Func_Camera_setLensFocus,
    Func_Camera_getLensFocus,
    Func_Camera_getLensFocusMin,
    Func_Camera_getLensFocusMax,

    Func_Camera_setLensAperture,
    Func_Camera_getLensAperture,
    Func_Camera_getLensApertureMin,
    Func_Camera_getLensApertureMax,



    // 99 - Deprecated functions
    Func_Camera_startAcquisitionCycle,		    //2009-05-05: DEPRECATED, please use Camera_softwareTrigger()
    Func_Camera_setTapCalibration,				//2009-03-10: DEPRECATED, please use Camera_setTapBalance()
    Func_Camera_getTapCalibration,				//2009-03-10: DEPRECATED, please use Camera_getTapBalance()
    Func_Camera_setLUTMode,						//2009-02-19: re-implemented for backward compatibility
    Func_Camera_getLUTMode,						//2009-02-19: re-implemented for backward compatibility
    Func_Camera_createLUTwhiteBalance,		    //2006-12-20: re-implemented by Camera_setWhiteBalance()
    Func_Camera_stampTimestamp,					//2008: removed, please use Camera_getTimestampCounter()
    Func_Camera_getTimestamp,					//2008: removed, please use Camera_getTimestampCounter()
    Func_Image_getDebugInfo,					//2010: forwarding debug values, for internal use
    Func_Camera_setTapBalance,					//2011-08-19: deprecated, please use Camera_setTapGain()
    Func_Camera_getTapBalance,					//2011-08-19: deprecated, please use Camera_getTapGain()
	Func_StreamingChannel_setChannelTimeout,    // removed: re-implemented for backward compatibility
    Func_StreamingChannel_getChannelTimeout,    // removed:  re-implemented for backward compatibility
	Func_Camera_setTapUserSettings,				// removed: re-implemented for backward compatibility
    Func_Camera_getTapUserSettings,				// removed: re-implemented for backward compatibility
    Func_Camera_saveTapBalanceSettings,			// deprecated:  please use Camera_setTapGain()
    Func_Camera_loadTapBalanceSettings,         // deprecated:  please use Camera_getTapGain()
	// 00 - Consistency check
	//
	// The following function will be used to
	// check whether consistency of a loaded
	// function table is OK.
	//
	Func_isVersionCompliantDLL_consistency_check,
  }
  SVGigE_FUNCTION;

/**
 *  Array of function pointers that will be obtained from the GigE DLL
 */
struct _GigEFunc
{
  FARPROC function_pointer;
  SVGigE_FUNCTION function_id;
  const char *function_name;
}
GigEFunc[] =
{
  // 0 - GigE DLL (implicitly called)
  NULL, Func_isVersionCompliantDLL,                    "isVersionCompliantDLL",
  NULL, Func_isDriverAvailable,                        "isDriverAvailable",

  // 1 - Camera: Discovery and bookkeeping
  NULL, Func_CameraContainer_create,                    "CameraContainer_create",
  NULL, Func_CameraContainer_delete,                    "CameraContainer_delete",
  NULL, Func_CameraContainer_discovery,                 "CameraContainer_discovery",
  NULL, Func_CameraContainer_getNumberOfCameras,        "CameraContainer_getNumberOfCameras",
  NULL, Func_CameraContainer_getCamera,                 "CameraContainer_getCamera",
  NULL, Func_CameraContainer_findCamera,                "CameraContainer_findCamera",

  // 2 - Camera: Connection
  NULL, Func_Camera_openConnection,                     "Camera_openConnection",
  NULL, Func_Camera_openConnectionEx,                   "Camera_openConnectionEx",
  NULL, Func_Camera_closeConnection,                    "Camera_closeConnection",
  NULL, Func_Camera_setIPAddress,                       "Camera_setIPAddress",
  NULL, Func_Camera_forceValidNetworkSettings,          "Camera_forceValidNetworkSettings",
  NULL, Func_Camera_restartIPConfiguration,             "Camera_restartIPConfiguration",

  // 3 - Camera: Information
  NULL, Func_Camera_getManufacturerName,                "Camera_getManufacturerName",
  NULL, Func_Camera_getModelName,                       "Camera_getModelName",
  NULL, Func_Camera_getDeviceVersion,                   "Camera_getDeviceVersion",
  NULL, Func_Camera_getManufacturerSpecificInformation, "Camera_getManufacturerSpecificInformation",
  NULL, Func_Camera_getSerialNumber,                    "Camera_getSerialNumber",
  NULL, Func_Camera_setUserDefinedName,                 "Camera_setUserDefinedName",
  NULL, Func_Camera_getUserDefinedName,                 "Camera_getUserDefinedName",
  NULL, Func_Camera_getMacAddress,                      "Camera_getMacAddress",
  NULL, Func_Camera_getIPAddress,                       "Camera_getIPAddress",
  NULL, Func_Camera_getSubnetMask,                      "Camera_getSubnetMask",
  NULL, Func_Camera_getPixelClock,                      "Camera_getPixelClock",
  NULL, Func_Camera_isCameraFeature,                    "Camera_isCameraFeature",
  NULL, Func_Camera_readXMLFile,                        "Camera_readXMLFile",
  NULL,Func_Camera_getSensorTemperature,                 "Camera_getSensorTemperature",

  // 4 - Stream: Channel creation and control
  NULL, Func_StreamingChannel_create,                   "StreamingChannel_create",
  NULL, Func_StreamingChannel_createEx,                 "StreamingChannel_createEx",
  NULL, Func_StreamingChannel_delete,                   "StreamingChannel_delete",
  NULL, Func_StreamingChannel_setReadoutTransfer,       "StreamingChannel_setReadoutTransfer",
  NULL, Func_StreamingChannel_getReadoutTransfer,       "StreamingChannel_getReadoutTransfer",

  // 5 - Stream: Channel statistics
  NULL, Func_StreamingChannel_getFrameLoss,             "StreamingChannel_getFrameLoss",
  NULL, Func_StreamingChannel_getActualFrameRate,       "StreamingChannel_getActualFrameRate",
  NULL, Func_StreamingChannel_getActualDataRate,        "StreamingChannel_getActualDataRate",
  NULL, Func_StreamingChannel_getPeakDataRate,          "StreamingChannel_getPeakDataRate",

  // 6 - Stream: Channel info
  NULL, Func_StreamingChannel_getPixelType,             "StreamingChannel_getPixelType",
  NULL, Func_StreamingChannel_getBufferData,            "StreamingChannel_getBufferData",
  NULL, Func_StreamingChannel_getBufferSize,            "StreamingChannel_getBufferSize",
  NULL, Func_StreamingChannel_getImagePitch,            "StreamingChannel_getImagePitch",
  NULL, Func_StreamingChannel_getImageSizeX,            "StreamingChannel_getImageSizeX",
  NULL, Func_StreamingChannel_getImageSizeY,            "StreamingChannel_getImageSizeY",

  // 7 - Stream: Transfer parameters
  NULL, Func_Camera_evaluateMaximalPacketSize,          "Camera_evaluateMaximalPacketSize",
  NULL, Func_Camera_setStreamingPacketSize,				"Camera_setStreamingPacketSize",
  NULL, Func_Camera_setInterPacketDelay,                "Camera_setInterPacketDelay",
  NULL, Func_Camera_getInterPacketDelay,                "Camera_getInterPacketDelay",
  NULL, Func_Camera_setMulticastMode,     				"Camera_setMulticastMode",
  NULL, Func_Camera_getMulticastMode,     				"Camera_getMulticastMode",
  NULL, Func_Camera_getMulticastGroup,     				"Camera_getMulticastGroup",

  // 8 - Stream: Image access
  NULL, Func_Image_getDataPointer,                      "Image_getDataPointer",
  NULL, Func_Image_getBufferIndex,                      "Image_getBufferIndex",
  NULL, Func_Image_getSignalType,                       "Image_getSignalType",
  NULL, Func_Image_getCamera,                           "Image_getCamera",
  NULL, Func_Image_release,                             "Image_release",

  // 9 - Stream: Image conversion
  NULL, Func_Image_getImageRGB,                         "Image_getImageRGB",
  NULL, Func_Image_getImageGray,                        "Image_getImageGray",
  NULL, Func_Image_getImage12bitAs8bit,                 "Image_getImage12bitAs8bit",
  NULL, Func_Image_getImage12bitAs16bit,                "Image_getImage12bitAs16bit",
  NULL, Func_Image_getImage16bitAs8bit,                 "Image_getImage16bitAs8bit",

  // 10 - Stream: Image characteristics
  NULL, Func_Image_getPixelType,                        "Image_getPixelType",
  NULL, Func_Image_getImageSize,                        "Image_getImageSize",
  NULL, Func_Image_getPitch,                            "Image_getPitch",
  NULL, Func_Image_getSizeX,                            "Image_getSizeX",
  NULL, Func_Image_getSizeY,                            "Image_getSizeY",

  // 11 - Stream: Image statistics
  NULL, Func_Image_getImageID,                          "Image_getImageID",
  NULL, Func_Image_getTimestamp,                        "Image_getTimestamp",
  NULL, Func_Image_getTransferTime,                     "Image_getTransferTime",
  NULL, Func_Image_getPacketCount,                      "Image_getPacketCount",
  NULL, Func_Image_getPacketResend,                     "Image_getPacketResend",

  // 12 - Stream: Messaging channel
  NULL, Func_Stream_createEvent,                        "Stream_createEvent",
  NULL, Func_Stream_addMessageType,                     "Stream_addMessageType",
  NULL, Func_Stream_removeMessageType,                  "Stream_removeMessageType",
  NULL, Func_Stream_isMessagePending,                   "Stream_isMessagePending",
  NULL, Func_Stream_registerEventCallback,              "Stream_registerEventCallback",
  NULL, Func_Stream_unregisterEventCallback,            "Stream_unregisterEventCallback",
  NULL, Func_Stream_getMessage,                         "Stream_getMessage",
  NULL, Func_Stream_getMessageData,                     "Stream_getMessageData",
  NULL, Func_Stream_getMessageTimestamp,                "Stream_getMessageTimestamp",
  NULL, Func_Stream_releaseMessage,                     "Stream_releaseMessage",
  NULL, Func_Stream_flushMessages,                      "Stream_flushMessages",
  NULL, Func_Stream_closeEvent,                         "Stream_closeEvent",

  // 13 - Controlling camera: Frame rate
  NULL, Func_Camera_setFrameRate,                       "Camera_setFrameRate",
  NULL, Func_Camera_getFrameRate,                       "Camera_getFrameRate",
  NULL, Func_Camera_getFrameRateMin,                    "Camera_getFrameRateMin",
  NULL, Func_Camera_getFrameRateMax,                    "Camera_getFrameRateMax",
  NULL, Func_Camera_getFrameRateRange,                  "Camera_getFrameRateRange",
  NULL, Func_Camera_getFrameRateIncrement,              "Camera_getFrameRateIncrement",

  // 14 - Controlling camera: Exposure
  NULL, Func_Camera_setExposureTime,                    "Camera_setExposureTime",
  NULL, Func_Camera_getExposureTime,                    "Camera_getExposureTime",
  NULL, Func_Camera_getExposureTimeMin,                 "Camera_getExposureTimeMin",
  NULL, Func_Camera_getExposureTimeMax,                 "Camera_getExposureTimeMax",
  NULL, Func_Camera_getExposureTimeRange,               "Camera_getExposureTimeRange",
  NULL, Func_Camera_getExposureTimeIncrement,           "Camera_getExposureTimeIncrement",
  NULL, Func_Camera_setExposureDelay,                   "Camera_setExposureDelay",
  NULL, Func_Camera_getExposureDelay,                   "Camera_getExposureDelay",
  NULL, Func_Camera_getExposureDelayMax,                "Camera_getExposureDelayMax",
  NULL, Func_Camera_getExposureDelayIncrement,          "Camera_getExposureDelayIncrement",

  // 15 - Controlling camera: Gain and offset
  NULL, Func_Camera_setGain,                            "Camera_setGain",
  NULL, Func_Camera_getGain,                            "Camera_getGain",
  NULL, Func_Camera_getGainMax,                         "Camera_getGainMax",
  NULL, Func_Camera_getGainMaxExtended,                 "Camera_getGainMaxExtended",
  NULL, Func_Camera_getGainIncrement,                   "Camera_getGainIncrement",
  NULL, Func_Camera_setOffset,                          "Camera_setOffset",
  NULL, Func_Camera_getOffset,                          "Camera_getOffset",
  NULL, Func_Camera_getOffsetMax,                       "Camera_getOffsetMax",

  // 16 - Controlling camera: Auto gain/exposure
  NULL, Func_Camera_setAutoGainEnabled,                 "Camera_setAutoGainEnabled",
  NULL, Func_Camera_getAutoGainEnabled,                 "Camera_getAutoGainEnabled",
  NULL, Func_Camera_setAutoGainBrightness,              "Camera_setAutoGainBrightness",
  NULL, Func_Camera_getAutoGainBrightness,              "Camera_getAutoGainBrightness",
  NULL, Func_Camera_setAutoGainDynamics,                "Camera_setAutoGainDynamics",
  NULL, Func_Camera_getAutoGainDynamics,                "Camera_getAutoGainDynamics",
  NULL, Func_Camera_setAutoGainLimits,                  "Camera_setAutoGainLimits",
  NULL, Func_Camera_getAutoGainLimits,                  "Camera_getAutoGainLimits",
  NULL, Func_Camera_setAutoExposureLimits,              "Camera_setAutoExposureLimits",
  NULL, Func_Camera_getAutoExposureLimits,              "Camera_getAutoExposureLimits",

  // 17 - Controlling camera: Acquisition trigger
  NULL, Func_Camera_setAcquisitionControl,              "Camera_setAcquisitionControl",
  NULL, Func_Camera_getAcquisitionControl,              "Camera_getAcquisitionControl",
  NULL, Func_Camera_setAcquisitionMode,                 "Camera_setAcquisitionMode",
  NULL, Func_Camera_setAcquisitionModeAndStart,         "Camera_setAcquisitionModeAndStart",
  NULL, Func_Camera_getAcquisitionMode,                 "Camera_getAcquisitionMode",
  NULL, Func_Camera_softwareTrigger,                    "Camera_softwareTrigger",
  NULL, Func_Camera_softwareTriggerID,                  "Camera_softwareTriggerID",
  NULL, Func_Camera_softwareTriggerIDEnable,            "Camera_softwareTriggerIDEnable",
  NULL, Func_Camera_setTriggerPolarity,                 "Camera_setTriggerPolarity",
  NULL, Func_Camera_getTriggerPolarity,                 "Camera_getTriggerPolarity",
  NULL, Func_Camera_setPivMode,                         "Camera_setPivMode",
  NULL, Func_Camera_getPivMode,                         "Camera_getPivMode",
  NULL, Func_Camera_setDebouncerDuration,               "Camera_setDebouncerDuration",
  NULL, Func_Camera_getDebouncerDuration,               "Camera_getDebouncerDuration",
  NULL,	Func_Camera_setPrescalerDevisor,                "Camera_setPrescalerDevisor",
  NULL,	Func_Camera_getPrescalerDevisor,                "Camera_getPrescalerDevisor",
  NULL,	Func_Camera_loadSequenceParameters,             "Camera_loadSequenceParameters",
  NULL, Func_Camera_startSequencer,                     "Camera_startSequencer",

  // 18 - Controlling camera: Strobe
  NULL, Func_Camera_setStrobePolarity,                  "Camera_setStrobePolarity",
  NULL, Func_Camera_setStrobePolarityExtended,          "Camera_setStrobePolarityExtended",
  NULL, Func_Camera_getStrobePolarity,                  "Camera_getStrobePolarity",
  NULL, Func_Camera_getStrobePolarityExtended,          "Camera_getStrobePolarityExtended",
  NULL, Func_Camera_setStrobePosition,                  "Camera_setStrobePosition",
  NULL, Func_Camera_setStrobePositionExtended,          "Camera_setStrobePositionExtended",
  NULL, Func_Camera_getStrobePosition,                  "Camera_getStrobePosition",
  NULL, Func_Camera_getStrobePositionExtended,          "Camera_getStrobePositionExtended",
  NULL, Func_Camera_getStrobePositionMax,               "Camera_getStrobePositionMax",
  NULL, Func_Camera_getStrobePositionIncrement,         "Camera_getStrobePositionIncrement",
  NULL, Func_Camera_setStrobeDuration,                  "Camera_setStrobeDuration",
  NULL, Func_Camera_setStrobeDurationExtended,          "Camera_setStrobeDurationExtended",
  NULL, Func_Camera_getStrobeDuration,                  "Camera_getStrobeDuration",
  NULL, Func_Camera_getStrobeDurationExtended,          "Camera_getStrobeDurationExtended",
  NULL, Func_Camera_getStrobeDurationMax,               "Camera_getStrobeDurationMax",
  NULL, Func_Camera_getStrobeDurationIncrement,         "Camera_getStrobeDurationIncrement",

  // 19 - Controlling camera: Tap balance
  NULL, Func_Camera_setTapConfiguration,                "Camera_setTapConfiguration",
  NULL, Func_Camera_getTapConfiguration,                "Camera_getTapConfiguration",
  NULL, Func_Camera_setTapConfigurationEx,              "Camera_setTapConfigurationEx",
  NULL, Func_Camera_getTapConfigurationEx,              "Camera_getTapConfigurationEx",
  NULL, Func_Camera_setAutoTapBalanceMode,              "Camera_setAutoTapBalanceMode",
  NULL, Func_Camera_getAutoTapBalanceMode,              "Camera_getAutoTapBalanceMode",
  NULL, Func_Camera_setTapGain,                         "Camera_setTapGain",
  NULL, Func_Camera_getTapGain,                         "Camera_getTapGain",

  // 20 - Controlling camera: Image parameter
  NULL, Func_Camera_getImagerWidth,                     "Camera_getImagerWidth",
  NULL, Func_Camera_getImagerHeight,                    "Camera_getImagerHeight",
  NULL, Func_Camera_getImageSize,                       "Camera_getImageSize",
  NULL, Func_Camera_getPitch,                           "Camera_getPitch",
  NULL, Func_Camera_getSizeX,                           "Camera_getSizeX",
  NULL, Func_Camera_getSizeY,                           "Camera_getSizeY",
  NULL, Func_Camera_setBinningMode,                     "Camera_setBinningMode",
  NULL, Func_Camera_getBinningMode,                     "Camera_getBinningMode",
  NULL, Func_Camera_setAreaOfInterest,                  "Camera_setAreaOfInterest",
  NULL, Func_Camera_getAreaOfInterest,                  "Camera_getAreaOfInterest",
  NULL, Func_Camera_getAreaOfInterestRange,             "Camera_getAreaOfInterestRange",
  NULL, Func_Camera_getAreaOfInterestIncrement,         "Camera_getAreaOfInterestIncrement",
  NULL, Func_Camera_resetTimestampCounter,              "Camera_resetTimestampCounter",
  NULL, Func_Camera_getTimestampCounter,                "Camera_getTimestampCounter",
  NULL, Func_Camera_getTimestampTickFrequency,          "Camera_getTimestampTickFrequency",
  NULL, Func_Camera_setFlippingMode,	                "Camera_setFlippingMode",
  NULL, Func_Camera_getFlippingMode,					"Camera_getFlippingMode",
  NULL,	Func_Camera_setShutterMode,						"Camera_setShutterMode",
  NULL,	Func_Camera_getShutterMode,						"Camera_getShutterMode",

  // 21 - Controlling camera: Image appearance
  NULL, Func_Camera_getPixelType,                       "Camera_getPixelType",
  NULL, Func_Camera_setPixelDepth,                      "Camera_setPixelDepth",
  NULL, Func_Camera_getPixelDepth,                      "Camera_getPixelDepth",
  NULL, Func_Camera_setWhiteBalance,                    "Camera_setWhiteBalance",
  NULL, Func_Camera_getWhiteBalance,                    "Camera_getWhiteBalance",
  NULL, Func_Camera_getWhiteBalanceMax,                 "Camera_getWhiteBalanceMax",
  NULL, Func_Camera_setGammaCorrection,                 "Camera_setGammaCorrection",
  NULL, Func_Camera_setGammaCorrectionExt,              "Camera_setGammaCorrectionExt",
  NULL, Func_Camera_setLowpassFilter,                   "Camera_setLowpassFilter",
  NULL, Func_Camera_getLowpassFilter,                   "Camera_getLowpassFilter",
  NULL, Func_Camera_setLookupTableMode,                 "Camera_setLookupTableMode",
  NULL, Func_Camera_getLookupTableMode,                 "Camera_getLookupTableMode",
  NULL, Func_Camera_setLookupTable,                     "Camera_setLookupTable",
  NULL, Func_Camera_getLookupTable,                     "Camera_getLookupTable",
  NULL, Func_Camera_startImageCorrection,               "Camera_startImageCorrection",
  NULL, Func_Camera_isIdleImageCorrection,              "Camera_isIdleImageCorrection",
  NULL, Func_Camera_setImageCorrection,                 "Camera_setImageCorrection",
  NULL, Func_Camera_getImageCorrection,                 "Camera_getImageCorrection",
  NULL, Func_Camera_setPixelsCorrectionMap,				"Camera_setPixelsCorrectionMap",
  NULL, Func_Camera_getPixelsCorrectionMap,				"Camera_getPixelsCorrectionMap",
  NULL, Func_Camera_setPixelsCorrectionControlEnabel,   "Camera_setPixelsCorrectionControlEnabel",
  NULL, Func_Camera_getPixelsCorrectionControlEnabel,	"Camera_getPixelsCorrectionControlEnabel",
  NULL, Func_Camera_setPixelsCorrectionControlMark,		"Camera_setPixelsCorrectionControlMark",
  NULL, Func_Camera_getPixelsCorrectionControlMark,		"Camera_getPixelsCorrectionControlMark",
  NULL, Func_Camera_setPixelsCorrectionMapOffset,		"Camera_setPixelsCorrectionMapOffset",
  NULL, Func_Camera_getPixelsCorrectionMapOffset,		"Camera_getPixelsCorrectionMapOffset",
  NULL, Func_Camera_getPixelsCorrectionMapSize,			"Camera_getPixelsCorrectionMapSize",
  NULL, Func_Camera_getMaximalPixelsCorrectionMapSize,	"Camera_getMaximalPixelsCorrectionMapSize",
  NULL, Func_Camera_setMapIndexCoordinate,				"Camera_setMapIndexCoordinate",
  NULL, Func_Camera_getMapIndexCoordinate,				"Camera_getMapIndexCoordinate",
  NULL, Func_Camera_deletePixelCoordinateFromMap,		"Camera_deletePixelCoordinateFromMap",

  // 22 - Special control: IOMux configuration
  NULL, Func_Camera_getMaxIOMuxIN,                      "Camera_getMaxIOMuxIN",
  NULL, Func_Camera_getMaxIOMuxOUT,                     "Camera_getMaxIOMuxOUT",
  NULL, Func_Camera_setIOAssignment,                    "Camera_setIOAssignment",
  NULL, Func_Camera_getIOAssignment,                    "Camera_getIOAssignment",

  // 23 - Special control: IO control
  NULL, Func_Camera_setIOMuxIN,                         "Camera_setIOMuxIN",
  NULL, Func_Camera_getIOMuxIN,                         "Camera_getIOMuxIN",
  NULL, Func_Camera_setIO,                              "Camera_setIO",
  NULL, Func_Camera_getIO,                              "Camera_getIO",
  NULL, Func_Camera_setAcqLEDOverride,                  "Camera_setAcqLEDOverride",
  NULL, Func_Camera_getAcqLEDOverride,                  "Camera_getAcqLEDOverride",
  NULL, Func_Camera_setLEDIntensity,                    "Camera_setLEDIntensity",
  NULL, Func_Camera_getLEDIntensity,                    "Camera_getLEDIntensity",

  // 24 - Special control: Serial communication
  NULL, Func_Camera_setUARTBuffer,                      "Camera_setUARTBuffer",
  NULL, Func_Camera_getUARTBuffer,                      "Camera_getUARTBuffer",
  NULL, Func_Camera_setUARTBaud,                        "Camera_setUARTBaud",
  NULL, Func_Camera_getUARTBaud,                        "Camera_getUARTBaud",

  // 25 - Special control: Direct register and memory access
  NULL, Func_Camera_setGigECameraRegister,              "Camera_setGigECameraRegister",
  NULL, Func_Camera_getGigECameraRegister,              "Camera_getGigECameraRegister",
  NULL, Func_Camera_writeGigECameraMemory,              "Camera_writeGigECameraMemory",
  NULL, Func_Camera_readGigECameraMemory,               "Camera_readGigECameraMemory",
  NULL, Func_Camera_forceOpenConnection,                "Camera_forceOpenConnection",

  // 26 - Special control: Persistent settings and recovery
  NULL, Func_Camera_writeEEPROM,                        "Camera_writeEEPROM",
  NULL, Func_Camera_readEEPROM,                         "Camera_readEEPROM",
  NULL, Func_Camera_restoreFactoryDefaults,             "Camera_restoreFactoryDefaults",
  NULL, Func_Camera_loadSettingsFromXml,                "Camera_loadSettingsFromXml",
  NULL, Func_Camera_SaveSettingsToXml,                  "Camera_SaveSettingsToXml",

  // 27 - General functions
  NULL, Func_SVGigE_estimateWhiteBalance,               "SVGigE_estimateWhiteBalance",
  NULL, Func_SVGigE_estimateWhiteBalanceExtended,       "SVGigE_estimateWhiteBalanceExtended",
  NULL, Func_SVGigE_writeImageToBitmapFile,             "SVGigE_writeImageToBitmapFile",
  NULL, Func_SVGigE_installFilterDriver,                "SVGigE_installFilterDriver",
  NULL, Func_SVGigE_uninstallFilterDriver,              "SVGigE_uninstallFilterDriver",

  // 28 - Diagnostics
  NULL, Func_Error_getMessage,                          "Error_getMessage",
  NULL, Func_Camera_registerForLogMessages,             "Camera_registerForLogMessages",

  // 29 - Special control: Lens conttrol
  NULL, Func_Camera_isLensAvailable,					"Camera_isLensAvailable",
  NULL, Func_Camera_getLensName,					"Camera_getLensName",

  NULL, Func_Camera_setLensFocalLenght,					"Camera_setLensFocalLenght",
  NULL, Func_Camera_getLensFocalLenght,					"Camera_getLensFocalLenght",
  NULL, Func_Camera_getLensFocalLenghtMin,				"Camera_getLensFocalLenghtMin",
  NULL, Func_Camera_getLensFocalLenghtMax,				"Camera_getLensFocalLenghtMax",

  NULL, Func_Camera_setLensFocusUnit,					"Camera_setLensFocusUnit",
  NULL, Func_Camera_getLensFocusUnit,					"Camera_getLensFocusUnit",
  NULL, Func_Camera_setLensFocus,					"Camera_setLensFocus",
  NULL, Func_Camera_getLensFocus,					"Camera_getLensFocus",
  NULL, Func_Camera_getLensFocusMin,					"Camera_getLensFocusMin",
  NULL, Func_Camera_getLensFocusMax,					"Camera_getLensFocusMax",


  NULL, Func_Camera_setLensAperture,					"Camera_setLensAperture",
  NULL, Func_Camera_getLensAperture,					"Camera_getLensAperture",
  NULL, Func_Camera_getLensApertureMin,					"Camera_getLensApertureMin",
  NULL, Func_Camera_getLensApertureMax,					"Camera_getLensApertureMax",


 // 99 - Deprecated functions
  NULL, Func_Camera_startAcquisitionCycle,              "Camera_startAcquisitionCycle",
  NULL, Func_Camera_setTapCalibration,                  "Camera_setTapCalibration",
  NULL, Func_Camera_getTapCalibration,                  "Camera_getTapCalibration",
  NULL, Func_Camera_setLUTMode,                         "Camera_setLUTMode",
  NULL, Func_Camera_getLUTMode,                         "Camera_getLUTMode",
  NULL, Func_Camera_createLUTwhiteBalance,              "Camera_createLUTwhiteBalance",
  NULL, Func_Camera_stampTimestamp,                     "Camera_stampTimestamp",
  NULL, Func_Camera_getTimestamp,                       "Camera_getTimestamp",
  NULL, Func_Image_getDebugInfo,                        "Image_getDebugInfo",
  NULL, Func_Camera_setTapBalance,                      "Camera_setTapBalance",
  NULL, Func_Camera_getTapBalance,                      "Camera_getTapBalance",
  NULL, Func_StreamingChannel_setChannelTimeout,        "StreamingChannel_setChannelTimeout",
  NULL, Func_StreamingChannel_getChannelTimeout,        "StreamingChannel_getChannelTimeout",
  NULL, Func_Camera_setTapUserSettings,                 "Camera_setTapUserSettings",
  NULL, Func_Camera_getTapUserSettings,                 "Camera_getTapUserSettings",
  NULL, Func_Camera_saveTapBalanceSettings,             "Camera_saveTapBalanceSettings",
  NULL, Func_Camera_loadTapBalanceSettings,             "Camera_loadTapBalanceSettings",


  // 00 - Consistency check
  //
  // The following function pointer will be used to
  // check whether consistency of the whole function
  // table is OK.
  //
  NULL, Func_isVersionCompliantDLL_consistency_check,   "isVersionCompliantDLL",
};

/**
 *  Specify function types to be used for casting function pointers that are retrieved from the DLL
 */

//-----------------------------------------------------------------------------------------
// 0 - GigE DLL (also implicitly called)
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_isVersionCompliantDLL)(SVGigE_VERSION *DllVersion,
                               SVGigE_VERSION *ExpectedVersion);

typedef SVGigE_RETURN
(*TFunc_isDriverAvailable)();

//-----------------------------------------------------------------------------------------
// 1 - Camera: Discovery and bookkeeping
//-----------------------------------------------------------------------------------------

typedef CameraContainerClient_handle
(*TFunc_CameraContainer_create)(SVGigETL_Type TransportLayerType);

typedef SVGigE_RETURN
(*TFunc_CameraContainer_delete)(CameraContainerClient_handle hCameraContainer);

typedef SVGigE_RETURN
(*TFunc_CameraContainer_discovery)(CameraContainerClient_handle hCameraContainer);

typedef int
(*TFunc_CameraContainer_getNumberOfCameras)(CameraContainerClient_handle hCameraContainer);

typedef  Camera_handle
(*TFunc_CameraContainer_getCamera)(CameraContainerClient_handle hCameraContainer,
                                   int CameraIndex);

typedef  Camera_handle
(*TFunc_CameraContainer_findCamera)(CameraContainerClient_handle hCameraContainer,
                                    char *CameraName);

//-----------------------------------------------------------------------------------------
// 2 - Camera: Connection
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_openConnection)(Camera_handle hCamera, float Timeout);

typedef SVGigE_RETURN
(*TFunc_Camera_openConnectionEx)(Camera_handle hCamera, float HeartbeatTimeout, int GVCPRetryCount, int GVCPTimeout);


typedef SVGigE_RETURN
(*TFunc_Camera_closeConnection)(Camera_handle hCamera);

typedef SVGigE_RETURN
(*TFunc_Camera_setIPAddress)(Camera_handle hCamera,
                             unsigned int IPAddress,
                             unsigned int SubnetMask);

typedef  SVGigE_RETURN
(*TFunc_Camera_forceValidNetworkSettings)(Camera_handle hCamera,
                                          unsigned int *IPAddress,
                                          unsigned int *SubnetMask);

typedef  SVGigE_RETURN
(*TFunc_Camera_restartIPConfiguration)(Camera_handle hCamera);

//-----------------------------------------------------------------------------------------
// 3 - Camera: Information
//-----------------------------------------------------------------------------------------

typedef const char *
(*TFunc_Camera_getManufacturerName)(Camera_handle hCamera);

typedef const char *
(*TFunc_Camera_getModelName)(Camera_handle hCamera);

typedef const char *
(*TFunc_Camera_getDeviceVersion)(Camera_handle hCamera);

typedef const char *
(*TFunc_Camera_getManufacturerSpecificInformation)(Camera_handle hCamera);

typedef const char *
(*TFunc_Camera_getSerialNumber)(Camera_handle hCamera);

typedef SVGigE_RETURN (*TFunc_Camera_setUserDefinedName)(Camera_handle hCamera, char *UserDefinedName);

typedef const char *
(*TFunc_Camera_getUserDefinedName)(Camera_handle hCamera);

typedef const char *
(*TFunc_Camera_getMacAddress)(Camera_handle hCamera);

typedef const char *
(*TFunc_Camera_getIPAddress)(Camera_handle hCamera);

typedef const char *
(*TFunc_Camera_getSubnetMask)(Camera_handle hCamera);

typedef SVGigE_RETURN
(*TFunc_Camera_getPixelClock)(Camera_handle hCamera,
                             int *PixelClock);

typedef bool
(*TFunc_Camera_isCameraFeature)(Camera_handle hCamera,
                                CAMERA_FEATURE Feature);

typedef SVGigE_RETURN
(*TFunc_Camera_readXMLFile)(Camera_handle hCamera,
                            char **XML);


typedef SVGigE_RETURN
(*TFunc_Camera_getSensorTemperature)(Camera_handle hCamera,
                                     unsigned int *SensorTemperature);


//-----------------------------------------------------------------------------------------
// 4 - Stream: Channel creation and control
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_create)(StreamingChannel_handle *hStreamingChannel,
                                 CameraContainerClient_handle hCameraContainer,
                                 Camera_handle hCamera,
                                 int BufferCount,
                                 StreamCallback CallbackFunction,
                                 void *Context);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_createEx)(StreamingChannel_handle *hStreamingChannel,
                                 CameraContainerClient_handle hCameraContainer,
                                 Camera_handle hCamera,
                                 int BufferCount,
								 int PacketResendTimeout,
                                 StreamCallback CallbackFunction,
                                 void *Context);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_delete)(StreamingChannel_handle hStreamingChannel);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_setReadoutTransfer)(StreamingChannel_handle hStreamingChannel,
                                             bool isReadoutTransfer);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getReadoutTransfer)(StreamingChannel_handle hStreamingChannel,
                                             bool *isReadoutTransfer);

//-----------------------------------------------------------------------------------------
// 5 - Stream: Channel statistics
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getFrameLoss)(StreamingChannel_handle hStreamingChannel,
                                       int *FrameLoss);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getActualFrameRate)(StreamingChannel_handle hStreamingChannel,
                                             float *ActualFrameRate);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getActualDataRate)(StreamingChannel_handle hStreamingChannel,
                                            float *ActualDataRate);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getPeakDataRate)(StreamingChannel_handle hStreamingChannel,
                                          float *PeakDataRate);

//-----------------------------------------------------------------------------------------
// 6 - Stream: Channel info
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getPixelType)(StreamingChannel_handle hStreamingChannel,
                                       GVSP_PIXEL_TYPE *ProgrammedPixelType);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getBufferData)(StreamingChannel_handle hStreamingChannel,
																			unsigned int BufferIndex,
																			unsigned char **BufferData);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getBufferSize)(StreamingChannel_handle hStreamingChannel,
                                        int *BufferSize);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getImagePitch)(StreamingChannel_handle hStreamingChannel,
                                        int *ImagePitch);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getImageSizeX)(StreamingChannel_handle hStreamingChannel,
                                        int *ImageSizeX);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getImageSizeY)(StreamingChannel_handle hStreamingChannel,
                                        int *ImageSizeY);

//-----------------------------------------------------------------------------------------
// 7 - Stream: Transfer parameters
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_evaluateMaximalPacketSize)(Camera_handle hCamera,
                                          int *MaximalPacketSize);

typedef SVGigE_RETURN
(*TFunc_Camera_setStreamingPacketSize)(Camera_handle hCamera,
                                       int StreamingPacketSize);

typedef SVGigE_RETURN
(*TFunc_Camera_setInterPacketDelay)(Camera_handle hCamera,
                                    float InterPacketDelay);

typedef SVGigE_RETURN
(*TFunc_Camera_getInterPacketDelay)(Camera_handle hCamera,
                                    float *ProgrammedInterPacketDelay);

typedef SVGigE_RETURN
(*TFunc_Camera_setMulticastMode)(Camera_handle hCamera,
                                 MULTICAST_MODE Multicast);

typedef SVGigE_RETURN
(*TFunc_Camera_getMulticastMode)(Camera_handle hCamera,
                                 MULTICAST_MODE *Multicast);

typedef SVGigE_RETURN
(*TFunc_Camera_getMulticastGroup)(Camera_handle hCamera,
                                  unsigned int *MulticastIP,
                                  unsigned int *MulticastPort);

//-----------------------------------------------------------------------------------------
// 8 - Stream: Image access
//-----------------------------------------------------------------------------------------

typedef unsigned char *
(*TFunc_Image_getDataPointer)(Image_handle hImage);

typedef int
(*TFunc_Image_getBufferIndex)(Image_handle hImage);

typedef SVGigE_SIGNAL_TYPE
(*TFunc_Image_getSignalType)(Image_handle hImage);

typedef Camera_handle
(*TFunc_Image_getCamera)(Image_handle hImage);

typedef SVGigE_RETURN
(*TFunc_Image_release)(Image_handle hImage);

//-----------------------------------------------------------------------------------------
// 9 - Stream: Image conversion
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Image_getImageRGB)(Image_handle hImage,
                           unsigned char *BufferRGB,
                           int BufferLength,
                           BAYER_METHOD BayerMethod);

typedef SVGigE_RETURN
(*TFunc_Image_getImageGray)(Image_handle hImage,
                            unsigned char *Buffer8bit,
                            int BufferLength);

typedef SVGigE_RETURN
(*TFunc_Image_getImage12bitAs8bit)(Image_handle hImage,
                                   unsigned char *Buffer8bit,
                                   int BufferLength);

typedef SVGigE_RETURN
(*TFunc_Image_getImage12bitAs16bit)(Image_handle hImage,
                                    unsigned char *Buffer16bit,
                                    int BufferLength);

typedef SVGigE_RETURN
(*TFunc_Image_getImage16bitAs8bit)(Image_handle hImage,
                                   unsigned char *Buffer8bit,
                                   int BufferLength);

//-----------------------------------------------------------------------------------------
// 10 - Stream: Image characteristics
//-----------------------------------------------------------------------------------------

typedef GVSP_PIXEL_TYPE
(*TFunc_Image_getPixelType)(Image_handle hImage);

typedef int
(*TFunc_Image_getImageSize)(Image_handle hImage);

typedef	int
(*TFunc_Image_getPitch)(Image_handle hImage);

typedef int
(*TFunc_Image_getSizeX)(Image_handle hImage);

typedef	int
(*TFunc_Image_getSizeY)(Image_handle hImage);

//-----------------------------------------------------------------------------------------
// 11 - Stream: Image statistics
//-----------------------------------------------------------------------------------------

typedef int
(*TFunc_Image_getImageID)(Image_handle hImage);

typedef double
(*TFunc_Image_getTimestamp)(Image_handle hImage);

typedef double
(*TFunc_Image_getTransferTime)(Image_handle hImage);

typedef int
(*TFunc_Image_getPacketCount)(Image_handle hImage);

typedef int
(*TFunc_Image_getPacketResend)(Image_handle hImage);

//-----------------------------------------------------------------------------------------
// 12 - Stream: Messaging channel
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Stream_createEvent)(StreamingChannel_handle hStreamingChannel,
                            Event_handle *EventID,
                            int SizeFIFO);

typedef SVGigE_RETURN
(*TFunc_Stream_addMessageType)(StreamingChannel_handle hStreamingChannel,
                               Event_handle EventID,
                               SVGigE_SIGNAL_TYPE MessageType);

typedef SVGigE_RETURN
(*TFunc_Stream_removeMessageType)(StreamingChannel_handle hStreamingChannel,
                                  Event_handle EventID,
                                  SVGigE_SIGNAL_TYPE MessageType);

typedef SVGigE_RETURN
(*TFunc_Stream_isMessagePending)(StreamingChannel_handle hStreamingChannel,
                                 Event_handle EventID,
                                 int Timeout_ms);

typedef SVGigE_RETURN
(*TFunc_Stream_registerEventCallback)(StreamingChannel_handle hStreamingChannel,
                                      Event_handle EventID,
                                      EventCallback Callback,
                                      void *Context);

typedef SVGigE_RETURN
(*TFunc_Stream_unregisterEventCallback)(StreamingChannel_handle hStreamingChannel,
                                        Event_handle EventID,
                                        EventCallback Callback);

typedef SVGigE_RETURN
(*TFunc_Stream_getMessage)(StreamingChannel_handle hStreamingChannel,
                           Event_handle EventID,
                           Message_handle *MessageID,
                           SVGigE_SIGNAL_TYPE *MessageType);

typedef SVGigE_RETURN
(*TFunc_Stream_getMessageData)(StreamingChannel_handle hStreamingChannel,
                               Event_handle EventID,
                               Message_handle MessageID,
                               void **MessageData,
                               int *MessageLength);

typedef SVGigE_RETURN
(*TFunc_Stream_getMessageTimestamp)(StreamingChannel_handle hStreamingChannel,
                                    Event_handle EventID,
                                    Message_handle MessageID,
                                    double *MessageTimestamp);

typedef SVGigE_RETURN
(*TFunc_Stream_releaseMessage)(StreamingChannel_handle hStreamingChannel,
                               Event_handle EventID,
                               Message_handle MessageID);

typedef SVGigE_RETURN
(*TFunc_Stream_flushMessages)(StreamingChannel_handle hStreamingChannel,
                              Event_handle EventID);

typedef SVGigE_RETURN
(*TFunc_Stream_closeEvent)(StreamingChannel_handle hStreamingChannel,
                           Event_handle EventID);


//-----------------------------------------------------------------------------------------
// 13 - Controlling camera: Frame rate
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_setFrameRate)(Camera_handle hCamera,
                             float Framerate);

typedef SVGigE_RETURN
(*TFunc_Camera_getFrameRate)(Camera_handle hCamera,
                             float *ProgrammedFramerate);

typedef SVGigE_RETURN
(*TFunc_Camera_getFrameRateMin)(Camera_handle hCamera,
                                float *MinFramerate);

typedef SVGigE_RETURN
(*TFunc_Camera_getFrameRateMax)(Camera_handle hCamera,
                                float *MaxFramerate);

typedef SVGigE_RETURN
(*TFunc_Camera_getFrameRateRange)(Camera_handle hCamera,
                                  float *MinFramerate,
                                  float *MaxFramerate);

typedef SVGigE_RETURN
(*TFunc_Camera_getFrameRateIncrement)(Camera_handle hCamera,
                                      float *FramerateIncrement);

//-----------------------------------------------------------------------------------------
// 14 - Controlling camera: Exposure
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_setExposureTime)(Camera_handle hCamera,
                                float ExposureTime);

typedef SVGigE_RETURN
(*TFunc_Camera_getExposureTime)(Camera_handle hCamera,
                                float *ProgrammedExposureTime);

typedef SVGigE_RETURN
(*TFunc_Camera_getExposureTimeMin)(Camera_handle hCamera,
                                   float *MinExposureTime);

typedef SVGigE_RETURN
(*TFunc_Camera_getExposureTimeMax)(Camera_handle hCamera,
                                   float *MaxExposureTime);

typedef SVGigE_RETURN
(*TFunc_Camera_getExposureTimeRange)(Camera_handle hCamera,
                                     float *MinExposureTime,
                                     float *MaxExposureTime);

typedef SVGigE_RETURN
(*TFunc_Camera_getExposureTimeIncrement)(Camera_handle hCamera,
                                         float *ExposureTimeIncrement);

typedef SVGigE_RETURN
(*TFunc_Camera_setExposureDelay)(Camera_handle hCamera,
                                 float ExposureDelay);

typedef SVGigE_RETURN
(*TFunc_Camera_getExposureDelay)(Camera_handle hCamera,
                                 float *ProgrammedExposureDelay);

typedef SVGigE_RETURN
(*TFunc_Camera_getExposureDelayMax)(Camera_handle hCamera,
                                    float *MaxExposureDelay);

typedef SVGigE_RETURN
(*TFunc_Camera_getExposureDelayIncrement)(Camera_handle hCamera,
                                          float *ExposureDelayIncrement);

//-----------------------------------------------------------------------------------------
// 15 - Controlling camera: Gain and offset
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_setGain)(Camera_handle hCamera,
                        float Gain);

typedef SVGigE_RETURN
(*TFunc_Camera_getGain)(Camera_handle hCamera,
                        float *ProgrammedGain);

typedef SVGigE_RETURN
(*TFunc_Camera_getGainMax)(Camera_handle hCamera,
                           float *MaxGain);

typedef SVGigE_RETURN
(*TFunc_Camera_getGainMaxExtended)(Camera_handle hCamera,
                                   float *MaxGainExtended);

typedef SVGigE_RETURN
(*TFunc_Camera_getGainIncrement)(Camera_handle hCamera,
                                 float *GainIncrement);

typedef SVGigE_RETURN
(*TFunc_Camera_setOffset)(Camera_handle hCamera,
                         float Offset);

typedef SVGigE_RETURN
(*TFunc_Camera_getOffset)(Camera_handle hCamera,
                          float *Offset);

typedef SVGigE_RETURN
(*TFunc_Camera_getOffsetMax)(Camera_handle hCamera,
                             float *MaxOffset);

//-----------------------------------------------------------------------------------------
// 16 - Controlling camera: Auto gain/exposure
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_setAutoGainEnabled)(Camera_handle Camera,
														       bool isAutoGainEnabled);

typedef SVGigE_RETURN
(*TFunc_Camera_getAutoGainEnabled)(Camera_handle Camera,
														       bool *isAutoGainEnabled);

typedef SVGigE_RETURN
(*TFunc_Camera_setAutoGainBrightness)(Camera_handle Camera,
                                      float Brightness);

typedef SVGigE_RETURN
(*TFunc_Camera_getAutoGainBrightness)(Camera_handle Camera,
                                      float *ProgrammedBrightness);

typedef SVGigE_RETURN
(*TFunc_Camera_setAutoGainDynamics)(Camera_handle Camera,
                                    float AutoGainParameterI,
                                    float AutoGainParameterD);

typedef SVGigE_RETURN
(*TFunc_Camera_getAutoGainDynamics)(Camera_handle Camera,
                                    float *ProgrammedAutoGainParameterI,
                                    float *ProgrammedAutoGainParameterD);

typedef SVGigE_RETURN
(*TFunc_Camera_setAutoGainLimits)(Camera_handle Camera,
                                  float MinGain,
                                  float MaxGain);

typedef SVGigE_RETURN
(*TFunc_Camera_getAutoGainLimits)(Camera_handle Camera,
                                  float *ProgrammedMinGain,
                                  float *ProgrammedMaxGain);

typedef SVGigE_RETURN
(*TFunc_Camera_setAutoExposureLimits)(Camera_handle Camera,
                                      float MinExposure,
                                      float MaxExposure);

typedef SVGigE_RETURN
(*TFunc_Camera_getAutoExposureLimits)(Camera_handle Camera,
                                      float *ProgrammedMinExposure,
                                      float *ProgrammedMaxExposure);

//-----------------------------------------------------------------------------------------
// 17 - Controlling camera: Acquisition trigger
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_setAcquisitionControl)(Camera_handle hCamera,
                                      ACQUISITION_CONTROL AcquisitionControl);

typedef SVGigE_RETURN
(*TFunc_Camera_getAcquisitionControl)(Camera_handle hCamera,
                                      ACQUISITION_CONTROL *ProgrammedAcquisitionControl);

typedef SVGigE_RETURN
(*TFunc_Camera_setAcquisitionMode)(Camera_handle hCamera,
                                  ACQUISITION_MODE AcquisitionMode);

typedef SVGigE_RETURN
(*TFunc_Camera_setAcquisitionModeAndStart)(Camera_handle hCamera,
                                           ACQUISITION_MODE AcquisitionMode,
                                           bool AcquisitionStart);

typedef SVGigE_RETURN
(*TFunc_Camera_getAcquisitionMode)(Camera_handle hCamera,
                                   ACQUISITION_MODE *ProgrammedAcquisitionMode);

typedef SVGigE_RETURN
(*TFunc_Camera_softwareTrigger)(Camera_handle hCamera);

typedef SVGigE_RETURN
(*TFunc_Camera_softwareTriggerID)(Camera_handle hCamera,
                                  int TriggerID);

typedef SVGigE_RETURN
(*TFunc_Camera_softwareTriggerIDEnable)(Camera_handle hCamera,
                                        bool TriggerIDEnable);

typedef SVGigE_RETURN
(*TFunc_Camera_setTriggerPolarity)(Camera_handle hCamera,
                                   TRIGGER_POLARITY TriggerPolarity);

typedef SVGigE_RETURN
(*TFunc_Camera_getTriggerPolarity)(Camera_handle hCamera,
                                   TRIGGER_POLARITY *ProgrammedTriggerPolarity);
//--------------Piv Mode-------------------
typedef SVGigE_RETURN
(*TFunc_Camera_setPivMode)(Camera_handle hCamera,
                           PIV_MODE  SelectPivMode);

typedef SVGigE_RETURN
(*TFunc_Camera_getPivMode)(Camera_handle hCamera,
                           PIV_MODE *ProgrammedPivMode);

//--------------Debouncer-------------------
typedef SVGigE_RETURN
(*TFunc_Camera_setDebouncerDuration)(Camera_handle hCamera,
                                     float  DebouncerDuration);

typedef SVGigE_RETURN
(*TFunc_Camera_getDebouncerDuration)(Camera_handle hCamera,
                                     float *ProgrammedDuration);

//--------------prescaler-------------------
typedef SVGigE_RETURN
(*TFunc_Camera_setPrescalerDevisor)(Camera_handle hCamera,
                                    unsigned int  PrescalerDevisor);

typedef SVGigE_RETURN
(*TFunc_Camera_getPrescalerDevisor)(Camera_handle hCamera,
                                    unsigned int *ProgrammedPrescalerDevisor);

//--------------Sequencer-------------------
typedef SVGigE_RETURN
(*TFunc_Camera_loadSequenceParameters)(Camera_handle hCamera,
                                       const char *Filename);

typedef SVGigE_RETURN
(*TFunc_Camera_startSequencer)(Camera_handle hCamera);


//-----------------------------------------------------------------------------------------
// 18 - Controlling camera: Strobe
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_setStrobePolarity)(Camera_handle hCamera,
                                  STROBE_POLARITY StrobePolarity);

typedef SVGigE_RETURN
(*TFunc_Camera_setStrobePolarityExtended)(Camera_handle hCamera,
                                  STROBE_POLARITY StrobePolarity, int StrobeIndex);

typedef SVGigE_RETURN
(*TFunc_Camera_getStrobePolarity)(Camera_handle hCamera,
                                  STROBE_POLARITY *ProgrammedStrobePolarity);

typedef SVGigE_RETURN
(*TFunc_Camera_getStrobePolarityExtended)(Camera_handle hCamera,
                                  STROBE_POLARITY *ProgrammedStrobePolarity, int StrobeIndex );

typedef SVGigE_RETURN
(*TFunc_Camera_setStrobePosition)(Camera_handle hCamera,
                                  float StrobePosition);

typedef SVGigE_RETURN
(*TFunc_Camera_setStrobePositionExtended)(Camera_handle hCamera,
                                  float StrobePosition, int StrobeIndex);

typedef SVGigE_RETURN
(*TFunc_Camera_getStrobePosition)(Camera_handle hCamera,
                                  float *ProgrammedStrobePosition);

typedef SVGigE_RETURN
(*TFunc_Camera_getStrobePositionExtended)(Camera_handle hCamera,
                                  float *ProgrammedStrobePosition,int StrobeIndex );

typedef SVGigE_RETURN
(*TFunc_Camera_getStrobePositionMax)(Camera_handle hCamera,
                                     float *MaxStrobePosition);

typedef SVGigE_RETURN
(*TFunc_Camera_getStrobePositionIncrement)(Camera_handle hCamera,
                                           float *StrobePositionIncrement);

typedef SVGigE_RETURN
(*TFunc_Camera_setStrobeDuration)(Camera_handle hCamera,
                                  float StrobeDuration);

typedef SVGigE_RETURN
(*TFunc_Camera_setStrobeDurationExtended)(Camera_handle hCamera,
                                  float StrobeDuration,int StrobeIndex);

typedef SVGigE_RETURN
(*TFunc_Camera_getStrobeDuration)(Camera_handle hCamera,
                                  float *ProgrammedStrobeDuration);

typedef SVGigE_RETURN
(*TFunc_Camera_getStrobeDurationExtended)(Camera_handle hCamera,
                                          float *ProgrammedStrobeDuration,int StrobeIndex);

typedef SVGigE_RETURN
(*TFunc_Camera_getStrobeDurationMax)(Camera_handle hCamera,
                                     float *MaxStrobeDuration);

typedef SVGigE_RETURN
(*TFunc_Camera_getStrobeDurationIncrement)(Camera_handle hCamera,
                                           float *StrobeDurationIncrement);

//-----------------------------------------------------------------------------------------
// 19 - Controlling camera: Tap balance
//-----------------------------------------------------------------------------------------



typedef SVGigE_RETURN
(*TFunc_Camera_setTapConfiguration)(Camera_handle hCamera,
                                    int TapCount);

typedef SVGigE_RETURN
(*TFunc_Camera_getTapConfiguration)(Camera_handle hCamera,
                                    int *TapCount);

typedef SVGigE_RETURN
(*TFunc_Camera_setTapConfigurationEx)(Camera_handle hCamera,
                                    SVGIGE_TAP_CONFIGURATION_SELECT SelectedTapConfig);

typedef SVGigE_RETURN
(*TFunc_Camera_getTapConfigurationEx)(Camera_handle hCamera,
                                    SVGIGE_TAP_CONFIGURATION_SELECT *ProgrammedTapConfig);


typedef SVGigE_RETURN
(*TFunc_Camera_setAutoTapBalanceMode)(Camera_handle hCamera,
                                      SVGIGE_AUTO_TAP_BALANCE_MODE AutoTapBalanceMode);

typedef SVGigE_RETURN
(*TFunc_Camera_getAutoTapBalanceMode)(Camera_handle hCamera,
                                      SVGIGE_AUTO_TAP_BALANCE_MODE *AutoTapBalanceMode);


typedef SVGigE_RETURN
(*TFunc_Camera_setTapGain)(Camera_handle hCamera,
													 float TapGain,
 													 SVGIGE_TAP_SELECT TapSelect);

typedef SVGigE_RETURN
(*TFunc_Camera_getTapGain)(Camera_handle hCamera,
													 float *TapGain,
													 SVGIGE_TAP_SELECT TapSelect);

//-----------------------------------------------------------------------------------------
// 20 - Controlling camera: Image parameter
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_getImagerWidth)(Camera_handle hCamera,
                               int *ImagerWidth);

typedef SVGigE_RETURN
(*TFunc_Camera_getImagerHeight)(Camera_handle hCamera,
                                int *ImagerHeight);

typedef SVGigE_RETURN
(*TFunc_Camera_getImageSize)(Camera_handle hCamera,
                             int *ImageSize);

typedef SVGigE_RETURN
(*TFunc_Camera_getPitch)(Camera_handle hCamera,
                         int *Pitch);

typedef SVGigE_RETURN
(*TFunc_Camera_getSizeX)(Camera_handle hCamera,
                         int *SizeX);

typedef SVGigE_RETURN
(*TFunc_Camera_getSizeY)(Camera_handle hCamera,
                         int *SizeY);

typedef SVGigE_RETURN
(*TFunc_Camera_setBinningMode)(Camera_handle hCamera,
                               BINNING_MODE BinningMode);

typedef SVGigE_RETURN
(*TFunc_Camera_getBinningMode)(Camera_handle hCamera,
                              BINNING_MODE *ProgrammedBinningMode);

typedef SVGigE_RETURN
(*TFunc_Camera_setAreaOfInterest)(Camera_handle hCamera,
                                  int SizeX,
                                  int SizeY,
                                  int OffsetX,
                                  int OffsetY);

typedef SVGigE_RETURN
(*TFunc_Camera_getAreaOfInterest)(Camera_handle hCamera,
                                  int *ProgrammedSizeX,
                                  int *ProgrammedSizeY,
                                  int *ProgrammedOffsetX,
                                  int *ProgrammedOffsetY);

typedef SVGigE_RETURN
(*TFunc_Camera_getAreaOfInterestRange)(Camera_handle hCamera,
                                       int *MinSizeX,
                                       int *MinSizeY,
                                       int *MaxSizeX,
                                       int *MaxSizeY);

typedef SVGigE_RETURN
(*TFunc_Camera_getAreaOfInterestIncrement)(Camera_handle hCamera,
                                           int *SizeXIncrement,
                                           int *SizeYIncrement,
                                           int *OffsetXIncrement,
                                           int *OffsetYIncrement);

typedef SVGigE_RETURN
(*TFunc_Camera_resetTimestampCounter)(Camera_handle hCamera);

typedef SVGigE_RETURN
(*TFunc_Camera_getTimestampCounter)(Camera_handle hCamera,
                                    double *TimestampCounter);

typedef SVGigE_RETURN
(*TFunc_Camera_getTimestampTickFrequency)(Camera_handle hCamera,
                                          double *TimestampCounter);

typedef SVGigE_RETURN
(*TFunc_Camera_setFlippingMode)(Camera_handle hCamera,
                        SVGIGE_FLIPPING_MODE  FlippingMode);

 typedef SVGigE_RETURN
(*TFunc_Camera_getFlippingMode)(Camera_handle hCamera,
                        SVGIGE_FLIPPING_MODE *ProgrammedFlippingMode);

  typedef SVGigE_RETURN
(*TFunc_Camera_setShutterMode)(Camera_handle hCamera,
                        SVGIGE_SHUTTER_MODE  ShutterMode);

   typedef SVGigE_RETURN
(*TFunc_Camera_getShutterMode)(Camera_handle hCamera,
                        SVGIGE_SHUTTER_MODE *ProgrammedShutterMode);
//-----------------------------------------------------------------------------------------
// 21 - Controlling camera: Image appearance
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_getPixelType)(Camera_handle hCamera,
                             GVSP_PIXEL_TYPE *PixelType);

typedef SVGigE_RETURN
(*TFunc_Camera_setPixelDepth)(Camera_handle hCamera,
                              SVGIGE_PIXEL_DEPTH PixelDepth);

typedef SVGigE_RETURN
(*TFunc_Camera_getPixelDepth)(Camera_handle hCamera,
                              SVGIGE_PIXEL_DEPTH *PixelDepth);

typedef SVGigE_RETURN
(*TFunc_Camera_setWhiteBalance)(Camera_handle hCamera,
                                float Red,
                                float Green ,
                                float Blue);

typedef SVGigE_RETURN
(*TFunc_Camera_getWhiteBalance)(Camera_handle hCamera,
                                float *Red,
                                float *Green ,
                                float *Blue);

typedef SVGigE_RETURN
(*TFunc_Camera_getWhiteBalanceMax)(Camera_handle hCamera,
                                   float *WhiteBalanceMax);

typedef SVGigE_RETURN
(*TFunc_Camera_setGammaCorrection)(Camera_handle hCamera,
  													       float GammaCorrection);

typedef SVGigE_RETURN
(*TFunc_Camera_setGammaCorrectionExt)(Camera_handle hCamera,
  													          float GammaCorrection,
                                      float DigitalGain,
                                      float DigitalOffset);

typedef SVGigE_RETURN
(*TFunc_Camera_setLowpassFilter)(Camera_handle hCamera,
  													     LOWPASS_FILTER LowpassFilter);

typedef SVGigE_RETURN
(*TFunc_Camera_getLowpassFilter)(Camera_handle hCamera,
  													     LOWPASS_FILTER *ProgrammedLowpassFilter);

typedef SVGigE_RETURN
(*TFunc_Camera_setLookupTableMode)(Camera_handle hCamera,
                                   LUT_MODE LUTMode);

typedef SVGigE_RETURN
(*TFunc_Camera_getLookupTableMode)(Camera_handle hCamera,
                                   LUT_MODE *ProgrammedLUTMode);

typedef SVGigE_RETURN
(*TFunc_Camera_setLookupTable)(Camera_handle hCamera,
                               unsigned char *LookupTable,
                               int LookupTableSize);

typedef SVGigE_RETURN
(*TFunc_Camera_getLookupTable)(Camera_handle hCamera,
                               unsigned char *LookupTable,
                               int LookupTableSize);

typedef SVGigE_RETURN
(*TFunc_Camera_startImageCorrection)(Camera_handle hCamera,
									IMAGE_CORRECTION_STEP ImageCorrectionStep);

typedef SVGigE_RETURN
(*TFunc_Camera_isIdleImageCorrection)(Camera_handle hCamera,
									 IMAGE_CORRECTION_STEP *ProgrammedImageCorrectionStep,
			 						 bool *isIdle);

typedef SVGigE_RETURN
(*TFunc_Camera_setImageCorrection)(Camera_handle hCamera,
								IMAGE_CORRECTION_MODE ImageCorrectionMode);

typedef SVGigE_RETURN
(*TFunc_Camera_getImageCorrection)(Camera_handle hCamera,
								 IMAGE_CORRECTION_MODE *ProgrammedImageCorrectionMode);


typedef SVGigE_RETURN
(*TFunc_Camera_setPixelsCorrectionMap)(Camera_handle hCamera,
								  PIXELS_CORRECTION_MAP_SELECT PixelsCorrectionMap);

 typedef SVGigE_RETURN
(*TFunc_Camera_getPixelsCorrectionMap)(Camera_handle hCamera,
								PIXELS_CORRECTION_MAP_SELECT * ProgrammedPixelsCorrectionMap);




typedef SVGigE_RETURN
(*TFunc_Camera_setPixelsCorrectionControlEnabel)(Camera_handle hCamera,
									bool isPixelsCorrectionEnabled);



typedef SVGigE_RETURN
(*TFunc_Camera_getPixelsCorrectionControlEnabel)(Camera_handle hCamera,
									bool *isPixelsCorrectionEnabled);


 typedef SVGigE_RETURN
(*TFunc_Camera_setPixelsCorrectionControlMark)(Camera_handle hCamera,
									bool isPixelsCorrectionMarked);



typedef SVGigE_RETURN
(*TFunc_Camera_getPixelsCorrectionControlMark)(Camera_handle hCamera,
									bool *isPixelsCorrectionMarked);


typedef SVGigE_RETURN
(*TFunc_Camera_setPixelsCorrectionMapOffset)(Camera_handle hCamera,
										 int  OffsetX,   int  OffsetY);


typedef SVGigE_RETURN
(*TFunc_Camera_getPixelsCorrectionMapOffset)(Camera_handle hCamera,
									  int *ProgrammedOffsetX,  int *ProgrammedOffsetY);


 typedef SVGigE_RETURN
(*TFunc_Camera_getPixelsCorrectionMapSize)(Camera_handle hCamera,
									 unsigned int *programmedMapSize);


 typedef SVGigE_RETURN
(*TFunc_Camera_getMaximalPixelsCorrectionMapSize)(Camera_handle hCamera,
											 unsigned int *MaximalprogrammedMapSize);


 typedef SVGigE_RETURN
(*TFunc_Camera_setMapIndexCoordinate)(Camera_handle hCamera,
									unsigned int MapIndex,
									unsigned int CoordinateX, unsigned int CoordinateY );


 typedef SVGigE_RETURN
(*TFunc_Camera_getMapIndexCoordinate)(Camera_handle hCamera,
									unsigned int MapIndex,
								unsigned int *ProgrammedCoordinateX, unsigned int *ProgrammedCoordinateY );


 typedef SVGigE_RETURN
(*TFunc_Camera_deletePixelCoordinateFromMap)(Camera_handle hCamera, unsigned int MapIndex);

//-----------------------------------------------------------------------------------------
// 22 - Special control: IOMux configuration
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_getMaxIOMuxIN)(Camera_handle hCamera,
                              int *MaxIOMuxINSignals);

typedef SVGigE_RETURN
(*TFunc_Camera_getMaxIOMuxOUT)(Camera_handle hCamera,
                               int *MaxIOMuxOUTSignals);

typedef SVGigE_RETURN
(*TFunc_Camera_setIOAssignment)(Camera_handle hCamera,
                                SVGigE_IOMux_OUT IOMuxOUT,
                                unsigned int SignalIOMuxIN);

typedef SVGigE_RETURN
(*TFunc_Camera_getIOAssignment)(Camera_handle hCamera,
                                SVGigE_IOMux_OUT IOMuxOUT,
                                unsigned int *ProgrammedIOMuxIN);

//-----------------------------------------------------------------------------------------
// 23 - Special control: IO control
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_setIOMuxIN)(Camera_handle hCamera,
                           unsigned int VectorIOMuxIN);

typedef SVGigE_RETURN
(*TFunc_Camera_getIOMuxIN)(Camera_handle hCamera,
                           unsigned int *ProgrammedVectorIOMuxIN);

typedef SVGigE_RETURN
(*TFunc_Camera_setIO)(Camera_handle hCamera,
                      SVGigE_IOMux_IN SignalIOMuxIN,
                      IO_SIGNAL SignalValue);

typedef SVGigE_RETURN
(*TFunc_Camera_getIO)(Camera_handle hCamera,
                      SVGigE_IOMux_IN SignalIOMuxIN,
                      IO_SIGNAL *ProgrammedSignalValue);

typedef SVGigE_RETURN
(*TFunc_Camera_setAcqLEDOverride)(Camera_handle hCamera,
                                  bool isOverrideActive);

typedef SVGigE_RETURN
(*TFunc_Camera_getAcqLEDOverride)(Camera_handle hCamera,
                                  bool *isOverrideActive);

typedef SVGigE_RETURN
(*TFunc_Camera_setLEDIntensity)(Camera_handle hCamera,
                                int LEDIntensity);

typedef SVGigE_RETURN
(*TFunc_Camera_getLEDIntensity)(Camera_handle hCamera,
                                int *ProgrammedLEDIntensity);

//-----------------------------------------------------------------------------------------
// 24 - Special control: Serial communication
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_setUARTBuffer)(Camera_handle hCamera,
                              unsigned char *Data,
                              int DataLength);

typedef SVGigE_RETURN
(*TFunc_Camera_getUARTBuffer)(Camera_handle hCamera,
                              unsigned char *Data,
                              int *DataLengthReceived,
                              int DataLengthMax,
                              float Timeout);

typedef SVGigE_RETURN
(*TFunc_Camera_setUARTBaud)(Camera_handle hCamera,
                            SVGigE_BaudRate BaudRate);

typedef SVGigE_RETURN
(*TFunc_Camera_getUARTBaud)(Camera_handle hCamera,
                            SVGigE_BaudRate *ProgrammedBaudRate);

//-----------------------------------------------------------------------------------------
// 25 - Special control: Direct register and memory access
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_setGigECameraRegister)(Camera_handle hCamera,
                                      unsigned int RegisterAddress,
                                      unsigned int RegisterValue,
                                      unsigned int GigECameraAccessKey);

typedef SVGigE_RETURN
(*TFunc_Camera_getGigECameraRegister)(Camera_handle hCamera,
                                      unsigned int RegisterAddress,
                                      unsigned int *ProgrammedRegisterValue,
                                      unsigned int GigECameraAccessKey);

typedef SVGigE_RETURN
(*TFunc_Camera_writeGigECameraMemory)(Camera_handle hCamera,
                                      unsigned int  MemoryAddress,
                                      unsigned char *DataBlock,
                                      unsigned int  DataLength,
                                      unsigned int  GigECameraAccessKey);

typedef SVGigE_RETURN
(*TFunc_Camera_readGigECameraMemory)(Camera_handle hCamera,
                                     unsigned int  MemoryAddress,
                                     unsigned char *DataBlock,
                                     unsigned int  DataLength,
                                     unsigned int  GigECameraAccessKey);

typedef SVGigE_RETURN
(*TFunc_Camera_forceOpenConnection)(Camera_handle hCamera, float Timeout);

//-----------------------------------------------------------------------------------------
// 26 - Special control: Persistent settings and recovery
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_writeEEPROM)(Camera_handle hCamera);

typedef SVGigE_RETURN
(*TFunc_Camera_readEEPROM)(Camera_handle hCamera);

typedef SVGigE_RETURN
(*TFunc_Camera_restoreFactoryDefaults)(Camera_handle hCamera);

typedef SVGigE_RETURN
(*TFunc_Camera_loadSettingsFromXml)(Camera_handle hCamera,
                                    const char *Filename);

typedef SVGigE_RETURN
(*TFunc_Camera_SaveSettingsToXml)(Camera_handle hCamera,
                                  const char *Filename);

//-----------------------------------------------------------------------------------------
// 27 - General functions
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_SVGigE_estimateWhiteBalance)(unsigned char *DataRGB, int DataRGBLength, float *Red, float *Green, float *Blue);

typedef SVGigE_RETURN
(*TFunc_SVGigE_estimateWhiteBalanceExtended)(unsigned char *DataRGB, int PixelNumber, int &Red, int &Green, int &Blue,SVGIGE_Whitebalance_SELECT  Whitebalance_Art );

typedef SVGigE_RETURN
(*TFunc_SVGigE_writeImageToBitmapFile)(const char *Filename, unsigned char *Data, int SizeX, int SizeY, GVSP_PIXEL_TYPE PixelType);

typedef SVGigE_RETURN
(*TFunc_SVGigE_installFilterDriver)( const char *PathToDriverPackage, const char *FilenameINF, const char *FilenameINF_m);

typedef SVGigE_RETURN
(*TFunc_SVGigE_uninstallFilterDriver)();

//-----------------------------------------------------------------------------------------
// 28 - Diagnostics
//-----------------------------------------------------------------------------------------

typedef char *
(*TFunc_Error_getMessage)(SVGigE_RETURN ReturnCode);

typedef SVGigE_RETURN
(*TFunc_Camera_registerForLogMessages)(Camera_handle hCamera,
                                       int  LogLevel,
                                       const char *LogFilename,
                                       LogMessageCallback LogCallback,
                                       void *MessageContext);


//-----------------------------------------------------------------------------------------
// 29 - Special control: Lens control
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_isLensAvailable)(Camera_handle hCamera, bool *isAvailable);

typedef char*
(*TFunc_Camera_getLensName)(Camera_handle hCamera);

//----------------------------------------FocalLenght--------------------------------------
typedef SVGigE_RETURN
(*TFunc_Camera_setLensFocalLenght)(Camera_handle hCamera, unsigned int ProgrammedFocalLenght);

typedef SVGigE_RETURN
(*TFunc_Camera_getLensFocalLenght)(Camera_handle hCamera, unsigned int *ProgrammedFocalLenght);

typedef SVGigE_RETURN
(*TFunc_Camera_getLensFocalLenghtMin)(Camera_handle hCamera, unsigned int *FocalLenghtMin);

typedef SVGigE_RETURN
(*TFunc_Camera_getLensFocalLenghtMax)(Camera_handle hCamera, unsigned int *FocalLenghtMax);


//----------------------------------------Focus------------------------------------------


typedef SVGigE_RETURN
(*TFunc_Camera_setLensFocusUnit)(Camera_handle hCamera, FOCUS_UNIT Selected_unit );

typedef SVGigE_RETURN
(*TFunc_Camera_getLensFocusUnit)(Camera_handle hCamera, FOCUS_UNIT *Selected_unit );

typedef SVGigE_RETURN
(*TFunc_Camera_setLensFocus)(Camera_handle hCamera, unsigned int ProgrammedFocus);

typedef SVGigE_RETURN
(*TFunc_Camera_getLensFocus)(Camera_handle hCamera, unsigned int *ProgrammedFocus);

typedef SVGigE_RETURN
(*TFunc_Camera_getLensFocusMin)(Camera_handle hCamera, unsigned int *FocusMin);

typedef SVGigE_RETURN
(*TFunc_Camera_getLensFocusMax)(Camera_handle hCamera, unsigned int *FocusMax);


//----------------------------------------Aperture------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_setLensAperture)(Camera_handle hCamera, unsigned int ProgrammedAperture);

typedef SVGigE_RETURN
(*TFunc_Camera_getLensAperture)(Camera_handle hCamera, unsigned int *ProgrammedAperture);

typedef SVGigE_RETURN
(*TFunc_Camera_getLensApertureMin)(Camera_handle hCamera, unsigned int *ApertureMin);

typedef SVGigE_RETURN
(*TFunc_Camera_getLensApertureMax)(Camera_handle hCamera, unsigned int *ApertureMax);


//-----------------------------------------------------------------------------------------
// 99 - Deprecated functions
//-----------------------------------------------------------------------------------------

typedef SVGigE_RETURN
(*TFunc_Camera_startAcquisitionCycle)(Camera_handle hCamera);

typedef SVGigE_RETURN
(*TFunc_Camera_setTapCalibration)(Camera_handle hCamera,
                                  unsigned int TapID,
                                  unsigned int Gain,
                                  unsigned int Offset);

typedef SVGigE_RETURN
(*TFunc_Camera_getTapCalibration)(Camera_handle hCamera,
                                  unsigned int TapID,
                                  unsigned int *Gain,
                                  unsigned int *Offset);

typedef SVGigE_RETURN
(*TFunc_Camera_setLUTMode)(Camera_handle hCamera,
                           LUT_MODE LUTMode);

typedef SVGigE_RETURN
(*TFunc_Camera_getLUTMode)(Camera_handle hCamera,
                           LUT_MODE *ProgrammedLUTMode);

typedef SVGigE_RETURN
(*TFunc_Camera_createLUTwhiteBalance)(Camera_handle hCamera,
                                      float Red,
                                      float Green ,
                                      float Blue);

typedef SVGigE_RETURN
(*TFunc_Camera_stampTimestamp)(Camera_handle hCamera,
                               int TimestampIndex);

typedef SVGigE_RETURN
(*TFunc_Camera_getTimestamp)(Camera_handle hCamera,
                             int TimestampIndex,
                             double *Timestamp);

typedef unsigned
(*TFunc_Image_getDebugInfo)(Image_handle hImage,
							int Index);

typedef SVGigE_RETURN
(*TFunc_Camera_setTapBalance)(Camera_handle hCamera,
                              float TapBalance);

typedef SVGigE_RETURN
(*TFunc_Camera_getTapBalance)(Camera_handle hCamera,
                              float *TapBalance);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_setChannelTimeout)(StreamingChannel_handle hStreamingChannel,
                                            float ChannelTimeout);

typedef SVGigE_RETURN
(*TFunc_StreamingChannel_getChannelTimeout)(StreamingChannel_handle hStreamingChannel,
                                            float *ProgrammedChannelTimeout);

typedef SVGigE_RETURN
(*TFunc_Camera_setTapUserSettings)(Camera_handle hCamera,
									float TapUserGain,
									float TapUserOffset);

typedef SVGigE_RETURN
(*TFunc_Camera_getTapUserSettings)(Camera_handle hCamera,
									float *TapUserGain,
									float *TapUserOffset);
typedef SVGigE_RETURN
(*TFunc_Camera_saveTapBalanceSettings)(Camera_handle hCamera,
                                       const char *Filename);

typedef SVGigE_RETURN
(*TFunc_Camera_loadTapBalanceSettings)(Camera_handle hCamera,
                                       const char *Filename);

//--------------------End-of-function-type-declarations------------------------------------


//*****************************************************************************************
// 0 - GigE DLL (implicitly called)
//*****************************************************************************************

bool isLoadedGigEDLL()
{

    // Try to load GigE DLL
    GigEDLL = LoadLibrary(SVGigE_DLL);

    // Check DLL availability
    if( NULL == GigEDLL )
      return false;


	// Check if size of function table matches the number of imported functions
	int FunctionCount = sizeof(GigEFunc) / sizeof(struct _GigEFunc);
	if( FunctionCount != Func_isVersionCompliantDLL_consistency_check + 1 )
		return false;

  // Obtain CameraContainer procedure addresses
  bool function_loaded = true;
  for( int function_index = Func_isVersionCompliantDLL;
       function_index < (sizeof(GigEFunc) / sizeof(struct _GigEFunc));
       function_index++
     )
  {
    GigEFunc[function_index].function_pointer = GetProcAddress(GigEDLL, GigEFunc[function_index].function_name);

    // Check if function was found
    if( NULL == GigEFunc[function_index].function_pointer )
      function_loaded = false;
  }

  // Check if all function pointers could successfully be obtained from the DLL
  if( function_loaded == false )
    return false;
  else
    return true;
}

SVGigE_RETURN
isVersionCompliantDLL(SVGigE_VERSION *DllVersion,
                      SVGigE_VERSION *ExpectedVersion)
{
  // Check DLL availability
  if( NULL == GigEDLL )
  {
    // Try to load SVGigE DLL
    if( !isLoadedGigEDLL() )
      return SVGigE_DLL_NOT_LOADED;
  }

  // Pass through function call to DLL
	//
	// 2011-08-22/EKantz: check consistency of the whole function pointer
	//                    table by calling the last function in that table.
	//
  return ((TFunc_isVersionCompliantDLL)
  GigEFunc[Func_isVersionCompliantDLL_consistency_check].function_pointer)(DllVersion, ExpectedVersion);
}

SVGigE_RETURN
isDriverAvailable()
{
  // Check DLL availability
  if( NULL == GigEDLL )
  {
    // Try to load SVGigE DLL
    if( !isLoadedGigEDLL() )
      return SVGigE_DLL_NOT_LOADED;
  }

  // Pass through function call to DLL
  return ((TFunc_isDriverAvailable)
  GigEFunc[Func_isDriverAvailable].function_pointer)();
}

//*****************************************************************************************
// 1 - Camera: Discovery and bookkeeping
//*****************************************************************************************

CameraContainerClient_handle
CameraContainer_create(SVGigETL_Type TransportLayerType)
{
  // Load DLL
  if( !isLoadedGigEDLL() )
    return SVGigE_NO_CLIENT;

  // Check DLL version against version at compile time
  SVGigE_VERSION DllVersion;
  SVGigE_VERSION ExpectedVersion;
  ExpectedVersion.MajorVersion  = SVGigE_VERSION_MAJOR;
  ExpectedVersion.MinorVersion  = SVGigE_VERSION_MINOR;
  ExpectedVersion.DriverVersion = SVGigE_VERSION_DRIVER;
  ExpectedVersion.BuildVersion  = SVGigE_VERSION_BUILD;
  if( SVGigE_SUCCESS != isVersionCompliantDLL(&DllVersion, &ExpectedVersion) )
    return SVGigE_NO_CLIENT;

  // Pass through function call to DLL
  return ((TFunc_CameraContainer_create)
  GigEFunc[Func_CameraContainer_create].function_pointer)(TransportLayerType);
}

SVGigE_RETURN
CameraContainer_delete(CameraContainerClient_handle hCameraContainer)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  SVGigE_RETURN Ret =
  ((TFunc_CameraContainer_delete)
  GigEFunc[Func_CameraContainer_delete].function_pointer)(hCameraContainer);

  // Release DLL (reference counter will be decreased)
  FreeLibrary(GigEDLL);

	return Ret;
}

SVGigE_RETURN
CameraContainer_discovery(CameraContainerClient_handle hCameraContainer)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_CameraContainer_discovery)
    GigEFunc[Func_CameraContainer_discovery].function_pointer)(hCameraContainer);
}

int
CameraContainer_getNumberOfCameras(CameraContainerClient_handle hCameraContainer)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_CameraContainer_getNumberOfCameras)
  GigEFunc[Func_CameraContainer_getNumberOfCameras].function_pointer)(hCameraContainer);
}

Camera_handle
CameraContainer_getCamera(CameraContainerClient_handle hCameraContainer,
                          int CameraIndex)
{
  // Check DLL availablility
  if( NULL == GigEDLL )
    return SVGigE_NO_CAMERA;

  // Pass through function call to DLL
  return ((TFunc_CameraContainer_getCamera)
  GigEFunc[Func_CameraContainer_getCamera].function_pointer)(hCameraContainer, CameraIndex);
}

Camera_handle
CameraContainer_findCamera(CameraContainerClient_handle hCameraContainer,
                           char *CameraItem)
{
  // Check DLL availablility
  if( NULL == GigEDLL )
    return SVGigE_NO_CAMERA;

  // Pass through function call to DLL
  return ((TFunc_CameraContainer_findCamera)
  GigEFunc[Func_CameraContainer_findCamera].function_pointer)(hCameraContainer, CameraItem);
}

//*****************************************************************************************
// 2 - Camera: Connection
//*****************************************************************************************

SVGigE_RETURN
Camera_openConnection(Camera_handle hCamera, float Timeout)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_openConnection)
  GigEFunc[Func_Camera_openConnection].function_pointer)(hCamera, Timeout);
}

SVGigE_RETURN
Camera_openConnectionEx(Camera_handle hCamera, float HeartbeatTimeout, int GVCPRetryCount, int GVCPTimeout)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_openConnectionEx)
  GigEFunc[Func_Camera_openConnectionEx].function_pointer)(hCamera,HeartbeatTimeout,GVCPRetryCount, GVCPTimeout);
}



SVGigE_RETURN
Camera_closeConnection(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_closeConnection)
  GigEFunc[Func_Camera_closeConnection].function_pointer)(hCamera);
}

SVGigE_RETURN
Camera_setIPAddress(Camera_handle hCamera,
                    unsigned int IPAddress,
                    unsigned int SubnetMask)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setIPAddress)
  GigEFunc[Func_Camera_setIPAddress].function_pointer)(hCamera, IPAddress, SubnetMask);
}

SVGigE_RETURN
Camera_forceValidNetworkSettings(Camera_handle hCamera,
                                 unsigned int *IPAddress,
                                 unsigned int *SubnetMask)
{
  // Check DLL availablility
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_forceValidNetworkSettings)
  GigEFunc[Func_Camera_forceValidNetworkSettings].function_pointer)(hCamera, IPAddress, SubnetMask);
}

SVGigE_RETURN
Camera_restartIPConfiguration(Camera_handle hCamera)
{
  // Check DLL availablility
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_restartIPConfiguration)
  GigEFunc[Func_Camera_restartIPConfiguration].function_pointer)(hCamera);
}

//*****************************************************************************************
// 3 - Camera: Information
//*****************************************************************************************

const char *
Camera_getManufacturerName(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getManufacturerName)
  GigEFunc[Func_Camera_getManufacturerName].function_pointer)(hCamera);
}

const char *
Camera_getModelName(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getModelName)
  GigEFunc[Func_Camera_getModelName].function_pointer)(hCamera);
}

const char *
Camera_getDeviceVersion(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getDeviceVersion)
  GigEFunc[Func_Camera_getDeviceVersion].function_pointer)(hCamera);
}

const char *
Camera_getManufacturerSpecificInformation(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getManufacturerSpecificInformation)
  GigEFunc[Func_Camera_getManufacturerSpecificInformation].function_pointer)(hCamera);
}

const char *
Camera_getSerialNumber(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getSerialNumber)
  GigEFunc[Func_Camera_getSerialNumber].function_pointer)(hCamera);
}

SVGigE_RETURN
Camera_setUserDefinedName(Camera_handle hCamera, char *UserDefinedName)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setUserDefinedName)
  GigEFunc[Func_Camera_setUserDefinedName].function_pointer)(hCamera, UserDefinedName);
}

const char *
Camera_getUserDefinedName(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getUserDefinedName)
  GigEFunc[Func_Camera_getUserDefinedName].function_pointer)(hCamera);
}

const char *
Camera_getMacAddress(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getMacAddress)
  GigEFunc[Func_Camera_getMacAddress].function_pointer)(hCamera);
}

const char *
Camera_getIPAddress(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getIPAddress)
  GigEFunc[Func_Camera_getIPAddress].function_pointer)(hCamera);
}

const char *
Camera_getSubnetMask(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getSubnetMask)
  GigEFunc[Func_Camera_getSubnetMask].function_pointer)(hCamera);
}

SVGigE_RETURN
Camera_getPixelClock(Camera_handle hCamera,
                    int *PixelClock)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelClock)
  GigEFunc[Func_Camera_getPixelClock].function_pointer)(hCamera, PixelClock);
}

bool
Camera_isCameraFeature(Camera_handle hCamera,
                       CAMERA_FEATURE Feature)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return false;

  // Pass through function call to DLL
  return ((TFunc_Camera_isCameraFeature)
  GigEFunc[Func_Camera_isCameraFeature].function_pointer)(hCamera, Feature);
}

SVGigE_RETURN
Camera_readXMLFile(Camera_handle hCamera,
                   char **XML)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_readXMLFile)
  GigEFunc[Func_Camera_readXMLFile].function_pointer)(hCamera, XML);
}

SVGigE_RETURN
Camera_getSensorTemperature(Camera_handle hCamera,
                            unsigned int *SensorTemperature)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getSensorTemperature)
  GigEFunc[Func_Camera_getSensorTemperature].function_pointer)(hCamera, SensorTemperature);
}

//*****************************************************************************************
// 4 - Stream: Channel creation and control
//*****************************************************************************************

SVGigE_RETURN
StreamingChannel_create(StreamingChannel_handle *hStreamingChannel,
                        CameraContainerClient_handle hCameraContainer,
                        Camera_handle hCamera,
                        int BufferCount,
                        StreamCallback CallbackFunction,
                        void *Context)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_create)
  GigEFunc[Func_StreamingChannel_create].function_pointer)(hStreamingChannel, hCameraContainer, hCamera, BufferCount, CallbackFunction, Context);
}

SVGigE_RETURN
StreamingChannel_createEx(StreamingChannel_handle *hStreamingChannel,
                        CameraContainerClient_handle hCameraContainer,
                        Camera_handle hCamera,
                        int BufferCount,
                        int PacketResendTimeout,
						StreamCallback CallbackFunction,
                        void *Context)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_createEx)
  GigEFunc[Func_StreamingChannel_createEx].function_pointer)(hStreamingChannel, hCameraContainer, hCamera, BufferCount, PacketResendTimeout, CallbackFunction, Context);
}


SVGigE_RETURN
StreamingChannel_delete(StreamingChannel_handle hStreamingChannel)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_delete)
  GigEFunc[Func_StreamingChannel_delete].function_pointer)(hStreamingChannel);
}



SVGigE_RETURN
StreamingChannel_setReadoutTransfer(StreamingChannel_handle hStreamingChannel,
                                    bool isReadoutTransfer)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_setReadoutTransfer)
  GigEFunc[Func_StreamingChannel_setReadoutTransfer].function_pointer)(hStreamingChannel,isReadoutTransfer);
}

SVGigE_RETURN
StreamingChannel_getReadoutTransfer(StreamingChannel_handle hStreamingChannel,
                                    bool *isReadoutTransfer)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getReadoutTransfer)
  GigEFunc[Func_StreamingChannel_getReadoutTransfer].function_pointer)(hStreamingChannel,isReadoutTransfer);
}

//*****************************************************************************************
// 5 - Stream: Channel statistics
//*****************************************************************************************

SVGigE_RETURN
StreamingChannel_getFrameLoss(StreamingChannel_handle hStreamingChannel,
                              int *FrameLoss)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getFrameLoss)
  GigEFunc[Func_StreamingChannel_getFrameLoss].function_pointer)(hStreamingChannel, FrameLoss);
}

SVGigE_RETURN
StreamingChannel_getActualFrameRate(StreamingChannel_handle hStreamingChannel,
                                    float *ActualFrameRate)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getActualFrameRate)
  GigEFunc[Func_StreamingChannel_getActualFrameRate].function_pointer)(hStreamingChannel, ActualFrameRate);
}

SVGigE_RETURN
StreamingChannel_getActualDataRate(StreamingChannel_handle hStreamingChannel,
                                   float *ActualDataRate)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getActualDataRate)
  GigEFunc[Func_StreamingChannel_getActualDataRate].function_pointer)(hStreamingChannel, ActualDataRate);
}

SVGigE_RETURN
StreamingChannel_getPeakDataRate(StreamingChannel_handle hStreamingChannel,
                                 float *PeakDataRate)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getPeakDataRate)
  GigEFunc[Func_StreamingChannel_getPeakDataRate].function_pointer)(hStreamingChannel,PeakDataRate);
}

//*****************************************************************************************
// 6 - Stream: Channel info
//*****************************************************************************************

SVGigE_RETURN
StreamingChannel_getPixelType(StreamingChannel_handle hStreamingChannel,
                              GVSP_PIXEL_TYPE *ProgrammedPixelType)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getPixelType)
  GigEFunc[Func_StreamingChannel_getPixelType].function_pointer)(hStreamingChannel,ProgrammedPixelType);
}

SVGigE_RETURN
StreamingChannel_getBufferData(StreamingChannel_handle hStreamingChannel,
															 unsigned int BufferIndex,
                               unsigned char **BufferData)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getBufferData)
  GigEFunc[Func_StreamingChannel_getBufferData].function_pointer)(hStreamingChannel,BufferIndex,BufferData);
}

SVGigE_RETURN
StreamingChannel_getBufferSize(StreamingChannel_handle hStreamingChannel,
                               int *BufferSize)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getBufferSize)
  GigEFunc[Func_StreamingChannel_getBufferSize].function_pointer)(hStreamingChannel,BufferSize);
}

SVGigE_RETURN
StreamingChannel_getImagePitch(StreamingChannel_handle hStreamingChannel,
                               int *ImagePitch)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getImagePitch)
  GigEFunc[Func_StreamingChannel_getImagePitch].function_pointer)(hStreamingChannel,ImagePitch);
}

SVGigE_RETURN
StreamingChannel_getImageSizeX(StreamingChannel_handle hStreamingChannel,
                               int *ImageSizeX)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getImageSizeX)
  GigEFunc[Func_StreamingChannel_getImageSizeX].function_pointer)(hStreamingChannel,ImageSizeX);
}

SVGigE_RETURN
StreamingChannel_getImageSizeY(StreamingChannel_handle hStreamingChannel,
                               int *ImageSizeY)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getImageSizeY)
  GigEFunc[Func_StreamingChannel_getImageSizeY].function_pointer)(hStreamingChannel,ImageSizeY);
}

//*****************************************************************************************
// 7 - Stream: Transfer parameters
//*****************************************************************************************

SVGigE_RETURN
Camera_evaluateMaximalPacketSize(Camera_handle hCamera,
                                 int *MaximalPacketSize)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_evaluateMaximalPacketSize)
  GigEFunc[Func_Camera_evaluateMaximalPacketSize].function_pointer)(hCamera, MaximalPacketSize);
}

SVGigE_RETURN
Camera_setStreamingPacketSize(Camera_handle hCamera,
                              int StreamingPacketSize)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setStreamingPacketSize)
  GigEFunc[Func_Camera_setStreamingPacketSize].function_pointer)(hCamera, StreamingPacketSize);
}

SVGigE_RETURN
Camera_setInterPacketDelay(Camera_handle hCamera,
                           float InterPacketDelay)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setInterPacketDelay)
  GigEFunc[Func_Camera_setInterPacketDelay].function_pointer)(hCamera, InterPacketDelay);
}

SVGigE_RETURN
Camera_getInterPacketDelay(Camera_handle hCamera,
                           float *ProgrammedInterPacketDelay)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getInterPacketDelay)
  GigEFunc[Func_Camera_getInterPacketDelay].function_pointer)(hCamera, ProgrammedInterPacketDelay);
}

SVGigE_RETURN
Camera_setMulticastMode(Camera_handle hCamera, MULTICAST_MODE MulticastMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setMulticastMode)
  GigEFunc[Func_Camera_setMulticastMode].function_pointer)(hCamera, MulticastMode);
}

SVGigE_RETURN
Camera_getMulticastMode(Camera_handle hCamera, MULTICAST_MODE *MulticastMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getMulticastMode)
  GigEFunc[Func_Camera_getMulticastMode].function_pointer)(hCamera, MulticastMode);
}

SVGigE_RETURN
Camera_getMulticastGroup(Camera_handle hCamera,
                         unsigned int *MulticastIP,
                         unsigned int *MulticastPort)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getMulticastGroup)
  GigEFunc[Func_Camera_getMulticastGroup].function_pointer)(hCamera, MulticastIP, MulticastPort);
}

//*****************************************************************************************
// 8 - Stream: Image access
//*****************************************************************************************

unsigned char *
Image_getDataPointer(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return NULL;

  // Pass through function call to DLL
  return ((TFunc_Image_getDataPointer)
  GigEFunc[Func_Image_getDataPointer].function_pointer)(hImage);
}

int
Image_getBufferIndex(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return NULL;

  // Pass through function call to DLL
  return ((TFunc_Image_getBufferIndex)
  GigEFunc[Func_Image_getBufferIndex].function_pointer)(hImage);
}

SVGigE_SIGNAL_TYPE
Image_getSignalType(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_SIGNAL_NONE;

  // Pass through function call to DLL
  return ((TFunc_Image_getSignalType)
  GigEFunc[Func_Image_getSignalType].function_pointer)(hImage);
}

Camera_handle
Image_getCamera(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_NO_CAMERA;

  // Pass through function call to DLL
  return ((TFunc_Image_getCamera)
  GigEFunc[Func_Image_getCamera].function_pointer)(hImage);
}

SVGigE_RETURN
Image_release(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_release)
  GigEFunc[Func_Image_release].function_pointer)(hImage);
}

//*****************************************************************************************
// 9 - Stream: Image conversion
//*****************************************************************************************

SVGigE_RETURN
Image_getImageRGB(Image_handle hImage,
                  unsigned char *BufferRGB,
                  int BufferLength,
                  BAYER_METHOD BayerMethod)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_getImageRGB)
  GigEFunc[Func_Image_getImageRGB].function_pointer)(hImage, BufferRGB, BufferLength, BayerMethod);
}

SVGigE_RETURN
Image_getImageGray(Image_handle hImage,
                   unsigned char *Buffer8bit,
                   int BufferLength)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_getImageGray)
  GigEFunc[Func_Image_getImageGray].function_pointer)(hImage, Buffer8bit, BufferLength);
}

SVGigE_RETURN
Image_getImage12bitAs8bit(Image_handle hImage,
                          unsigned char *Buffer8bit,
                          int BufferLength)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_getImage12bitAs8bit)
  GigEFunc[Func_Image_getImage12bitAs8bit].function_pointer)(hImage, Buffer8bit, BufferLength);
}

SVGigE_RETURN
Image_getImage12bitAs16bit(Image_handle hImage,
                           unsigned char *Buffer16bit,
                           int BufferLength)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_getImage12bitAs16bit)
  GigEFunc[Func_Image_getImage12bitAs16bit].function_pointer)(hImage, Buffer16bit, BufferLength);
}

SVGigE_RETURN
Image_getImage16bitAs8bit(Image_handle hImage,
                          unsigned char *Buffer8bit,
                          int BufferLength)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_getImage16bitAs8bit)
  GigEFunc[Func_Image_getImage16bitAs8bit].function_pointer)(hImage, Buffer8bit, BufferLength);
}

//*****************************************************************************************
// 10 - Stream: Image characteristics
//*****************************************************************************************

GVSP_PIXEL_TYPE
Image_getPixelType(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return GVSP_PIX_MONO8;

  // Pass through function call to DLL
  return ((TFunc_Image_getPixelType)
  GigEFunc[Func_Image_getPixelType].function_pointer)(hImage);
}

int
Image_getImageSize(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getImageSize)
  GigEFunc[Func_Image_getImageSize].function_pointer)(hImage);
}

int
Image_getPitch(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getPitch)
  GigEFunc[Func_Image_getPitch].function_pointer)(hImage);
}

int
Image_getSizeX(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getSizeX)
  GigEFunc[Func_Image_getSizeX].function_pointer)(hImage);
}

int
Image_getSizeY(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getSizeY)
  GigEFunc[Func_Image_getSizeY].function_pointer)(hImage);
}

//*****************************************************************************************
// 11 - Stream: Image statistics
//*****************************************************************************************

int
Image_getImageID(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getImageID)
  GigEFunc[Func_Image_getImageID].function_pointer)(hImage);
}

double
Image_getTimestamp(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getTimestamp)
  GigEFunc[Func_Image_getTimestamp].function_pointer)(hImage);
}

double
Image_getTransferTime(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getTransferTime)
  GigEFunc[Func_Image_getTransferTime].function_pointer)(hImage);
}

int
Image_getPacketCount(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getPacketCount)
  GigEFunc[Func_Image_getPacketCount].function_pointer)(hImage);
}

int
Image_getPacketResend(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getPacketResend)
  GigEFunc[Func_Image_getPacketResend].function_pointer)(hImage);
}

//*****************************************************************************************
// 12 - Stream: Messaging channel
//*****************************************************************************************

SVGigE_RETURN
Stream_createEvent(StreamingChannel_handle hStreamingChannel,
                   Event_handle *EventID,
                   int SizeFIFO)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_createEvent)
  GigEFunc[Func_Stream_createEvent].function_pointer)(hStreamingChannel, EventID, SizeFIFO);
}

SVGigE_RETURN
Stream_addMessageType(StreamingChannel_handle hStreamingChannel,
                      Event_handle EventID,
                      SVGigE_SIGNAL_TYPE MessageType)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_addMessageType)
  GigEFunc[Func_Stream_addMessageType].function_pointer)(hStreamingChannel, EventID, MessageType);
}

SVGigE_RETURN
Stream_removeMessageType(StreamingChannel_handle hStreamingChannel,
                         Event_handle EventID,
                         SVGigE_SIGNAL_TYPE MessageType)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_removeMessageType)
  GigEFunc[Func_Stream_removeMessageType].function_pointer)(hStreamingChannel, EventID, MessageType);
}

SVGigE_RETURN
Stream_isMessagePending(StreamingChannel_handle hStreamingChannel,
                        Event_handle EventID,
                        int Timeout_ms)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_isMessagePending)
  GigEFunc[Func_Stream_isMessagePending].function_pointer)(hStreamingChannel, EventID, Timeout_ms);
}

SVGigE_RETURN
Stream_registerEventCallback(StreamingChannel_handle hStreamingChannel,
                             Event_handle EventID,
                             EventCallback Callback,
                             void *Context)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_registerEventCallback)
  GigEFunc[Func_Stream_registerEventCallback].function_pointer)(hStreamingChannel, EventID, Callback, Context);
}

SVGigE_RETURN
Stream_unregisterEventCallback(StreamingChannel_handle hStreamingChannel,
                               Event_handle EventID,
                               EventCallback Callback)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_unregisterEventCallback)
  GigEFunc[Func_Stream_unregisterEventCallback].function_pointer)(hStreamingChannel, EventID, Callback);
}

SVGigE_RETURN
Stream_getMessage(StreamingChannel_handle hStreamingChannel,
                  Event_handle EventID,
                  Message_handle *MessageID,
                  SVGigE_SIGNAL_TYPE *MessageType)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_getMessage)
  GigEFunc[Func_Stream_getMessage].function_pointer)(hStreamingChannel, EventID, MessageID, MessageType);
}

SVGigE_RETURN
Stream_getMessageData(StreamingChannel_handle hStreamingChannel,
                      Event_handle EventID,
                      Message_handle MessageID,
                      void **MessageData,
                      int *MessageLength)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_getMessageData)
  GigEFunc[Func_Stream_getMessageData].function_pointer)(hStreamingChannel, EventID, MessageID, MessageData, MessageLength);
}

SVGigE_RETURN
Stream_getMessageTimestamp(StreamingChannel_handle hStreamingChannel,
                           Event_handle EventID,
                           Message_handle MessageID,
                           double *MessageTimestamp)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_getMessageTimestamp)
  GigEFunc[Func_Stream_getMessageTimestamp].function_pointer)(hStreamingChannel, EventID, MessageID, MessageTimestamp);
}

SVGigE_RETURN
Stream_releaseMessage(StreamingChannel_handle hStreamingChannel,
                      Event_handle EventID,
                      Message_handle MessageID)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_releaseMessage)
  GigEFunc[Func_Stream_releaseMessage].function_pointer)(hStreamingChannel, EventID, MessageID);
}

SVGigE_RETURN
Stream_flushMessages(StreamingChannel_handle hStreamingChannel,
                     Event_handle EventID)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_flushMessages)
  GigEFunc[Func_Stream_flushMessages].function_pointer)(hStreamingChannel, EventID);
}

SVGigE_RETURN
Stream_closeEvent(StreamingChannel_handle hStreamingChannel,
                  Event_handle EventID)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Stream_closeEvent)
  GigEFunc[Func_Stream_closeEvent].function_pointer)(hStreamingChannel, EventID);
}

//*****************************************************************************************
// 13 - Controlling camera: Frame rate
//*****************************************************************************************

SVGigE_RETURN
Camera_setFrameRate(Camera_handle hCamera,
                    float Framerate)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setFrameRate)
  GigEFunc[Func_Camera_setFrameRate].function_pointer)(hCamera, Framerate);
}

SVGigE_RETURN
Camera_getFrameRate(Camera_handle hCamera,
                    float *ProgrammedFramerate)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFrameRate)
  GigEFunc[Func_Camera_getFrameRate].function_pointer)(hCamera, ProgrammedFramerate);
}

SVGigE_RETURN
Camera_getFrameRateMin(Camera_handle hCamera,
                       float *MinFramerate)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFrameRateMin)
  GigEFunc[Func_Camera_getFrameRateMin].function_pointer)(hCamera, MinFramerate);
}

SVGigE_RETURN
Camera_getFrameRateMax(Camera_handle hCamera,
                       float *MaxFramerate)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFrameRateMax)
  GigEFunc[Func_Camera_getFrameRateMax].function_pointer)(hCamera, MaxFramerate);
}

SVGigE_RETURN
Camera_getFrameRateRange(Camera_handle hCamera,
                         float *MinFramerate,
                         float *MaxFramerate)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFrameRateRange)
  GigEFunc[Func_Camera_getFrameRateRange].function_pointer)(hCamera, MinFramerate, MaxFramerate);
}

SVGigE_RETURN
Camera_getFrameRateIncrement(Camera_handle hCamera,
                             float *FramerateIncrement)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFrameRateIncrement)
  GigEFunc[Func_Camera_getFrameRateIncrement].function_pointer)(hCamera, FramerateIncrement);
}

//*****************************************************************************************
// 14 - Controlling camera: Exposure
//*****************************************************************************************

SVGigE_RETURN
Camera_setExposureTime(Camera_handle hCamera,
                       float ExposureTime)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setExposureTime)
  GigEFunc[Func_Camera_setExposureTime].function_pointer)(hCamera, ExposureTime);
}

SVGigE_RETURN
Camera_getExposureTime(Camera_handle hCamera,
                       float *ProgrammedExposureTime)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureTime)
  GigEFunc[Func_Camera_getExposureTime].function_pointer)(hCamera, ProgrammedExposureTime);
}

SVGigE_RETURN
Camera_getExposureTimeMin(Camera_handle hCamera,
                          float *MinExposureTime)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureTimeMin)
  GigEFunc[Func_Camera_getExposureTimeMin].function_pointer)(hCamera, MinExposureTime);
}

SVGigE_RETURN
Camera_getExposureTimeMax(Camera_handle hCamera,
                          float *MaxExposureTime)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureTimeMax)
  GigEFunc[Func_Camera_getExposureTimeMax].function_pointer)(hCamera, MaxExposureTime);
}

SVGigE_RETURN
Camera_getExposureTimeRange(Camera_handle hCamera,
                            float *MinExposureTime,
                            float *MaxExposureTime)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureTimeRange)
  GigEFunc[Func_Camera_getExposureTimeRange].function_pointer)(hCamera, MinExposureTime, MaxExposureTime);
}

SVGigE_RETURN
Camera_getExposureTimeIncrement(Camera_handle hCamera,
                                float *ExposureTimeIncrement)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureTimeIncrement)
  GigEFunc[Func_Camera_getExposureTimeIncrement].function_pointer)(hCamera, ExposureTimeIncrement);
}

SVGigE_RETURN
Camera_setExposureDelay(Camera_handle hCamera,
                         float ExposureDelay)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setExposureDelay)
  GigEFunc[Func_Camera_setExposureDelay].function_pointer)(hCamera, ExposureDelay);
}

SVGigE_RETURN
Camera_getExposureDelay(Camera_handle hCamera,
                        float *ProgrammedExposureDelay)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureDelay)
  GigEFunc[Func_Camera_getExposureDelay].function_pointer)(hCamera, ProgrammedExposureDelay);
}

SVGigE_RETURN
Camera_getExposureDelayMax(Camera_handle hCamera,
                           float *MaxExposureDelay)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureDelayMax)
  GigEFunc[Func_Camera_getExposureDelayMax].function_pointer)(hCamera, MaxExposureDelay);
}

SVGigE_RETURN
Camera_getExposureDelayIncrement(Camera_handle hCamera,
                                 float *ExposureDelayIncrement)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureDelayIncrement)
  GigEFunc[Func_Camera_getExposureDelayIncrement].function_pointer)(hCamera, ExposureDelayIncrement);
}

//*****************************************************************************************
// 15 - Controlling camera: Gain and offset
//*****************************************************************************************

SVGigE_RETURN
Camera_setGain(Camera_handle hCamera,
               float Gain)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setGain)
  GigEFunc[Func_Camera_setGain].function_pointer)(hCamera, Gain);
}

SVGigE_RETURN
Camera_getGain(Camera_handle hCamera,
               float *ProgrammedGain)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getGain)
  GigEFunc[Func_Camera_getGain].function_pointer)(hCamera, ProgrammedGain);
}

SVGigE_RETURN
Camera_getGainMax(Camera_handle hCamera,
                  float *MaxGain)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getGainMax)
  GigEFunc[Func_Camera_getGainMax].function_pointer)(hCamera, MaxGain);
}

SVGigE_RETURN
Camera_getGainMaxExtended(Camera_handle hCamera,
                          float *MaxGainExtended)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getGainMaxExtended)
  GigEFunc[Func_Camera_getGainMaxExtended].function_pointer)(hCamera, MaxGainExtended);
}

SVGigE_RETURN
Camera_getGainIncrement(Camera_handle hCamera,
                        float *GainIncrement)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getGainIncrement)
  GigEFunc[Func_Camera_getGainIncrement].function_pointer)(hCamera, GainIncrement);
}

SVGigE_RETURN
Camera_setOffset(Camera_handle hCamera,
                 float Offset)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setOffset)
  GigEFunc[Func_Camera_setOffset].function_pointer)(hCamera, Offset);
}

SVGigE_RETURN
Camera_getOffset(Camera_handle hCamera,
                 float *ProgrammedOffset)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getOffset)
  GigEFunc[Func_Camera_getOffset].function_pointer)(hCamera, ProgrammedOffset);
}

SVGigE_RETURN
Camera_getOffsetMax(Camera_handle hCamera,
                    float *MaxOffset)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getOffsetMax)
  GigEFunc[Func_Camera_getOffsetMax].function_pointer)(hCamera, MaxOffset);
}

//*****************************************************************************************
// 16 - Controlling camera: Auto gain/exposure
//*****************************************************************************************

SVGigE_RETURN
Camera_setAutoGainEnabled(Camera_handle hCamera,
													bool isAutoGainEnabled)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoGainEnabled)
  GigEFunc[Func_Camera_setAutoGainEnabled].function_pointer)(hCamera, isAutoGainEnabled);
}

SVGigE_RETURN
Camera_getAutoGainEnabled(Camera_handle hCamera,
														bool *isAutoGainEnabled)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoGainEnabled)
  GigEFunc[Func_Camera_getAutoGainEnabled].function_pointer)(hCamera, isAutoGainEnabled);
}

SVGigE_RETURN
Camera_setAutoGainBrightness(Camera_handle hCamera,
                               float Brightness)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoGainBrightness)
  GigEFunc[Func_Camera_setAutoGainBrightness].function_pointer)(hCamera, Brightness);
}

SVGigE_RETURN
Camera_getAutoGainBrightness(Camera_handle hCamera,
                             float *ProgrammedBrightness)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoGainBrightness)
  GigEFunc[Func_Camera_getAutoGainBrightness].function_pointer)(hCamera, ProgrammedBrightness);
}

SVGigE_RETURN
Camera_setAutoGainDynamics(Camera_handle hCamera,
                           float AutoGainParameterI,
                           float AutoGainParameterD)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoGainDynamics)
  GigEFunc[Func_Camera_setAutoGainDynamics].function_pointer)(hCamera, AutoGainParameterI, AutoGainParameterD);
}

SVGigE_RETURN
Camera_getAutoGainDynamics(Camera_handle hCamera,
                           float *ProgrammedAutoGainParameterI,
                           float *ProgrammedAutoGainParameterD)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoGainDynamics)
  GigEFunc[Func_Camera_getAutoGainDynamics].function_pointer)(hCamera, ProgrammedAutoGainParameterI, ProgrammedAutoGainParameterD);
}

SVGigE_RETURN
Camera_setAutoGainLimits(Camera_handle hCamera,
                         float MinGain,
                         float MaxGain)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoGainLimits)
  GigEFunc[Func_Camera_setAutoGainLimits].function_pointer)(hCamera, MinGain, MaxGain);
}

SVGigE_RETURN
Camera_getAutoGainLimits(Camera_handle hCamera,
                         float *ProgrammedMinGain,
                         float *ProgrammedMaxGain)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoGainLimits)
  GigEFunc[Func_Camera_getAutoGainLimits].function_pointer)(hCamera, ProgrammedMinGain, ProgrammedMaxGain);
}

SVGigE_RETURN
Camera_setAutoExposureLimits(Camera_handle hCamera,
                             float MinExposure,
                             float MaxExposure)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoExposureLimits)
  GigEFunc[Func_Camera_setAutoExposureLimits].function_pointer)(hCamera, MinExposure, MaxExposure);
}

SVGigE_RETURN
Camera_getAutoExposureLimits(Camera_handle hCamera,
                             float *ProgrammedMinExposure,
                             float *ProgrammedMaxExposure)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoExposureLimits)
  GigEFunc[Func_Camera_getAutoExposureLimits].function_pointer)(hCamera, ProgrammedMinExposure, ProgrammedMaxExposure);
}

//*****************************************************************************************
// 17 - Controlling camera: Acquisition trigger
//*****************************************************************************************

SVGigE_RETURN
Camera_setAcquisitionControl(Camera_handle hCamera,
                             ACQUISITION_CONTROL AcquisitionControl)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAcquisitionControl)
  GigEFunc[Func_Camera_setAcquisitionControl].function_pointer)(hCamera, AcquisitionControl);
}

SVGigE_RETURN
Camera_getAcquisitionControl(Camera_handle hCamera,
                             ACQUISITION_CONTROL *ProgrammedAcquisitionControl)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAcquisitionControl)
  GigEFunc[Func_Camera_getAcquisitionControl].function_pointer)(hCamera, ProgrammedAcquisitionControl);
}

SVGigE_RETURN
Camera_setAcquisitionMode(Camera_handle hCamera,
                          ACQUISITION_MODE AcquisitionMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAcquisitionMode)
  GigEFunc[Func_Camera_setAcquisitionMode].function_pointer)(hCamera, AcquisitionMode);
}

SVGigE_RETURN
Camera_setAcquisitionModeAndStart(Camera_handle hCamera,
                                  ACQUISITION_MODE AcquisitionMode,
                                  bool AcquisitionStart)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAcquisitionModeAndStart)
  GigEFunc[Func_Camera_setAcquisitionModeAndStart].function_pointer)(hCamera, AcquisitionMode, AcquisitionStart);
}

SVGigE_RETURN
Camera_getAcquisitionMode(Camera_handle hCamera,
                          ACQUISITION_MODE *ProgrammedAcquisitionMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAcquisitionMode)
  GigEFunc[Func_Camera_getAcquisitionMode].function_pointer)(hCamera, ProgrammedAcquisitionMode);
}

SVGigE_RETURN
Camera_softwareTrigger(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_softwareTrigger)
  GigEFunc[Func_Camera_softwareTrigger].function_pointer)(hCamera);
}

SVGigE_RETURN
Camera_softwareTriggerID(Camera_handle hCamera,
                         int TriggerID)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_softwareTriggerID)
  GigEFunc[Func_Camera_softwareTriggerID].function_pointer)(hCamera, TriggerID);
}

SVGigE_RETURN
Camera_softwareTriggerIDEnable(Camera_handle hCamera,
                               bool TriggerIDEnable)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_softwareTriggerIDEnable)
  GigEFunc[Func_Camera_softwareTriggerIDEnable].function_pointer)(hCamera, TriggerIDEnable);
}

SVGigE_RETURN
Camera_setTriggerPolarity(Camera_handle hCamera,
                          TRIGGER_POLARITY TriggerPolarity)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setTriggerPolarity)
  GigEFunc[Func_Camera_setTriggerPolarity].function_pointer)(hCamera, TriggerPolarity);
}

SVGigE_RETURN
Camera_getTriggerPolarity(Camera_handle hCamera,
                          TRIGGER_POLARITY *ProgrammedTriggerPolarity)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTriggerPolarity)
  GigEFunc[Func_Camera_getTriggerPolarity].function_pointer)(hCamera, ProgrammedTriggerPolarity);
}

//---------------PIV-mode--------------------------------
SVGigE_RETURN
Camera_setPivMode(Camera_handle hCamera,
                  PIV_MODE SelectPivMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPivMode)
  GigEFunc[Func_Camera_setPivMode].function_pointer)(hCamera, SelectPivMode);
}

SVGigE_RETURN
Camera_getPivMode(Camera_handle hCamera,
                  PIV_MODE *ProgrammedPivMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPivMode)
  GigEFunc[Func_Camera_getPivMode].function_pointer)(hCamera, ProgrammedPivMode);
}

//---------------debouncer-------------------------------------
SVGigE_RETURN
Camera_setDebouncerDuration(Camera_handle hCamera,
                            float  DebouncerDuration)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setDebouncerDuration)
  GigEFunc[Func_Camera_setDebouncerDuration].function_pointer)(hCamera, DebouncerDuration);
}

SVGigE_RETURN
Camera_getDebouncerDuration(Camera_handle hCamera,
                            float *ProgrammedDuration)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getDebouncerDuration)
  GigEFunc[Func_Camera_getDebouncerDuration].function_pointer)(hCamera, ProgrammedDuration);
}

//---------------prescaler-------------------------------------
SVGigE_RETURN
Camera_setPrescalerDevisor(Camera_handle hCamera,
                           unsigned int  PrescalerDevisor)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPrescalerDevisor)
  GigEFunc[Func_Camera_setPrescalerDevisor].function_pointer)(hCamera, PrescalerDevisor);
}

SVGigE_RETURN
Camera_getPrescalerDevisor(Camera_handle hCamera,
                            unsigned int *ProgrammedPrescalerDevisor)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPrescalerDevisor)
  GigEFunc[Func_Camera_getPrescalerDevisor].function_pointer)(hCamera, ProgrammedPrescalerDevisor);
}

//---------------sequencer-------------------------------------
SVGigE_RETURN
 Camera_loadSequenceParameters(Camera_handle hCamera,
                               const char *Filename)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_loadSequenceParameters)
  GigEFunc[Func_Camera_loadSequenceParameters].function_pointer)(hCamera, Filename);
}

SVGigE_RETURN
Camera_startSequencer(Camera_handle hCamera)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_startSequencer)
  GigEFunc[Func_Camera_startSequencer].function_pointer)(hCamera);
}

//*****************************************************************************************
// 18 - Controlling camera: Strobe
//*****************************************************************************************

SVGigE_RETURN
Camera_setStrobePolarity(Camera_handle hCamera,
                         STROBE_POLARITY StrobePolarity)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setStrobePolarity)
  GigEFunc[Func_Camera_setStrobePolarity].function_pointer)(hCamera, StrobePolarity);
}

SVGigE_RETURN
Camera_setStrobePolarityExtended(Camera_handle hCamera,
                                 STROBE_POLARITY StrobePolarity, int StrobeIndex )
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setStrobePolarityExtended)
  GigEFunc[Func_Camera_setStrobePolarityExtended].function_pointer)(hCamera, StrobePolarity,  StrobeIndex);
}


SVGigE_RETURN
Camera_getStrobePolarity(Camera_handle hCamera,
                         STROBE_POLARITY *ProgrammedStrobePolarity)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobePolarity)
  GigEFunc[Func_Camera_getStrobePolarity].function_pointer)(hCamera, ProgrammedStrobePolarity);
}


SVGigE_RETURN
Camera_getStrobePolarityExtended(Camera_handle hCamera,
                                 STROBE_POLARITY *ProgrammedStrobePolarity,int StrobeIndex)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobePolarityExtended)
  GigEFunc[Func_Camera_getStrobePolarityExtended].function_pointer)(hCamera, ProgrammedStrobePolarity, StrobeIndex);
}


SVGigE_RETURN
Camera_setStrobePosition(Camera_handle hCamera,
                         float StrobePosition)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setStrobePosition)
  GigEFunc[Func_Camera_setStrobePosition].function_pointer)(hCamera, StrobePosition);
}



SVGigE_RETURN
Camera_setStrobePositionExtended(Camera_handle hCamera,
                                 float StrobePosition,int StrobeIndex)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setStrobePositionExtended)
  GigEFunc[Func_Camera_setStrobePositionExtended].function_pointer)(hCamera, StrobePosition,StrobeIndex);
}

SVGigE_RETURN
Camera_getStrobePosition(Camera_handle hCamera,
                         float *ProgrammedStrobePosition)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobePosition)
  GigEFunc[Func_Camera_getStrobePosition].function_pointer)(hCamera, ProgrammedStrobePosition);
}

SVGigE_RETURN
Camera_getStrobePositionExtended(Camera_handle hCamera,
                                 float *ProgrammedStrobePosition, int StrobeIndex )
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobePositionExtended)
  GigEFunc[Func_Camera_getStrobePositionExtended].function_pointer)(hCamera, ProgrammedStrobePosition, StrobeIndex);
}

SVGigE_RETURN
Camera_getStrobePositionMax(Camera_handle hCamera,
                            float *MaxStrobePosition)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobePositionMax)
  GigEFunc[Func_Camera_getStrobePositionMax].function_pointer)(hCamera, MaxStrobePosition);
}

SVGigE_RETURN
Camera_getStrobePositionIncrement(Camera_handle hCamera,
                                  float *StrobePositionIncrement)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobePositionIncrement)
  GigEFunc[Func_Camera_getStrobePositionIncrement].function_pointer)(hCamera, StrobePositionIncrement);
}

SVGigE_RETURN
Camera_setStrobeDuration(Camera_handle hCamera,
                         float StrobeDuration)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setStrobeDuration)
  GigEFunc[Func_Camera_setStrobeDuration].function_pointer)(hCamera, StrobeDuration);
}

SVGigE_RETURN
Camera_setStrobeDurationExtended(Camera_handle hCamera,
                                 float StrobeDuration,int StrobeIndex  )
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setStrobeDurationExtended)
  GigEFunc[Func_Camera_setStrobeDurationExtended].function_pointer)(hCamera, StrobeDuration, StrobeIndex);
}

SVGigE_RETURN
Camera_getStrobeDuration(Camera_handle hCamera,
                         float *ProgrammedStrobeDuration)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobeDuration)
  GigEFunc[Func_Camera_getStrobeDuration].function_pointer)(hCamera, ProgrammedStrobeDuration);
}

SVGigE_RETURN
Camera_getStrobeDurationExtended(Camera_handle hCamera,
                                 float *ProgrammedStrobeDuration, int StrobeIndex )
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobeDurationExtended)
  GigEFunc[Func_Camera_getStrobeDurationExtended].function_pointer)(hCamera, ProgrammedStrobeDuration,StrobeIndex);
}

SVGigE_RETURN
Camera_getStrobeDurationMax(Camera_handle hCamera,
                            float *MaxStrobeDuration)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobeDurationMax)
  GigEFunc[Func_Camera_getStrobeDurationMax].function_pointer)(hCamera, MaxStrobeDuration);
}

SVGigE_RETURN
Camera_getStrobeDurationIncrement(Camera_handle hCamera,
                                  float *StrobeDurationIncrement)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobeDurationIncrement)
  GigEFunc[Func_Camera_getStrobeDurationIncrement].function_pointer)(hCamera, StrobeDurationIncrement);
}

//*****************************************************************************************
// 19 - Controlling camera: Tap balance
//*****************************************************************************************



SVGigE_RETURN
Camera_setTapConfiguration(Camera_handle hCamera,
                           int TapCount)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setTapConfiguration)
  GigEFunc[Func_Camera_setTapConfiguration].function_pointer)(hCamera, TapCount);
}

SVGigE_RETURN
Camera_getTapConfiguration(Camera_handle hCamera,
                           int *TapCount)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTapConfiguration)
  GigEFunc[Func_Camera_getTapConfiguration].function_pointer)(hCamera, TapCount);
}


SVGigE_RETURN
Camera_setTapConfigurationEx(Camera_handle hCamera,
                         SVGIGE_TAP_CONFIGURATION_SELECT SelectedTapConfig)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setTapConfigurationEx)
  GigEFunc[Func_Camera_setTapConfigurationEx].function_pointer)(hCamera, SelectedTapConfig);
}

SVGigE_RETURN
Camera_getTapConfigurationEx(Camera_handle hCamera,
                         SVGIGE_TAP_CONFIGURATION_SELECT *ProgrammedTapConfig)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTapConfigurationEx)
  GigEFunc[Func_Camera_getTapConfigurationEx].function_pointer)(hCamera, ProgrammedTapConfig);
}

SVGigE_RETURN
Camera_setAutoTapBalanceMode(Camera_handle hCamera,
                             SVGIGE_AUTO_TAP_BALANCE_MODE AutoTapBalanceMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoTapBalanceMode)
  GigEFunc[Func_Camera_setAutoTapBalanceMode].function_pointer)(hCamera, AutoTapBalanceMode);
}

SVGigE_RETURN
Camera_getAutoTapBalanceMode(Camera_handle hCamera,
                             SVGIGE_AUTO_TAP_BALANCE_MODE *AutoTapBalanceMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoTapBalanceMode)
  GigEFunc[Func_Camera_getAutoTapBalanceMode].function_pointer)(hCamera, AutoTapBalanceMode);
}

SVGigE_RETURN
Camera_setTapGain(Camera_handle hCamera,
									float TapGain,
									SVGIGE_TAP_SELECT Tap)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setTapGain)
  GigEFunc[Func_Camera_setTapGain].function_pointer)(hCamera, TapGain, Tap);
}

SVGigE_RETURN
Camera_getTapGain(Camera_handle hCamera,
									float *TapGain,
									SVGIGE_TAP_SELECT Tap)
{
// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTapGain)
  GigEFunc[Func_Camera_getTapGain].function_pointer)(hCamera, TapGain, Tap);
}








//*****************************************************************************************
// 20 - Controlling camera: Image parameter
//*****************************************************************************************

SVGigE_RETURN
Camera_getImagerWidth(Camera_handle hCamera,
                      int *ImagerWidth)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getImagerWidth)
  GigEFunc[Func_Camera_getImagerWidth].function_pointer)(hCamera, ImagerWidth);
}

SVGigE_RETURN
Camera_getImagerHeight(Camera_handle hCamera,
                       int *ImagerHeight)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getImagerHeight)
  GigEFunc[Func_Camera_getImagerHeight].function_pointer)(hCamera, ImagerHeight);
}

SVGigE_RETURN
Camera_getImageSize(Camera_handle hCamera,
                    int *ImageSize)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getImageSize)
  GigEFunc[Func_Camera_getImageSize].function_pointer)(hCamera, ImageSize);
}

SVGigE_RETURN
Camera_getPitch(Camera_handle hCamera,
                int *Pitch)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPitch)
  GigEFunc[Func_Camera_getPitch].function_pointer)(hCamera, Pitch);
}

SVGigE_RETURN
Camera_getSizeX(Camera_handle hCamera,
                int *SizeX)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getSizeX)
  GigEFunc[Func_Camera_getSizeX].function_pointer)(hCamera, SizeX);
}

SVGigE_RETURN
Camera_getSizeY(Camera_handle hCamera,
                int *SizeY)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getSizeY)
  GigEFunc[Func_Camera_getSizeY].function_pointer)(hCamera, SizeY);
}

SVGigE_RETURN
Camera_setBinningMode(Camera_handle hCamera,
                      BINNING_MODE BinningMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setBinningMode)
  GigEFunc[Func_Camera_setBinningMode].function_pointer)(hCamera, BinningMode);
}

SVGigE_RETURN
Camera_getBinningMode(Camera_handle hCamera,
                      BINNING_MODE *ProgrammedBinningMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getBinningMode)
  GigEFunc[Func_Camera_getBinningMode].function_pointer)(hCamera, ProgrammedBinningMode);
}

SVGigE_RETURN
Camera_setAreaOfInterest(Camera_handle hCamera,
                         int SizeX,
                         int SizeY,
                         int OffsetX,
                         int OffsetY)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAreaOfInterest)
  GigEFunc[Func_Camera_setAreaOfInterest].function_pointer)(hCamera, SizeX, SizeY, OffsetX, OffsetY);
}

SVGigE_RETURN
Camera_getAreaOfInterest(Camera_handle hCamera,
                         int *ProgrammedSizeX,
                         int *ProgrammedSizeY,
                         int *ProgrammedOffsetX,
                         int *ProgrammedOffsetY)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAreaOfInterest)
  GigEFunc[Func_Camera_getAreaOfInterest].function_pointer)(hCamera, ProgrammedSizeX, ProgrammedSizeY, ProgrammedOffsetX, ProgrammedOffsetY);
}

SVGigE_RETURN
Camera_getAreaOfInterestRange(Camera_handle hCamera,
                              int *MinSizeX,
                              int *MinSizeY,
                              int *MaxSizeX,
                              int *MaxSizeY)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAreaOfInterestRange)
  GigEFunc[Func_Camera_getAreaOfInterestRange].function_pointer)(hCamera, MinSizeX, MinSizeY, MaxSizeX, MaxSizeY);
}

SVGigE_RETURN
Camera_getAreaOfInterestIncrement(Camera_handle hCamera,
                                  int *SizeXIncrement,
                                  int *SizeYIncrement,
                                  int *OffsetXIncrement,
                                  int *OffsetYIncrement)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAreaOfInterestIncrement)
  GigEFunc[Func_Camera_getAreaOfInterestIncrement].function_pointer)(hCamera, SizeXIncrement, SizeYIncrement, OffsetXIncrement, OffsetYIncrement);
}

SVGigE_RETURN
Camera_resetTimestampCounter(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_resetTimestampCounter)
  GigEFunc[Func_Camera_resetTimestampCounter].function_pointer)(hCamera);
}

SVGigE_RETURN
Camera_getTimestampCounter(Camera_handle hCamera,
                           double *TimestampCounter)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTimestampCounter)
  GigEFunc[Func_Camera_getTimestampCounter].function_pointer)(hCamera, TimestampCounter);
}

SVGigE_RETURN
Camera_getTimestampTickFrequency(Camera_handle hCamera,
                                 double *TimestampTickFrequency)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTimestampTickFrequency)
  GigEFunc[Func_Camera_getTimestampTickFrequency].function_pointer)(hCamera, TimestampTickFrequency);
}

SVGigE_RETURN
Camera_setFlippingMode(Camera_handle hCamera,
						SVGIGE_FLIPPING_MODE  FlippingMode)
{
 // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setFlippingMode)
  GigEFunc[Func_Camera_setFlippingMode].function_pointer)(hCamera, FlippingMode);
}

SVGigE_RETURN
Camera_getFlippingMode(Camera_handle hCamera,
                        SVGIGE_FLIPPING_MODE *ProgrammedFlippingMode)
{
	 // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFlippingMode)
  GigEFunc[Func_Camera_getFlippingMode].function_pointer)(hCamera, ProgrammedFlippingMode);
}

SVGigE_RETURN
Camera_setShutterMode(Camera_handle hCamera,
                        SVGIGE_SHUTTER_MODE  ShutterMode)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setShutterMode)
  GigEFunc[Func_Camera_setShutterMode].function_pointer)(hCamera, ShutterMode);
 }

 SVGigE_RETURN
Camera_getShutterMode(Camera_handle hCamera,
                        SVGIGE_SHUTTER_MODE *ProgrammedShutterMode)
{
		 // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getShutterMode)
  GigEFunc[Func_Camera_getShutterMode].function_pointer)(hCamera, ProgrammedShutterMode);
}

//*****************************************************************************************
// 21 - Controlling camera: Image appearance
//*****************************************************************************************

SVGigE_RETURN
Camera_getPixelType(Camera_handle hCamera,
                    GVSP_PIXEL_TYPE *PixelType)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelType)
  GigEFunc[Func_Camera_getPixelType].function_pointer)(hCamera, PixelType);
}

SVGigE_RETURN
Camera_setPixelDepth(Camera_handle hCamera,
                     SVGIGE_PIXEL_DEPTH PixelDepth)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPixelDepth)
  GigEFunc[Func_Camera_setPixelDepth].function_pointer)(hCamera, PixelDepth);
}

SVGigE_RETURN
Camera_getPixelDepth(Camera_handle hCamera,
                     SVGIGE_PIXEL_DEPTH *PixelDepth)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelDepth)
  GigEFunc[Func_Camera_getPixelDepth].function_pointer)(hCamera, PixelDepth);
}



SVGigE_RETURN
Camera_setWhiteBalance(Camera_handle hCamera,
                       float Red,
                       float Green,
                       float Blue)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setWhiteBalance)
  GigEFunc[Func_Camera_setWhiteBalance].function_pointer)(hCamera, Red, Green, Blue);
}

SVGigE_RETURN
Camera_getWhiteBalance(Camera_handle hCamera,
                       float *Red,
                       float *Green,
                       float *Blue)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getWhiteBalance)
  GigEFunc[Func_Camera_getWhiteBalance].function_pointer)(hCamera, Red, Green, Blue);
}

SVGigE_RETURN
Camera_getWhiteBalanceMax(Camera_handle hCamera,
                            float *WhiteBalanceMax)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getWhiteBalanceMax)
  GigEFunc[Func_Camera_getWhiteBalanceMax].function_pointer)(hCamera, WhiteBalanceMax);
}

SVGigE_RETURN
Camera_setGammaCorrection(Camera_handle hCamera,
  										    float GammaCorrection)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setGammaCorrection)
  GigEFunc[Func_Camera_setGammaCorrection].function_pointer)(hCamera, GammaCorrection);
}

SVGigE_RETURN
Camera_setGammaCorrectionExt(Camera_handle hCamera,
  													 float GammaCorrection,
                             float DigitalGain,
                             float DigitalOffset)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setGammaCorrectionExt)
  GigEFunc[Func_Camera_setGammaCorrectionExt].function_pointer)(hCamera, GammaCorrection, DigitalGain, DigitalOffset);
}

SVGigE_RETURN
Camera_setLowpassFilter(Camera_handle hCamera,
                        LOWPASS_FILTER LowpassFilter)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLowpassFilter)
  GigEFunc[Func_Camera_setLowpassFilter].function_pointer)(hCamera, LowpassFilter);
}

SVGigE_RETURN
Camera_getLowpassFilter(Camera_handle hCamera,
                        LOWPASS_FILTER *ProgrammedLowpassFilter)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLowpassFilter)
  GigEFunc[Func_Camera_getLowpassFilter].function_pointer)(hCamera, ProgrammedLowpassFilter);
}

SVGigE_RETURN
Camera_setLookupTableMode(Camera_handle hCamera,
                          LUT_MODE LUTMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLookupTableMode)
  GigEFunc[Func_Camera_setLookupTableMode].function_pointer)(hCamera, LUTMode);
}

SVGigE_RETURN
Camera_getLookupTableMode(Camera_handle hCamera,
                          LUT_MODE *ProgrammedLUTMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLookupTableMode)
  GigEFunc[Func_Camera_getLookupTableMode].function_pointer)(hCamera, ProgrammedLUTMode);
}

SVGigE_RETURN
Camera_setLookupTable(Camera_handle hCamera,
                      unsigned char *LookupTable,
                      int LookupTableSize)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLookupTable)
  GigEFunc[Func_Camera_setLookupTable].function_pointer)(hCamera, LookupTable, LookupTableSize);
}

SVGigE_RETURN
Camera_getLookupTable(Camera_handle hCamera,
                      unsigned char *LookupTable,
                      int LookupTableSize)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLookupTable)
  GigEFunc[Func_Camera_getLookupTable].function_pointer)(hCamera, LookupTable, LookupTableSize);
}

__usrdllexport__ SVGigE_RETURN
Camera_startImageCorrection(Camera_handle hCamera,
                            IMAGE_CORRECTION_STEP ImageCorrectionStep)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_startImageCorrection)
  GigEFunc[Func_Camera_startImageCorrection].function_pointer)(hCamera, ImageCorrectionStep);
}

__usrdllexport__ SVGigE_RETURN
Camera_isIdleImageCorrection(Camera_handle hCamera,
														 IMAGE_CORRECTION_STEP *ProgrammedImageCorrectionStep,
			 											 bool *isIdle)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_isIdleImageCorrection)
  GigEFunc[Func_Camera_isIdleImageCorrection].function_pointer)(hCamera, ProgrammedImageCorrectionStep, isIdle);
}

__usrdllexport__ SVGigE_RETURN
Camera_setImageCorrection(Camera_handle hCamera,
													IMAGE_CORRECTION_MODE ImageCorrectionMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setImageCorrection)
  GigEFunc[Func_Camera_setImageCorrection].function_pointer)(hCamera, ImageCorrectionMode);
}

__usrdllexport__ SVGigE_RETURN
Camera_getImageCorrection(Camera_handle hCamera,
													IMAGE_CORRECTION_MODE *ProgrammedImageCorrectionMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getImageCorrection)
  GigEFunc[Func_Camera_getImageCorrection].function_pointer)(hCamera, ProgrammedImageCorrectionMode);
}



__usrdllexport__ SVGigE_RETURN
Camera_setPixelsCorrectionMap(Camera_handle hCamera,
							  PIXELS_CORRECTION_MAP_SELECT PixelsCorrectionMap)
{
// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPixelsCorrectionMap)
  GigEFunc[Func_Camera_setPixelsCorrectionMap].function_pointer)(hCamera,PixelsCorrectionMap );
}

__usrdllexport__ SVGigE_RETURN
Camera_getPixelsCorrectionMap(Camera_handle hCamera,
								PIXELS_CORRECTION_MAP_SELECT * ProgrammedPixelsCorrectionMap)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelsCorrectionMap)
  GigEFunc[Func_Camera_getPixelsCorrectionMap].function_pointer)(hCamera, ProgrammedPixelsCorrectionMap);
}




__usrdllexport__ SVGigE_RETURN
Camera_setPixelsCorrectionControlEnabel(Camera_handle hCamera,
										bool isPixelsCorrectionEnabled)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPixelsCorrectionControlEnabel)
  GigEFunc[Func_Camera_setPixelsCorrectionControlEnabel].function_pointer)(hCamera, isPixelsCorrectionEnabled);
}



__usrdllexport__ SVGigE_RETURN
Camera_getPixelsCorrectionControlEnabel(Camera_handle hCamera,
										 bool *isPixelsCorrectionEnabled)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelsCorrectionControlEnabel)
  GigEFunc[Func_Camera_getPixelsCorrectionControlEnabel].function_pointer)(hCamera,isPixelsCorrectionEnabled );
}


__usrdllexport__ SVGigE_RETURN
Camera_setPixelsCorrectionControlMark(Camera_handle hCamera,
									  bool isPixelsCorrectionMarked)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPixelsCorrectionControlMark)
  GigEFunc[Func_Camera_setPixelsCorrectionControlMark].function_pointer)(hCamera,isPixelsCorrectionMarked );
}



__usrdllexport__ SVGigE_RETURN
Camera_getPixelsCorrectionControlMark(Camera_handle hCamera,
									   bool *isPixelsCorrectionMarked)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelsCorrectionControlMark)
  GigEFunc[Func_Camera_getPixelsCorrectionControlMark].function_pointer)(hCamera,isPixelsCorrectionMarked );
}


__usrdllexport__ SVGigE_RETURN
Camera_setPixelsCorrectionMapOffset(Camera_handle hCamera,
									 int  OffsetX,   int  OffsetY)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPixelsCorrectionMapOffset)
  GigEFunc[Func_Camera_setPixelsCorrectionMapOffset].function_pointer)(hCamera, OffsetX, OffsetY );
}

__usrdllexport__ SVGigE_RETURN
Camera_getPixelsCorrectionMapOffset(Camera_handle hCamera,
									  int *ProgrammedOffsetX, int *ProgrammedOffsetY)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelsCorrectionMapOffset)
  GigEFunc[Func_Camera_getPixelsCorrectionMapOffset].function_pointer)(hCamera, ProgrammedOffsetX,ProgrammedOffsetY );
}


__usrdllexport__ SVGigE_RETURN
Camera_getPixelsCorrectionMapSize(Camera_handle hCamera,
								   unsigned int *programmedMapSize)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelsCorrectionMapSize)
  GigEFunc[Func_Camera_getPixelsCorrectionMapSize].function_pointer)(hCamera, programmedMapSize );
}


__usrdllexport__ SVGigE_RETURN
Camera_getMaximalPixelsCorrectionMapSize(Camera_handle hCamera,
										 unsigned int *MaximalprogrammedMapSize)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getMaximalPixelsCorrectionMapSize)
  GigEFunc[Func_Camera_getMaximalPixelsCorrectionMapSize].function_pointer)(hCamera, MaximalprogrammedMapSize );
}


__usrdllexport__ SVGigE_RETURN
Camera_setMapIndexCoordinate(Camera_handle hCamera,
									unsigned int MapIndex,
									unsigned int CoordinateX, unsigned int CoordinateY )
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setMapIndexCoordinate)
  GigEFunc[Func_Camera_setMapIndexCoordinate].function_pointer)(hCamera, MapIndex,  CoordinateX, CoordinateY );
}


__usrdllexport__ SVGigE_RETURN
Camera_getMapIndexCoordinate(Camera_handle hCamera,
									unsigned int MapIndex,
									unsigned int *ProgrammedCoordinateX, unsigned int *ProgrammedCoordinateY )
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getMapIndexCoordinate)
  GigEFunc[Func_Camera_getMapIndexCoordinate].function_pointer)(hCamera, MapIndex, ProgrammedCoordinateX, ProgrammedCoordinateY);
}


__usrdllexport__ SVGigE_RETURN
Camera_deletePixelCoordinateFromMap(Camera_handle hCamera, unsigned int MapIndex)
{
	// Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_deletePixelCoordinateFromMap)
  GigEFunc[Func_Camera_deletePixelCoordinateFromMap].function_pointer)(hCamera, MapIndex );

}




//*****************************************************************************************
// 22 - Special control: IOMux configuration
//*****************************************************************************************

SVGigE_RETURN
Camera_getMaxIOMuxIN(Camera_handle hCamera,
                     int *MaxIOMuxINSignals)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getMaxIOMuxIN)
  GigEFunc[Func_Camera_getMaxIOMuxIN].function_pointer)(hCamera, MaxIOMuxINSignals);
}

SVGigE_RETURN
Camera_getMaxIOMuxOUT(Camera_handle hCamera,
                      int *MaxIOMuxOUTSignals)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getMaxIOMuxOUT)
  GigEFunc[Func_Camera_getMaxIOMuxOUT].function_pointer)(hCamera, MaxIOMuxOUTSignals);
}

SVGigE_RETURN
Camera_setIOAssignment(Camera_handle hCamera,
                         SVGigE_IOMux_OUT IOMuxOUT,
                         unsigned int SignalIOMuxIN)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setIOAssignment)
  GigEFunc[Func_Camera_setIOAssignment].function_pointer)(hCamera, IOMuxOUT, SignalIOMuxIN);
}

SVGigE_RETURN
Camera_getIOAssignment(Camera_handle hCamera,
                       SVGigE_IOMux_OUT IOMuxOUT,
                       unsigned int *ProgrammedIOMuxIN)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getIOAssignment)
  GigEFunc[Func_Camera_getIOAssignment].function_pointer)(hCamera, IOMuxOUT, ProgrammedIOMuxIN);
}

//*****************************************************************************************
// 23 - Special control: IO control
//*****************************************************************************************

SVGigE_RETURN
Camera_setIOMuxIN(Camera_handle hCamera,
                  unsigned int VectorIOMuxIN)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setIOMuxIN)
  GigEFunc[Func_Camera_setIOMuxIN].function_pointer)(hCamera, VectorIOMuxIN);
}

SVGigE_RETURN
Camera_getIOMuxIN(Camera_handle hCamera,
                  unsigned int *ProgrammedVectorIOMuxIN)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getIOMuxIN)
  GigEFunc[Func_Camera_getIOMuxIN].function_pointer)(hCamera, ProgrammedVectorIOMuxIN);
}

SVGigE_RETURN
Camera_setIO(Camera_handle hCamera,
             SVGigE_IOMux_IN SignalIOMuxIN,
             IO_SIGNAL SignalValue)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setIO)
  GigEFunc[Func_Camera_setIO].function_pointer)(hCamera, SignalIOMuxIN, SignalValue);
}

SVGigE_RETURN
Camera_getIO(Camera_handle hCamera,
             SVGigE_IOMux_IN SignalIOMuxIN,
             IO_SIGNAL *ProgrammedSignalValue)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getIO)
  GigEFunc[Func_Camera_getIO].function_pointer)(hCamera, SignalIOMuxIN, ProgrammedSignalValue);
}

SVGigE_RETURN
Camera_setAcqLEDOverride(Camera_handle hCamera,
                         bool isOverrideActive)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAcqLEDOverride)
  GigEFunc[Func_Camera_setAcqLEDOverride].function_pointer)(hCamera, isOverrideActive);
}

SVGigE_RETURN
Camera_getAcqLEDOverride(Camera_handle hCamera,
                         bool *isOverrideActive)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAcqLEDOverride)
  GigEFunc[Func_Camera_getAcqLEDOverride].function_pointer)(hCamera, isOverrideActive);
}

SVGigE_RETURN
Camera_setLEDIntensity(Camera_handle hCamera,
                       int LEDIntensity)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLEDIntensity)
  GigEFunc[Func_Camera_setLEDIntensity].function_pointer)(hCamera, LEDIntensity);
}

SVGigE_RETURN
Camera_getLEDIntensity(Camera_handle hCamera,
                       int *ProgrammedLEDIntensity)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLEDIntensity)
  GigEFunc[Func_Camera_getLEDIntensity].function_pointer)(hCamera, ProgrammedLEDIntensity);
}


//*****************************************************************************************
// 24 - Special control: Serial communication
//*****************************************************************************************

SVGigE_RETURN
Camera_setUARTBuffer(Camera_handle hCamera,
                     unsigned char *Data,
                     int DataLength)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setUARTBuffer)
  GigEFunc[Func_Camera_setUARTBuffer].function_pointer)(hCamera, Data, DataLength);
}

SVGigE_RETURN
Camera_getUARTBuffer(Camera_handle hCamera,
                     unsigned char *Data,
                     int *DataLengthReceived,
                     int DataLengthMax,
                     float Timeout)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getUARTBuffer)
  GigEFunc[Func_Camera_getUARTBuffer].function_pointer)(hCamera, Data, DataLengthReceived, DataLengthMax, Timeout);
}

SVGigE_RETURN
Camera_setUARTBaud(Camera_handle hCamera,
                   SVGigE_BaudRate BaudRate)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setUARTBaud)
  GigEFunc[Func_Camera_setUARTBaud].function_pointer)(hCamera, BaudRate);
}

SVGigE_RETURN
Camera_getUARTBaud(Camera_handle hCamera,
                   SVGigE_BaudRate *ProgrammedBaudRate)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getUARTBaud)
  GigEFunc[Func_Camera_getUARTBaud].function_pointer)(hCamera, ProgrammedBaudRate);
}

//*****************************************************************************************
// 25 - Special control: Direct register and memory access
//*****************************************************************************************

SVGigE_RETURN
Camera_setGigECameraRegister(Camera_handle hCamera,
                             unsigned int RegisterAddress,
                             unsigned int RegisterValue,
                             unsigned int GigECameraAccessKey)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setGigECameraRegister)
  GigEFunc[Func_Camera_setGigECameraRegister].function_pointer)(hCamera, RegisterAddress, RegisterValue, GigECameraAccessKey);
}

SVGigE_RETURN
Camera_getGigECameraRegister(Camera_handle hCamera,
                             unsigned int RegisterAddress,
                             unsigned int *ProgramedRegisterValue,
                             unsigned int GigECameraAccessKey)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getGigECameraRegister)
  GigEFunc[Func_Camera_getGigECameraRegister].function_pointer)(hCamera, RegisterAddress, ProgramedRegisterValue, GigECameraAccessKey);
}

SVGigE_RETURN
Camera_writeGigECameraMemory(Camera_handle hCamera,
                             unsigned int  MemoryAddress,
                             unsigned char *DataBlock,
                             unsigned int  DataLength,
                             unsigned int  GigECameraAccessKey)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_writeGigECameraMemory)
  GigEFunc[Func_Camera_writeGigECameraMemory].function_pointer)(hCamera, MemoryAddress, DataBlock, DataLength, GigECameraAccessKey);
}

SVGigE_RETURN
Camera_readGigECameraMemory(Camera_handle hCamera,
                            unsigned int  MemoryAddress,
                            unsigned char *DataBlock,
                            unsigned int  DataLength,
                            unsigned int  GigECameraAccessKey)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_readGigECameraMemory)
  GigEFunc[Func_Camera_readGigECameraMemory].function_pointer)(hCamera, MemoryAddress, DataBlock, DataLength, GigECameraAccessKey);
}

SVGigE_RETURN
Camera_forceOpenConnection(Camera_handle hCamera, float Timeout)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_forceOpenConnection)
  GigEFunc[Func_Camera_forceOpenConnection].function_pointer)(hCamera, Timeout);
}

//*****************************************************************************************
// 26 - Special control: Persistent settings and recovery
//*****************************************************************************************

SVGigE_RETURN
Camera_writeEEPROM(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_writeEEPROM)
  GigEFunc[Func_Camera_writeEEPROM].function_pointer)(hCamera);
}

SVGigE_RETURN
Camera_readEEPROM(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_readEEPROM)
  GigEFunc[Func_Camera_readEEPROM].function_pointer)(hCamera);
}

SVGigE_RETURN
Camera_restoreFactoryDefaults(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_restoreFactoryDefaults)
  GigEFunc[Func_Camera_restoreFactoryDefaults].function_pointer)(hCamera);
}


SVGigE_RETURN
Camera_loadSettingsFromXml(Camera_handle hCamera,
                           const char *Filename)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_loadSettingsFromXml)
  GigEFunc[Func_Camera_loadSettingsFromXml].function_pointer)(hCamera,
                                Filename);
}


SVGigE_RETURN
Camera_SaveSettingsToXml(Camera_handle hCamera,
                         const char *Filename)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_SaveSettingsToXml)
  GigEFunc[Func_Camera_SaveSettingsToXml].function_pointer)(hCamera,Filename );

}







//*****************************************************************************************
// 27 - General functions
//*****************************************************************************************

SVGigE_RETURN
SVGigE_estimateWhiteBalance(unsigned char *DataRGB, int DataRGBLength, float *Red, float *Green, float *Blue)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_SVGigE_estimateWhiteBalance)
  GigEFunc[Func_SVGigE_estimateWhiteBalance].function_pointer)(DataRGB, DataRGBLength, Red, Green, Blue);
}

SVGigE_RETURN
SVGigE_estimateWhiteBalanceExtended(unsigned char *DataRGB, int PixelNumber, int &Red, int &Green, int &Blue,SVGIGE_Whitebalance_SELECT  Whitebalance_Art )
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_SVGigE_estimateWhiteBalanceExtended)
  GigEFunc[Func_SVGigE_estimateWhiteBalanceExtended].function_pointer)(DataRGB, PixelNumber, Red, Green, Blue,Whitebalance_Art);
}


SVGigE_RETURN
SVGigE_writeImageToBitmapFile(const char *Filename, unsigned char *Data, int SizeX, int SizeY, GVSP_PIXEL_TYPE PixelType)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_SVGigE_writeImageToBitmapFile)
  GigEFunc[Func_SVGigE_writeImageToBitmapFile].function_pointer)(Filename, Data, SizeX, SizeY, PixelType);
}

SVGigE_RETURN
SVGigE_installFilterDriver(const char *PathToDriverPackage, const char *FilenameINF, const char *FilenameINF_m)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_SVGigE_installFilterDriver)
  GigEFunc[Func_SVGigE_installFilterDriver].function_pointer)(PathToDriverPackage,FilenameINF,FilenameINF_m);
}

SVGigE_RETURN
SVGigE_uninstallFilterDriver()
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_SVGigE_uninstallFilterDriver)
  GigEFunc[Func_SVGigE_uninstallFilterDriver].function_pointer)();
}

//*****************************************************************************************
// 28 - Diagnostics
//*****************************************************************************************

const char* Error_getMessage(SVGigE_RETURN ReturnCode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Error_getMessage)
  GigEFunc[Func_Error_getMessage].function_pointer)(ReturnCode);
}

SVGigE_RETURN
Camera_registerForLogMessages(Camera_handle hCamera,
                              int  LogLevel,
                              const char *LogFilename,
                              LogMessageCallback LogCallback,
                              void *MessageContext)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_registerForLogMessages)
  GigEFunc[Func_Camera_registerForLogMessages].function_pointer)(hCamera, LogLevel, LogFilename, LogCallback, MessageContext);
}

//*****************************************************************************************
// 29 - Lens Control
//*****************************************************************************************


SVGigE_RETURN Camera_isLensAvailable(Camera_handle hCamera, bool *isAvailable)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_isLensAvailable)
  GigEFunc[Func_Camera_isLensAvailable].function_pointer)(hCamera, isAvailable);
}

const char *
Camera_getLensName(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return NULL;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensName)
  GigEFunc[Func_Camera_getLensName].function_pointer)(hCamera);
}

SVGigE_RETURN	Camera_setLensFocalLenght(Camera_handle hCamera, unsigned int ProgrammedFocalLenght)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLensFocalLenght)
  GigEFunc[Func_Camera_setLensFocalLenght].function_pointer)(hCamera, ProgrammedFocalLenght);
}

SVGigE_RETURN	Camera_getLensFocalLenght(Camera_handle hCamera, unsigned int *ProgrammedFocalLenght)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocalLenght)
  GigEFunc[Func_Camera_getLensFocalLenght].function_pointer)(hCamera, ProgrammedFocalLenght);
}

SVGigE_RETURN Camera_getLensFocalLenghtMin(Camera_handle hCamera, unsigned int *FocalLenghtMin)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocalLenghtMin)
  GigEFunc[Func_Camera_getLensFocalLenghtMin].function_pointer)(hCamera, FocalLenghtMin);
}

SVGigE_RETURN	Camera_getLensFocalLenghtMax(Camera_handle hCamera, unsigned int *FocalLenghtMax)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocalLenghtMax)
  GigEFunc[Func_Camera_getLensFocalLenghtMax].function_pointer)(hCamera, FocalLenghtMax);
}


SVGigE_RETURN Camera_setLensFocusUnit(Camera_handle hCamera, FOCUS_UNIT Selected_unit )
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLensFocusUnit)
  GigEFunc[Func_Camera_setLensFocusUnit].function_pointer)(hCamera, Selected_unit);
}

SVGigE_RETURN Camera_getLensFocusUnit(Camera_handle hCamera, FOCUS_UNIT *Selected_unit )
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocusUnit)
  GigEFunc[Func_Camera_getLensFocusUnit].function_pointer)(hCamera, Selected_unit);
}


SVGigE_RETURN Camera_setLensFocus(Camera_handle hCamera, unsigned int ProgrammedFocus)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLensFocus)
  GigEFunc[Func_Camera_setLensFocus].function_pointer)(hCamera, ProgrammedFocus);
}

SVGigE_RETURN Camera_getLensFocus(Camera_handle hCamera, unsigned int *ProgrammedFocus)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocus)
  GigEFunc[Func_Camera_getLensFocus].function_pointer)(hCamera, ProgrammedFocus);
}

SVGigE_RETURN Camera_getLensFocusMin(Camera_handle hCamera, unsigned int *FocusMin)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocusMin)
  GigEFunc[Func_Camera_getLensFocusMin].function_pointer)(hCamera, FocusMin);
}

SVGigE_RETURN Camera_getLensFocusMax(Camera_handle hCamera, unsigned int *FocusMax)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocusMax)
  GigEFunc[Func_Camera_getLensFocusMax].function_pointer)(hCamera, FocusMax);
}

SVGigE_RETURN	Camera_setLensAperture(Camera_handle hCamera, unsigned int ProgrammedAperture)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLensAperture)
  GigEFunc[Func_Camera_setLensAperture].function_pointer)(hCamera, ProgrammedAperture);
}

SVGigE_RETURN	Camera_getLensAperture(Camera_handle hCamera, unsigned int *ProgrammedAperture)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensAperture)
  GigEFunc[Func_Camera_getLensAperture].function_pointer)(hCamera, ProgrammedAperture);
}

SVGigE_RETURN	Camera_getLensApertureMin(Camera_handle hCamera, unsigned int *ApertureMin)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensApertureMin)
  GigEFunc[Func_Camera_getLensApertureMin].function_pointer)(hCamera, ApertureMin);
}

SVGigE_RETURN	Camera_getLensApertureMax(Camera_handle hCamera, unsigned int *ApertureMax)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensApertureMax)
  GigEFunc[Func_Camera_getLensApertureMax].function_pointer)(hCamera, ApertureMax);
}



//*****************************************************************************************
// 99 - Deprecated functions
//*****************************************************************************************

SVGigE_RETURN
Camera_startAcquisitionCycle(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_startAcquisitionCycle)
  GigEFunc[Func_Camera_startAcquisitionCycle].function_pointer)(hCamera);
}

SVGigE_RETURN
Camera_setTapCalibration(Camera_handle hCamera,
                         unsigned int TapID,
                         unsigned int Gain,
                         unsigned int Offset)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setTapCalibration)
  GigEFunc[Func_Camera_setTapCalibration].function_pointer)(hCamera, TapID, Gain, Offset);
}

SVGigE_RETURN
Camera_getTapCalibration(Camera_handle hCamera,
                         unsigned int TapID,
                         unsigned int *Gain,
                         unsigned int *Offset)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTapCalibration)
  GigEFunc[Func_Camera_getTapCalibration].function_pointer)(hCamera, TapID, Gain, Offset);
}

SVGigE_RETURN
Camera_setLUTMode(Camera_handle hCamera,
                  LUT_MODE LUTMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLookupTableMode)
  GigEFunc[Func_Camera_setLookupTableMode].function_pointer)(hCamera, LUTMode);
}

SVGigE_RETURN
Camera_getLUTMode(Camera_handle hCamera,
                  LUT_MODE *ProgrammedLUTMode)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLookupTableMode)
  GigEFunc[Func_Camera_getLookupTableMode].function_pointer)(hCamera, ProgrammedLUTMode);
}

SVGigE_RETURN
Camera_createLUTwhiteBalance(Camera_handle hCamera,
                             float Red,
                             float Green ,
                             float Blue)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_createLUTwhiteBalance)
  GigEFunc[Func_Camera_createLUTwhiteBalance].function_pointer)(hCamera, Red, Green, Blue);
}

SVGigE_RETURN
Camera_stampTimestamp(Camera_handle hCamera,
                      int TimestampIndex)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_stampTimestamp)
  GigEFunc[Func_Camera_stampTimestamp].function_pointer)(hCamera, TimestampIndex);
}

SVGigE_RETURN
Camera_getTimestamp(Camera_handle hCamera,
                    int TimestampIndex,
                    double *Timestamp)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTimestamp)
  GigEFunc[Func_Camera_getTimestamp].function_pointer)(hCamera, TimestampIndex, Timestamp);
}

unsigned
Image_getDebugInfo(Image_handle hImage, int Index)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_getDebugInfo)
  GigEFunc[Func_Image_getDebugInfo].function_pointer)(hImage, Index);
}

SVGigE_RETURN
Camera_setTapBalance(Camera_handle hCamera,
                     float TapBalance)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setTapBalance)
  GigEFunc[Func_Camera_setTapBalance].function_pointer)(hCamera, TapBalance);
}

SVGigE_RETURN
Camera_getTapBalance(Camera_handle hCamera,
                     float *TapBalance)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTapBalance)
  GigEFunc[Func_Camera_getTapBalance].function_pointer)(hCamera, TapBalance);
}

SVGigE_RETURN
StreamingChannel_setChannelTimeout(StreamingChannel_handle hStreamingChannel,
                                   float ChannelTimeout)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_setChannelTimeout)
  GigEFunc[Func_StreamingChannel_setChannelTimeout].function_pointer)(hStreamingChannel,ChannelTimeout);
}

SVGigE_RETURN
StreamingChannel_getChannelTimeout(StreamingChannel_handle hStreamingChannel,
                                   float *ProgrammedChannelTimeout)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getChannelTimeout)
  GigEFunc[Func_StreamingChannel_getChannelTimeout].function_pointer)(hStreamingChannel,ProgrammedChannelTimeout);
}


SVGigE_RETURN
Camera_setTapUserSettings(Camera_handle hCamera,
													float TapUserGain,
													float TapUserOffset)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setTapUserSettings)
  GigEFunc[Func_Camera_setTapUserSettings].function_pointer)(hCamera, TapUserGain, TapUserOffset);
}

SVGigE_RETURN
Camera_getTapUserSettings(Camera_handle hCamera,
													float *TapUserGain,
													float *TapUserOffset)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTapUserSettings)
  GigEFunc[Func_Camera_getTapUserSettings].function_pointer)(hCamera, TapUserGain, TapUserOffset);
}

SVGigE_RETURN
Camera_saveTapBalanceSettings(Camera_handle hCamera,
                              const char *Filename)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_saveTapBalanceSettings)
  GigEFunc[Func_Camera_saveTapBalanceSettings].function_pointer)(hCamera, Filename);
}

SVGigE_RETURN
Camera_loadTapBalanceSettings(Camera_handle hCamera,
                              const char *Filename)
{
  // Check DLL availability
  if( NULL == GigEDLL )
    return SVGigE_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_loadTapBalanceSettings)
  GigEFunc[Func_Camera_loadTapBalanceSettings].function_pointer)(hCamera, Filename);
}

/******************************************************************************************
 * 1394-Based Digital Camera Control Library
 * Bayer pattern decoding functions
 * Copyright (C) Damien Douxchamps <ddouxchamps@users.sf.net>
 *
 * Written by Damien Douxchamps and Frederic Devernay
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *****************************************************************************************/

#ifdef BAYER_CONVERSION_ALGORITHMS

#define CLIP(in, out)\
   in = in < 0 ? 0 : in;\
   in = in > 255 ? 255 : in;\
   out=in;

#define CLIP16(in, out, bits)\
   in = in < 0 ? 0 : in;\
   in = in > ((1<<bits)-1) ? ((1<<bits)-1) : in;\
   out=in;

void
ClearBorders(unsigned char *rgb, int sx, int sy, int w)
{
    int i, j;
    // black edges are added with a width w:
    i = 3 * sx * w - 1;
    j = 3 * sx * sy - 1;
    while (i >= 0) {
	rgb[i--] = 0;
	rgb[j--] = 0;
    }
    i = sx * (sy - 1) * 3 - 1 + w * 3;
    while (i > sx) {
	j = 6 * w;
	while (j > 0) {
	    rgb[i--] = 0;
	    j--;
	}
	i -= (sx - 2 * w) * 3;
    }
}

void
ClearBorders_uint16(uint16_t * rgb, int sx, int sy, int w)
{
    int i, j;

    // black edges:
    i = 3 * sx * w - 1;
    j = 3 * sx * sy - 1;
    while (i >= 0) {
	rgb[i--] = 0;
	rgb[j--] = 0;
    }

    i = sx * (sy - 1) * 3 - 1 + w * 3;
    while (i > sx) {
	j = 6 * w;
	while (j > 0) {
	    rgb[i--] = 0;
	    j--;
	}
	i -= (sx - 2 * w) * 3;
    }

}

/**************************************************************
 *     Color conversion functions for cameras that can        *
 * output raw-Bayer pattern images, such as some Basler and   *
 * Point Grey camera. Most of the algos presented here come   *
 * from http://www-ise.stanford.edu/~tingchen/ and have been  *
 * converted from Matlab to C and extended to all elementary  *
 * patterns.                                                  *
 **************************************************************/

/* 8-bits versions */
/* insprired by OpenCV's Bayer decoding */

void
dc1394_bayer_NearestNeighbor(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;
    int i, imax, iinc;

    /* add black border */
    imax = sx * sy * 3;
    for (i = sx * (sy - 1) * 3; i < imax; i++) {
	rgb[i] = 0;
    }
    iinc = (sx - 1) * 3;
    for (i = (sx - 1) * 3; i < imax; i += iinc) {
	rgb[i++] = 0;
	rgb[i++] = 0;
	rgb[i++] = 0;
    }

    rgb += 1;
    width -= 1;
    height -= 1;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
      //int t0, t1;
	const unsigned char *bayerEnd = bayer + width;

        if (start_with_green) {
            rgb[-blue] = bayer[1];
            rgb[0] = bayer[bayerStep + 1];
            rgb[blue] = bayer[bayerStep];
            bayer++;
            rgb += 3;
        }

        if (blue > 0) {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[-1] = bayer[0];
                rgb[0] = bayer[1];
                rgb[1] = bayer[bayerStep + 1];

                rgb[2] = bayer[2];
                rgb[3] = bayer[bayerStep + 2];
                rgb[4] = bayer[bayerStep + 1];
            }
        } else {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[1] = bayer[0];
                rgb[0] = bayer[1];
                rgb[-1] = bayer[bayerStep + 1];

                rgb[4] = bayer[2];
                rgb[3] = bayer[bayerStep + 2];
                rgb[2] = bayer[bayerStep + 1];
            }
        }

        if (bayer < bayerEnd) {
            rgb[-blue] = bayer[0];
            rgb[0] = bayer[1];
            rgb[blue] = bayer[bayerStep + 1];
            bayer++;
            rgb += 3;
        }

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}

/* OpenCV's Bayer decoding */
void
dc1394_bayer_Bilinear(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    /*
       the two letters  of the OpenCV name are respectively
       the 4th and 3rd letters from the blinky name,
       and we also have to switch R and B (OpenCV is BGR)

       CV_BayerBG2BGR <-> DC1394_COLOR_FILTER_BGGR
       CV_BayerGB2BGR <-> DC1394_COLOR_FILTER_GBRG
       CV_BayerGR2BGR <-> DC1394_COLOR_FILTER_GRBG

       int blue = tile == CV_BayerBG2BGR || tile == CV_BayerGB2BGR ? -1 : 1;
       int start_with_green = tile == CV_BayerGB2BGR || tile == CV_BayerGR2BGR;
     */
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;

    ClearBorders(rgb, sx, sy, 1);
    rgb += rgbStep + 3 + 1;
    height -= 2;
    width -= 2;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
	int t0, t1;
	const unsigned char *bayerEnd = bayer + width;

	if (start_with_green) {
	    /* OpenCV has a bug in the next line, which was
	       t0 = (bayer[0] + bayer[bayerStep * 2] + 1) >> 1; */
	    t0 = (bayer[1] + bayer[bayerStep * 2 + 1] + 1) >> 1;
	    t1 = (bayer[bayerStep] + bayer[bayerStep + 2] + 1) >> 1;
	    rgb[-blue] = (unsigned char) t0;
	    rgb[0] = bayer[bayerStep + 1];
	    rgb[blue] = (unsigned char) t1;
	    bayer++;
	    rgb += 3;
	}

	if (blue > 0) {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		      bayer[bayerStep * 2 + 2] + 2) >> 2;
		t1 = (bayer[1] + bayer[bayerStep] +
		      bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		      2) >> 2;
		rgb[-1] = (unsigned char) t0;
		rgb[0] = (unsigned char) t1;
		rgb[1] = bayer[bayerStep + 1];

		t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
		t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		      1) >> 1;
		rgb[2] = (unsigned char) t0;
		rgb[3] = bayer[bayerStep + 2];
		rgb[4] = (unsigned char) t1;
	    }
	} else {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		      bayer[bayerStep * 2 + 2] + 2) >> 2;
		t1 = (bayer[1] + bayer[bayerStep] +
		      bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		      2) >> 2;
		rgb[1] = (unsigned char) t0;
		rgb[0] = (unsigned char) t1;
		rgb[-1] = bayer[bayerStep + 1];

		t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
		t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		      1) >> 1;
		rgb[4] = (unsigned char) t0;
		rgb[3] = bayer[bayerStep + 2];
		rgb[2] = (unsigned char) t1;
	    }
	}

	if (bayer < bayerEnd) {
	    t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		  bayer[bayerStep * 2 + 2] + 2) >> 2;
	    t1 = (bayer[1] + bayer[bayerStep] +
		  bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		  2) >> 2;
	    rgb[-blue] = (unsigned char) t0;
	    rgb[0] = (unsigned char) t1;
	    rgb[blue] = bayer[bayerStep + 1];
	    bayer++;
	    rgb += 3;
	}

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}

/* High-Quality Linear Interpolation For Demosaicing Of
   Bayer-Patterned Color Images, by Henrique S. Malvar, Li-wei He, and
   Ross Cutler, in ICASSP'04 */
void
dc1394_bayer_HQLinear(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;

    ClearBorders(rgb, sx, sy, 2);
    rgb += 2 * rgbStep + 6 + 1;
    height -= 4;
    width -= 4;

    /* We begin with a (+1 line,+1 column) offset with respect to bilinear decoding, so start_with_green is the same, but blue is opposite */
    blue = -blue;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
	int t0, t1;
	const unsigned char *bayerEnd = bayer + width;
	const int bayerStep2 = bayerStep * 2;
	const int bayerStep3 = bayerStep * 3;
	const int bayerStep4 = bayerStep * 4;

	if (start_with_green) {
	    /* at green pixel */
	    rgb[0] = bayer[bayerStep2 + 2];
	    t0 = rgb[0] * 5
		+ ((bayer[bayerStep + 2] + bayer[bayerStep3 + 2]) << 2)
		- bayer[2]
		- bayer[bayerStep + 1]
		- bayer[bayerStep + 3]
		- bayer[bayerStep3 + 1]
		- bayer[bayerStep3 + 3]
		- bayer[bayerStep4 + 2]
		+ ((bayer[bayerStep2] + bayer[bayerStep2 + 4] + 1) >> 1);
	    t1 = rgb[0] * 5 +
		((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3]) << 2)
		- bayer[bayerStep2]
		- bayer[bayerStep + 1]
		- bayer[bayerStep + 3]
		- bayer[bayerStep3 + 1]
		- bayer[bayerStep3 + 3]
		- bayer[bayerStep2 + 4]
		+ ((bayer[2] + bayer[bayerStep4 + 2] + 1) >> 1);
	    t0 = (t0 + 4) >> 3;
	    CLIP(t0, rgb[-blue]);
	    t1 = (t1 + 4) >> 3;
	    CLIP(t1, rgb[blue]);
	    bayer++;
	    rgb += 3;
	}

	if (blue > 0) {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		/* B at B */
		rgb[1] = bayer[bayerStep2 + 2];
		/* R at B */
		t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		       bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
		    -
		    (((bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						     2]) * 3 + 1) >> 1)
		    + rgb[1] * 6;
		/* G at B */
		t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		       bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2]) << 1)
		    - (bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		    + (rgb[1] << 2);
		t0 = (t0 + 4) >> 3;
		CLIP(t0, rgb[-1]);
		t1 = (t1 + 4) >> 3;
		CLIP(t1, rgb[0]);
		/* at green pixel */
		rgb[3] = bayer[bayerStep2 + 3];
		t0 = rgb[3] * 5
		    + ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
		    - bayer[3]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep4 + 3]
		    +
		    ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
		      1) >> 1);
		t1 = rgb[3] * 5 +
		    ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
		    - bayer[bayerStep2 + 1]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep2 + 5]
		    + ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
		t0 = (t0 + 4) >> 3;
		CLIP(t0, rgb[2]);
		t1 = (t1 + 4) >> 3;
		CLIP(t1, rgb[4]);
	    }
	} else {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		/* R at R */
		rgb[-1] = bayer[bayerStep2 + 2];
		/* B at R */
		t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		       bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
		    -
		    (((bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						     2]) * 3 + 1) >> 1)
		    + rgb[-1] * 6;
		/* G at R */
		t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		       bayer[bayerStep2 + 3] + bayer[bayerStep * 3 +
						     2]) << 1)
		    - (bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		    + (rgb[-1] << 2);
		t0 = (t0 + 4) >> 3;
		CLIP(t0, rgb[1]);
		t1 = (t1 + 4) >> 3;
		CLIP(t1, rgb[0]);

		/* at green pixel */
		rgb[3] = bayer[bayerStep2 + 3];
		t0 = rgb[3] * 5
		    + ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
		    - bayer[3]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep4 + 3]
		    +
		    ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
		      1) >> 1);
		t1 = rgb[3] * 5 +
		    ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
		    - bayer[bayerStep2 + 1]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep2 + 5]
		    + ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
		t0 = (t0 + 4) >> 3;
		CLIP(t0, rgb[4]);
		t1 = (t1 + 4) >> 3;
		CLIP(t1, rgb[2]);
	    }
	}

	if (bayer < bayerEnd) {
	    /* B at B */
	    rgb[blue] = bayer[bayerStep2 + 2];
	    /* R at B */
	    t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		   bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
		-
		(((bayer[2] + bayer[bayerStep2] +
		   bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						 2]) * 3 + 1) >> 1)
		+ rgb[blue] * 6;
	    /* G at B */
	    t1 = (((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		    bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2])) << 1)
		- (bayer[2] + bayer[bayerStep2] +
		   bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		+ (rgb[blue] << 2);
	    t0 = (t0 + 4) >> 3;
	    CLIP(t0, rgb[-blue]);
	    t1 = (t1 + 4) >> 3;
	    CLIP(t1, rgb[0]);
	    bayer++;
	    rgb += 3;
	}

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}

/* coriander's Bayer decoding (GPL) */
/* Edge Sensing Interpolation II from http://www-ise.stanford.edu/~tingchen/ */
/*   (Laroche,Claude A.  "Apparatus and method for adaptively
     interpolating a full color image utilizing chrominance gradients"
     U.S. Patent 5,373,322) */
void
dc1394_bayer_EdgeSense(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    unsigned char *outR, *outG, *outB;
    register int i, j;
    int dh, dv;
    int tmp;

    // sx and sy should be even
    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
	outR = &rgb[0];
	outG = &rgb[1];
	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
	outR = &rgb[2];
	outG = &rgb[1];
	outB = &rgb[0];
	break;
    default:
 //	fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
 //	return;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_GBRG:
	// copy original RGB data to output images
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 0; j < sx; j += 2) {
	  outG[(i + j) * 3] = bayer[i + j];
	  outG[(i + sx + (j + 1)) * 3] = bayer[i + sx + (j + 1)];
	  outR[(i + j + 1) * 3] = bayer[i + j + 1];
	  outB[(i + sx + j) * 3] = bayer[i + sx + j];
	}
      }
      // process GREEN channel
      for (i = 3*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 3; j += 2) {
	  dh = abs(((outB[(i + j - 2) * 3] +
		     outB[(i + j + 2) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  dv = abs(((outB[(i - (sx<<1) + j) * 3] +
		     outB[(i + (sx<<1) + j) * 3]) >> 1)  -
		   outB[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP(tmp, outG[(i + j) * 3]);
	}
      }

      for (i = 2*sx; i < (sy - 3)*sx; i += (sx<<1)) {
	for (j = 3; j < sx - 2; j += 2) {
	  dh = abs(((outR[(i + j - 2) * 3] +
		     outR[(i + j + 2) * 3]) >>1 ) -
		   outR[(i + j) * 3]);
	  dv = abs(((outR[(i - (sx<<1) + j) * 3] +
		     outR[(i + (sx<<1) + j) * 3]) >>1 ) -
		   outR[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP(tmp, outG[(i + j) * 3]);
	}
      }
      // process RED channel
      for (i = 0; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outR[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
      }
      for (i = sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 1; j < sx; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outR[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outR[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outR[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outR[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
      }

      // process BLUE channel
      for (i = sx; i < sy*sx; i += (sx<<1)) {
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outB[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
      }
      for (i = 2*sx; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 0; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outB[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outB[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outB[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outB[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
      }
      break;

    case DC1394_COLOR_FILTER_BGGR:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_RGGB:
	// copy original RGB data to output images
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 0; j < sx; j += 2) {
	  outB[(i + j) * 3] = bayer[i + j];
	  outR[(i + sx + (j + 1)) * 3] = bayer[i + sx + (j + 1)];
	  outG[(i + j + 1) * 3] = bayer[i + j + 1];
	  outG[(i + sx + j) * 3] = bayer[i + sx + j];
	}
      }
      // process GREEN channel
      for (i = 2*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 3; j += 2) {
	  dh = abs(((outB[(i + j - 2) * 3] +
		    outB[(i + j + 2) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  dv = abs(((outB[(i - (sx<<1) + j) * 3] +
		    outB[(i + (sx<<1) + j) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP(tmp, outG[(i + j) * 3]);
	}
      }
      for (i = 3*sx; i < (sy - 3)*sx; i += (sx<<1)) {
	for (j = 3; j < sx - 2; j += 2) {
	  dh = abs(((outR[(i + j - 2) * 3] +
		    outR[(i + j + 2) * 3]) >> 1) -
		   outR[(i + j) * 3]);
	  dv = abs(((outR[(i - (sx<<1) + j) * 3] +
		    outR[(i + (sx<<1) + j) * 3]) >> 1) -
		   outR[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >>1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >>1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >>2;
	  }
	  CLIP(tmp, outG[(i + j) * 3]);
	}
      }
      // process RED channel
      for (i = sx; i < (sy - 1)*sx; i += (sx<<1)) {	// G-points (1/2)
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outR[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >>1);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
      }
      for (i = 2*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 1; j < sx; j += 2) {	// G-points (2/2)
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outR[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
	for (j = 2; j < sx - 1; j += 2) {	// B-points
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outR[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outR[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outR[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
      }

      // process BLUE channel
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outB[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
      }
      for (i = sx; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 0; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outB[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outB[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outB[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outB[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
      }
      break;
    default:			//---------------------------------------------------------
//      fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
//      return;
      break;
    }

    ClearBorders(rgb, sx, sy, 3);
}

/* coriander's Bayer decoding (GPL) */
void
dc1394_bayer_Downsample(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    unsigned char *outR, *outG, *outB;
    register int i, j;
    int tmp;

    sx *= 2;
    sy *= 2;

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
	outR = &rgb[0];
	outG = &rgb[1];
	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
	outR = &rgb[2];
	outG = &rgb[1];
	outB = &rgb[0];
	break;
    default:
//	fprintf(stderr, "Bad Bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_GBRG:
	for (i = 0; i < sy*sx; i += (sx<<1)) {
	    for (j = 0; j < sx; j += 2) {
		tmp =
		    ((bayer[i + j] + bayer[i + sx + j + 1]) >> 1);
		CLIP(tmp, outG[((i >> 2) + (j >> 1)) * 3]);
		tmp = bayer[i + sx + j + 1];
		CLIP(tmp, outR[((i >> 2) + (j >> 1)) * 3]);
		tmp = bayer[i + sx + j];
		CLIP(tmp, outB[((i >> 2) + (j >> 1)) * 3]);
	    }
	}
	break;
    case DC1394_COLOR_FILTER_BGGR:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_RGGB:
	for (i = 0; i < sy*sx; i += (sx<<1)) {
	    for (j = 0; j < sx; j += 2) {
		tmp =
		    ((bayer[i + sx + j] + bayer[i + j + 1]) >> 1);
		CLIP(tmp, outG[((i >> 2) + (j >> 1)) * 3]);
		tmp = bayer[i + sx + j + 1];
		CLIP(tmp, outR[((i >> 2) + (j >> 1)) * 3]);
		tmp = bayer[i + j];
		CLIP(tmp, outB[((i >> 2) + (j >> 1)) * 3]);
	    }
	}
	break;
    default:			//---------------------------------------------------------
//	fprintf(stderr, "Bad Bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

}

/* this is the method used inside AVT cameras. See AVT docs. */
void
dc1394_bayer_Simple(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    int blue = tile == DC1394_COLOR_FILTER_BGGR
        || tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
        || tile == DC1394_COLOR_FILTER_GRBG;
    int i, imax, iinc;

    /* add black border */
    imax = sx * sy * 3;
    for (i = sx * (sy - 1) * 3; i < imax; i++) {
        rgb[i] = 0;
    }
    iinc = (sx - 1) * 3;
    for (i = (sx - 1) * 3; i < imax; i += iinc) {
        rgb[i++] = 0;
        rgb[i++] = 0;
        rgb[i++] = 0;
    }

    rgb += 1;
    width -= 1;
    height -= 1;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
        const unsigned char *bayerEnd = bayer + width;

        if (start_with_green) {
            rgb[-blue] = bayer[1];
            rgb[0] = (bayer[0] + bayer[bayerStep + 1] + 1) >> 1;
            rgb[blue] = bayer[bayerStep];
            bayer++;
            rgb += 3;
        }

        if (blue > 0) {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[-1] = bayer[0];
                rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
                rgb[1] = bayer[bayerStep + 1];

                rgb[2] = bayer[2];
                rgb[3] = (bayer[1] + bayer[bayerStep + 2] + 1) >> 1;
                rgb[4] = bayer[bayerStep + 1];
            }
        } else {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[1] = bayer[0];
                rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
                rgb[-1] = bayer[bayerStep + 1];

                rgb[4] = bayer[2];
                rgb[3] = (bayer[1] + bayer[bayerStep + 2] + 1) >> 1;
                rgb[2] = bayer[bayerStep + 1];
            }
        }

        if (bayer < bayerEnd) {
            rgb[-blue] = bayer[0];
            rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
            rgb[blue] = bayer[bayerStep + 1];
            bayer++;
            rgb += 3;
        }

        bayer -= width;
        rgb -= width * 3;

        blue = -blue;
        start_with_green = !start_with_green;
    }
}

/* 16-bits versions */

/* insprired by OpenCV's Bayer decoding */
void
dc1394_bayer_NearestNeighbor_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;
    int i, iinc, imax;

    /* add black border */
    imax = sx * sy * 3;
    for (i = sx * (sy - 1) * 3; i < imax; i++) {
	rgb[i] = 0;
    }
    iinc = (sx - 1) * 3;
    for (i = (sx - 1) * 3; i < imax; i += iinc) {
	rgb[i++] = 0;
	rgb[i++] = 0;
	rgb[i++] = 0;
    }

    rgb += 1;
    height -= 1;
    width -= 1;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
      //int t0, t1;
	const uint16_t *bayerEnd = bayer + width;

        if (start_with_green) {
            rgb[-blue] = bayer[1];
            rgb[0] = bayer[bayerStep + 1];
            rgb[blue] = bayer[bayerStep];
            bayer++;
            rgb += 3;
        }

        if (blue > 0) {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[-1] = bayer[0];
                rgb[0] = bayer[1];
                rgb[1] = bayer[bayerStep + 1];

                rgb[2] = bayer[2];
                rgb[3] = bayer[bayerStep + 2];
                rgb[4] = bayer[bayerStep + 1];
            }
        } else {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[1] = bayer[0];
                rgb[0] = bayer[1];
                rgb[-1] = bayer[bayerStep + 1];

                rgb[4] = bayer[2];
                rgb[3] = bayer[bayerStep + 2];
                rgb[2] = bayer[bayerStep + 1];
            }
        }

        if (bayer < bayerEnd) {
            rgb[-blue] = bayer[0];
            rgb[0] = bayer[1];
            rgb[blue] = bayer[bayerStep + 1];
            bayer++;
            rgb += 3;
        }

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}
/* OpenCV's Bayer decoding */
void
dc1394_bayer_Bilinear_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;

    rgb += rgbStep + 3 + 1;
    height -= 2;
    width -= 2;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
	int t0, t1;
	const uint16_t *bayerEnd = bayer + width;

	if (start_with_green) {
	    /* OpenCV has a bug in the next line, which was
	       t0 = (bayer[0] + bayer[bayerStep * 2] + 1) >> 1; */
	    t0 = (bayer[1] + bayer[bayerStep * 2 + 1] + 1) >> 1;
	    t1 = (bayer[bayerStep] + bayer[bayerStep + 2] + 1) >> 1;
	    rgb[-blue] = (uint16_t) t0;
	    rgb[0] = bayer[bayerStep + 1];
	    rgb[blue] = (uint16_t) t1;
	    bayer++;
	    rgb += 3;
	}

	if (blue > 0) {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		      bayer[bayerStep * 2 + 2] + 2) >> 2;
		t1 = (bayer[1] + bayer[bayerStep] +
		      bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		      2) >> 2;
		rgb[-1] = (uint16_t) t0;
		rgb[0] = (uint16_t) t1;
		rgb[1] = bayer[bayerStep + 1];

		t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
		t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		      1) >> 1;
		rgb[2] = (uint16_t) t0;
		rgb[3] = bayer[bayerStep + 2];
		rgb[4] = (uint16_t) t1;
	    }
	} else {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		      bayer[bayerStep * 2 + 2] + 2) >> 2;
		t1 = (bayer[1] + bayer[bayerStep] +
		      bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		      2) >> 2;
		rgb[1] = (uint16_t) t0;
		rgb[0] = (uint16_t) t1;
		rgb[-1] = bayer[bayerStep + 1];

		t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
		t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		      1) >> 1;
		rgb[4] = (uint16_t) t0;
		rgb[3] = bayer[bayerStep + 2];
		rgb[2] = (uint16_t) t1;
	    }
	}

	if (bayer < bayerEnd) {
	    t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		  bayer[bayerStep * 2 + 2] + 2) >> 2;
	    t1 = (bayer[1] + bayer[bayerStep] +
		  bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		  2) >> 2;
	    rgb[-blue] = (uint16_t) t0;
	    rgb[0] = (uint16_t) t1;
	    rgb[blue] = bayer[bayerStep + 1];
	    bayer++;
	    rgb += 3;
	}

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}

/* High-Quality Linear Interpolation For Demosaicing Of
   Bayer-Patterned Color Images, by Henrique S. Malvar, Li-wei He, and
   Ross Cutler, in ICASSP'04 */
void
dc1394_bayer_HQLinear_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    /*
       the two letters  of the OpenCV name are respectively
       the 4th and 3rd letters from the blinky name,
       and we also have to switch R and B (OpenCV is BGR)

       CV_BayerBG2BGR <-> DC1394_COLOR_FILTER_BGGR
       CV_BayerGB2BGR <-> DC1394_COLOR_FILTER_GBRG
       CV_BayerGR2BGR <-> DC1394_COLOR_FILTER_GRBG

       int blue = tile == CV_BayerBG2BGR || tile == CV_BayerGB2BGR ? -1 : 1;
       int start_with_green = tile == CV_BayerGB2BGR || tile == CV_BayerGR2BGR;
     */
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;

    ClearBorders_uint16(rgb, sx, sy, 2);
    rgb += 2 * rgbStep + 6 + 1;
    height -= 4;
    width -= 4;

    /* We begin with a (+1 line,+1 column) offset with respect to bilinear decoding, so start_with_green is the same, but blue is opposite */
    blue = -blue;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
	int t0, t1;
	const uint16_t *bayerEnd = bayer + width;
	const int bayerStep2 = bayerStep * 2;
	const int bayerStep3 = bayerStep * 3;
	const int bayerStep4 = bayerStep * 4;

	if (start_with_green) {
	    /* at green pixel */
	    rgb[0] = bayer[bayerStep2 + 2];
	    t0 = rgb[0] * 5
		+ ((bayer[bayerStep + 2] + bayer[bayerStep3 + 2]) << 2)
		- bayer[2]
		- bayer[bayerStep + 1]
		- bayer[bayerStep + 3]
		- bayer[bayerStep3 + 1]
		- bayer[bayerStep3 + 3]
		- bayer[bayerStep4 + 2]
		+ ((bayer[bayerStep2] + bayer[bayerStep2 + 4] + 1) >> 1);
	    t1 = rgb[0] * 5 +
		((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3]) << 2)
		- bayer[bayerStep2]
		- bayer[bayerStep + 1]
		- bayer[bayerStep + 3]
		- bayer[bayerStep3 + 1]
		- bayer[bayerStep3 + 3]
		- bayer[bayerStep2 + 4]
		+ ((bayer[2] + bayer[bayerStep4 + 2] + 1) >> 1);
	    t0 = (t0 + 4) >> 3;
	    CLIP16(t0, rgb[-blue], bits);
	    t1 = (t1 + 4) >> 3;
	    CLIP16(t1, rgb[blue], bits);
	    bayer++;
	    rgb += 3;
	}

	if (blue > 0) {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		/* B at B */
		rgb[1] = bayer[bayerStep2 + 2];
		/* R at B */
		t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		       bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
		    -
		    (((bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						     2]) * 3 + 1) >> 1)
		    + rgb[1] * 6;
		/* G at B */
		t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		       bayer[bayerStep2 + 3] + bayer[bayerStep * 3 +
						     2]) << 1)
		    - (bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		    + (rgb[1] << 2);
		t0 = (t0 + 4) >> 3;
		CLIP16(t0, rgb[-1], bits);
		t1 = (t1 + 4) >> 3;
		CLIP16(t1, rgb[0], bits);
		/* at green pixel */
		rgb[3] = bayer[bayerStep2 + 3];
		t0 = rgb[3] * 5
		    + ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
		    - bayer[3]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep4 + 3]
		    +
		    ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
		      1) >> 1);
		t1 = rgb[3] * 5 +
		    ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
		    - bayer[bayerStep2 + 1]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep2 + 5]
		    + ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
		t0 = (t0 + 4) >> 3;
		CLIP16(t0, rgb[2], bits);
		t1 = (t1 + 4) >> 3;
		CLIP16(t1, rgb[4], bits);
	    }
	} else {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		/* R at R */
		rgb[-1] = bayer[bayerStep2 + 2];
		/* B at R */
		t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		       bayer[bayerStep * 3 + 1] + bayer[bayerStep3 +
							3]) << 1)
		    -
		    (((bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						     2]) * 3 + 1) >> 1)
		    + rgb[-1] * 6;
		/* G at R */
		t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		       bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2]) << 1)
		    - (bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		    + (rgb[-1] << 2);
		t0 = (t0 + 4) >> 3;
		CLIP16(t0, rgb[1], bits);
		t1 = (t1 + 4) >> 3;
		CLIP16(t1, rgb[0], bits);

		/* at green pixel */
		rgb[3] = bayer[bayerStep2 + 3];
		t0 = rgb[3] * 5
		    + ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
		    - bayer[3]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep4 + 3]
		    +
		    ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
		      1) >> 1);
		t1 = rgb[3] * 5 +
		    ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
		    - bayer[bayerStep2 + 1]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep2 + 5]
		    + ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
		t0 = (t0 + 4) >> 3;
		CLIP16(t0, rgb[4], bits);
		t1 = (t1 + 4) >> 3;
		CLIP16(t1, rgb[2], bits);
	    }
	}

	if (bayer < bayerEnd) {
	    /* B at B */
	    rgb[blue] = bayer[bayerStep2 + 2];
	    /* R at B */
	    t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		   bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
		-
		(((bayer[2] + bayer[bayerStep2] +
		   bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						 2]) * 3 + 1) >> 1)
		+ rgb[blue] * 6;
	    /* G at B */
	    t1 = (((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		    bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2])) << 1)
		- (bayer[2] + bayer[bayerStep2] +
		   bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		+ (rgb[blue] << 2);
	    t0 = (t0 + 4) >> 3;
	    CLIP16(t0, rgb[-blue], bits);
	    t1 = (t1 + 4) >> 3;
	    CLIP16(t1, rgb[0], bits);
	    bayer++;
	    rgb += 3;
	}

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}

/* coriander's Bayer decoding (GPL) */
void
dc1394_bayer_EdgeSense_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    uint16_t *outR, *outG, *outB;
    register int i, j;
    int dh, dv;
    int tmp;

    // sx and sy should be even
    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
	outR = &rgb[0];
	outG = &rgb[1];
	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
	outR = &rgb[2];
	outG = &rgb[1];
	outB = &rgb[0];
	break;
    default:
//	fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_GBRG:
	// copy original RGB data to output images
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 0; j < sx; j += 2) {
	  outG[(i + j) * 3] = bayer[i + j];
	  outG[(i + sx + (j + 1)) * 3] = bayer[i + sx + (j + 1)];
	  outR[(i + j + 1) * 3] = bayer[i + j + 1];
	  outB[(i + sx + j) * 3] = bayer[i + sx + j];
	}
      }
      // process GREEN channel
      for (i = 3*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 3; j += 2) {
	  dh = abs(((outB[(i + j - 2) * 3] +
		     outB[(i + j + 2) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  dv = abs(((outB[(i - (sx<<1) + j) * 3] +
		     outB[(i + (sx<<1) + j) * 3]) >> 1)  -
		   outB[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }

      for (i = 2*sx; i < (sy - 3)*sx; i += (sx<<1)) {
	for (j = 3; j < sx - 2; j += 2) {
	  dh = abs(((outR[(i + j - 2) * 3] +
		     outR[(i + j + 2) * 3]) >>1 ) -
		   outR[(i + j) * 3]);
	  dv = abs(((outR[(i - (sx<<1) + j) * 3] +
		     outR[(i + (sx<<1) + j) * 3]) >>1 ) -
		   outR[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      // process RED channel
      for (i = 0; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outR[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      for (i = sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 1; j < sx; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outR[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outR[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outR[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outR[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }

      // process BLUE channel
      for (i = sx; i < sy*sx; i += (sx<<1)) {
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outB[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      for (i = 2*sx; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 0; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outB[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outB[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outB[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outB[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      break;

    case DC1394_COLOR_FILTER_BGGR:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_RGGB:
	// copy original RGB data to output images
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 0; j < sx; j += 2) {
	  outB[(i + j) * 3] = bayer[i + j];
	  outR[(i + sx + (j + 1)) * 3] = bayer[i + sx + (j + 1)];
	  outG[(i + j + 1) * 3] = bayer[i + j + 1];
	  outG[(i + sx + j) * 3] = bayer[i + sx + j];
	}
      }
      // process GREEN channel
      for (i = 2*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 3; j += 2) {
	  dh = abs(((outB[(i + j - 2) * 3] +
		    outB[(i + j + 2) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  dv = abs(((outB[(i - (sx<<1) + j) * 3] +
		    outB[(i + (sx<<1) + j) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      for (i = 3*sx; i < (sy - 3)*sx; i += (sx<<1)) {
	for (j = 3; j < sx - 2; j += 2) {
	  dh = abs(((outR[(i + j - 2) * 3] +
		    outR[(i + j + 2) * 3]) >> 1) -
		   outR[(i + j) * 3]);
	  dv = abs(((outR[(i - (sx<<1) + j) * 3] +
		    outR[(i + (sx<<1) + j) * 3]) >> 1) -
		   outR[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >>1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >>1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >>2;
	  }
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      // process RED channel
      for (i = sx; i < (sy - 1)*sx; i += (sx<<1)) {	// G-points (1/2)
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outR[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >>1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      for (i = 2*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 1; j < sx; j += 2) {	// G-points (2/2)
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outR[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
	for (j = 2; j < sx - 1; j += 2) {	// B-points
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outR[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outR[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outR[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }

      // process BLUE channel
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outB[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      for (i = sx; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 0; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outB[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outB[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outB[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outB[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      break;
    default:			//---------------------------------------------------------
//      fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
//      return;
      break;
    }

    ClearBorders_uint16(rgb, sx, sy, 3);
}

/* coriander's Bayer decoding (GPL) */
void
dc1394_bayer_Downsample_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    uint16_t *outR, *outG, *outB;
    register int i, j;
    int tmp;

    sx *= 2;
    sy *= 2;

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
	outR = &rgb[0];
	outG = &rgb[1];
	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
	outR = &rgb[2];
	outG = &rgb[1];
	outB = &rgb[0];
	break;
    default:
//	fprintf(stderr, "Bad Bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_GBRG:
	for (i = 0; i < sy*sx; i += (sx<<1)) {
	    for (j = 0; j < sx; j += 2) {
		tmp =
		    ((bayer[i + j] + bayer[i + sx + j + 1]) >> 1);
		CLIP16(tmp, outG[((i >> 2) + (j >> 1)) * 3], bits);
		tmp = bayer[i + sx + j + 1];
		CLIP16(tmp, outR[((i >> 2) + (j >> 1)) * 3], bits);
		tmp = bayer[i + sx + j];
		CLIP16(tmp, outB[((i >> 2) + (j >> 1)) * 3], bits);
	    }
	}
	break;
    case DC1394_COLOR_FILTER_BGGR:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_RGGB:
	for (i = 0; i < sy*sx; i += (sx<<1)) {
	    for (j = 0; j < sx; j += 2) {
		tmp =
		    ((bayer[i + sx + j] + bayer[i + j + 1]) >> 1);
		CLIP16(tmp, outG[((i >> 2) + (j >> 1)) * 3], bits);
		tmp = bayer[i + sx + j + 1];
		CLIP16(tmp, outR[((i >> 2) + (j >> 1)) * 3], bits);
		tmp = bayer[i + j];
		CLIP16(tmp, outB[((i >> 2) + (j >> 1)) * 3], bits);
	    }
	}
	break;
    default:			//---------------------------------------------------------
//	fprintf(stderr, "Bad Bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

}

/* coriander's Bayer decoding (GPL) */
void dc1394_bayer_Simple_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    uint16_t *outR, *outG, *outB;
    register int i, j;
    int tmp, base;

    // sx and sy should be even
    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
//	outR = &rgb[0];
//	outG = &rgb[1];
//	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
//	outR = &rgb[2];
//	outG = &rgb[1];
//	outB = &rgb[0];
	break;
    default:
//	fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
	outR = &rgb[0];
	outG = &rgb[1];
	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
	outR = &rgb[2];
	outG = &rgb[1];
	outB = &rgb[0];
	break;
    default:
	outR = NULL;
	outG = NULL;
	outB = NULL;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_GBRG:
	for (i = 0; i < sy - 1; i += 2) {
	    for (j = 0; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base] + bayer[base + sx + 1]) >> 1);
		CLIP16(tmp, outG[base * 3], bits);
		tmp = bayer[base + 1];
		CLIP16(tmp, outR[base * 3], bits);
		tmp = bayer[base + sx];
		CLIP16(tmp, outB[base * 3], bits);
	    }
	}
	for (i = 0; i < sy - 1; i += 2) {
	    for (j = 1; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base + 1] + bayer[base + sx]) >> 1);
		CLIP16(tmp, outG[(base) * 3], bits);
		tmp = bayer[base];
		CLIP16(tmp, outR[(base) * 3], bits);
		tmp = bayer[base + 1 + sx];
		CLIP16(tmp, outB[(base) * 3], bits);
	    }
	}
	for (i = 1; i < sy - 1; i += 2) {
	    for (j = 0; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base + sx] + bayer[base + 1]) >> 1);
		CLIP16(tmp, outG[base * 3], bits);
		tmp = bayer[base + sx + 1];
		CLIP16(tmp, outR[base * 3], bits);
		tmp = bayer[base];
		CLIP16(tmp, outB[base * 3], bits);
	    }
	}
	for (i = 1; i < sy - 1; i += 2) {
	    for (j = 1; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base] + bayer[base + 1 + sx]) >> 1);
		CLIP16(tmp, outG[(base) * 3], bits);
		tmp = bayer[base + sx];
		CLIP16(tmp, outR[(base) * 3], bits);
		tmp = bayer[base + 1];
		CLIP16(tmp, outB[(base) * 3], bits);
	    }
	}
	break;
    case DC1394_COLOR_FILTER_BGGR:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_RGGB:
	for (i = 0; i < sy - 1; i += 2) {
	    for (j = 0; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base + sx] + bayer[base + 1]) >> 1);
		CLIP16(tmp, outG[base * 3], bits);
		tmp = bayer[base + sx + 1];
		CLIP16(tmp, outR[base * 3], bits);
		tmp = bayer[base];
		CLIP16(tmp, outB[base * 3], bits);
	    }
	}
	for (i = 1; i < sy - 1; i += 2) {
	    for (j = 0; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base] + bayer[base + 1 + sx]) >> 1);
		CLIP16(tmp, outG[(base) * 3], bits);
		tmp = bayer[base + 1];
		CLIP16(tmp, outR[(base) * 3], bits);
		tmp = bayer[base + sx];
		CLIP16(tmp, outB[(base) * 3], bits);
	    }
	}
	for (i = 0; i < sy - 1; i += 2) {
	    for (j = 1; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base] + bayer[base + sx + 1]) >> 1);
		CLIP16(tmp, outG[base * 3], bits);
		tmp = bayer[base + sx];
		CLIP16(tmp, outR[base * 3], bits);
		tmp = bayer[base + 1];
		CLIP16(tmp, outB[base * 3], bits);
	    }
	}
	for (i = 1; i < sy - 1; i += 2) {
	    for (j = 1; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base + 1] + bayer[base + sx]) >> 1);
		CLIP16(tmp, outG[(base) * 3], bits);
		tmp = bayer[base];
		CLIP16(tmp, outR[(base) * 3], bits);
		tmp = bayer[base + 1 + sx];
		CLIP16(tmp, outB[(base) * 3], bits);
	    }
	}
	break;
    default:			//---------------------------------------------------------
//	fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

    /* add black border */
    for (i = sx * (sy - 1) * 3; i < sx * sy * 3; i++) {
	rgb[i] = 0;
    }
    for (i = (sx - 1) * 3; i < sx * sy * 3; i += (sx - 1) * 3) {
	rgb[i++] = 0;
	rgb[i++] = 0;
	rgb[i++] = 0;
    }
}

int dc1394_bayer_decoding_8bit(const unsigned char *bayer, unsigned char *rgb, unsigned int sx, unsigned int sy, unsigned int tile, unsigned int method)
{
  switch (method) {
  case DC1394_BAYER_METHOD_NEAREST:
    dc1394_bayer_NearestNeighbor(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_SIMPLE:
    dc1394_bayer_Simple(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_BILINEAR:
    dc1394_bayer_Bilinear(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_HQLINEAR:
    dc1394_bayer_HQLinear(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_DOWNSAMPLE:
    dc1394_bayer_Downsample(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_EDGESENSE:
    dc1394_bayer_EdgeSense(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  }

  return DC1394_INVALID_BAYER_METHOD;
}

int
dc1394_bayer_decoding_16bit(const uint16_t *bayer, uint16_t *rgb, uint_t sx, uint_t sy, uint_t tile, uint_t bits, uint_t method)
{
  switch (method) {
  case DC1394_BAYER_METHOD_NEAREST:
    dc1394_bayer_NearestNeighbor_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_SIMPLE:
    dc1394_bayer_Simple_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_BILINEAR:
    dc1394_bayer_Bilinear_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_HQLINEAR:
    dc1394_bayer_HQLinear_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_DOWNSAMPLE:
    dc1394_bayer_Downsample_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_EDGESENSE:
    dc1394_bayer_EdgeSense_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  }

  return DC1394_INVALID_BAYER_METHOD;
}

#endif
