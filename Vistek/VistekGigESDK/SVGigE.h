/*******************************************************************************
 * SVS GigE API   Declaration of GigE camera access functions
 *******************************************************************************
 *
 * Version:     1.5.2.251
 *
 * Copyright:   SVS VISTEK GmbH
 *
 * Function categories:
 * ---------------------------------------------------------
 *    1 - Camera: Discovery and bookkeeping
 *    2 - Camera: Connection
 *    3 - Camera: Information
 *    4 - Stream: Channel creation and control
 *    5 - Stream: Channel statistics
 *    6 - Stream: Channel info
 *    7 - Stream: Transfer parameters
 *    8 - Stream: Image access
 *    9 - Stream: Image conversion
 *   10 - Stream: Image characteristics
 *   11 - Stream: Image statistics
 *   12 - Stream: Messaging channel
 *   13 - Controlling camera: Frame rate
 *   14 - Controlling camera: Exposure
 *   15 - Controlling camera: Gain and offset
 *   16 - Controlling camera: Auto gain/exposure
 *   17 - Controlling camera: Acquisition trigger
 *   18 - Controlling camera: Strobe
 *   19 - Controlling camera: Tap balance
 *   20 - Controlling camera: Image parameter
 *   21 - Controlling camera: Image appearance
 *   22 - Special control: IOMux configuration
 *   23 - Special control: IO control
 *   24 - Special control: Serial communication
 *   25 - Special control: Direct register and memory access
 *   26 - Special control: Persistent settings and recovery
 *   27 - General functions
 *   28 - Diagnostics
 *   29 - Special control: Lens control
 * ---------------------------------------------------------
 *   99 - Deprecated functions
 *
 *******************************************************************************
 *
 *
 * Revision history:
 *
 ** Version 1.5.1
 * --------------------
 *	A negative value for PixelsCorrectionMapOffset is allowed.
 *	Improvements in the c# interface.
 *	Bug fixing when using an Adapter with multiple IP addresses.
 *
 ** Version 1.5.0
 * --------------------
 * The SVS build system has been re-structured for enabling
 * “nightly builds” as well as for making InstallShield merge
 * modules available for the main “SVCam GigE Software” components
 *
 ** Version 1.4.26.62
 * --------------------
 *   	+ Camera_setLensFocusUnit()
 *		+ Camera_getLensFocusUnit()
 *
 ** Version 1.4.26.61-2
 * --------------------
 *      + Camera_setTapConfigurationEx(()
 *	    + Camera_getTapConfigurationEx()
 *  	+ Camera_setFlippingMode()
 *	    + Camera_getFlippingMode()
 *	    + Camera_setShutterMode()
 *	    + Camera_getShutterMode()
 *
 ** Version 1.4.26.61-1
 * --------------------
 *    - bugfix for communication timeout.
 *		ACQUISITION_MODE_NO_ACQUISITION is no longer available
 *
 *
 * Version 1.4.26.61
 * --------------------
 *    - bugfix for Multicast (thus new driver version 1.4.26)
 *
 *
 * Version 1.4.25.60-2
 * --------------------
 *    - functions added:
 *      + Camera_setPixelsCorrectionMap()
 *	    + Camera_getPixelsCorrectionMap()
 *  	+ Camera_setPixelsCorrectionControlEnabel()
 *	    + Camera_getPixelsCorrectionControlEnabel()
 *	    + Camera_setPixelsCorrectionControlMark()
 *	    + Camera_getPixelsCorrectionControlMark()
 *      + Camera_setPixelsCorrectionMapOffset()
 *      + Camera_getPixelsCorrectionMapOffset()
 *      + Camera_getPixelsCorrectionMapSize()
 *      + Camera_getMaximalPixelsCorrectionMapSize()
 *      +  Camera_getMapIndexCoordinate()
 *      + Camera_deletePixelCoordinateFromMap()
 *
 * Version 1.4.25.60-1
 * -------------------
 *    - functions added:
 *      + Camera_isLensAvailable()
 *	    + Camera_getLensName()
 *  	+ Camera_setLensFocalLenght()
 *	    + Camera_getLensFocalLenght()
 *	    + Camera_getLensFocalLenghtMin()
 *	    + Camera_getLensFocalLenghtMax()
 *      + Camera_setLensFocus()
 *      + Camera_getLensFocus()
 *      + Camera_getLensFocusMin()
 *      + Camera_getLensFocusMax()
 *      + Camera_setLensAperture()
 *      + Camera_getLensAperture()
 *      + Camera_getLensApertureMin()
 *      + Camera_getLensApertureMax()
 *

 * Version 1.4.25.60
 * ------------------
 *    - functions added:
 *    + SVGigE_estimateWhiteBalanceExtended()
 *	  + Camera_openConnectionEx()
 *	  + StreamingChannel_createEx()

 * Version 1.4.24.59
 * -----------------
 *   - functions added:
 *     + Camera_getSensorTemperature()
 *     + Camera_setPivMode()
 *     + Camera_getPivMode()
 *     + Camera_setPrescalerDevisor()
 *     + Camera_getPrescalerDevisor()
 *     + Camera_setDebouncerDuration()
 *     + Camera_getDebouncerDuration()
 *     + Camera_loadSequenceParameters()
 *     + Camera_startSequencer()
 *     + Camera_setStrobePolarityExtended()
 *     + Camera_getStrobePolarityExtended()
 *     + Camera_setStrobePositionExtended()
 *     + Camera_getStrobePositionExtended()
 *     + Camera_getStrobePositionIncrement()
 *     + Camera_setStrobeDurationExtended()
 *     + Camera_getStrobeDurationExtended()
 *     + Camera_getTapGain()
 *     + Camera_setTapGain()
 *     + Camera_loadSettingsFromXml()
 *     + Camera_SaveSettingsToXml()
 *     + Camera_loadSequenceParameters()
 *     + Camera_startSequencer()
 *   - removed
 *     - Camera_setUserTapGain()
 *     - Camera_getUserTapGain()
 *
 * Version 1.4.24.58
 * -----------------
 *   - firmware for 'eco' camera series updated to build 1759 with support for 'trigger violation'
 *
 * Version 1.4.24.57
 * -----------------
 *   - messages from camera added, like e.g. "trigger violation"
 *   - signals 9/10 exchanged for TL consistency ("camera found"/"multicast message")
 *   - GEV(GigE Vision) return codes properly translated into GigE_GEV_ codes (-301..-314)
 *   - custom GEV codes translated to GigE_SVCAM_STATUS_ codes (-130..-132,-140..-145)
 *   - functions added:
 *		 + Camera_startImageCorrection()
 *     + Camera_isIdleImageCorrection()
 *     + Camera_setImageCorrection()
 *     + Camera_getImageCorrection()
 *     + Camera_setTapUserSettings()
 *     + Camera_getTapUserSettings()
 *   - functions deprecated
 *     - Camera_setTapBalance()
 *     - Camera_getTapBalance()
 *   - "MultiStream" (removed)->deprecated

 *
 * Version 1.4.23.56
 * -----------------
 *   - functions deprecated
 *		 - Image_getImageGray()
 *     - StreamingChannel_createMultiStream()
 *     - Camera_forceOpenConnection()
 *
 * Version 1.4.23.55
 * -----------------
 *   - functions added:
 *     + isDriverAvailable()
 *
 * Version 1.4.23.53
 * -----------------
 *   - SVGigE_IOMux_IN_READOUT added
 *	 - SVGigE_IOMux_IN_EXPOSE added
 *	 - SVGigE_IOMux_IN_PWMB added
 *	 - SVGigE_IOMux_IN_PWMA added
 *   - SVGigE_IOMux_IN_STROBE_0and1 added
 *	 - SVGigE_IOMux_IN_STROBE1 added
 *	 - SVGigE_IOMux_IN_STROBE0 added
 *   - SVGigE_IOMux_IN_FIXED_LOW  has changed from SVGigE_IOMUX_IN7 to SVGigE_IOMUX_IN30
 *   - SVGigE_IOMux_IN_FIXED_HIGH has changed from SVGigE_IOMUX_IN8 to SVGigE_IOMUX_IN31
 *
 * Version 1.4.23.51
 * -----------------
 *   - path to driver packages for SVGigE_installFilterDriver() changed
 *
 * Version 1.4.20.47
 * -----------------
 *   - binning mode 3x3 and 4x4 added
 *
 * Version 1.4.20.46
 * -----------------
 *   - functions added:
 *      + SVGigE_installFilterDriver()
 *      + SVGigE_uninstallFilterDriver()
 *
 * Version 1.4.20.45
 * -----------------
 *   - minimal value for auto exposure limited to 10 us
 *
 * Version 1.4.20.44
 * -----------------
 *   - functions added:
 *     + Camera_restartIPConfiguration
 *
 * Version 1.4.19.43
 * -----------------
 *   - stability issues fixed (receiving "C002" when network packet got lost during 'start stream')
 *   - stream blocking resolved when switching mode after ImageID=65535
 *
 * Version 1.4.18.42
 * -----------------
 *   - functions added:
 *     + StreamingChannel_createMultiStream
 *     + StreamingChannel_setChannelTimeout()
 *     + StreamingChannel_getChannelTimeout()
 *     + StreamingChannel_getPixelType()
 *     + StreamingChannel_getBufferData()
 *     + StreamingChannel_getBufferSize()
 *     + StreamingChannel_getImagePitch()
 *     + StreamingChannel_getImageSizeX()
 *     + StreamingChannel_getImageSizeY()
 *
 * Version 1.4.16.40
 * -----------------
 *   - functions added:
 *     + Camera_resetTimestampCounter()
 *     + Camera_getTimestampCounter()
 *     + Camera_getTimestampTickFrequency()
 *
 * Version 1.4.16.39-2
 * -------------------
 *  - softwareTrigger() fixed
 *
 * Version 1.4.16.39
 * -----------------
 *  - initial AutoTapBalance fixed (mode: ONCE preselected now)
 *
 * Version 1.4.16.36
 * -----------------
 *   - functions added:
 *     + Stream_createEvent()
 *     + Stream_addEventType()
 *     + Stream_removeEventType()
 *     + Stream_registerMessageCallback()
 *     + Stream_getEventID()
 *     + Stream_getMessageType()
 *     + Stream_getMessageData()
 *     + Stream_releaseMessage()
 *     + Stream_isMessagePending()
 *     + Stream_flushMessages()
 *     + Stream_closeEvent()
 *
 * Version 1.4.16.36
 * -----------------
 *   - functions added:
 *     + Camera_getSubnetMask()
 *     + Camera_getFrameRateMin()
 *     + Camera_getFrameRateMax()
 *     + Camera_getFrameRateIncrement()
 *     + Camera_getExposureTimeMin()
 *     + Camera_getExposureTimeMax()
 *     + Camera_getExposureTimeIncrement()
 *     + Camera_getExposureDelayMax()
 *     + Camera_getExposureDelayIncrement()
 *     + Camera_getGainIncrement()
 *     + Camera_getOffsetMax()
 *     + Camera_getWhiteBalanceMax()
 *     + Camera_setGammaCorrection()
 *     + Camera_softwareTrigger()
 *     + Camera_softwareTriggerID()
 *     + Camera_getStrobePositionMax()
 *     + Camera_getStrobePositionIncrement()
 *     + Camera_getStrobeDurationMax()
 *     + Camera_getStrobeDurationIncrement()
 *     + Camera_getAreaOfInterestRange()
 *     + Camera_getAreaOfInterestIncrement()
 *     + Camera_setAutoGainEnabled()
 *     + Camera_getAutoGainEnabled()
 *     + Camera_setAutoGainBrightness()
 *     + Camera_getAutoGainBrightness()
 *     + Camera_setAutoGainDynamics()
 *     + Camera_getAutoGainDynamics()
 *     + Camera_setAutoGainLimits()
 *     + Camera_getAutoGainLimits()
 *     + Camera_setAutoExposureLimits()
 *     + Camera_getAutoExposureLimits()
 *     + Camera_getMaxIOMuxIN()
 *     + Camera_getMaxIOMuxOUT()
 *     + Camera_setIOAssignment()
 *     + Camera_getIOAssignment()
 *     + Camera_setIOMuxIN()
 *     + Camera_getIOMuxIN()
 *     + Camera_setIO()
 *     + Camera_getIO()
 *     + Camera_setAcqLEDOverride()
 *     + Camera_getAcqLEDOverride()
 *     + Camera_setLEDIntensity()
 *     + Camera_getLEDIntensity()
 *     + Camera_setUARTBuffer()
 *     + Camera_getUARTBuffer()
 *     + Camera_setUARTBaud()
 *     + Camera_getUARTBaud()
 *     + StreamingChannel_setReadoutTransfer()
 *     + StreamingChannel_getReadoutTransfer()
 *     + StreamingChannel_getPeakDataRate()
 *
 * Version 1.4.16.32
 * -----------------
 *   - functions added:
 *     + Bayer conversion limited to a HQ (high quality) and a HP (high performance) algorithm
 *     + Image_getImageGray()
 *     + Camera_setMulticastMode()
 *     + Camera_getMulticastMode()
 *     + Camera_getMulticastGroup()
 *     + Camera_setLUTMode() re-implemented for backward compatibility
 *     + Camera_getLUTMode() re-implemented for backward compatibility
 *
 * Version 1.4.15.30
 * -----------------
 *   - functions added:
 *     + Image_getTransferTime()
 *     + Camera_setAutoTapBalanceMode()
 *     + Camera_getAutoTapBalanceMode()
 *     + Camera_setTapBalance()
 *     + Camera_getTapBalance()
 *
 * Version 1.4.14.26
 * -----------------
 *   - a need for firmware upgrade is checked on Camera_open()
 *
 *   - functions added:
 *     + Camera_setAcquisitionModeAndStart()
 *     + Camera_forceValidNetworkSettings()
 *     + Camera_setIPAddress()
 *
 * Version 1.4.14.22
 * -----------------
 *   - functions added:
 *     + Camera_setLookupTableMode()
 *     + Camera_getLookupTableMode()
 *     + Camera_setLookupTable()
 *     + Camera_getLookupTable()
 *     + Camera_setAcquisitionControl()
 *     + Camera_getAcquisitionControl()
 *     + Image_getImage12bitAs8bit()
 *     + Image_getImage12bitAs16bit()
 *     + Image_getImage16bitAs8bit()
 *
 * Version 1.4.13.20
 * -----------------
 *   - functions added:
 *     + Camera_setWhiteBalance()
 *     + Camera_getWhiteBalance()
 *     + SVGigE_estimateWhiteBalance()
 *     + Camera_registerForLogMessages()
 *
 * Version 1.4.12.18
 * -----------------
 *   - functions added:
 *     + Camera_getPixelClock()
 *
 * Version 1.4.11.1
 * -----------------
 *   - functions added:
 *     + Camera_writeGigECameraMemory()
 *     + Camera_readGigECameraMemory()
 *
 * Version 1.4.8.13
 * ----------------
 *   - fast switching of tap configuration enabled (< 200 ms)
 *
 * Version 1.4.7.11
 * ----------------
 *   - isVersionCompliantDLL() can now run before other functions
 *   - CameraContainer_create() checks also SVGigE driver availability
 *
 *   - functions added:
 *     + Camera_setPixelDepth()
 *     + Camera_getPixelDepth()
 *     + Camera_saveTapBalanceSettings()
 *     + Camera_loadTapBalanceSettings()
 *     + Camera_setTapCalibration()
 *     + Camera_getTapCalibration()
 *     + Camera_setTapConfiguration()
 *     + Camera_getTapConfiguration()
 *
 * Version 1.4.6.9
 * ---------------
 *   - function added:
 *     + Camera_readXMLFile()
 *
 * Version 1.4.3.1
 * ---------------
 *   - packet resend capabilities improved
 *     NOTE: The image pointer will be NULL for uncomplete images
 *           which may be the case for unsufficient network bandwidth
 *
 * Version 1.4.1.4
 * ---------------
 *   - 64-bit supported
 *   - function added:
 *     + Camera_setStreamingPacketSize()
 *
 * Version 1.4.1.3
 * ---------------
 *   - non-admin account: All users are granted access to the SVGigE driver
 *
 * Version 1.4.0.0
 * ---------------
 *   - shorter names: SVGigE_RETURN instead of SVS_GigE_API_RETURN
 *   - CameraContainer_create(), parameter added: SVGigETL_Type TransportLayerType
 *   - Camera_openConnection(), parameter added: float Timeout
 *   - StreamingChannel_create(), parameter removed: bool DriverEnabled
 *
 *   - functions added:
 *     + Camera_evaluateMaximalPacketSize()
 *     + Camera_getImagerWidth()
 *     + Camera_getImagerHeight()
 *     + Camera_getFrameRateRange()
 *     + Camera_getExposureTimeRange()
 *     + Camera_setAreaOfInterest()
 *     + Camera_getAreaOfInterest()
 *     + StreamingChannel_getFrameLoss()
 *     + StreamingChannel_getActualFrameRate()
 *     + StreamingChannel_getActualDataRate()
 *     + Image_getCamera()
 *     + Image_getPacketCount()
 *     + SVGigE_writeImageToBitmapFile()
 *     + Camera_isCameraFeature()
 *
 *   - function deactivated temporarily
 *     ~ Camera_createLUTwhiteBalance()
 *
 *
 *******************************************************************************
 * Detailed function listing
 *******************************************************************************
 *
 * 0 - GigE DLL and driver
 * -------------------------------------
 *  isVersionCompliantDLL()
 *  isDriverAvailable()
 *
 * 1 - Camera: Discovery and bookkeeping
 * -------------------------------------
 *  CameraContainer_create()
 *  CameraContainer_delete()
 *  CameraContainer_discovery()
 *  CameraContainer_getNumberOfCameras()
 *  CameraContainer_getCamera()
 *  CameraContainer_findCamera()
 *
 * 2 - Camera: Connection
 * ----------------------
 *  Camera_openConnection()
 *  Camera_closeConnection()
 *  Camera_openConnectionEx()
 *  Camera_setIPAddress()
 *  Camera_forceValidNetworkSettings()
 *  Camera_restartIPConfiguration()
 *
 * 3 - Camera: Information
 * -----------------------
 *  Camera_getManufacturerName()
 *  Camera_getModelName()
 *  Camera_getDeviceVersion()
 *  Camera_getManufacturerSpecificInformation()
 *  Camera_getSerialNumber()
 *  Camera_setUserDefinedName()
 *  Camera_getUserDefinedName()
 *  Camera_getMacAddress()
 *  Camera_getIPAddress()
 *  Camera_getSubnetMask()
 *  Camera_getPixelClock()
 *  Camera_isCameraFeature()
 *  Camera_readXMLFile()
 *  Camera_getSensorTemperature()
 *
 * 4 - Stream: Channel creation and control
 * ----------------------------------------
 *  StreamingChannel_create()
 *  StreamingChannel_createEx()
 *  StreamingChannel_delete()
 *  StreamingChannel_setChannelTimeout()
 *  StreamingChannel_getChannelTimeout()
 *  StreamingChannel_setReadoutTransfer()
 *  StreamingChannel_getReadoutTransfer()
 *
 * 5 - Stream: Channel statistics
 * ------------------------------------------
 *  StreamingChannel_getFrameLoss()
 *  StreamingChannel_getActualFrameRate()
 *  StreamingChannel_getActualDataRate()
 *  StreamingChannel_getPeakDataRate()
 *
 * 6 - Stream: Channel info
 * ------------------------
 *  StreamingChannel_getPixelType()
 *  StreamingChannel_getBufferData()
 *  StreamingChannel_getBufferSize()
 *  StreamingChannel_getImagePitch()
 *  StreamingChannel_getImageSizeX()
 *  StreamingChannel_getImageSizeY()
 *
 * 7 - Stream: Transfer parameters
 * -------------------------------
 *  Camera_evaluateMaximalPacketSize()
 *  Camera_setStreamingPacketSize()
 *  Camera_setInterPacketDelay()
 *  Camera_getInterPacketDelay()
 *  Camera_setMulticastMode()
 *  Camera_getMulticastMode()
 *  Camera_getMulticastGroup()
 *
 * 8 - Stream: Image access
 * ------------------------
 *  Image_getDataPointer()
 *  Image_getBufferIndex()
 *  Image_getSignalType()
 *  Image_getCamera()
 *  Image_release()
 *
 * 9 - Stream: Image conversion
 * ----------------------------
 *  Image_getImageRGB()
 *  Image_getImage12bitAs8bit()
 *  Image_getImage12bitAs16bit()
 *  Image_getImage16bitAs8bit()
 *
 * 10 - Stream: Image characteristics
 * ----------------------------------
 *  Image_getPixelType()
 *  Image_getImageSize()
 *  Image_getPitch()
 *  Image_getSizeX()
 *  Image_getSizeY()
 *
 * 11 - Stream: Image statistics
 * ----------------------------
 *  Image_getImageID()
 *  Image_getTimestamp()
 *  Image_getTransferTime()
 *  Image_getPacketCount()
 *  Image_getPacketResend()
 *
 * 12 - Stream: Messaging channel
 * ------------------------------
 *  Stream_createEvent()
 *  Stream_addMessageType()
 *  Stream_removeMessageType()
 *  Stream_isMessagePending()
 *  Stream_registerEventCallback()
 *  Stream_unregisterEventCallback()
 *  Stream_getMessage()
 *  Stream_getMessageData()
 *  Stream_getMessageTimestamp()
 *  Stream_releaseMessage()
 *  Stream_flushMessages()
 *  Stream_closeEvent()
 *
 * 13 - Controlling camera: Frame rate
 * -----------------------------------
 *  Camera_setFrameRate()
 *  Camera_getFrameRate()
 *  Camera_getFrameRateMin()
 *  Camera_getFrameRateMax()
 *  Camera_getFrameRateRange()
 *  Camera_getFrameRateIncrement()
 *
 * 14 - Controlling camera: Exposure
 * ---------------------------------
 *  Camera_setExposureTime()
 *  Camera_getExposureTime()
 *  Camera_getExposureTimeMin()
 *  Camera_getExposureTimeMax()
 *  Camera_getExposureTimeRange()
 *  Camera_getExposureTimeIncrement()
 *  Camera_setExposureDelay()
 *  Camera_getExposureDelay()
 *  Camera_getExposureDelayMax()
 *  Camera_getExposureDelayIncrement()
 *
 * 15 - Controlling camera: Gain and offset
 * ----------------------------------------
 *  Camera_setGain()
 *  Camera_getGain()
 *  Camera_getGainMax()
 *  Camera_getGainMaxExtended()
 *  Camera_getGainIncrement()
 *  Camera_setOffset()
 *  Camera_getOffset()
 *  Camera_getOffsetMax()
 *
 * 16 - Controlling camera: Auto gain/exposure
 * -------------------------------------------
 *  Camera_setAutoGainEnabled()
 *  Camera_getAutoGainEnabled()
 *  Camera_setAutoGainBrightness()
 *  Camera_getAutoGainBrightness()
 *  Camera_setAutoGainDynamics()
 *  Camera_getAutoGainDynamics()
 *  Camera_setAutoGainLimits()
 *  Camera_getAutoGainLimits()
 *  Camera_setAutoExposureLimits()
 *  Camera_getAutoExposureLimits()
 *
 * 17 - Controlling camera: Acquisition trigger
 * --------------------------------------------
 *  Camera_setAcquisitionControl()
 *  Camera_getAcquisitionControl()
 *  Camera_setAcquisitionMode()
 *  Camera_setAcquisitionModeAndStart()
 *  Camera_getAcquisitionMode()
 *  Camera_softwareTrigger()
 *  Camera_softwareTriggerID()
 *  Camera_softwareTriggerIDEnable()
 *  Camera_setTriggerPolarity()
 *  Camera_getTriggerPolarity()
 *  Camera_setPivMode()
 *  Camera_getPivMode()
 *  Camera_setDebouncerDuration()
 *  Camera_getDebouncerDuration()
 *  Camera_setPrescalerDevisor()
 *  Camera_getPrescalerDevisor()
 *  Camera_loadSequenceParameters()
 *  Camera_startSequencer()
 *
 * 18 - Controlling camera: Strobe
 * -------------------------------
 *  Camera_setStrobePolarity()
 *  Camera_setStrobePolarityExtended()
 *  Camera_getStrobePolarity()
 *  Camera_getStrobePolarityExtended()
 *  Camera_setStrobePosition()
 *  Camera_setStrobePositionExtended()
 *  Camera_getStrobePosition()
 *  Camera_getStrobePositionExtended()
 *  Camera_getStrobePositionMax()
 *  Camera_getStrobePositionIncrement()
 *  Camera_setStrobeDuration()
 *  Camera_setStrobeDurationExtended()
 *  Camera_getStrobeDuration()
 *  Camera_getStrobeDurationExtended()
 *  Camera_getStrobeDurationMax()
 *  Camera_getStrobeDurationIncrement()
 *
 * 19 - Controlling camera: Tap balance
 * ------------------------------------
 *  Camera_saveTapBalanceSettings()
 *  Camera_loadTapBalanceSettings()
 *  Camera_setTapConfiguration()
 *  Camera_getTapConfiguration()
 *  Camera_setTapConfigurationEx()
 *  Camera_getTapConfigurationEx()
 *  Camera_setAutoTapBalanceMode()
 *  Camera_getAutoTapBalanceMode()
 *  Camera_setTapGain()
 *  Camera_getTapGain()
 *
 * 20 - Controlling camera: Image parameter
 * ---------------------------------------
 *  Camera_getImagerWidth()
 *  Camera_getImagerHeight()
 *  Camera_getImageSize()
 *  Camera_getPitch()
 *  Camera_getSizeX()
 *  Camera_getSizeY()
 *  Camera_setBinningMode()
 *  Camera_getBinningMode()
 *  Camera_setAreaOfInterest()
 *  Camera_getAreaOfInterest()
 *  Camera_getAreaOfInterestRange()
 *  Camera_getAreaOfInterestIncrement()
 *  Camera_resetTimestampCounter()
 *  Camera_getTimestampCounter()
 *  Camera_getTimestampTickFrequency()
 *  Camera_setFlippingMode()
 *	Camera_getFlippingMode()
 *	Camera_setShutterMode()
 *	Camera_getShutterMode()
 *

 * 21 - Controlling camera: Image appearance
 * -----------------------------------------
 *  Camera_getPixelType()
 *  Camera_setPixelDepth()
 *  Camera_getPixelDepth()
 *  Camera_setWhiteBalance()
 *  Camera_getWhiteBalance()
 *  Camera_getWhiteBalanceMax()
 *  Camera_setGammaCorrection()
 *  Camera_setGammaCorrectionExt()
 *  Camera_setLowpassFilter()
 *  Camera_getLowpassFilter()
 *  Camera_setLookupTableMode()
 *  Camera_getLookupTableMode()
 *  Camera_setLookupTable()
 *  Camera_getLookupTable()
 *  Camera_startImageCorrection()
 *  Camera_isIdleImageCorrection()
 *  Camera_setImageCorrection()
 *  Camera_getImageCorrection()
 *  Camera_setPixelsCorrectionMap()
 *  Camera_getPixelsCorrectionMap()
 *  Camera_setPixelsCorrectionControlEnabel()
 *	Camera_getPixelsCorrectionControlEnabel()
 *	Camera_setPixelsCorrectionControlMark()
 *	Camera_getPixelsCorrectionControlMark()
 *  Camera_setPixelsCorrectionMapOffset()
 *  Camera_getPixelsCorrectionMapOffset()
 *  Camera_getPixelsCorrectionMapSize()
 *  Camera_getMaximalPixelsCorrectionMapSize()
 *  Camera_getMapIndexCoordinate()
 *  Camera_deletePixelCoordinateFromMap()
 *
 *
 * 22 - Special control: IOMux configuration
 * -------------------------------------------------------
 *  Camera_getMaxIOMuxIN()
 *  Camera_getMaxIOMuxOUT()
 *  Camera_setIOAssignment()
 *  Camera_getIOAssignment()
 *
 * 23 - Special control: IO control
 * -------------------------------------------------------
 *  Camera_setIOMuxIN()
 *  Camera_getIOMuxIN()
 *  Camera_setIO()
 *  Camera_getIO()
 *  Camera_setAcqLEDOverride()
 *  Camera_getAcqLEDOverride()
 *  Camera_setLEDIntensity()
 *  Camera_getLEDIntensity()
 *
 * 24 - Special control: Serial communication
 * -------------------------------------------------------
 *  Camera_setUARTBuffer()
 *  Camera_getUARTBuffer()
 *  Camera_setUARTBaud()
 *  Camera_getUARTBaud()
 *
 * 25 - Special control: Direct register and memory access
 * -------------------------------------------------------
 *  Camera_setGigECameraRegister()
 *  Camera_getGigECameraRegister()
 *  Camera_writeGigECameraMemory()
 *  Camera_readGigECameraMemory()
 *
 * 26 - Special control: Persistent settings and recovery
 * ------------------------------------------------------
 *  Camera_writeEEPROM()
 *  Camera_readEEPROM()
 *  Camera_restoreFactoryDefaults()
 *  Camera_loadSettingsFromXml()
 *  Camera_SaveSettingsToXml()
 *
 * 27 - General functions
 * ----------------------
 *  SVGigE_estimateWhiteBalance()
 *  SVGigE_estimateWhiteBalanceExtended()
 *  SVGigE_writeImageToBitmapFile()
 *  SVGigE_installFilterDriver()
 *  SVGigE_uninstallFilterDriver()
 *
 * 28 - Diagnostics
 * ----------------
 *  getErrorMessage()
 *  Camera_registerForLogMessages()
 *

 * 29 - Special control: Lens control
 * ------------------------------------------------------
 *  Camera_isLensAvailable()
 *  Camera_getLensName()
 *  Camera_setLensFocalLenght()
 *  Camera_getLensFocalLenght()
 *  Camera_getLensFocalLenghtMin()
 *  Camera_getLensFocalLenghtMax()
 *  Camera_setLensFocusUnit()
 *  Camera_getLensFocusUnit()
 *  Camera_setLensFocus()
 *  Camera_getLensFocus()
 *  Camera_getLensFocusMin()
 *  Camera_getLensFocusMax()
 *  Camera_setLensAperture()
 *  Camera_getLensAperture()
 *  Camera_getLensApertureMin()
 *  Camera_getLensApertureMax()

 *
 * ---------------------------------------------------------
 * 99 - Deprecated functions
 * ---------------------------------------------------------
 *  Camera_startAcquisitionCycle()
 *  Camera_setTapCalibration()
 *  Camera_getTapCalibration()
 *  Camera_setLUTMode()
 *  Camera_getLUTMode()
 *  Camera_createLUTwhiteBalance()
 *  Camera_stampTimestamp()
 *  Camera_getTimestamp()
 *  Image_getImageGray()
 *  Camera_forceOpenConnection()
 *  StreamingChannel_createMultiStream()
 *  Camera_setTapBalance()
 *  Camera_getTapBalance()
 *	StreamingChannel_setChannelTimeout()
 *  StreamingChannel_getChannelTimeout()
 *	Camera_setTapUserSettings()
 *	Camera_getTapUserSettings()
 *
 *
 *******************************************************************************
 */

#ifndef SVGigEH
#define SVGigEH

#ifdef __BORLANDC__
  // Generate enums of integer size
#pragma option push -b
#endif

#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>

//---------------------------------------------------------------------------

extern "C"
{
  /** Version information
   *  The particular components of the version information will be represented
   *  in the following way
   */
#ifndef SVGigE_VERSION_DEFINED
#define SVGigE_VERSION_DEFINED
  typedef struct
  {
    unsigned char	MajorVersion;
    unsigned char MinorVersion;
    unsigned char	DriverVersion;
    unsigned char	BuildVersion;

  } SVGigE_VERSION;

  /** Version settings
   *  The following version settings apply to current SVGigE SDK:
   */
  #define SVGigE_VERSION_MAJOR			 1
  #define SVGigE_VERSION_MINOR			 5
  // 2013-09-30: _DRIVER changed to _REVISION
  #define SVGigE_VERSION_REVISION		 2
  // 2013-09-30: for backward compatibility
  #define SVGigE_VERSION_DRIVER  SVGigE_VERSION_REVISION
  #define SVGigE_VERSION_BUILD			251
#endif


  /** Camera Container client handle.
   *  A camera container client handle serves as a reference for managing multiple
   *  clients that are connected to a single camera container (which has no notion
   *  about clients). A value of SVGigE_NO_CLIENT serves as an indicator for
   *  an invalid camera container client before obtaining a valid handle.
   */
  typedef int CameraContainerClient_handle;
  #define SVGigE_NO_CLIENT	(CameraContainerClient_handle)-1

  /** Camera handle.
   *  A camera handle serves as a reference for access functions to camera
   *  functionality. A value of SVGigE_NO_CAMERA serves as an indicator for an
   *  invalid camera handle before obtaining a camera from a camera container.
   */
  typedef void * Camera_handle;
  #define SVGigE_NO_CAMERA	(Camera_handle)-1

  /** Streaming channel handle.
   *  A streaming channel handle serves as a reference for an image stream
   */
  typedef void * StreamingChannel_handle;
  #define SVGigE_NO_STREAMING_CHANNEL	(StreamingChannel_handle)-1

  /** Event handle.
   *  An event handle serves as a reference for a messaging channel
   */
  typedef void * Event_handle;
  #define SVGigE_NO_EVENT	(Event_handle)-1

  /** Message handle.
   *  A message handle serves as a reference for a single message
   */
  typedef int Message_handle;
  #define SVGigE_NO_MESSAGE	(Message_handle)-1

  /** Image handle.
   *  An image handle serves as a reference for picture access and
   *  processing functions
   */
  typedef int Image_handle;

  /** Function returns.
   *  API Functions may return success or error codes by this data type unless
   *  they return specific values
   */
  typedef enum
  {
    SVGigE_SUCCESS                                      =  0,
    SVGigE_ERROR                                        = -1,
    SVGigE_DLL_NOT_LOADED                               = -2,
    SVGigE_DLL_VERSION_MISMATCH                         = -3,
    SVGigE_DRIVER_VERSION_MISMATCH                      = -4,
    SVGigE_WINSOCK_INITIALIZATION_FAILED                = -5,
    SVGigE_CAMERA_CONTAINER_NOT_AVAILABLE               = -6,
    SVGigE_NO_CAMERAS_DETECTED                          = -7,
    SVGigE_CAMERA_NOT_FOUND                             = -8,
    SVGigE_CAMERA_OPEN_FAILED                           = -9,
    SVGigE_CAMERA_COMMUNICATION_FAILED                  = -10,
    SVGigE_CAMERA_REGISTER_ACCESS_DENIED                = -11,
    SVGigE_UNKNOWN_LUT_MODE                             = -12,
    SVGigE_STREAMING_FUNCTION_ALREADY_REGISTERED        = -13,
    SVGigE_STREAMING_NOT_INITIALIZED                    = -14,
    SVGigE_OUT_OF_MEMORY                                = -15,
    SVGigE_INVALID_CALLBACK_INITIALIZATION              = -16,
    SVGigE_INVALID_CALLBACK_FUNCTION_POINTER            = -17,
    SVGigE_IMAGE_NOT_AVAILABLE                          = -18,
    SVGigE_INSUFFICIENT_RGB_BUFFER_PROVIDED             = -19,
    SVGigE_STREAMING_CHANNEL_NOT_AVAILABLE              = -20,
    SVGigE_INVALID_PARAMETERS                           = -21,
    SVGigE_PIXEL_TYPE_NOT_SUPPORTED                     = -22,
    SVGigE_FILE_COULD_NOT_BE_OPENED                     = -23,
    SVGigE_TRANSPORT_LAYER_NOT_AVAILABLE                = -24,
    SVGigE_XML_FILE_NOT_AVAILABLE                       = -25,
    SVGigE_WHITE_BALANCE_FAILED                         = -26,
    SVGigE_DEPRECATED_FUNCTION                          = -27,
    SVGigE_WRONG_DESTINATION_BUFFER_SIZE                = -28,
    SVGigE_INSUFFICIENT_DESTINATION_BUFFER_SIZE         = -29,
    SVGigE_CAMERA_NOT_IN_CURRENT_SUBNET                 = -30,
    SVGigE_CAMERA_MOVED_TO_FOREIGN_SUBNET               = -31,
    SVGigE_IMAGE_SKIPPED_IN_CALLBACK                    = -32,
    SVGigE_MESSAGE_CHANNEL_NOT_SUPPORTED                = -33,
    SVGigE_MESSAGE_CHANNEL_NOT_OPENED                   = -34,
    SVGigE_MESSAGE_TYPE_NOT_SUPPORTED                   = -35,

    // Mapped camera return codes
    SVGigE_SVCAM_STATUS_ERROR                           = -101,
    SVGigE_SVCAM_STATUS_SOCKET_ERROR                    = -102,
    SVGigE_SVCAM_STATUS_ALREADY_CONNECTED				= -103,
    SVGigE_SVCAM_STATUS_INVALID_MAC						= -104,
    SVGigE_SVCAM_STATUS_UNREACHABLE						= -105,
    SVGigE_SVCAM_STATUS_ACCESS_DENIED					= -106,
    SVGigE_SVCAM_STATUS_BUSY							= -107,
    SVGigE_SVCAM_STATUS_LOCAL_PROBLEM					= -108,
    SVGigE_SVCAM_STATUS_MSG_MISMATCH					= -109,
    SVGigE_SVCAM_STATUS_PROTOCOL_ID_MISMATCH            = -110,
    SVGigE_SVCAM_STATUS_NOT_ACKNOWLEDGED                = -111,
    SVGigE_SVCAM_STATUS_RECEIVE_LENGTH_MISMATCH         = -112,
    SVGigE_SVCAM_STATUS_ACKNOWLEDGED_LENGTH_MISMATCH    = -113,
    SVGigE_SVCAM_STATUS_NO_ACK_BUFFER_PROVIDED          = -114,
    SVGigE_SVCAM_STATUS_CONNECTION_LOST                 = -115,
    SVGigE_SVCAM_STATUS_TL_NOT_AVAILABLE                = -116,
    SVGigE_SVCAM_STATUS_DRIVER_VERSION_MISMATCH         = -117,
    SVGigE_SVCAM_STATUS_FEATURE_NOT_SUPPORTED           = -118,
    SVGigE_SVCAM_STATUS_PENDING_CHANNEL_DETECTED        = -119,
    SVGigE_SVCAM_STATUS_LUT_NOT_AVAILABLE               = -120,
    SVGigE_SVCAM_STATUS_LUT_SIZE_MISMATCH               = -121,
    SVGigE_SVCAM_STATUS_NO_MATCHING_IP_ADDRESS          = -122,
    SVGigE_SVCAM_STATUS_DISCOVERY_INFO_CHANGED          = -123,
    SVGigE_SVCAM_STATUS_FIRMWARE_UPGRADE_REQUIRED       = -124,
    SVGigE_SVCAM_STATUS_MULTICAST_NOT_SUPPORTED         = -125,
    SVGigE_SVCAM_STATUS_PIXELDEPTH_NOT_SUPPORTED        = -126,
    SVGigE_SVCAM_STATUS_IO_CONFIG_NOT_SUPPORTED         = -127,
    SVGigE_SVCAM_STATUS_USER_DEFINED_NAME_TOO_LONG      = -128,
    SVGigE_SVCAM_STATUS_INVALID_RESULT_POINTER          = -129,
    SVGigE_SVCAM_STATUS_ARP_REQUEST_FAILED              = -130,
    SVGigE_SVCAM_STATUS_ALREADY_STREAMING               = -131,
    SVGigE_SVCAM_STATUS_NO_STREAMING_CHANNEL            = -132,
    SVGigE_SVCAM_STATUS_CAMERA_OCCUPIED                 = -133,
    SVGigE_SVCAM_STATUS_SECOND_LINK_MISSING             = -134,
    SVGigE_SVCAM_STATUS_CAMERA_NOT_CONNECTED            = -135,
    SVGigE_SVCAM_STATUS_FILTER_DRIVER_NOT_AVAILABLE     = -136,
    SVGigE_SVCAM_STATUS_FILTER_INF_FILE_NOT_FOUND       = -137,
    SVGigE_SVCAM_STATUS_FILTER_INF_FILE_COPY_FAILED     = -138,
    SVGigE_SVCAM_STATUS_BINNING_MODE_NOT_AVAILABLE      = -139,
    SVGigE_SVCAM_STATUS_CREATE_BUFFERS_FAILED       	= -140,
    SVGigE_SVCAM_STATUS_FFC_INVALID_PARAMETER      	    = -141,
    SVGigE_SVCAM_STATUS_FFC_ACQUISITION_RUNNING    	    = -142,
    SVGigE_SVCAM_STATUS_FFC_COLDEPTH_NOT_12BPP        	= -143,
    SVGigE_SVCAM_STATUS_FFC_ACTIVE                      = -144,
    SVGigE_SVCAM_STATUS_ONLY_OFFSET_NOT_ACTIVE         	= -145,
    // mapping from SVGigE_GEV_STATUS_ERROR
    SVGigE_SVCAM_STATUS_CAMERA_COMMUNICATION_ERROR      = -199,

// Mapped transport layer return codes
   SVGigE_TL_SUCCESS                                    =    0,
    SVGigE_TL_DLL_NOT_LOADED                            = -201,
    SVGigE_TL_DLL_VERSION_MISMATCH                      = -202,
    SVGigE_TL_DLL_ALIGNMENT_PROBLEM                     = -203,
    SVGigE_TL_OPERATING_SYSTEM_NOT_SUPPORTED            = -204,
	SVGigE_TL_WINSOCK_INITIALIZATION_FAILED			    = -205,
    SVGigE_TL_CAMERA_NOT_FOUND                          = -206,
    SVGigE_TL_CAMERA_OPEN_FAILED                        = -207,
    SVGigE_TL_CAMERA_NOT_OPEN                           = -208,
    SVGigE_TL_CAMERA_UNKNWON_COMMAND                    = -209,
    SVGigE_TL_CAMERA_PAYLOAD_LENGTH_EXCEEDED            = -210,
    SVGigE_TL_CAMERA_PAYLOAD_LENGTH_INVALID             = -211,
    SVGigE_TL_CAMERA_COMMUNICATION_TIMEOUT              = -212,
	SVGigE_TL_CAMERA_ACCESS_DENIED						= -213,
	SVGigE_TL_CAMERA_CONNECTION_LOST					= -214,
    SVGigE_TL_STREAMING_FUNCTION_ALREADY_REGISTERED     = -215,
    SVGigE_TL_NO_STREAMING_PORT_FOUND                   = -216,
    SVGigE_TL_OUT_OF_MEMORY                             = -217,
    SVGigE_TL_INVALID_CALLBACK_FUNCTION_POINTER         = -218,
    SVGigE_TL_STREAMING_CHANNEL_NOT_AVAILABLE           = -219,
    SVGigE_TL_STREAMING_CHANNEL_VERSION_MISMATCH        = -220,
    SVGigE_TL_CALLBACK_INVALID_PARAMETER                = -221,
    SVGigE_TL_CALLBACK_IMAGE_DATA_MISSING               = -222,
    SVGigE_TL_OPENING_STREAMING_CHANNEL_FAILED          = -223,
    SVGigE_TL_CHANNEL_CREATION_FAILED                   = -224,
    SVGigE_TL_DRIVER_NOT_INSTALLED                      = -225,
    SVGigE_TL_PENDING_CHANNEL_DETECTED                  = -226,
	SVGigE_TL_UNSUPPORTED_PACKET_FORMAT                	= -227,
	SVGigE_TL_INCORRECT_BLOCK_ID                       	= -228,
	SVGigE_TL_INVALID_PARAMETER                        	= -229,
	SVGigE_TL_STREAMING_CHANNEL_LOOSING_FRAMES         	= -230,
	SVGigE_TL_FRAME_BUFFER_ALLOCATION_FAILED           	= -231,
	SVGigE_TL_FRAME_BUFFER_INFO_NOT_AVAILABLE           = -232,
	SVGigE_TL_MULTICAST_MANAGEMENT_FAILED              	= -233,
	SVGigE_TL_CAMERA_SIGNAL_IGNORED                     = -234,
    SVGigE_TL_FILTER_DRIVER_INSTALLATION_NOT_SUPPORTED  = -235,
    SVGigE_TL_FILTER_DRIVER_NOT_AVAILABLE               = -236,
    SVGigE_TL_FILTER_INF_FILE_NOT_FOUND                 = -237,
    SVGigE_TL_FILTER_INF_FILE_COPY_FAILED               = -238,
    SVGigE_TL_FILTER_INITIALIZING_COM_FAILED            = -239,
    SVGigE_TL_FILTER_DRIVER_INSTALLATION_FAILED         = -240,
    SVGigE_TL_FILTER_DRIVER_DEINSTALLATION_FAILED       = -241,
    SVGigE_TL_DRIVER_INSTALL_REQUIRES_ADMIN_PRIVILEGES  = -242,
	SVGigE_TL_DRIVER_INSTALL_REQUIRES_REBOOT			= -243,
	SVGigE_TL_DRIVER_REINSTALL_AFTER_REBOOT_REQUIRED	= -244,
	SVGigE_TL_DRIVER_X86_ON_X64_NOT_SUPPORTED			= -245,

	  // Mapped GEV return codes
	  SVGigE_GEV_STATUS_NOT_IMPLEMENTED                 = -301,
	  SVGigE_GEV_STATUS_INVALID_PARAMETER               = -302,
	  SVGigE_GEV_STATUS_INVALID_ADDRESS                 = -303,
	  SVGigE_GEV_STATUS_WRITE_PROTECT                   = -304,
	  SVGigE_GEV_STATUS_BAD_ALIGNMENT                   = -305,
	  SVGigE_GEV_STATUS_ACCESS_DENIED                   = -306,
	  SVGigE_GEV_STATUS_BUSY                            = -307,
	  SVGigE_GEV_STATUS_LOCAL_PROBLEM                   = -308,
	  SVGigE_GEV_STATUS_MSG_MISMATCH                    = -309,
	  SVGigE_GEV_STATUS_INVALID_PROTOCOL                = -310,
	  SVGigE_GEV_STATUS_NO_MSG                          = -311,
	  SVGigE_GEV_STATUS_PACKET_UNAVAILABLE              = -312,
	  SVGigE_GEV_STATUS_DATA_OVERRUN                    = -313,
	  SVGigE_GEV_STATUS_INVALID_HEADER                  = -314,
  	SVGigE_GEV_STATUS_ERROR                             = -399, // never issued, take SVGigE_SVCAM_STATUS_CAMERA_COMMUNICATION_ERROR instead

    // Mapped MessagingChannel return codes
    SVGigE_MC_MESSAGING_CHANNEL_NOT_AVAILABLE           = -400,
    SVGigE_MC_BUFFER_OVERRUN                            = -401,
    SVGigE_MC_MEMORY_PROBLEM                            = -402,
    SVGigE_MC_EVENT_NOT_FOUND                           = -403,
    SVGigE_MC_MESSAGE_TYPE_EVENT_NOT_FOUND              = -404,
    SVGigE_MC_MESSAGE_TYPE_EVENT_EXISTS_ALREADY         = -405,
    SVGigE_MC_MESSAGE_PENDING                           = -406,
    SVGigE_MC_MESSAGE_TIMEOUT                           = -407,
    SVGigE_MC_MESSAGE_QUEU_EMPTY                        = -408,
    SVGigE_MC_MESSAGE_EVENT_ERROR                       = -409,
    SVGigE_MC_MESSAGE_ID_MISMATCH                       = -410,
    SVGigE_MC_CALLBACK_INVALID                          = -411,
    SVGigE_MC_CALLBACK_NOT_REGISTERED                   = -412,
    SVGigE_MC_CALLBACK_REGISTERED_ALREADY               = -413,
    SVGigE_MC_CALLBACK_THREAD_NOT_RUNNING               = -414,
    SVGigE_MC_CALLBACK_THREAD_STILL_RUNNING             = -415,

  } SVGigE_RETURN;

	/** GigE transport layer type.
	 *  The SVGigE functionality is exposed through a transport layer which
	 *  connects to cameras in the network and which delivers images as well
	 *  as signals.
	 *  There are different types of transport layers, e.g. a NDIS filter
	 *  driver which assembles images from particular network packets in an
	 *  efficient way. A transport layer based on Winsock is another kind of
	 *  transport layer which has a higher CPU load footprint but does not have
	 *  a need for a driver being installed.
	 *  It is recommended to use a driver for performance and reliability reasons.
	 */
	typedef enum
	{
		SVGigETL_TypeNone				= 0,
		SVGigETL_TypeFilter				= 1,    // SVGigE filter driver (must be installed)
		SVGigETL_TypeWinsock			= 2,    // sockets available on Windows platforms

	}SVGigETL_Type;

  /** Camera feature information.
   *  A camera can support several items from the following set of camera features.
   */
  typedef enum
  {
    CAMERA_FEATURE_SOFTWARE_TRIGGER                    = 0,  // camera can be triggered by software
    CAMERA_FEATURE_HARDWARE_TRIGGER                    = 1,  // hardware trigger supported as well as trigger polarity
    CAMERA_FEATURE_HARDWARE_TRIGGER_EXTERNAL_EXPOSURE  = 2,  // hardware trigger with internal exposure supported as well as trigger polarity
    CAMERA_FEATURE_FRAMERATE_IN_FREERUNNING_MODE       = 3,  // framerate can be adjusted (in free-running mode)
    CAMERA_FEATURE_EXPOSURE_TIME                       = 4,  // exposure time can be adjusted
    CAMERA_FEATURE_EXPOSURE_DELAY                      = 5,  // exposure delay can be adjusted
    CAMERA_FEATURE_STROBE                              = 6,  // strobe is supported (polarity, duration and delay)
    CAMERA_FEATURE_AUTOGAIN                            = 7,  // autogain is supported
    CAMERA_FEATURE_ADCGAIN                             = 8,  // 2009-05-15/deprecated
    CAMERA_FEATURE_GAIN                                = 8,  // the ADC's gain can be adjusted
    CAMERA_FEATURE_AOI                                 = 9,  // image acquisition can be done for an AOI (area of interest)
    CAMERA_FEATURE_BINNING                             = 10, // binning is supported
    CAMERA_FEATURE_UPDATE_REGISTER                     = 11, // streaming channel related registers can be pre-set and then updated at once (e.g. for changing an AOI)
    CAMERA_FEATURE_NOT_USED                            = 12, // not in use
    CAMERA_FEATURE_COLORDEPTH_8BPP                     = 13, // a pixel depth of 8bit is supported
    CAMERA_FEATURE_COLORDEPTH_10BPP                    = 14, // a pixel depth of 10bit is supported
    CAMERA_FEATURE_COLORDEPTH_12BPP                    = 15, // a pixel depth of 12bit is supported
    CAMERA_FEATURE_COLORDEPTH_16BPP                    = 16, // a pixel depth of 16bit is supported
    CAMERA_FEATURE_ADCOFFSET                           = 17, // the ADC's offset can be adjusted
    CAMERA_FEATURE_SENSORDATA                          = 18, // the camera's sensor/ADC settings can be adjusted (for dual tap cameras)
    CAMERA_FEATURE_WHITEBALANCE                        = 19, // a LUT for whitebalancing is available
    CAMERA_FEATURE_LUT_10TO8                           = 20, // a LUT from 10 bit to 8 bit is available
    CAMERA_FEATURE_LUT_12TO8                           = 21, // a LUT from 12 bit to 8 bit is available
	CAMERA_FEATURES_FLAGS           				   = 22, // streaming state and image availability can be queried from camera
	CAMERA_FEATURES_READOUT_CONTROL      			   = 23, // time of image read out from camera can be controlled by application
	CAMERA_FEATURES_TAP_CONFIG         				   = 24, // the tap configuration can be changed (switching between one and two taps)
	CAMERA_FEATURES_ACQUISITION        				   = 25, // acquisition can be controlled by start/stop
	CAMERA_FEATURES_TAPBALANCE						   = 26, // camera is capable of running auto tap balance
	CAMERA_FEATURES_BINNING_V2						   = 27, // camera offers extended binning modes
    CAMERA_FEATURES_ROTATE_180                         = 28, // image is rotated by 180°
    CAMERA_FEATURES_CAMMODE_NG                         = 29, // camera has a next-generation register mapping
    CAMERA_FEATURES_CAMMODE_CLASSIC                    = 30, // camera has a classic register mapping
    CAMERA_FEATURES_NEXT_FEATUREVECTOR                 = 31, // a subsequent feature vector is available
    // Extended feature vector
    CAMERA_FEATURES2_START                             = 32, // first extended camera feature
	CAMERA_FEATURES2_1_TAP			                   = 32, // camera supports a single tap sensor
	CAMERA_FEATURES2_2_TAP			                   = 33, // camera supports a dual tap sensor
	CAMERA_FEATURES2_3_TAP			                   = 34, // camera supports a triple tap sensor
	CAMERA_FEATURES2_4_TAP			                   = 35, // camera supports a quadruple tap sensor
	CAMERA_FEATURES2_REBOOT		                       = 36, // a remote reboot command is supported
	CAMERA_FEATURES2_IOMUX			                   = 37, // IO multiplexer functionality is available
    CAMERA_FEATURES2_SOFTWARE_TRIGGER_ID               = 38, // writing a software trigger ID into images is supported
	CAMERA_FEATURES2_KNEE_POINTS					   = 39, // knee points available
	CAMERA_FEATURES2_NOISEFILTER					   = 40, // noise filter available
	CAMERA_FEATURES2_TRIGGERDEBOUNCE  				   = 41, // trigger bounce can be activated
	CAMERA_FEATURES2_TEMPERATURE_SENSOR				   = 42, // temperature sensor available
	CAMERA_FEATURES2_IOMUX_PWM						   = 43, // PWM A and B signals are available in IO multiplexer
	CAMERA_FEATURES2_IOMUX_STROBE2					   = 44, // STROBE0 and STROBE1 signals are available in IO multiplexer
	CAMERA_FEATURES2_PIV							   = 45, // PIV Mode
	CAMERA_FEATURES2_IOMUX_EXPOSE					   = 46, // EXPOSE signal is available in IO multiplexer
	CAMERA_FEATURES2_IOMUX_READOUT					   = 47, // READOUT signal is available in IO multiplexer
	CAMERA_FEATURES2_FLATFIELDCORRECTION			   = 48, // FLATFIELDCORRECTION is available
	CAMERA_FEATURES2_SHADINGCORRECTION				   = 49, // SHADINGCORRECTION is available
	CAMERA_FEATURES2_DEFECTPIXEL					   = 50, // DEFECTPIXEL treatment is available
    CAMERA_FEATURES2_TRIGGERBOTHEDGES                  = 51, // TRIGGERBOTHEDGES is available
    CAMERA_FEATURES2_USERGAIN                          = 52, // USERGAIN is available
    CAMERA_FEATURES2_USEROFFSET                        = 53, // USEROFFSET is available
    CAMERA_FEATURES2_BINNING_X2                        = 54, // BINNING_X2 is availble
    CAMERA_FEATURES2_BINNING_X3                        = 55, // BINNING_X3 is available
    CAMERA_FEATURES2_BINNING_X4                        = 56, // BINNING_X4 is available
	CAMERA_FEATURES2_IOMUX_LOGIC					   = 57, // IOMUX_LOGIC is available
	CAMERA_FEATURES2_IOMUX_STROBE4                     = 58, // IOMUX_STROBE4 is available
	CAMERA_FEATURES2_LENSCONTROL       				   = 59, // LENSCONTROL is supported
	CAMERA_FEATURES2_1_TAP_1X_1Y			  		   = 60, // camera supports a single tap sensor
	CAMERA_FEATURES2_2_TAP_2XE_1Y			  		   = 61, // camera supports a dual tap left/right sensor
	CAMERA_FEATURES2_2_TAP_1X_2YE			  		   = 62, // camera supports a dual tap top/bottom sensor
	CAMERA_FEATURES2_4_TAP_2XE_2YE					   = 63, // camera supports a quad tap sensor
	// Extended feature vector
	CAMERA_FEATURES3_START               			   = 64, // second extended camera feature
	CAMERA_FEATURES3_REVERSE_X	              		   = 64, // camera supports horizontal flipping
	CAMERA_FEATURES3_REVERSE_Y	                	   = 65, // camera supports vertical flipping
	CAMERA_FEATURES3_GLOBAL_SHUTTER                    = 66, // camera supports GLOBAL SHUTTER  Mode
	CAMERA_FEATURES3_ROLLING_SHUTTER                   = 67, // camera supports ROLLING SHUTTER Mode
	CAMERA_FEATURES3_MFT_FOCUS_UNIT                    = 68, // MFT focus unit can be changed,
	} CAMERA_FEATURE;


  /** Look-up table mode.
   *  A camera can be instructed to apply a look-up table. Usually this will
   *  be used for running a gamma correction. However, other goals can also
   *  be implemented by a look-up table, like converting a gray-scale picture
   *  into a binary black/white picture.
   */
  typedef enum
  {
    LUT_MODE_DISABLED               = 0,
    LUT_MODE_WHITE_BALANCE          = 1,  // 2006-12-20: deactivated, use
										  // Camera_setWhiteBalance() instead
	LUT_MODE_ENABLED                = 2,

	} LUT_MODE;

	/** Binning mode.
   *  A camera can be set to one of the following pre-defined binning modes
   */
  typedef enum
  {
		BINNING_MODE_OFF			= 0,
		BINNING_MODE_HORIZONTAL		= 1,
		BINNING_MODE_VERTICAL		= 2,
		BINNING_MODE_2x2			= 3,
		BINNING_MODE_3x3			= 4,
		BINNING_MODE_4x4			= 5,

  } BINNING_MODE;

  /** Lowpass filter.
   *  A lowpass filter can be activated/deactivated according to the
   *  following options.
   */
  typedef enum
  {
    LOWPASS_FILTER_NONE   = 0,
    LOWPASS_FILTER_3X3    = 1,

  } LOWPASS_FILTER;

  /** Multicast mode
   *  An application can receive images from a camera as a multicast controller,
   *  multicast listener or by non-multicast (default, unicast).
   */
  typedef enum
  {
    MULTICAST_MODE_NONE         = 0,
    MULTICAST_MODE_LISTENER     = 1,
    MULTICAST_MODE_CONTROLLER   = 2

  } MULTICAST_MODE;

  /** PIV mode
   *  A  camera can be set to enabled or disabled PIV mode
   */
  typedef enum
  {
    PIV_MODE_DISABLED  = 0,
    PIV_MODE_ENABLED   = 1,
  } PIV_MODE ;

  /** Acquisition mode.
   *  A camera can be set to one of the following acquisition modes
   */
  typedef enum
  {
    ACQUISITION_MODE_NO_ACQUISITION             = 0,  // 2013-07-08: deprecated
    ACQUISITION_MODE_FREE_RUNNING               = 1,  // 2008-05-06: deprecated, replaced by "fixed frequency"
    ACQUISITION_MODE_FIXED_FREQUENCY            = 1,  // 2011-08-29: deprecated, replaced by "programmable framerate"
    ACQUISITION_MODE_PROGRAMMABLE_FRAMERATE     = 1,
    ACQUISITION_MODE_INT_TRIGGER                = 2,  // 2008-05-06: deprecated, replaced by "software trigger"
    ACQUISITION_MODE_SOFTWARE_TRIGGER           = 2,
    ACQUISITION_MODE_EXT_TRIGGER_INT_EXPOSURE   = 3,
    ACQUISITION_MODE_EXT_TRIGGER_EXT_EXPOSURE   = 4,

  } ACQUISITION_MODE;

  /** Acquisition control
   *  A camera can be set to the following acquisition control modes
   */
  typedef enum
	{
    ACQUISITION_CONTROL_STOP    = 0,
    ACQUISITION_CONTROL_START   = 1,

  } ACQUISITION_CONTROL;

  /** Trigger polarity.
   *  A camera can be set to positive or negative trigger polarity
   */
  typedef enum
  {
	  TRIGGER_POLARITY_POSITIVE = 0,
	  TRIGGER_POLARITY_NEGATIVE = 1,

  } TRIGGER_POLARITY;

  /** Strobe polarity.
   *  A camera can be set to positive or negative strobe polarity
   */
  typedef enum
  {
    STROBE_POLARITY_POSITIVE = 0,
    STROBE_POLARITY_NEGATIVE = 1,

  } STROBE_POLARITY;

  /** Bayer conversion method
   */
  typedef enum
  {
    BAYER_METHOD_NONE         = -1,
    BAYER_METHOD_NEAREST      = 0,  // 2009-03-30: deprecated, mapped to BAYER_METHOD_SIMPLE
    BAYER_METHOD_SIMPLE       = 1,
    BAYER_METHOD_BILINEAR     = 2,  // 2009-03-30: deprecated, mapped to BAYER_METHOD_HQLINEAR
    BAYER_METHOD_HQLINEAR     = 3,
    BAYER_METHOD_EDGESENSE    = 4,  // 2009-03-30: deprecated, mapped to BAYER_METHOD_HQLINEAR
    BAYER_METHOD_GRAY         = 5,  // 2011-03-08: deprecated, user is sopposed to provide for own conversion

  } BAYER_METHOD;

  /** Pixel depth defines.
   *  The following pixel depths can be supported by a camera
   */
  typedef enum
  {
    SVGIGE_PIXEL_DEPTH_8     =0,
	//SVGIGE_PIXEL_DEPTH_10  =1, // not supported
    SVGIGE_PIXEL_DEPTH_12    =2,
    SVGIGE_PIXEL_DEPTH_16    =3,

  }SVGIGE_PIXEL_DEPTH;

	/** Correction step.
	 *  While performing image correction, a sequence of
	 *  particular steps is needed as they are defined below .
	 */
	typedef enum
	{
		IMAGE_CORRECTION_IDLE				    = 0,
		IMAGE_CORRECTION_ACQUIRE_BLACK_IMAGE	= 1,
		IMAGE_CORRECTION_ACQUIRE_WHITE_IMAGE	= 2,
		IMAGE_CORRECTION_SAVE_TO_EEPROM		    = 3,

	} IMAGE_CORRECTION_STEP;

	/** Correction mode.
	 *  After a successful image correction run, one of
	 *  the following modes can be activated in order to
	 *  control what image correction method is used.
	 */
	typedef enum
	{
		IMAGE_CORRECTION_NONE		  = 0,
		IMAGE_CORRECTION_OFFSET_ONLY  = 1,
		IMAGE_CORRECTION_ENABLED      = 2,

	} IMAGE_CORRECTION_MODE;


	/** Tap selection defines.
	 *  Each tap of a 2-tap or 4-tap camera can be selected
	 *  by using one of the following defines.
	 */
	typedef enum
	{
		SVGIGE_TAP_SELECT_TAP0		= 0,
		SVGIGE_TAP_SELECT_TAP1		= 1,
		SVGIGE_TAP_SELECT_TAP2		= 2,
		SVGIGE_TAP_SELECT_TAP3		= 3,

	}SVGIGE_TAP_SELECT;


	/** Tap configuration	selection defines.
	 *  Each configuration  of a 1-tap, 2-tap (horizontal and vertical) or 4-tap can be selected
	 *  by using one of the following defines.
	 */
	typedef enum
	{
		SVGIGE_SELECT_SINGLE_TAP	= 0,
		SVGIGE_SELECT_DUAL_TAP_H	= 1,
		SVGIGE_SELECT_DUAL_TAP_V	= 2,
		SVGIGE_SELECT_QUAD		 	= 3,

	}SVGIGE_TAP_CONFIGURATION_SELECT;


	/** flipping mode selection defines.
	 *  the following modes of flipping are available
	 */
	typedef enum
	{
		SVGIGE_REVERSE_OFF	= 0,
		SVGIGE_REVERSE_X	= 1,
		SVGIGE_REVERSE_Y	= 2,
		SVGIGE_REVERSE_X_Y	= 3,
	} SVGIGE_FLIPPING_MODE ;

	/** Shutter mode selection defines.
	 *  the following modes of Shutter are available
	 */
	typedef enum
	{
		SVGIGE_GLOBAL_SHUTTER  = 0,
		SVGIGE_ROLLING_SHUTTER = 1,
	}	SVGIGE_SHUTTER_MODE ;


	/** Auto tap balance modes.
	 *  The following modes of auto tap balancing are available
	 */
	typedef enum
	{
		SVGIGE_AUTO_TAP_BALANCE_MODE_OFF			=0,
		SVGIGE_AUTO_TAP_BALANCE_MODE_ONCE			=1,
		SVGIGE_AUTO_TAP_BALANCE_MODE_CONTINUOUS		=2,
		SVGIGE_AUTO_TAP_BALANCE_MODE_RESET			=3,

	}SVGIGE_AUTO_TAP_BALANCE_MODE;

	/** Whitebalance Arts.
	 *  The following Art of Whitebalance are available
	 */
	typedef enum
	{
		Histogramm_Based_WB =0,
		Gray_Based_WB =1,

	}SVGIGE_Whitebalance_SELECT;


	 /** The following maps for pixels correction are available
	 */
	typedef enum
	{
		Factory_Map =0,
		SVS_Map =1,
		Custom_Map =2,

	} PIXELS_CORRECTION_MAP_SELECT ;

	/** following focus units are available
    */
	typedef enum
		{
		 One_mm_Unit = 0,		//  focus  unit: 1 mm
		 Dec_mm_Unit =1,	   //   focus  unit:  1/10 mm
		} FOCUS_UNIT;

  /** Pixel type defines.
   *  The pixel type as it is defined in the GigE Vision specification
   */

  // Indicate that pixel is monochrome
  #define GVSP_PIX_MONO                       0x01000000
  #define GVSP_PIX_RGB                        0x02000000

  // Indicate effective number of bits occupied by the pixel (including padding).
  // This can be used to compute amount of memory required to store an image.
  #define GVSP_PIX_OCCUPY8BIT                 0x00080000
  #define GVSP_PIX_OCCUPY12BIT                0x000C0000
  #define GVSP_PIX_OCCUPY16BIT                0x00100000
  #define GVSP_PIX_OCCUPY24BIT                0x00180000

  // Bit masks
  #define GVSP_PIX_COLOR_MASK                 0xFF000000
  #define GVSP_PIX_EFFECTIVE_PIXELSIZE_MASK   0x00FF0000
  #define GVSP_PIX_ID_MASK                    0x0000FFFF

  // Bit shift value
  #define GVSP_PIX_EFFECTIVE_PIXELSIZE_SHIFT  16

  typedef enum
  {
    // Unknown pixel format
    GVSP_PIX_UNKNOWN          = 0x0000,

    // Mono buffer format defines
    GVSP_PIX_MONO8            = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT  | 0x0001),
    GVSP_PIX_MONO10           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0003),
    GVSP_PIX_MONO10_PACKED    = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY12BIT | 0x0004),
    GVSP_PIX_MONO12           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0005),
    GVSP_PIX_MONO12_PACKED    = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY12BIT | 0x0006),
    GVSP_PIX_MONO16           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0007),

    // Bayer buffer format defines
    GVSP_PIX_BAYGR8           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT  | 0x0008),
    GVSP_PIX_BAYRG8           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT  | 0x0009),
    GVSP_PIX_BAYGB8           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT  | 0x000A),
    GVSP_PIX_BAYBG8           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT  | 0x000B),
    GVSP_PIX_BAYGR10          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x000C),
    GVSP_PIX_BAYRG10          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x000D),
    GVSP_PIX_BAYGB10          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x000E),
    GVSP_PIX_BAYBG10          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x000F),
    GVSP_PIX_BAYGR12          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0010),
    GVSP_PIX_BAYRG12          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0011),
    GVSP_PIX_BAYGB12          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0012),
    GVSP_PIX_BAYBG12          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0013),

    // Color buffer format defines
    GVSP_PIX_RGB24            = (GVSP_PIX_RGB  | GVSP_PIX_OCCUPY24BIT),

    // Define for a gray image that was converted from a bayer coded image
    GVSP_PIX_GRAY8            = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT),

  } GVSP_PIXEL_TYPE;

  /** Signal types
   *  Each image that is delivered to an application by a callback will have a related signal
   *  which informs about the circumstances why a callback was triggered.
   *
   *  Usually a complete image will be delivered with the SVGigE_SIGNAL_FRAME_COMPLETED signal.
   */
  typedef enum
  {
	SVGigE_SIGNAL_NONE             					= 0,
	SVGigE_SIGNAL_FRAME_COMPLETED 					= 1,   // new image available, transfer was successful
	SVGigE_SIGNAL_FRAME_ABANDONED 					= 2,   // an image could not be completed in time and was therefore abandoned
	SVGigE_SIGNAL_END_OF_EXPOSURE  					= 3,   // end of exposure is currently mapped to transfer started
	SVGigE_SIGNAL_START_OF_TRANSFER					= 3,   // transfer of a new image has started
	SVGigE_SIGNAL_BANDWIDTH_EXCEEDED				= 4,   // available network bandwidth has been exceeded
	SVGigE_SIGNAL_OLD_STYLE_DATA_PACKETS  			= 5,   // driver problem due to old-style driver behavior (prior to 2003, not WDM driver)
	SVGigE_SIGNAL_TEST_PACKET						= 6,   // a test packet arrived
	SVGigE_SIGNAL_CAMERA_IMAGE_TRANSFER_DONE		= 7,   // the camera has finished an image transfer
	SVGigE_SIGNAL_CAMERA_CONNECTION_LOST			= 8,   // connection to camera got lost
	SVGigE_SIGNAL_MULTICAST_MESSAGE					= 9,   // an exceptional situation occurred during a multicast transmission
	SVGigE_SIGNAL_FRAME_INCOMPLETE					= 10,  // a frame could not be properly completed
	SVGigE_SIGNAL_MESSAGE_FIFO_OVERRUN				= 11,  // a next entry was put into the message FIFO before the old one was released
	SVGigE_SIGNAL_CAMERA_SEQ_DONE					= 12,  // the camera has finished a shutter sequence
	SVGigE_SIGNAL_CAMERA_TRIGGER_VIOLATION			= 13,  // the camera detected a trigger violation

  } SVGigE_SIGNAL_TYPE;

  /** IO multiplexer IN signals (signal sources).
   *  A camera provides for a flexible IO signal handling where one or
   *  multiple IN signals (signal sources) can be assigned to an OUT
   *  signal (signal sink). The following IN signals are defined.
   */
  typedef enum
	{
	SVGigE_IOMUX_IN0       = (1 <<  0),
	SVGigE_IOMUX_IN1       = (1 <<  1),
	SVGigE_IOMUX_IN2       = (1 <<  2),
	SVGigE_IOMUX_IN3       = (1 <<  3),
	SVGigE_IOMUX_IN4       = (1 <<  4),
	SVGigE_IOMUX_IN5       = (1 <<  5),
    SVGigE_IOMUX_IN6       = (1 <<  6),
    SVGigE_IOMUX_IN7       = (1 <<  7),
    SVGigE_IOMUX_IN8       = (1 <<  8),
    SVGigE_IOMUX_IN9       = (1 <<  9),
    SVGigE_IOMUX_IN10      = (1 << 10),
    SVGigE_IOMUX_IN11      = (1 << 11),
    SVGigE_IOMUX_IN12      = (1 << 12),
    SVGigE_IOMUX_IN13      = (1 << 13),
    SVGigE_IOMUX_IN14      = (1 << 14),
    SVGigE_IOMUX_IN15      = (1 << 15),
    SVGigE_IOMUX_IN16      = (1 << 16),
    SVGigE_IOMUX_IN17      = (1 << 17),
    SVGigE_IOMUX_IN18      = (1 << 18),
    SVGigE_IOMUX_IN19      = (1 << 19),
    SVGigE_IOMUX_IN20      = (1 << 20),
    SVGigE_IOMUX_IN21      = (1 << 21),
    SVGigE_IOMUX_IN22      = (1 << 22),
    SVGigE_IOMUX_IN23      = (1 << 23),
    SVGigE_IOMUX_IN24      = (1 << 24),
    SVGigE_IOMUX_IN25      = (1 << 25),
    SVGigE_IOMUX_IN26      = (1 << 26),
    SVGigE_IOMUX_IN27      = (1 << 27),
    SVGigE_IOMUX_IN28      = (1 << 28),
    SVGigE_IOMUX_IN29      = (1 << 29),
    SVGigE_IOMUX_IN30      = (1 << 30),
	SVGigE_IOMUX_IN31      = (1 << 31),
  } SVGigE_IOMux_IN;

  /** Some of the multiplexer's IN signals (signal sources) have a
   *  pre-defined usage:
   */


	#define SVGigE_IOMux_IN_STROBE3					SVGigE_IOMUX_IN21
  // STROBE0 signal from the camera (same as STROBE)
	#define SVGigE_IOMux_IN_STROBE2					SVGigE_IOMUX_IN20
  // for logic trigger
	#define SVGigE_IOMUX_IN_MASK_LOGIC				SVGigE_IOMUX_IN19
  // pre-scaler for trigger purposes
	#define SVGigE_IOMUX_IN_MASK_PRESCALE			SVGigE_IOMUX_IN17
  // Reject short pulses (debounce)
	#define SVGigE_IOMUX_IN_MASK_DEBOUNCE			SVGigE_IOMUX_IN16
  // PWMA signal (pulse width modulator)
	#define SVGigE_IOMux_IN_PWMD					SVGigE_IOMUX_IN14
  // PWMA signal (pulse width modulator)
	#define SVGigE_IOMux_IN_PWMC				   	SVGigE_IOMUX_IN13
  // READOUT signal from camera
  #define SVGigE_IOMUX_IN_MASK_PULSE				SVGigE_IOMUX_IN12
  // READOUT signal from camera
	#define SVGigE_IOMux_IN_READOUT			  		SVGigE_IOMUX_IN11
  // EXPOSE signal from camera
  #define SVGigE_IOMux_IN_EXPOSE					SVGigE_IOMUX_IN10
  // PWMB signal (pulse width modulator)
	#define SVGigE_IOMux_IN_PWMB				  	SVGigE_IOMUX_IN9
  // PWMA signal (pulse width modulator)
	#define SVGigE_IOMux_IN_PWMA					SVGigE_IOMUX_IN8
  // STROBE1 and STROBE2 signal from the camera
	#define SVGigE_IOMux_IN_STROBE_0and1			(SVGigE_IOMUX_IN6 | SVGigE_IOMUX_IN7)
  // STROBE1 signal from the camera
	#define SVGigE_IOMux_IN_STROBE1					SVGigE_IOMUX_IN7
  // STROBE0 signal from the camera (same as STROBE)
	#define SVGigE_IOMux_IN_STROBE0					SVGigE_IOMUX_IN6
  // STROBE signal from the camera
	#define SVGigE_IOMux_IN_STROBE					SVGigE_IOMUX_IN6
  // Transmitter output from UART
	#define SVGigE_IOMux_IN_UART_OUT				SVGigE_IOMUX_IN5
  // Receiver IO line from camera connector
	#define SVGigE_IOMux_IN_IO_RXD					SVGigE_IOMUX_IN4
  // Fixed High signal (2010-09-16/EKantz: changed from 8 to 31)
	#define SVGigE_IOMux_IN_FIXED_HIGH				SVGigE_IOMUX_IN31
  // Fixed Low signal (2010-09-16/EKantz: changed from 7 to 30)
	#define SVGigE_IOMux_IN_FIXED_LOW				SVGigE_IOMUX_IN30

	/** Signal values for a particular signal
   */
  typedef enum
  {
    IO_SIGNAL_OFF  = 0,
    IO_SIGNAL_ON   = 1
  } IO_SIGNAL;

  /** IO multiplexer OUT signals (signal sinks).
   *  A camera provides for a flexible IO signal handling where one or
   *  multiple IN signals (signal sources) can be assigned to an OUT
   *  signal (signal sink). The following OUT signals are defined.
   */
  typedef enum
  {
    SVGigE_IOMUX_OUT0       =  0,
    SVGigE_IOMUX_OUT1       =  1,
    SVGigE_IOMUX_OUT2       =  2,
    SVGigE_IOMUX_OUT3       =  3,
    SVGigE_IOMUX_OUT4       =  4,
    SVGigE_IOMUX_OUT5       =  5,
    SVGigE_IOMUX_OUT6       =  6,
    SVGigE_IOMUX_OUT7       =  7,
    SVGigE_IOMUX_OUT8       =  8,
    SVGigE_IOMUX_OUT9       =  9,
    SVGigE_IOMUX_OUT10      = 10,
    SVGigE_IOMUX_OUT11      = 11,
    SVGigE_IOMUX_OUT12      = 12,
    SVGigE_IOMUX_OUT13      = 13,
    SVGigE_IOMUX_OUT14      = 14,
    SVGigE_IOMUX_OUT15      = 15,
    SVGigE_IOMUX_OUT16      = 16,
    SVGigE_IOMUX_OUT17      = 17,
    SVGigE_IOMUX_OUT18      = 18,
    SVGigE_IOMUX_OUT19      = 19,
    SVGigE_IOMUX_OUT20      = 20,
    SVGigE_IOMUX_OUT21      = 21,
    SVGigE_IOMUX_OUT22      = 22,
    SVGigE_IOMUX_OUT23      = 23,
    SVGigE_IOMUX_OUT24      = 24,
    SVGigE_IOMUX_OUT25      = 25,
    SVGigE_IOMUX_OUT26      = 26,
    SVGigE_IOMUX_OUT27      = 27,
    SVGigE_IOMUX_OUT28      = 28,
    SVGigE_IOMUX_OUT29      = 29,
    SVGigE_IOMUX_OUT30      = 30,
    SVGigE_IOMUX_OUT31      = 31,
  } SVGigE_IOMux_OUT;


  /** Some of the multiplexer's OUT signals (signal sinks) have a
   *  pre-defined usage:
   */
  // Trigger signal to camera
  #define SVGigE_IOMux_OUT_TRIGGER    SVGigE_IOMUX_OUT6
  // Receiver input to UART
  #define SVGigE_IOMux_OUT_UART_IN    SVGigE_IOMUX_OUT5
  // Transmitter IO line from camera connector
  #define SVGigE_IOMux_OUT_IO_TXD     SVGigE_IOMUX_OUT4

  /** Baud rate for a camera's UART
   *  A camera's UART can be set to the following pre-defined baud rates.
   */
  typedef enum
  {
    SVGigE_BaudRate_110        =    110,
    SVGigE_BaudRate_300        =    300,
    SVGigE_BaudRate_1200       =   1200,
    SVGigE_BaudRate_2400       =   2400,
    SVGigE_BaudRate_4800       =   4800,
    SVGigE_BaudRate_9600       =   9600,
    SVGigE_BaudRate_19200      =  19200,
    SVGigE_BaudRate_38400      =  38400,
    SVGigE_BaudRate_57600      =  57600,
    SVGigE_BaudRate_115200     = 115200,
  } SVGigE_BaudRate;

  /** Camera buffer structure
	  *  Camera information will be stored and transferred to the application
	  *  using the following structure
	  */
  typedef struct {
    unsigned int controlIP;
    unsigned int controlSubnet;
    unsigned int controlGateway;
    unsigned long macHigh;
    unsigned long macLow;
    char manufacturer[32];
    char model[32];
    char specificInformation[48];
    char deviceVersion[32];
    char serialNumber[16];
    char userName[16];
    unsigned short versionMajor;
    unsigned short versionMinor;
    unsigned int localIP;
  } SVGigE_CAMERA;

  /** Image buffer structure
	  *  An image data pointer along with accompanying data will be stored and
	  *  transferred to the application using the following structure
	  */
  typedef struct {
	unsigned char *ImageData;		// pointer to image data
	int ImageSize;					// image buffer size

	unsigned int ImageID;			// image ID assigned by camera

	int BufferIndex;				// buffer index of current image
	int ImageCount;					// total number of images
	int FrameLoss;					// lost images in streaming channel instance
	int PacketCount;				// packet count for one image
	int PacketResend;				// number of resend packets in one image

	int TransferTime; 				// Total packet transfer time from receiving first packet until
									// finishing image and sending to user space [microseconds]

	float ActualFrameRate;			// actual frame rate in streaming channel
	float ActualDataRate;			// actual data rate in streaming channel

	unsigned char Header[576];		// header data of current image

	SVGigE_SIGNAL_TYPE SignalType;	// type of signal

	bool ImageLocked;				// flag for locking image while doing image processing

  } SVGigE_IMAGE;

  /** Signal structure.
   * A camera signal conveys information about an event that took place
   * during image acquisition. In case of the end-of-transfer signal
   * an image pointer along with accompanying image data will be delivered
   * to the application contained in a data transport structure.
   */
  typedef struct {
	  SVGigE_SIGNAL_TYPE SignalType;
	  void *Data;
	  int DataLength;
  } SVGigE_SIGNAL;

  /** Streaming channel callback function.
   *  The user-defined streaming channel callback function will be called each
   *  time when an image acquisition from camera has been finished and the image
   *  is ready for processing
   *
   *  NOTE: The callback function will return a NULL pointer in case an image
   *        could not be completely received over the network due to a timeout,
   *        e.g. in the result of insufficient bandwidth
   */
  typedef SVGigE_RETURN (__stdcall *StreamCallback)(Image_handle Image, void* Context);

  /** Messaging channel callback function.
   *  The user-defined messaging channel callback function will be called each time
   *  when an event was signaled which arrived from a camera or from intermediate software
   *  layers.
   *
   *  An application should retrieve, process appropriately and finally release any message
   *  that has arrived when the event callback function gets control. There might be one or
   *  multiple messages waiting for processing in a FIFO on entry to the callback function.
   *
   *  HINT: If the size of the FIFO was not sufficient for handling all messages that
   *  arrived from one callback to the next callback then an exception will be raised.
   *  An application must catch this exception and should react with an appropriate
   *  user message, log entry or the like which informs an operator about this
   *  exceptional situation.
   */
  typedef SVGigE_RETURN (__stdcall *EventCallback)(Event_handle EventID, void* Context);


#ifdef __USRDLL__
  #define __usrdllexport__  __declspec(dllexport)
#else
  #define __usrdllexport__
#endif


//------------------------------------------------------------------------------
// 0 - GigE DLL and driver
//------------------------------------------------------------------------------

  /** isVersionCompliant.
   *  The DLL's version at compile time will be checked against an expected
   *  version at runtime. The calling program knows the runtime version that
   *  it needs for proper functioning and can handle a version mismatch, e.g.
   *  by displaying the expected and the found DLL version to the user.
   *
	 *  NOTE: If the DLL version matches expected version, then subsequently a
	 *        check for driver availability will be performed. The result may be:
	 *          - "not installed" (if no SVGigE driver could be found)
	 *          - "not available" (if a driver is installed but communication failed, e.g. x86 on x64)
   *
   *  @param DllVersion a pointer to a version structure for the current DLL version
   *  @param ExpectedVersion a pointer to a version structure with the expected DLL version
   *  @return SVGigE_SUCCESS or an appropriate SVGigE error code
   */
  __usrdllexport__ SVGigE_RETURN
  isVersionCompliantDLL(SVGigE_VERSION *DllVersion,
                        SVGigE_VERSION *ExpectedVersion);

  /** isDriverAvailable.
   *  It will be checked whether a FilterDriver is available. The following return codes apply:
	 *    - "success" - driver is installed and available for work
	 *    - "TL not loaded" - a FilterDriver transport layer is not available
	 *    - "not installed" - no FilterDriver installed
	 *    - "not available" - FilterDriver installed but not available (e.g. x86 on x64)
   *
   *  @return SVGigE_SUCCESS or an appropriate SVGigE return code
   */
  __usrdllexport__ SVGigE_RETURN
  isDriverAvailable();

	//------------------------internal-use----------------------------------------
  /** isLoadedGigEDLL
   *  A check is performed if the GigE DLL has already been loaded and if this
   *  is not the case it will be tried to load it.
   */
  bool isLoadedGigEDLL();

//------------------------------------------------------------------------------
// 1 - Camera: Discovery and bookkeeping
//------------------------------------------------------------------------------

  /** Create camera container.
   *  A handle is obtained that references a management object for discovering
   *  and selecting GigE cameras in the network segments that a computer is
   *  connected to.
   *
	 *  @TransportLayerType the type of the transport layer DLL to be used
   *  @return on success a handle for subsequent camera container function calls
   *         or -1 if creating a camera container failed
   */
  __usrdllexport__ CameraContainerClient_handle
  CameraContainer_create(SVGigETL_Type TransportLayerType);

  /** Delete Camera container.
   *  A previously created camera container reference will be released. After
   *  deleting a camera container all camera specific functions are no longer
   *  available
   *
   *  @param hCameraContainer a handle received from CameraContainer_create()
   */
  __usrdllexport__ SVGigE_RETURN
  CameraContainer_delete(CameraContainerClient_handle hCameraContainer);

  /** Device discovery.
   *  All network segments that a computer is connected to will be serached for
   *  GigE cameras by sending out a network broadcast and subsequently analyzing
   *  camera responses.
   *
   *  @param hCameraContainer a handle received from CameraContainer_create()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  CameraContainer_discovery(CameraContainerClient_handle hCameraContainer);

  /** Get number of connected cameras.
   *  The maximal index is returned that cameras can be accessed with in the
   *  camera container
   *
   *  @param hCameraContainer a handle received from CameraContainer_create()
   *  @return number of available cameras
   */
  __usrdllexport__ int
  CameraContainer_getNumberOfCameras(CameraContainerClient_handle hCameraContainer);

  /** Get camera.
   *  A camera handle is obtained accordingly to an index that was specified
   *  as an input parameter
   *
   *  @param hCameraContainer a handle received from CameraContainer_create()
   *  @param CameraIndex camera index from zero to one less the number of available cameras
   *  @return a handle for subsequent camera function calls or NULL in case the
   *         index was specified out of range
   */
  __usrdllexport__ Camera_handle
  CameraContainer_getCamera(CameraContainerClient_handle hCameraContainer,
                            int CameraIndex);

  /** Find camera.
   *  A camera handle is obtained accordingly to a search string that will be
   *  matched against the following items:
   *   - MAC address
   *   - IP address
   *   - serial number
   *   - user defined name
   *   If one of those items can be matched a valid handle is returned.
   *   Otherwise a SVGigE_NO_CAMERA return value will be generated
   *
   *  @param hCameraContainer a handle received from CameraContainer_create()
   *  @param CameraItem a string for matching it against the above items
   *  @return a handle for subsequent camera function calls or NULL in case the
   *          index was specified out of range
   */
  __usrdllexport__ Camera_handle
  CameraContainer_findCamera(CameraContainerClient_handle hCameraContainer,
                             const char *CameraItem);



//-------------------------------------------
  /** Set CommandTransferTimeout
	*  The user can set the appropriate Timeout for the transfer.
	*  this have to be occurred befor opening the camera, if not the default timeout will be used, in this case 200 ms.
	*  @param hCameraContainer a handle received from CameraContainer_create()
	*  @param UserTransferTimeout the current timeout to be used
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
  */
  __usrdllexport__ Camera_handle
  CameraContainer_setCommandTransferTimeout(CameraContainerClient_handle hCameraContainer,
                             int UserTransferTimeout);

 /** Get the current used timeout
	*  Current used Transfer Timeout will be returned.
	*  @param hCameraContainer a handle from a camera that has been opened before
	*  @param UserTransferTimeout  the current timeout used
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
  */
   __usrdllexport__ Camera_handle
  CameraContainer_getCommandTransferTimeout(CameraContainerClient_handle hCameraContainer,
                             int *UserTransferTimeout);
  //---------------------------------------------------------

//------------------------------------------------------------------------------
// 2 - Camera: Connection
//------------------------------------------------------------------------------

  /** Open connection to camera.
   *  A TCP/IP control channel will be established.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Timeout the time without traffic or heartbeat after which a camera drops a connection (default: 3.0 sec.)
   *   NOTE: Values between 0.0 to 0.5 sec. will be mapped to the default value (3.0 sec.)
	 * HINT: It is recommended to use an extended timeout for debug sessions (e.g. 30 sec.).
	 * This prevents from loosing connection to a camera due to missing heartbeat when
	 * stepping through a program in debug mode.
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_openConnection(Camera_handle hCamera, float Timeout);

  /** Open connection to camera.
   *  A TCP/IP control channel will be established.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param HeartbeatTimeout the time without traffic or heartbeat after which a camera drops a connection (default: 3.0 sec.)
   	* NOTE: Values between 0.0 to 0.5 sec. will be mapped to the default value (3.0 sec.)
	* HINT: It is recommended to use an extended timeout for debug sessions (e.g. 30 sec.).
	* This prevents from loosing connection to a camera due to missing heartbeat when
	* stepping through a program in debug mode.
   *  @param GVCPRetryCount retrys before giving up, valid values between 1 and n. Default is 3
   *  @param GVCPTimeout Timeout before next retry in msec. Default is 200 msec
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_openConnectionEx(Camera_handle hCamera, float HeartbeatTimeout, int GVCPRetryCount, int GVCPTimeout);


  /** Disconnect camera.
   *  The TCP/IP control channel will be closed
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_closeConnection(Camera_handle hCamera);

  /** Set IP address.
   *  The camera will get a new persistent IP address assigned. If the camera
   *  is currently unavailable in the subnet where it is attached to, then a
   *  temporary IP address will be forced into the camera first. In any case
   *  the camera will have the new IP address assigned as a persistent IP
   *  which will apply after camera's next reboot.
   *
   *  HINT:
   *  If an IP address is set that is not inside the subnet where the camera
   *  is currently connected to, then the camera becomes unavailable after next
   *  reboot. This can be avoided by having a valid IP address assigned automatically
   *  by setting both values to zero, IPAddress and NetMask
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param IPAddress a valid IP address or zero (for automatically assigning a valid IP/netmask)
   *  @param NetMask a matching net mask or zero (for automatically assigning a valid IP/netmask)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setIPAddress(Camera_handle hCamera,
                      unsigned int IPAddress,
                      unsigned int NetMask);

  /** Force valid network settings
   *  A camera's availability will be evaluated. If it is outside current subnet
   *  then it will be forced to valid network settings inside current subnet.
   *  Valid network settings will be reported back to caller.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param IPAddress the new IP address that has been selected and forced
   *  @param SubnetMask the new subnet mask that has been selected and forced
   *  @return success or error code
   *
   *  HINT: If the return code is SVGigE_SVCAM_STATUS_CAMERA_OCCUPIED then the
   *        IPAdress will show the IP of the host that occupies the camera.
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_forceValidNetworkSettings(Camera_handle hCamera,
                                   unsigned int *IPAddress,
                                   unsigned int *SubnetMask);

  /** Restart IP configuration
   *  A camera's IP configuration process will be restarted. Usually this
   *  function will be used when a new DHCP address configuration should be
   *  performed. If no DHCP server is available then a fallback to LLA will
   *  take place. If, however, the camera has already got a persistent IP
   *  assigned then the result of the IP configuration process will be the
   *  already assigned fixed IP address.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_restartIPConfiguration(Camera_handle hCamera);


//------------------------------------------------------------------------------
// 3 - Camera: Information
//------------------------------------------------------------------------------

  /** Get manufacturer name.
   *  The manufacturer name that is obtained from the camera firmware will be
   *  returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ const char *
  Camera_getManufacturerName(Camera_handle hCamera);

  /** Get model name.
   *  The model name that is obtained from the camera firmware will be
   *  returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ const char *
  Camera_getModelName(Camera_handle hCamera);

  /** Get device version.
   *  The device version that is obtained from the camera firmware will be
   *  returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ const char *
  Camera_getDeviceVersion(Camera_handle hCamera);

  /** Get manufacturer specific information.
   *  The manufacturer specific information that is obtained from the camera
   *  firmware will be returned
   *
   *  @see CameraContainer_getCamera()fo
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ const char *
  Camera_getManufacturerSpecificInformation(Camera_handle hCamera);

  /** Get serial number.
   *  The (manufacturer-assigned) serial number will be obtained from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ const char *
  Camera_getSerialNumber(Camera_handle hCamera);

  /** Set user-defined name
   *  A user-defined name will be uploaded to a camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param UserDefinedName the name to be transferred to the camera
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setUserDefinedName(Camera_handle hCamera, const char *UserDefinedName);

  /** Get user-defined name
   *  A name that has been assigned to a camera by the user will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ const char *
  Camera_getUserDefinedName(Camera_handle hCamera);

  /** Get MAC address.
   *  The MAC address that is obtained from the camera firmware will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ const char *
  Camera_getMacAddress(Camera_handle hCamera);

  /** Get IP address.
   *  The IP address that the camera is currently working on will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ const char *
  Camera_getIPAddress(Camera_handle hCamera);

  /** Get subnet mask.
   *  The subnet mask that the camera is currently working with will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ const char *
  Camera_getSubnetMask(Camera_handle hCamera);

  /** Get pixel clock.
   *  The camera's pixel clock will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelClock a reference to the pixel clock value
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getPixelClock(Camera_handle hCamera,
                       int *PixelClock);

  /** Is camera feature.
   *  The camera will be queried whether a feature is available or not.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Feature a feature which availability will be determined
   *  @return a boolean value indicating whether the queried feature is available or not
   */
  __usrdllexport__ bool
  Camera_isCameraFeature(Camera_handle hCamera, CAMERA_FEATURE Feature);

  /** Read XML file.
   *  The camera's XML file will be retrieved and made available for further
   *  processing by an application. The returned pointer to the content of the
   *  XML file will be valid until the function is called again or until
   *  the camera is closed.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param XML a pointer to a zero-terminated string containing the complete XML file
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_readXMLFile(Camera_handle hCamera, const char **XML);

 /**  Get Sensor temperature.
   * The current camera's Sensor temperature  will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SensorTemperatur  the actual sensor temperature of the camera [°C]
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getSensorTemperature(Camera_handle hCamera, unsigned int *SensorTemperatur);

//------------------------------------------------------------------------------
// 4 - Stream: Channel creation and control
//------------------------------------------------------------------------------

  /** Create streaming channel.
   *  A UDP streaming channel will be established for image data transfer.
   *  A connection to the camera has to be successfully opened first using
   *  Camera_openConnection() before a streaming channel can be established
   *
   *  @see Camera_openConnection()
   *  @param hStreamingChannel a handle for the streaming channel will be returned
   *  @param hCameraContainer a camera container handle received from CameraContainer_create()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BufferCount specifies the number of image buffers (default: 3 buffers)
   *                    NOTE: A value of 0 will be mapped to the default value (3 buffers)
   *  @param PacketResendTimeout Timeout for packet resend in msec. Default is 16 ms
   *  @param CallbackFunction user-defined callback function for image processing
   *  @param Context user-defined data that will be returned on each callback
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_createEx(StreamingChannel_handle *hStreamingChannel,
                          CameraContainerClient_handle hCameraContainer,
                          Camera_handle hCamera,
                          int BufferCount,
						  int PacketResendTimeout,
                          StreamCallback CallbackFunction,
                          void *Context);
  /** Create streaming channel.
   *  A UDP streaming channel will be established for image data transfer.
   *  A connection to the camera has to be successfully opened first using
   *  Camera_openConnection() before a streaming channel can be established
   *
   *  @see Camera_openConnection()
   *  @param hStreamingChannel a handle for the streaming channel will be returned
   *  @param hCameraContainer a camera container handle received from CameraContainer_create()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BufferCount specifies the number of image buffers (default: 3 buffers)
   *                    NOTE: A value of 0 will be mapped to the default value (3 buffers)
   *  @param CallbackFunction user-defined callback function for image processing
   *  @param Context user-defined data that will be returned on each callback
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_create(StreamingChannel_handle *hStreamingChannel,
                          CameraContainerClient_handle hCameraContainer,
                          Camera_handle hCamera,
                          int BufferCount,
                          StreamCallback CallbackFunction,
                          void *Context);

  /** Delete streaming channel.
   *  A streaming channel will be closed and all resources will be released that
   *  have been occupied by the streaming channel
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_delete(StreamingChannel_handle hStreamingChannel);



  /** Set readout transfer.
   *  The readout transfer of a streaming channel will be enabled or disabled
   *  dependent on an isReadoutTransfer parameter. If false, then the camera
   *  would capture an image but it would not transfer the image to the host.
   *  The application can request the streaming channel to send the already
   *  captured data by toggling the isRedoutTransfer parameter to true at
   *  any time after image exposure has finished.
   *  If the isReadoutTransfer parameter is toggled to true before image
   *  exposure has finished then the default behavior will take place where
   *  the camera starts image data transfer immediately after image exposure
   *  has finished.
   *  Controlling readout transfer might be useful when operating multiple
   *  cameras that are triggered all at the same time. The application will
   *  be able to request data in a pre-defined way on a one-by-one basis and
   *  to avoid bandwidth bottlenecks this way which otherwise might occur.
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param isReadoutTransfer whether an image will be transferred to the host after readout
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_setReadoutTransfer(StreamingChannel_handle hStreamingChannel,
                                      bool isReadoutTransfer);

  /** Get readout transfer.
   *  The readout transfer of a streaming channel will be retrieved. If false,
   *  then the camera would capture an image but it would not transfer the image
   *  to the host. The application can request the streaming channel to send the
   *  already captured data by toggling the isRedoutTransfer parameter to true at
   *  any time after image exposure has finished.
   *  If the isReadoutTransfer parameter is toggled to true before image
   *  exposure has finished then the default behavior will take place where
   *  the camera starts image data transfer immediately after image exposure
   *  has finished.
   *  Controlling readout transfer might be useful when operating multiple
   *  cameras that are triggered all at the same time. The application will
   *  be able to request data in a pre-defined way on a one-by-one basis and
   *  to avoid bandwidth bottlenecks this way which otherwise might occur.
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param isReadoutTransfer whether an image will be transferred to the host after readout
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getReadoutTransfer(StreamingChannel_handle hStreamingChannel,
                                      bool *isReadoutTransfer);

//------------------------------------------------------------------------------
// 5 - Stream: Channel statistics
//------------------------------------------------------------------------------

  /** Get frame loss.
   *  The number of lost frames in a streaming channel will be returned
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param FrameLoss the number of frames that have been lost since the streaming channel was opened
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getFrameLoss(StreamingChannel_handle hStreamingChannel,
                                int *FrameLoss);

  /** Get actual frame rate.
   *  The actual frame rate calculated from received images will be returned
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param ActualFrameRate the actual frame rate measured based on received images
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getActualFrameRate(StreamingChannel_handle hStreamingChannel,
                                      float *ActualFrameRate);

  /** Get actual data rate.
   *  The actual data rate calculated from received image data will be returned
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param ActualDataRate the actual data rate measured based on received image data
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getActualDataRate(StreamingChannel_handle hStreamingChannel,
                                     float *ActualDataRate);

  /** Get peak data rate.
   *  The peak data rate will be returned. It is determined for a single image
   *  transfer by measuring the transfer time from first to last network packet
   *  which belong to a single image. The peak data rate is received by dividing
   *  the amount of data of one image by that transfer time. It can be used for
   *  evaluating the bandwidth situation when operating multiple GigE cameras
   *  over a single GigE line.
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param PeakDataRate the actual frame rate measured based on received image data
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getPeakDataRate(StreamingChannel_handle hStreamingChannel,
                                   float *PeakDataRate);

//------------------------------------------------------------------------------
// 6 - Stream: Channel info
//------------------------------------------------------------------------------

  /** Get pixel type.
	*  The pixel type will be returned that applies to the output image
	*
	*  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
	*  @param PixelType the programmed pixel type that has been set for the output image/view
	*  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getPixelType(StreamingChannel_handle hStreamingChannel,
                                GVSP_PIXEL_TYPE *PixelType);

  /** Get buffer data.
   *  A streaming channel will be queried for information about one of its image buffers.
	 *  On success, a pointer to image data will be returned.
	 *  Since the buffer's data pointer is queried asynchronously with regard to image
   *  acquisition, no assumption can be made for the content that is in the buffer at
   *  time of running this function. Image access functions have to be used for obtaining
   *  actual images that were captured by the camera.
	 *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
	 *  @param BufferIndex an index of the buffer where information is requested for
	 *  @param BufferData a pointer to a data pointer that will return the buffer address
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getBufferData(StreamingChannel_handle hStreamingChannel,
																 unsigned int BufferIndex,
	                               unsigned char **BufferData);

  /** Get buffer size.
   *  The buffer size will be returned that applies to the output image
	 *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
	 *  @param BufferSize the buffer size for the output image/view
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getBufferSize(StreamingChannel_handle hStreamingChannel,
                                 int *BufferSize);

  /** Get image pitch.
   *  The image pitch will be returned that applies to the output image
	 *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
	 *  @param ImagePitch the image pitch for the output image/view
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getImagePitch(StreamingChannel_handle hStreamingChannel,
                                 int *ImagePitch);

  /** Get image size X.
   *  The image size X will be returned that applies to the output image
	 *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
	 *  @param ImageSizeX the image size X for the output image/view
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getImageSizeX(StreamingChannel_handle hStreamingChannel,
                                 int *ImageSizeX);

  /** Get image size Y.
   *  The image size Y will be returned that applies to the output image
	 *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
	 *  @param ImageSizeY the image size Y for the output image/view
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getImageSizeY(StreamingChannel_handle hStreamingChannel,
                                 int *ImageSizeY);

//------------------------------------------------------------------------------
// 7 - Stream: Transfer Parameters
//------------------------------------------------------------------------------

  /** Evaluate maximal packet size.
   *  A test will be performed which determines the maximal usable packet size
   *  based on given network hardware. This value will be used when opening a
   *  streaming channel.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
	 *  @param MaximalPacketSize the maximal possible packet size without fragmentation
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_evaluateMaximalPacketSize(Camera_handle hCamera,
                                   int *MaximalPacketSize);

	/** Set streaming packet size.
	 *  The packet size is set which will be generated by the camera for streaming
	 *  image data as payload packets to the host
	 *
	 *  WARNING! Explicitly setting network packet size to values above 1500 bytes
	 *           may provide to unpredictable results and also to a system crash.
	 *           Please use "Camera_evaluateMaximalPacketSize" for a save adjustment
	 *           to jumbo packets.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
	 *  @param StreamingPacketSize the packet size used by the camera for image packets
   *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
  Camera_setStreamingPacketSize(Camera_handle hCamera,
                                int StreamingPacketSize);

  /** Set inter-packet delay
   *  A delay between network packets will be introduced and set to a specified
   *  number of ticks.
   *
   *  NOTE: The resulting total image transfer time must not exceed 250 ms.
   *  Otherwise a timeout condition will be reached in the SVGigE driver.
   *  The dependency between inter-packet delay and total image transfer time
   *  depends on pixel clock, image size and has to be determine case by case.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param InterPacketDelay the new value for inter-packet delay (0..1000, relative number)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setInterPacketDelay(Camera_handle hCamera,
                             float InterPacketDelay);

  /** Get inter-packet delay
   *  The delay between network packets will be returned as a relative number
   *  of ticks.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param InterPackedDelay the currently programmed value for inter-packet delay (0..1000, relative number)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getInterPacketDelay(Camera_handle hCamera,
                             float *InterPackedDelay);

  /** Set multicast mode
   *  The multicast mode will be set to none (default), listener or controller.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
	 *  @param MulticastMode the intended new multicast mode
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setMulticastMode(Camera_handle hCamera,
                          MULTICAST_MODE MulticastMode);

  /** Get multicast mode
   *  Current multicast mode will be retrieved from the camera.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
	 *  @param MulticastMode current multicast mode will be returned
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getMulticastMode(Camera_handle hCamera,
                          MULTICAST_MODE *MulticastMode);

  /** Get multicast group
   *  Current multicast group (IP) and port will be retrieved from the camera.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
	 *  @param MulticastGroup current multicast group (IP) will be returned
	 *  @param MulticastPort current multicast port will be returned
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getMulticastGroup(Camera_handle hCamera,
                           unsigned int *MulticastGroup,
                           unsigned int *MulticastPort);

//------------------------------------------------------------------------------
// 8 - Stream: Image access
//------------------------------------------------------------------------------

  /** Get image pointer.
   *  A pointer to the image data will be returned
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return a pointer to the image data
   */
  __usrdllexport__ unsigned char *
  Image_getDataPointer(Image_handle hImage);

  /** Get buffer index.
   *  The index of the current image buffer will be returned
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return a pointer to the image data
   */
  __usrdllexport__ int
  Image_getBufferIndex(Image_handle hImage);

  /** Get signal type
   *  The signal type that is related to a callback will be returned.
   *
   *  HINT: If the camera cable is disconnected while a connection to that camera
   *        is open, the image callback will return a NULL pointer. When the
   *        Image_getSignalType() function is called in that situation it will
   *        return a value: SVGigE_SIGNAL_CAMERA_CONNECTION_LOST. That signal
   *        value may be used for taking further actions.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the signal type which informs why a callback was triggered
   */
	__usrdllexport__ SVGigE_SIGNAL_TYPE
  Image_getSignalType(Image_handle hImage);

  /** Get camera handle
   *  A handle of the camera that captured the image will be returned
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return a camera handle
   */
  __usrdllexport__ Camera_handle
  Image_getCamera(Image_handle hImage);

  /** Release image.
   *  An image which availability was indicated by a StreamCallback function needs
   *  to be released after processing it by a user application in order to free
   *  the occupied buffer space for a subsequent image acquisition.
   *
   *  After releasing an image, none image access function must be called anymore
   *  which would use the released image handle.
   *
   *  NOTE: The Image_release() function is called automatically when leaving the
   *        callback function. Therefore, afterwards a buffer will stay available
   *        with image content only as long as a buffer overrun occurs. A buffer
   *        overrun can be prevented by using a sufficient number of buffers.
   *
   *  HINT: An application is responsible for providing a sufficient number of
   *        buffers to the driver in order to prevent from buffer overrun. It is
   *        recommended to assign e.g. at least 60 MByte total buffer space
   *        (number of buffers times the size of a particular buffer). That amount
   *        of buffer space would make sure that images are available also in worst
   *        case for at least 500 ms because of a limited bandwidth on a GigE line
   *        (< 120 MB/s).
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Image_release(Image_handle hImage);

//------------------------------------------------------------------------------
// 9 - Stream: Image conversion
//------------------------------------------------------------------------------

  /** Get image RGB
   *
   *  The image will be converted by a selectable Bayer conversion algorithm into
   *  a RGB image. The caller needs to provide a sufficient buffer.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @param BufferRGB a buffer for RGB image data
   *  @param BufferLength the length of the image buffer
   *  @param BayerMethod the a demosaicking method (Bayer method)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Image_getImageRGB(Image_handle hImage,
                    unsigned char *BufferRGB,
                    int BufferLength,
                    BAYER_METHOD BayerMethod);

	/** Get image 12-bit as 8-bit
	 *  A 12-bit image will be converted into a 8-bit image. The caller needs to
	 *  provide for a sufficient buffer for the 8-bit image.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @param Buffer8bit a buffer for 8-bit image data
   *  @param BufferLength the length of the image buffer
   *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
	Image_getImage12bitAs8bit(Image_handle hImage,
                            unsigned char *Buffer8bit,
                            int BufferLength);

	/** Get image 12-bit as 16-bit
	 *  A 12-bit image will be converted into a 16-bit image. The caller needs to
	 *  provide for a sufficiently large buffer (2 x width x height bytes) for the
   *  16-bit image.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @param Buffer16bit a buffer for 16-bit image data
   *  @param BufferLength the length of the image buffer
   *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
	Image_getImage12bitAs16bit(Image_handle hImage,
                             unsigned char *Buffer16bit,
                             int BufferLength);

	/** Get image 16-bit as 8-bit
	 *  A 16-bit image will be converted into a 8-bit image. The caller needs to
	 *  provide for a sufficient buffer for the 8-bit image.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @param Buffer8bit a buffer for 8-bit image data
   *  @param BufferLength the length of the image buffer
   *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
	Image_getImage16bitAs8bit(Image_handle hImage,
                            unsigned char *Buffer8bit,
                            int BufferLength);

//------------------------------------------------------------------------------
// 10 - Stream: Image characteristics
//------------------------------------------------------------------------------

  /** Get pixel type
   *  The pixel type as indicated by the camera will be returned
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the pixel type as indicated by the camera
   */
  __usrdllexport__ GVSP_PIXEL_TYPE
  Image_getPixelType(Image_handle hImage);

  /** Get image size.
   *  The number of bytes in the image data field will be returned.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the number of bytes in the image data field
   */
  __usrdllexport__ int
  Image_getImageSize(Image_handle hImage);

  /** Get image pitch
   *  The number of bytes in a row (pitch) will be returned as received from the camera
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the image's pitch as received from the camera
   */
	__usrdllexport__ int
  Image_getPitch(Image_handle hImage);

  /** Get image size X
   *  The horizontal pixel number will be returned as received from the camera
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the image's size X as received from the camera
   */
  __usrdllexport__ int
  Image_getSizeX(Image_handle hImage);

  /** Get image size Y
   *  The vertical pixel number will be returned as received from the camera
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the image's size Y as received from the camera
   */
	__usrdllexport__ int
  Image_getSizeY(Image_handle hImage);

//------------------------------------------------------------------------------
// 11 - Stream: Image statistics
//------------------------------------------------------------------------------

  /** Get image identifier.
   *
   *  An image identifier will be returned as it was assigned by the camera.
   *  Usually the camera will assign an increasing sequence of integer numbers
   *  to subsequent images which will wrap at some point and jump back to 1.
   *  The 0 identifier will not be used as it is defined in the GigE Vision
   *  specification
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return an integer number that is unique inside a certain sequence of numbers
   */
  __usrdllexport__ int
  Image_getImageID(Image_handle hImage);

  /** Get image timestamp
   *  The timestamp that was assigned to an image by the camera on image
   *  acquisition will be returned
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return a timestamp as it was received from the camera in ticks elapsed after last camera reboot
   */
  __usrdllexport__ double
  Image_getTimestamp(Image_handle hImage);

  /** Get image transfer time
   *  The time that elapsed from image's first network packet arriving on PC side
	 *  until image completion will be determined, including possible packet resends.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return image's transfer time as it was explained above
   */
  __usrdllexport__ double
  Image_getTransferTime(Image_handle hImage);

  /** Get packet count
   *  The number of packets that belong to a frame will be returned
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the pixel type as indicated by the camera
   */
	__usrdllexport__ int
  Image_getPacketCount(Image_handle hImage);

  /** Get packet resend
   *  The number of packets that have been resent will be reported
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the pixel type as indicated by the camera
   */
	__usrdllexport__ int
  Image_getPacketResend(Image_handle hImage);

//------------------------------------------------------------------------------
// 12 - Stream: Messaging channel
//------------------------------------------------------------------------------

  /** Create event.
   *  An event will be created inside GigE API which is capable of waiting for
   *  messages that are issued inside a streaming channel. One or more message
   *  types have to be added to the event before messages will be actually
   *  delivered to the application.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID a pointer to the identifier of a messaging channel
   *  @param SizeFIFO the number of entries in a message FIFO (0 = no FIFO)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_createEvent(StreamingChannel_handle hStreamingChannel,
                     Event_handle *EventID,
                     int SizeFIFO);

  /** Add message type.
   *  A message type will be added to a previously created messaging channel.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID the identifier of a messaging channel
   *  @param MessageType one of pre-defined message types
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_addMessageType(StreamingChannel_handle hStreamingChannel,
                        Event_handle EventID,
                        SVGigE_SIGNAL_TYPE MessageType);

  /** Remove message type.
   *  A message type will be removed from a previously created messaging channel.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID the identifier of a messaging channel
   *  @param MessageType one of pre-defined message types
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_removeMessageType(StreamingChannel_handle hStreamingChannel,
                           Event_handle EventID,
                           SVGigE_SIGNAL_TYPE MessageType);

  /** Is message pending.
   *  A messaging channel with a given EventID will be checked whether pending
   *  messages are available. The function will return a result immediately if
   *  the timeout is set to zero. Otherwise it will wait for a message atmost
   *  till the timeout elapses.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID the identifier of a messaging channel
   *  @param Timeout_ms a timeout value in milliseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_isMessagePending(StreamingChannel_handle hStreamingChannel,
                          Event_handle EventID,
                          int Timeout_ms);

  /** Register event callback.
   *  A callback function will be registered which will be called whenever an event is
   *  signalled in the messaging channel. One parameter of the callback is a Context
   *  which was registered along with the callback.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID the identifier of a messaging channel
   *  @param Callback the pointer of an EventCallback function
   *  @param Context an arbitrary value that will be returned with each call to EventCallback
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_registerEventCallback(StreamingChannel_handle hStreamingChannel,
                               Event_handle EventID,
                               EventCallback Callback,
                               void *Context);

  /** Unregister event callback.
   *  A previously registered callback function will be unregistered from message channel.
   *  This will effectively stop any further calls to that function.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID the identifier of a messaging channel
   *  @param Callback the pointer of an EventCallback function
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_unregisterEventCallback(StreamingChannel_handle hStreamingChannel,
                                 Event_handle EventID,
                                 EventCallback Callback);

  /** Get message.
   *  A subsequent MessageID will be retrieved for a previously received EventID.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID an ID of a messaging channel from EventCallback() or from isMessagePending()
   *  @param MessageID a pointer to a MessageID variable
   *  @param MessageType a pointer to a MessageType variable
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_getMessage(StreamingChannel_handle hStreamingChannel,
                    Event_handle EventID,
                    Message_handle *MessageID,
                    SVGigE_SIGNAL_TYPE *MessageType);

  /** Get message data.
   *  The data pointer and length will be retrieved for a previously received MessageID.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID the identifier of a messaging channel
   *  @param MessageID a message identifier received from getMessage()
   *  @param MessageData a pointer to a void* variable
   *  @param MessageLength a pointer to a data length variable
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_getMessageData(StreamingChannel_handle hStreamingChannel,
                        Event_handle EventID,
                        Message_handle MessageID,
                        void **MessageData,
                        int *MessageLength);

  /** Get message timestamp.
   *  A message's timestamp will be retrieved for a previously received MessageID.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID the identifier of a messaging channel
   *  @param MessageID a message identifier received from getMessage()
   *  @param MessageTimestamp a pointer to a timestamp variable
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_getMessageTimestamp(StreamingChannel_handle hStreamingChannel,
                             Event_handle EventID,
                             Message_handle MessageID,
                             double *MessageTimestamp);

  /** Release message.
   *  A previously received MessageID will be released. No further access must happen
   *  for the released MessageID since it will be removed from memory.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID the identifier of a messaging channel
   *  @param MessageID a message identifier received from getMessage()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_releaseMessage(StreamingChannel_handle hStreamingChannel,
                        Event_handle EventID,
                        Message_handle MessageID);

  /** Flush messages.
   *  All messages in the message FIFO will be removed.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID the identifier of a messaging channel
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_flushMessages(StreamingChannel_handle hStreamingChannel,
                       Event_handle EventID);

  /** Close event.
   *  The messaging channel with given EventID will be closed and all resources
   *  will be freed.
   *
   *  @see CameraContainer_getCamera()
   *  @param hStreamingChannel a handle to a valid streaming channel
   *  @param EventID the identifier of a messaging channel
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Stream_closeEvent(StreamingChannel_handle hStreamingChannel,
                    Event_handle EventID);


//------------------------------------------------------------------------------
// 13 - Controlling camera: Frame rate
//------------------------------------------------------------------------------

  /** Set frame rate.
   *  The camera will be adjusted to a new frame rate
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Framerate new frame rate in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setFrameRate(Camera_handle hCamera,
                      float Framerate);

  /** Get frame rate.
   *  The currently programmed frame rate will be retrieved
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Framerate currently programmed frame rate in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getFrameRate(Camera_handle hCamera,
                      float *Framerate);

  /** Get frame rate min.
   *  The minimal available frame rate will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MinFramerate the minimal frame rate in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getFrameRateMin(Camera_handle hCamera,
                         float *MinFramerate);

  /** Get frame rate max.
   *  The maximal available frame rate will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxFramerate the maximal frame rate in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getFrameRateMax(Camera_handle hCamera,
                         float *MaxFramerate);

  /** Get frame rate range.
   *  The currently available frame rate range will be retrieved
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MinFramerate currently available minimal frame rate in 1/s
   *  @param MaxFramerate currently available maximal frame rate in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getFrameRateRange(Camera_handle hCamera,
                           float *MinFramerate,
                           float *MaxFramerate);

  /** Get frame rate increment.
   *  The frame rate increment will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FramerateIncrement the frame rate increment in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getFrameRateIncrement(Camera_handle hCamera,
                               float *FramerateIncrement);

//------------------------------------------------------------------------------
// 14 - Controlling camera: Exposure
//------------------------------------------------------------------------------

  /** Set exposure time.
   *  The camera will be adjusted to a new exposure time
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureTime new exposure time in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setExposureTime(Camera_handle hCamera,
                         float ExposureTime);

  /** Get exposure time.
   *  The currently programmed exposure time will be retrieved
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureTime the currently programmed exposure time in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getExposureTime(Camera_handle hCamera,
                         float *ExposureTime);

  /** Get exposure time min.
   *  The minimal setting for exposure time will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MinExposureTime the minimal exposure time setting in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getExposureTimeMin(Camera_handle hCamera,
                            float *MinExposureTime);

  /** Get exposure time max.
   *  The maximal setting for exposure will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxExposureTime the maximal exposure time setting in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getExposureTimeMax(Camera_handle hCamera,
                            float *MaxExposureTime);

  /** Get exposure time range.
   *  The currently available exposure time range will be retrieved.
   *  NOTE: The received values will apply to free-running mode. In triggered mode the
   *        usual exposure time is limited to slightly more than 1 second. Exposure
   *        times above 1 second require changes in internal camera settings. Please
   *        contact SVS VISTEK if the camera needs to run in that exposure time range.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MinExposureTime currently available minimal exposure time in microseconds
   *  @param MaxExposureTime currently available maximal exposure time in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getExposureTimeRange(Camera_handle hCamera,
                              float *MinExposureTime,
                              float *MaxExposureTime);

  /** Get exposure time increment.
   *  The increment for exposure time will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureTimeIncrement the exposure time increment in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getExposureTimeIncrement(Camera_handle hCamera,
                            float *ExposureTimeIncrement);

  /** Set exposure delay.
   *  The camera's exposure delay in micro seconds relative to the trigger
   *  pulse will be set to the provided value. The delay will become active
   *  each time an active edge of an internal or external trigger pulse arrives
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureDelay the new value for exposure delay in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setExposureDelay(Camera_handle hCamera,
                           float ExposureDelay);

  /** Get exposure delay.
   *  The camera's current exposure delay will be returned in micro seconds
   *  relative to the trigger pulse
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureDelay the currently programmed value for exposure delay in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getExposureDelay(Camera_handle hCamera,
                          float *ExposureDelay);

  /** Get maximal exposure delay.
   *  The camera's maximal exposure delay will be returned in micro seconds
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxExposureDelay the maximal value for exposure delay in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getExposureDelayMax(Camera_handle hCamera,
                             float *MaxExposureDelay);

  /** Get exposure delay increment.
   *  The camera's exposure delay increment will be returned in micro seconds
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureDelayIncrement the exposure delay increment in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getExposureDelayIncrement(Camera_handle hCamera,
                                   float *ExposureDelayIncrement);

//------------------------------------------------------------------------------
// 15 - Controlling camera: Gain and offset
//------------------------------------------------------------------------------

  /** Set gain.
   *  The camera will be adjusted to a new analog gain
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Gain new analogue gain (0..18 dB)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setGain(Camera_handle hCamera,
                 float Gain);

  /** Get gain.
   *  The currently programmed analog gain will be retrieved
   *  Note: A gain of 1.0 is represented as integer 128 in the appropriate camera
   *  register. Thus the gain can be adjusted only in discrete steps.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Gain the currently programmed analog gain (0..18 dB)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getGain(Camera_handle hCamera,
                 float *Gain);

  /** Get maximal gain.
   *  The maximal analog gain will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxGain the maximal analog gain
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getGainMax(Camera_handle hCamera,
                    float *MaxGain);

  /** Get maximal extended gain.
   *  The maximal extended analog gain will be returned. An extended gain
   *  allows for operating a camera outside specified range. Noise and
   *  distortions will increase above those values that are met inside
   *  specified range.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxGainExtended the maximal analog gain
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getGainMaxExtended(Camera_handle hCamera,
                            float *MaxGainExtended);

  /** Get gain increment.
   *  The analog gain increment will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param GainIncrement the analog gain increment
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getGainIncrement(Camera_handle hCamera,
                          float *GainIncrement);

  /** Set offset
   *  The ofset value will be set to the provided value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Offset the new value for pixel offset (0..255)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setOffset(Camera_handle hCamera,
                   float Offset);

  /** Get offset
   *  The current offset value will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Offset the currently programmed value for pixel offset (0..255)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getOffset(Camera_handle hCamera,
                   float *Offset);

  /** Get maximal offset
   *  The maximal offset value will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxOffset the maximal value for pixel offset
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getOffsetMax(Camera_handle hCamera,
                      float *MaxOffset);

//------------------------------------------------------------------------------
// 16 - Controlling camera: Auto gain/exposure
//------------------------------------------------------------------------------

 /** Set auto gain enabled
	*  The auto gain status will be switched on or off.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param isAutoGainEnabled whether auto gain has to be enabled or disabled
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
  */
  __usrdllexport__ SVGigE_RETURN
	Camera_setAutoGainEnabled(Camera_handle Camera,
														bool isAutoGainEnabled);

 /** Get auto gain enabled
	*  Current auto gain status will be returned.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param isAutoGainEnabled whether auto gain is enabled or disabled
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
  */
  __usrdllexport__ SVGigE_RETURN
	Camera_getAutoGainEnabled(Camera_handle Camera,
														bool *isAutoGainEnabled);

 /** Set auto gain brightness
	*  The target brightness (0..255) will be set which the camera tries to
  *  reach automatically when auto gain/exposure is enabled. The range
  *  0..255 always applies independently from pixel depth.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param Brightness the target brightness for auto gain enabled
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
	*/
  __usrdllexport__ SVGigE_RETURN
	Camera_setAutoGainBrightness(Camera_handle Camera,
                               float Brightness);

 /** Get auto gain brightness
	*  The target brightness (0..255) will be returned that the camera tries
  *  to reach automatically when auto gain/exposure is enabled.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param Brightness the programmed brightness value
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
	*/
  __usrdllexport__ SVGigE_RETURN
	Camera_getAutoGainBrightness(Camera_handle Camera,
                               float *Brightness);

 /** Set auto gain dynamics
	*  AutoGain PID regulator's time constants for the I (integration) and
  *  D (differentiation) components will be set to new values.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param AutoGainParameterI the I parameter in a PID regulation loop
	*  @param AutoGainParameterD the D parameter in a PID regulation loop
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
	*/
  __usrdllexport__ SVGigE_RETURN
	Camera_setAutoGainDynamics(Camera_handle Camera,
                             float AutoGainParameterI,
                             float AutoGainParameterD);

 /** Get auto gain dynamics
	*  AutoGain PID regulator's time constants for the I (integration) and
  *  D (differentiation) components will be retrieved from the camera.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param AutoGainParameterI the programmed I parameter in a PID regulation loop
	*  @param AutoGainParameterD the programmed D parameter in a PID regulation loop
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
	*/
  __usrdllexport__ SVGigE_RETURN
	Camera_getAutoGainDynamics(Camera_handle Camera,
                             float *AutoGainParameterI,
                             float *AutoGainParameterD);

 /** Set auto gain limits
	*  The minimal and maximal gain will be determined that the camera
	*  must not exceed in auto gain/exposure mode.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param MinGain the minimal allowed gain value
	*  @param MaxGain the maximal allowed gain value
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
	*/
  __usrdllexport__ SVGigE_RETURN
	Camera_setAutoGainLimits(Camera_handle Camera,
                           float MinGain,
                           float MaxGain);

 /** Get auto gain limits
	*  The minimal and maximal gain will be returned that the camera
	*  must not exceed in auto gain/exposure mode.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param MinGain a pointer to the programmed minimal allowed gain value
	*  @param MaxGain a pointer to the programmed maximal allowed gain value
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
	*/
  __usrdllexport__ SVGigE_RETURN
	Camera_getAutoGainLimits(Camera_handle Camera,
                           float *MinGain,
                           float *MaxGain);

 /** Set auto exposure limits
	*  The minimal and maximal exposure will be determined that the camera
	*  must not exceed in auto gain/exposure mode.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param MinExposure the minimal allowed exposure value
	*  @param MaxExposure the maximal allowed exposure value
	*  @return SVGigE_SUCCESS or an appropriate SVGigE error code
	*/
  __usrdllexport__ SVGigE_RETURN
	Camera_setAutoExposureLimits(Camera_handle Camera,
                               float MinExposure,
                               float MaxExposure);

 /** Set auto exposure limits
  *  The minimal and maximal exposure will be determined that the camera
  *  must not exceed in auto gain/exposure mode.
  *
  *  @param Camera a handle from a camera that has been opened before
  *  @param MinExposure the programmed minimal allowed exposure value
  *  @param MaxExposure the programmed maximal allowed exposure value
  *  @return SVGigE_SUCCESS or an appropriate SVGigE error code
  */
  __usrdllexport__ SVGigE_RETURN
	Camera_getAutoExposureLimits(Camera_handle Camera,
                               float *MinExposure,
                               float *MaxExposure);

//------------------------------------------------------------------------------
// 17 - Controlling camera: Acquisition trigger
//------------------------------------------------------------------------------

  /** Set acquisition control.
   *  The camera's acquisition will be controlled (start/stop).
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AcquisitionControl the new setting for acquisition control
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setAcquisitionControl(Camera_handle hCamera,
                               ACQUISITION_CONTROL AcquisitionControl);

  /** Get acquisition control.
   *  The camera's current acquisition control (start/stop) will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AcquisitionControl the currently programmed setting for acquisition control
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getAcquisitionControl(Camera_handle hCamera,
                               ACQUISITION_CONTROL *AcquisitionControl);

  /** Set acquisition mode and control.
   *  The camera's acquisition mode will be set to the selected value
   *  It can be determined whether camera control should switch to START immediately.
   *  First function always forces a START whereas second function allows to choose
   *  whether acquisition should start immediately or if previous control state
   *  will be preserved.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AcquisitionMode the new setting for acquisition mode
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setAcquisitionMode(Camera_handle hCamera,
                            ACQUISITION_MODE AcquisitionMode);

  /** Set Acquisition mode and start
   *  In addition to setting the acquisition mode it can be determined whether
   *  acquisition control will go to enabled or stay on disabled after the new
   *  acquisition mode has been adjusted
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AcquisitionMode the new setting for acquisition mode
   *  @param AcquisitionStart whether camera control switches to START immediately
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setAcquisitionModeAndStart(Camera_handle hCamera,
                                    ACQUISITION_MODE AcquisitionMode,
                                    bool AcquisitionStart);

  /** Get acquisition mode.
   *  The camera's current acquisition mode will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AcquisitionMode the currently programmed setting for acquisition mode
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getAcquisitionMode(Camera_handle hCamera,
                            ACQUISITION_MODE *AcquisitionMode);

  /** Software trigger.
   *  The camera will be triggered for starting a new acquisition cycle.
   *  A mandatory pre-requisition for a successful software trigger is to have
   *  the camera set to ACQUISITION_MODE_SOFTWARE_TIGGER before.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_softwareTrigger(Camera_handle hCamera);

  /** Software trigger ID. (defined but not yet available)
   *  The camera will be triggered for starting a new acquisition cycle.
   *  A mandatory pre-requisition for a successful software trigger is to have
   *  the camera set to ACQUISITION_MODE_SOFTWARE_TIGGER before.
   *  In addition to a usual software trigger, an ID will be accepted that
   *  can be written into the image on demand, e.g. for maintaining a unique
   *  trigger/image reference
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TriggerID an ID to be transferred into first bytes of resulting image data
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_softwareTriggerID(Camera_handle hCamera,
                           int TriggerID);

  /** Software trigger ID enable. (defined but not yet available)
   *  The "software trigger ID" mode will be enabled respectively disabled
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TriggerIDEnable whether "trigger ID" will be enabled or disabled
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_softwareTriggerIDEnable(Camera_handle hCamera,
                                 bool TriggerIDEnable);

  /** Set trigger polarity
   *  The camera's trigger polarity will be set to the selected value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TriggerPolarity the new setting for trigger polarity
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setTriggerPolarity(Camera_handle hCamera,
                            TRIGGER_POLARITY TriggerPolarity);

  /** Get trigger polarity
   *  The camera's current trigger polarity will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TriggerPolarity the currently programmed setting for trigger polarity
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getTriggerPolarity(Camera_handle hCamera,
                            TRIGGER_POLARITY *TriggerPolarity);

   /** Set a new PIV mode
    *  The camera's PIV mode will be enabled or disabled.
    *
    *  @see CameraContainer_getCamera()
    *  @param hCamera a camera handle received from CameraContainer_getCamera()
    *  @param PivMode the new setting for PIV mode
    *  @return success or error code
    */
  __usrdllexport__ SVGigE_RETURN
  Camera_setPivMode(Camera_handle hCamera,
                    PIV_MODE  PivMode);

   /** Get PIV Mode
    *  Check if camera's PIV mode is enabled or disabled.
    *  The state of camera's current PivMode will be returned
    *
    *  @see CameraContainer_getCamera()
    *  @param hCamera a camera handle received from CameraContainer_getCamera()
    *  @param PivMode the currently programmed setting for PIV mode
    *  @return success or error code
    */
  __usrdllexport__ SVGigE_RETURN
  Camera_getPivMode(Camera_handle hCamera,
                    PIV_MODE *PivMode);

 //---------------- Debouncer----------------------

  /** Set debouncer  duration
   *   The camera's Debouncer duration will be set to the selected value
   *
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param DebouncerDuration the new setting for debouncer Duration in ticks
   *  one tick corresponds to (1/66666666) sec
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setDebouncerDuration(Camera_handle hCamera,
                              float  DebouncerDuration);

 /** Get debouncer  duration
  *  The camera's debouncer  duration will be returned
  *
  *  @see CameraContainer_getCamera()
  *  @param hCamera a camera handle received from CameraContainer_getCamera()
  *  @param DebouncerDuration the currently programmed debouncer duration in ticks
  *	 one tick corresponds to (1/66666666) sec
  *  @return success or error code
  */
  __usrdllexport__ SVGigE_RETURN
  Camera_getDebouncerDuration(Camera_handle hCamera,
                              float *DebouncerDuration);

  //-----------------prescaler-----------------------

  /** Set prescaler devisor
   *   The camera's prescaler Devisor will be set to the selected value
   *
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PrescalerDevisor the new setting for PrescalerDevisor
   *  @return success or error code
   */
   __usrdllexport__ SVGigE_RETURN
   Camera_setPrescalerDevisor(Camera_handle hCamera,
                             unsigned int PrescalerDevisor);

 /** Get prescaler devisor
  *  The camera's prescaler devisor will be returned
  *
  *  @see CameraContainer_getCamera()
  *  @param hCamera a camera handle received from CameraContainer_getCamera()
  *  @param PrescalerDevisor the currently programmed setting for prescaler devisor.
  *  @return success or error code
  */
  __usrdllexport__ SVGigE_RETURN
  Camera_getPrescalerDevisor(Camera_handle hCamera,
                             unsigned int *PrescalerDevisor);

  //-----------------Sequencer-----------------------

  /** load Sequence parameters
   *  New sequence parameters will be loaded from a XML file into the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Filename a complete path and file name where to load the settings from
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_loadSequenceParameters(Camera_handle hCamera,
                                const char *Filename);

  /** Start Sequencer
   * Start acquisition using sequencer.
   * This will occur after loading the appropriate sequence parameters.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_startSequencer(Camera_handle hCamera);

//------------------------------------------------------------------------------
// 18 - Controlling camera: Strobe
//------------------------------------------------------------------------------

  /** Set strobe polarity
   *  The camera's strobe polarity will be set to the selected value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePolarity the new setting for strobe polarity
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setStrobePolarity(Camera_handle hCamera,
                           STROBE_POLARITY StrobePolarity);

  /** Set strobe polarity extended
   *  The camera's strobe polarity will be set to the selected value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePolarity the new setting for strobe polarity
   *  @param StrobIndex  the index of the current strobe channel. it can be 1,2,3 or 4.
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
   Camera_setStrobePolarityExtended(Camera_handle hCamera,
                                    STROBE_POLARITY StrobePolarity, int StrobIndex);

  /** Get strobe polarity
   *  The camera's current strobe polarity will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePolarity the currently programmed setting for strobe polarity
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getStrobePolarity(Camera_handle hCamera,
                           STROBE_POLARITY *StrobePolarity);

   /** Get strobe polarity extended
   *  The camera's current strobe polarity will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePolarity the currently programmed setting for strobe polarity
   *  @param StrobIndex the index of the current strobe channel, it can take a value 1,2,3 or 4.
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getStrobePolarityExtended(Camera_handle hCamera,
                                   STROBE_POLARITY *StrobePolarity, int StrobIndex);

  /** Set strobe position
   *  The camera's strobe position in micro seconds relative to the trigger
   *  pulse will be set to the selected value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePosition the new value for strobe position in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setStrobePosition(Camera_handle hCamera,
                           float StrobePosition);

  /** Set strobe position extended
   *  The camera's strobe position in micro seconds relative to the trigger
   *  pulse will be set to the selected value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePosition the new value for strobe position in microseconds
   *  @param StrobIndex the index of the current strobe channel, it can take a value 1,2,3 or 4.
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setStrobePositionExtended(Camera_handle hCamera,
                                   float StrobePosition,
                                   int StrobIndex);

  /** Get strobe position
   *  The camera's current strobe position will be returned in micro seconds
   *  relative to the trigger pulse
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePosition the currently programmed value for strobe position in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getStrobePosition(Camera_handle hCamera,
                           float *StrobePosition);

  /** Get strobe position extended
   *  The camera's current strobe position will be returned in micro seconds
   *  relative to the trigger pulse
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePosition the currently programmed value for strobe position in microseconds
   *  @param StrobIndex the index of the current strobe channel it can take a value 1,2,3 or 4.
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getStrobePositionExtended(Camera_handle hCamera,
                                   float *StrobePosition,
                                   int StrobIndex);

  /** Get maximal strobe position
   *  The camera's maximal strobe position (delay) will be returned in micro seconds
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxStrobePosition the maximal value for strobe position in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getStrobePositionMax(Camera_handle hCamera,
                           float *MaxStrobePosition);

  /** Get strobe position increment
   *  The camera's strobe position increment will be returned in micro seconds
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePositionIncrement the strobe position increment in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getStrobePositionIncrement(Camera_handle hCamera,
                                    float *StrobePositionIncrement);

  /** Set strobe duration
   *  The camera's strobe duration in micro seconds will be set to the selected
   *  value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobeDuration the new value for strobe duration in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setStrobeDuration(Camera_handle hCamera,
                           float StrobeDuration);

  /** Set strobe duration extended
   *  The camera's strobe duration in micro seconds will be set to the selected
   *  value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobeDuration the new value for strobe duration in microseconds
   *  @param StrobIndex the index of the current strobe channel it can take a value 1,2,3 or 4.
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setStrobeDurationExtended(Camera_handle hCamera,
                                   float StrobeDuration,
                                   int StrobIndex);

  /** Get strobe duration
   *  The camera's current strobe duration in micro seconds will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobeDuration the currently programmed value for strobe duration in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getStrobeDuration(Camera_handle hCamera,
                           float *StrobeDuration);

  /** Get strobe duration extended
   *  The camera's current strobe duration in micro seconds will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobeDuration the currently programmed value for strobe duration in microseconds
   *  @param StrobIndex the index of the current strobe channel it can take a value 1,2,3 or 4.
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getStrobeDurationExtended(Camera_handle hCamera,
                                   float *StrobeDuration,
                                   int StrobIndex);

  /** Get maximal strobe duration
   *  The camera's maximal strobe duration in micro seconds will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxStrobeDuration the maximal value for strobe duration in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getStrobeDurationMax(Camera_handle hCamera,
                              float *MaxStrobeDuration);

  /** Get strobe duration increment
   *  The camera's strobe duration increment in micro seconds will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobeDurationIncrement the strobe duration increment in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getStrobeDurationIncrement(Camera_handle hCamera,
                                    float *StrobeDurationIncrement);

//------------------------------------------------------------------------------
// 19 - Controlling camera: Tap balance
//------------------------------------------------------------------------------



  /** Set tap configuration
   *  The camera will be controlled for working with one or two taps
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TapCount the number of taps (1, 2) to be used by the camera
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setTapConfiguration(Camera_handle hCamera,
                             int TapCount);

  /** Get tap configuration
   *  The camera will be queried whether it is working with one or two taps
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TapCount the number of taps (1, 2) currently used by the camera is returned
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getTapConfiguration(Camera_handle hCamera,
                             int *TapCount);

	/**
	 *	Set tap Configuration extended
	 *  The camera will be controlled for working with one of the following tap configurations:
	 *  SVGIGE_SELECT_SINGLE_TAP
	 *	SVGIGE_SELECT_DUAL_TAP_H
	 *	SVGIGE_SELECT_DUAL_TAP_V
	 *	SVGIGE_SELECT_QUAD
	 *
	 *  @see CameraContainer_getCamera()
     *  @param hCamera a camera handle received from CameraContainer_getCamera()
	 *  @param SelectedTapConfig the selected tap configuration to be used by the camera
     *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
  Camera_setTapConfigurationEx(Camera_handle hCamera,
                             SVGIGE_TAP_CONFIGURATION_SELECT SelectedTapConfig);

	/**
	 *  getTapConfiguration
	 *  The camera will be queried whether it is working with one of the following configurations:
     *  SVGIGE_SELECT_SINGLE_TAP
	 *	SVGIGE_SELECT_DUAL_TAP_H
	 *	SVGIGE_SELECT_DUAL_TAP_V
	 *	SVGIGE_SELECT_QUAD
	 *
     *  @see CameraContainer_getCamera()
     *  @param hCamera a camera handle received from CameraContainer_getCamera()
	 *  @param TapConfig the tap configuration currently used by the camera is returned
     *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
  Camera_getTapConfigurationEx(Camera_handle hCamera,
                             SVGIGE_TAP_CONFIGURATION_SELECT *TapConfig);


  /** Set auto tap balance mode
   *  One of the modes (OFF,ONCE,CONTINUOUS,RESET) will be applied for an auto
   *  tap balance procedure.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AutoTapBalanceMode the mode for auto tap balancing to be activated
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setAutoTapBalanceMode(Camera_handle hCamera,
                               SVGIGE_AUTO_TAP_BALANCE_MODE AutoTapBalanceMode);

  /** Get auto tap balance mode
   *  Currently adjusted auto tap balance mode will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AutoTapBalanceMode a pointer to a value for auto tap balancing
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getAutoTapBalanceMode(Camera_handle hCamera,
                               SVGIGE_AUTO_TAP_BALANCE_MODE *AutoTapBalanceMode);



  /** Set tap gain
   *  A provided tap gain in [dB] will be transferred to camera.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()

   *  @param TapGain  one of the defined tap selectors
   *  @param TapSelect the new value for tap gain
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setTapGain(Camera_handle hCamera,
										float TapGain,
                    SVGIGE_TAP_SELECT TapSelect);

  /** Get tap gain
   *  Currently adjusted tap gain in [dB] will be queried from camera.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TapSelect one of the defined tap selectors
   *  @param TapGain the new value for tap gain
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getTapGain(Camera_handle hCamera,
										float *TapGain,
                    SVGIGE_TAP_SELECT TapSelect);

//------------------------------------------------------------------------------
// 20 - Controlling camera: Image parameter
//------------------------------------------------------------------------------

  /** Get imager width.
   *  The imager width will be retrieved from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImagerWidth a reference to the imager width value
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getImagerWidth(Camera_handle hCamera,
                        int *ImagerWidth);

  /** Get imager height.
   *  The imager height will be retrieved from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImagerHeight a reference to the imager height value
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getImagerHeight(Camera_handle hCamera,
                         int *ImagerHeight);

  /** Get image size.
   *  The number of bytes in an image will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImageSize a reference to the image size value
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getImageSize(Camera_handle hCamera,
                      int *ImageSize);

  /** Get pitch.
   *  The number of bytes in a row will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Pitch a reference to the pitch value
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getPitch(Camera_handle hCamera,
                  int *Pitch);

  /** Get size X.
   *  The currently used horizontal picture size X will be retrieved from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SizeX a reference to the size X value
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getSizeX(Camera_handle hCamera,
                  int *SizeX);

  /** Get size Y.
   *  The currently used vertical picture size Y will be retrieved from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SizeY a reference to the size Y value
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getSizeY(Camera_handle hCamera,
                  int *SizeY);

  /** Set binning mode.
   *  The camera's binning mode will be set to the selected value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BinningMode the new setting for binning mode
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setBinningMode(Camera_handle hCamera,
                        BINNING_MODE BinningMode);

  /** Get binning mode.
   *  The camera's current binning mode will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BinningMode the currently programmed setting for binning mode
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getBinningMode(Camera_handle hCamera,
                        BINNING_MODE *BinningMode);

  /** Set area of interest (AOI)
   *  The camera will be switched to partial scan mode and an AOI will be set
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SizeX the number of pixels in one row
   *  @param SizeY the number of scan lines
   *  @param OffsetX a left side offset of the scanned area
   *  @param OffsetY an upper side offset of the scanned area
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setAreaOfInterest(Camera_handle hCamera,
                           int SizeX,
                           int SizeY,
                           int OffsetX,
                           int OffsetY);

  /** Get area of interest(AOI)
   *  The currently set parameters for partial scan will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SizeX the programmed number of pixels in one row
   *  @param SizeY the programmed number of scan lines
   *  @param OffsetX a programmed left side offset of the scanned area
   *  @param OffsetY an programmed upper side offset of the scanned area
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getAreaOfInterest(Camera_handle hCamera,
                           int *SizeX,
                           int *SizeY,
                           int *OffsetX,
                           int *OffsetY);

  /** Get minimal/maximal area of interest(AOI).
   *  The boundaries for partial scan will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MinSizeX the minimal AOI width
   *  @param MinSizeY the minimal AOI height
   *  @param MaxSizeX the maximal AOI width
   *  @param MaxSizeY the maximal AOI height
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getAreaOfInterestRange(Camera_handle hCamera,
                                int *MinSizeX,
                                int *MinSizeY,
                                int *MaxSizeX,
                                int *MaxSizeY);

  /** Get area of interest(AOI) increment
   *  The increment for partial scan parameters will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SizeXIncrement the increment for AOI width
   *  @param SizeYIncrement the increment for AOI height
   *  @param OffsetXIncrement the increment for AOI width offset
   *  @param OffsetYIncrement the increment for AOI height offset
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getAreaOfInterestIncrement(Camera_handle hCamera,
                                    int *SizeXIncrement,
                                    int *SizeYIncrement,
                                    int *OffsetXIncrement,
                                    int *OffsetYIncrement);

  /** Reset timestamp counter
   *  The camera's timestamp counter will be set to zero.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_resetTimestampCounter(Camera_handle hCamera);

  /** Get timestamp counter
   *  Current value of the camera's timestamp counter will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TimestampCounter the current value of the timestamp counter
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getTimestampCounter(Camera_handle hCamera,
                             double *TimestampCounter);

  /** Get timestamp tick frequency
   *  A camera's timestamp tick frequency will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TimestampTickFrequency the camera's timestamp tick frequency
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getTimestampTickFrequency(Camera_handle hCamera,
                                   double *TimestampTickFrequency);



//-----------------------------------------------------------------------------------------------------

  /** Set Flipping mode.
   *   The camera will be controlled for working with the following flipping mode if selected:
   *   REVERSE_OFF (without flipping)
   *   REVERSE_X (vertical flipping)
   *   REVERSE_Y (horizontal flipping)
   *   REVERSE_X_Y( horizontal and vertical flipping)
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FlippingMode the new setting for flipping mode
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
  Camera_setFlippingMode(Camera_handle hCamera,
                        SVGIGE_FLIPPING_MODE  FlippingMode);

  /** Get Flipping mode.
   *  The camera will be queried whether it is working with one of the following flipping mode:
   *   REVERSE_OFF (without flipping)
   *   REVERSE_X (vertical flipping)
   *   REVERSE_Y (horizontal flipping)
   *   REVERSE_X_Y( horizontal and vertical flipping)
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FlippingMode  the currently programmed setting for flipping mode
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getFlippingMode(Camera_handle hCamera,
                        SVGIGE_FLIPPING_MODE *FlippingMode);


//-----------------------------------------------------------------------------------------------------


  /** Set Shutter mode.
   *  The camera will be controlled for working with the following shutter mode if selected:
   *  GLOBAL_SHUTTER
   *  ROLLING_SHUTTER
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ShutterMode the new setting for shutter mode
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
  Camera_setShutterMode(Camera_handle hCamera,
                        SVGIGE_SHUTTER_MODE  ShutterMode);

  /** Get Shutter mode.
   *  The camera will be queried whether it is working with one of the following shutter mode:
   *  GLOBAL_SHUTTER
   *  ROLLING_SHUTTER
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ShutterMode  the currently programmed setting for shutter mode
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getShutterMode(Camera_handle hCamera,
                        SVGIGE_SHUTTER_MODE *ShutterMode);


//------------------------------------------------------------------------------
// 21 - Controlling camera: Image appearance
//------------------------------------------------------------------------------

  /** Get pixel type.
   *  The pixel type will be retrieved from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelType a reference to the pixel type value
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getPixelType(Camera_handle hCamera,
                      GVSP_PIXEL_TYPE *PixelType);

  /** Set pixel depth.
   *  The number of bits for a pixel will be set to 8, 12 or 16 bits. Before this function
   *  is called the camera's feature vector should be queried whether the desired pixel depth
   *  is supported
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelDepth the intended value for pixel depth
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setPixelDepth(Camera_handle hCamera,
                       SVGIGE_PIXEL_DEPTH PixelDepth);

  /** Get pixel depth.
   *  The camera's current setting for pixel depth will be queried.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelDepth an enum for the number of bits in a pixel will be returned
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getPixelDepth(Camera_handle hCamera,
                       SVGIGE_PIXEL_DEPTH *PixelDepth);

  /** setWhiteBalance.
   *  The provided values will be applied for white balance. Color strengths can be
	 *  given with any reference value, e.g. 100 or 1.0 or others. Thus RGB=100/110/130
	 *  is the same as RGB=1.0/1.1/1.3
   *
   *  NOTE: The color component strength for Red, Green and Blue can either be
   *        provided by user or they can conveniently be calculated inside an image
   *        callback using the Image_estimateWhiteBalance() function.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Red balanced value for red color
   *  @param Green balanced value for green color
   *  @param Blue balanced value for blue color
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setWhiteBalance(Camera_handle hCamera,
                         float Red,
                         float Green ,
                         float Blue);

  /** getWhiteBalance.
   *  Currently set values for white balance will be returned.
   *  Previously adjusted values will be returned either unchanged or adjusted
   *  if necessary. The returned values will be 100 and above where the color
   *  which got 100 assigned will be transferred unchanged, however two
   *  other color components might be enhanced above 100 for each image.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Red balanced value for red color
   *  @param Green balanced value for green color
   *  @param Blue balanced value for blue color
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getWhiteBalance(Camera_handle hCamera,
                         float *Red,
                         float *Green ,
                         float *Blue);

  /** getWhiteBalanceMax.
   *  The maximal white-balance value for amplifying colors will be returned.
   *  A value of 1.0 is the reference for a balanced situation.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param WhiteBalanceMax the maximal white-balance (e.g. 4.0 or 2.0)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getWhiteBalanceMax(Camera_handle hCamera,
                            float *WhiteBalanceMax);

  /** setGammaCorrection.
   *  A lookup table will be generated based on given gamma correction.
   *  Subsequently the lookup table will be uploaded to the camera.
   *  A gamma correction is supported in a range 0.4 - 2.5
   *
   *  @param Camera a handle from a camera that has been opened before
   *  @param GammaCorrection a gamma correction factor
   *  @return SVGigE_SUCCESS or an appropriate SVGigE error code
   */
  __usrdllexport__ SVGigE_RETURN
	Camera_setGammaCorrection(Camera_handle Camera,
												    float GammaCorrection);

  /** setGammaCorrectionExt.
   *  A lookup table will be generated based on given gamma correction.
   *  Additionally, a digital gain and offset will be taken into account.
   *  Subsequently the lookup table will be uploaded to the camera.
   *  A gamma correction is supported in a range 0.4 - 2.5
   *
   *  @param Camera a handle from a camera that has been opened before
   *  @param GammaCorrection a gamma correction factor
   *  @param DigitalGain a digital gain used
   *  @param DigitalOffset a digital offset used
   *  @return SVGigE_SUCCESS or an appropriate SVGigE error code
   */
  __usrdllexport__ SVGigE_RETURN
	Camera_setGammaCorrectionExt(Camera_handle Camera,
												       float GammaCorrection,
                               float DigitalGain,
                               float DigitalOffset);

  /** setLowpassFilter.
    *  A filter will be enabled/disabled which smoothes an image inside
    *  a camera accordingly to a given algorithm, e.g. 3x3.
    *
    *  @param Camera a handle from a camera that has been opened before
    *  @param LowpassFilter a control value for activating/deactivating smoothing
    *  @return SVGigE_SUCCESS or an appropriate SVGigE error code
    */
  __usrdllexport__ SVGigE_RETURN
  Camera_setLowpassFilter(Camera_handle Camera,
                          LOWPASS_FILTER LowpassFilter);

  /** getLowpassFilter.
    *  Current mode of a low pass filter will be retrieved from camera.
    *
    *  @param Camera a handle from a camera that has been opened before
    *  @param LowpassFilter the currently programmed low pass filter will be returned
    *  @return SVGigE_SUCCESS or an appropriate SVGigE error code
    */
  __usrdllexport__ SVGigE_RETURN
  Camera_getLowpassFilter(Camera_handle Camera,
                          LOWPASS_FILTER *LowpassFilter);

  /** setLookupTableMode.
   *  The look-up table mode will be switched on or off
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LUTMode new setting for look-up table mode
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setLookupTableMode(Camera_handle hCamera,
                            LUT_MODE LUTMode);

  /** Get look-up table mode.
   *  The currently programmed look-up table mode will be retrieved
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LUTMode currently programmed look-up table mode
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getLookupTableMode(Camera_handle hCamera,
                            LUT_MODE *LUTMode);

  /** setLookupTable.
   *  A user-defined lookup table will be uploaded to the camera. The size has to match
   *  the lookup table size that is supported by the camera (1024 for 10to8 or 4096 for 12to8).
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LookupTable an array of user-defined lookup table values (bytes)
   *  @param LookupTableSize the size of the user-defined lookup table
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setLookupTable(Camera_handle hCamera,
                        unsigned char *LookupTable,
                        int LookupTableSize);

  /** getLookupTable.
   *  The currently installed lookup table will be downloaded from the camera. The size of the
   *  reserved download space has to match the lookup table size (1024 for 10to8 or 4096 for 12to8).
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LookupTable an array for downloading the lookup table from camera
   *  @param LookupTableSize the size of the provided lookup table space
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getLookupTable(Camera_handle hCamera,
                        unsigned char *LookupTable,
                        int LookupTableSize);

  /** startImageCorrection.
   *  A particular step inside of acquiring a correction image for either,
	 *  flat field correction (FFC) or shading correction will be started:
	 *    - acquire a black image
	 *    - acquire a white image
	 *    - save a correction image to camera's persistent memory
	 *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImageCorrectionStep a particular step from running an image correction
   *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
  Camera_startImageCorrection(Camera_handle hCamera,
                              IMAGE_CORRECTION_STEP ImageCorrectionStep);

  /** isIdleImageCorrection.
   *  A launched image correction process will be checked whether a recently
	 *  initiated image correction step has be finished:
	 *    - acquire a black image
	 *    - acquire a white image
	 *    - save a correction image to camera's persistent memory
	 *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImageCorrectionStep a programmed particular step from running an image correction
   *  @param isIdle  whether a recently initiated image correction is Idle
   *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
  Camera_isIdleImageCorrection(Camera_handle hCamera,
															 IMAGE_CORRECTION_STEP *ImageCorrectionStep,
				 											 bool *isIdle);

  /** setImageCorrection.
   *  A camera will be switched to one of the following image correction modes
	 *    - None (image correction is off)
	 *    - Offset only (available for Flat Field Correction, FFC)
	 *    - Enabled (image correction is on)
	 *  If image correction is enabled, then it depends on the camera whether
	 *  Flat Field Correction (FFC) is enabled (gain and offset for each pixel)
	 *  or Shading Correction (gain interpolation for a group of adjacent pixels)
	 *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImageCorrectionMode one of above image correction modes
   *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
  Camera_setImageCorrection(Camera_handle hCamera,
														IMAGE_CORRECTION_MODE ImageCorrectionMode);

  /** getImageCorrection.
   *  A camera will be queried for current image correction mode, either of:
	 *    - None (image correction is off)
	 *    - Offset only (available for Flat Field Correction, FFC)
	 *    - Enabled (image correction is on)
	 *  If image correction is enabled, then it depends on the camera whether
	 *  Flat Field Correction (FFC) is enabled (gain and offset for each pixel)
	 *  or Shading Correction (gain interpolation for a group of adjacent pixels)
	 *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImageCorrectionMode currently programmed correction modes
   *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
  Camera_getImageCorrection(Camera_handle hCamera,
														IMAGE_CORRECTION_MODE *ImageCorrectionMode);
  /** Set pixels correction Map
   * A camera will be switched to one of the following pixels correction maps if selected:
	  *    - factory map
	  *    - SVS map
	  *    - custom map
      * the pixels correction control for the currently selected map is enabled per default.
	  *
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelsCorrectionMap one of above pixels correction maps
   *  @return success or error code
	 */
  __usrdllexport__ SVGigE_RETURN
  Camera_setPixelsCorrectionMap(Camera_handle hCamera,
								  PIXELS_CORRECTION_MAP_SELECT PixelsCorrectionMap);

  /** Get pixels correction map
   *  A camera will be queried for current selected pixels correction map
   *
   *  @see Camera_isCameraFeature()
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelsCorrectionMap a currently selected map
   *  @return success or error code
	 */
   __usrdllexport__ SVGigE_RETURN
  Camera_getPixelsCorrectionMap(Camera_handle hCamera,
								PIXELS_CORRECTION_MAP_SELECT *PixelsCorrectionMap);

    /** Set Pixels Correction Control enable.
   *  The Pixels Correction Control status will be switched on or off.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isPixelsCorrectionEnabled whether Pixels Correction control has to be enabled or disabled
   *  @return success or error code
	 */
 __usrdllexport__ SVGigE_RETURN
  Camera_setPixelsCorrectionControlEnabel(Camera_handle hCamera,
									bool isPixelsCorrectionEnabled);

  /** Get pixels correction control enabel.
   *  A camera will be queried for current pixels correction control status.
   *  the pixels correction can be enabled or disabled.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isPixelsCorrectionEnabled the programmed status of the pixels correction control.
   *  @return success or error code
	 */
 __usrdllexport__ SVGigE_RETURN
  Camera_getPixelsCorrectionControlEnabel(Camera_handle hCamera,
									bool *isPixelsCorrectionEnabled);

  /** Set Pixels Correction Control mark.
   *  The defect pixels will be marked or not.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isPixelsCorrectionMarked whether defect pixels have to be marked or not
   *  @return success or error code
	 */
 __usrdllexport__ SVGigE_RETURN
  Camera_setPixelsCorrectionControlMark(Camera_handle hCamera,
									bool isPixelsCorrectionMarked);

  /** Get pixels correction control mark.
   *  A camera will be queried for current pixels correction control status.
   *  the defect pixels can be marked or not.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isPixelsCorrectionMarked the programmed mark status of the pixels correction control
   *  @return success or error code
	 */
 __usrdllexport__ SVGigE_RETURN
  Camera_getPixelsCorrectionControlMark(Camera_handle hCamera,
									bool *isPixelsCorrectionMarked);


  /** set pixels correction map offset
   *  The offset x and y of the currently slected map will be set to the provided values
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param OffsetX the new value for pixel offset in x-axis
   *  @param OffsetY the new value for pixel offset in y-axis
   *  @return success or error code
   */

  __usrdllexport__ SVGigE_RETURN
 Camera_setPixelsCorrectionMapOffset(Camera_handle hCamera,
										  int  OffsetX, int  OffsetY);

  /** Get pixels correction map offset
   *  The offset x and y values of the currently slected map will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param OffsetX the currently programmed value for pixel offset in x-axis
   *  @param OffsetY the currently programmed value for pixel offset in Y-axis
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
 Camera_getPixelsCorrectionMapOffset(Camera_handle hCamera,
									int *OffsetX, int *OffsetY);


  /** Get pixels correction map size
   *  The currently coordinates number of defect pixels in selected map will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MapSize the currently programmed number of pixels coordinates in selected map
   *  @return success or error code
   */
   __usrdllexport__ SVGigE_RETURN
  Camera_getPixelsCorrectionMapSize(Camera_handle hCamera,
									 unsigned int *MapSize);

  /** Get maximal pixels correction map size
   *  The Maximal pixels coordinates number per map will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaximalMapSize the Maximal programmed number of defect pixels coordinates per map
   *  @return success or error code
   */
    __usrdllexport__ SVGigE_RETURN
  Camera_getMaximalPixelsCorrectionMapSize(Camera_handle hCamera,
											 unsigned int *MaximalMapSize);


  /** Set map index coordinate
   *  write a new X and Y coordinate accordingly to a map index that was specified
   *  as an input parameter
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MapIndex the map index from zero to one less the Map size
   *  @param X_Coordinate the new X coordinate to be written
   *  @param Y_Coordinate the new Y coordinate to be written
   *  @return success or error code
   */
   __usrdllexport__ SVGigE_RETURN
  Camera_setMapIndexCoordinate(Camera_handle hCamera,
									unsigned int MapIndex,
									unsigned int X_Coordinate, unsigned int Y_Coordinate );


	/** Get map index coordinate
   *  get X and Y coordinate accordingly to a map index that was specified
   *  as an input parameter
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MapIndex the map index from zero to one less the map size
   *  @param CoordinateX the programmed X coordinate accordingly to a map index
   *  @param CoordinateY the programmed Y coordinate accordingly to a map index
   *  @return success or error code
   */
   __usrdllexport__ SVGigE_RETURN
  Camera_getMapIndexCoordinate(Camera_handle hCamera,
									unsigned int MapIndex,
									unsigned int *CoordinateX, unsigned int *CoordinateY );
  /** Delete Pixel coordinate from map
   *  delete Pixel Coordinate accordingly to a map index that was specified
   *  as an input parameter
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MapIndex the map index from zero to one less the map size
   *  @return success or error code
   */

   __usrdllexport__ SVGigE_RETURN
  Camera_deletePixelCoordinateFromMap(Camera_handle hCamera, unsigned int MapIndex);


//------------------------------------------------------------------------------
// 22 - Special control: IOMux configuration
//------------------------------------------------------------------------------

  /** getMaxIOMuxIN.
   *  The maximal number of IN signals (signal sources) to the multiplexer will
   *  be returned that are currently available in the camera for connecting them
   *  to the multiplexer's OUT signals.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxIOMuxINSignals the currently supported number of IN signals (signal sources)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getMaxIOMuxIN(Camera_handle hCamera,
                       int *MaxIOMuxINSignals);

  /** getMaxIOMuxOut.
   *  The maximal number of OUT signals (signal sinks) will be returned that
   *  are currently activated in the camera's IO multiplexer.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxIOMuxOUTSignals the currently supported number of OUT signals (signal sinks)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getMaxIOMuxOUT(Camera_handle hCamera,
                        int *MaxIOMuxOUTSignals);

  /** setIOAssignment.
   *  An OUT signal (signal sink) will get one or multiple IN signals (signal
   *  sources) assigned in a camera's multiplexer. In case of multiple signal
   *  sources (IN signals) those signals will be or'd for combining them to
   *  one 32-bit value that will subsequently be assigned to an OUT signal.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param IOMuxOUT the multiplexer's OUT signal (signal sink) to be configured
   *  @param SignalIOMuxIN the IN signal vector (signal sources) to be activated
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setIOAssignment(Camera_handle hCamera,
                         SVGigE_IOMux_OUT IOMuxOUT,
                         unsigned int SignalIOMuxIN);

  /** getIOAssignment.
   *  Current assignment of IN signals (signal sources) to an OUT signal
   *  (signal sink) will be retrieved from a camera's multiplexer.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param IOMuxOUT the multiplexer's OUT signal (signal sink) to be queried
   *  @param ProgrammedIOMuxIN the IN signal vector (signal sources) connected to the OUT signal
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getIOAssignment(Camera_handle hCamera,
                         SVGigE_IOMux_OUT IOMuxOUT,
                         unsigned int *ProgrammedIOMuxIN);

//------------------------------------------------------------------------------
// 23 - Special control: IO control
//------------------------------------------------------------------------------

  /** setIOMuxIN.
   *  The complete vector of IN signals (source signals, max 32 bits) will be
   *  set in a camera's multiplexer in one go.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param VectorIOMuxIN the IN signal vector's new state to be assigned
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setIOMuxIN(Camera_handle hCamera,
                    unsigned int VectorIOMuxIN);

  /** getIOMuxIN.
   *  The complete vector of IN signals (source signals, max 32 bits) will be
   *  read out from a camera's multiplexer in one go.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param VectorIOMuxIN the IN signal vector's current state
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getIOMuxIN(Camera_handle hCamera,
                    unsigned int *VectorIOMuxIN);

  /** setIO.
   *  A single IN signal (source signal, one out of max 32 bits) will be set.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SignalIOMuxIN a particular signal from the IN signal vector
   *  @param SignalValue the signal value to be assigned to the IN signal
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setIO(Camera_handle hCamera,
               SVGigE_IOMux_IN SignalIOMuxIN,
               IO_SIGNAL SignalValue);

  /** getIO.
   *  A single IN signal (source signal, one out of max 32 bits) will be read.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SignalIOMuxIN a particular signal from the IN signal vector
   *  @param SignalValue  the current value of the selected IN signal
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getIO(Camera_handle hCamera,
               SVGigE_IOMux_IN SignalIOMuxIN,
               IO_SIGNAL *SignalValue);

  /** setAcqLEDOverride.
   *  Override default LED mode by an alternative behavior:
   *  - blue:    waiting for trigger
   *  - cyan:    exposure
   *  - magenta: read-out
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isOverrideActive whether LED override will be activated or deactivated
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setAcqLEDOverride(Camera_handle hCamera,
                           bool isOverrideActive);

  /** getAcqLEDOverride.
   *  Check whether default LED mode was overridden by an alternative behavior:
   *  - blue:    waiting for trigger
   *  - cyan:    exposure
   *  - magenta: read-out
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isOverrideActive a flag indicating whether LED override is currently activated
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getAcqLEDOverride(Camera_handle hCamera,
                           bool *isOverrideActive);

  /** setLEDIntensity.
   *  The LED intensity will be controlled in the range 0..255 as follows:
   *  0   - dark
   *  255 - light
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LEDIntensity the new intensity (0..255=max) to be assigned to the LED
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setLEDIntensity(Camera_handle hCamera,
                         int LEDIntensity);

  /** getLEDIntensity.
   *  The LED intensity will be retrieved from camera with the following meaning:
   *  0   - dark
   *  255 - light
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LEDIntensity currently assigned LED intensity
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getLEDIntensity(Camera_handle hCamera,
                         int *LEDIntensity);

//------------------------------------------------------------------------------
// 24 - Special control: Serial communication
//------------------------------------------------------------------------------

  /** setUARTBuffer.
   *  A block of data (max 512 bytes) will be sent to the camera's UART for
   *  transmitting it over the serial line to a receiver.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Data a pointer to a block of data to be sent over the camera's UART
   *  @param DataLength the length of the data block (1..512)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setUARTBuffer(Camera_handle hCamera,
                       unsigned char *Data,
                       int DataLength);

  /** getUARTBuffer.
   *  A block of data will be retrieved which has arrived in the camera's UART
   *  receiver buffer. If this function returns the maximal possible byte count
   *  of the serial buffer (=512 bytes) then there might be more data available
   *  which should be retrieved by a subsequent call to this function.
   *
   *  NOTE: If DataLengthMax is set to less than the serial buffer size (512 bytes)
   *        and if DataLengthMax is not sufficient for returning all buffered data
   *        then data loss will occur.
   *
   *  HINT: If more data will be transferred than the serial buffer size (512 bytes),
   *        then DataLengthMax has to match that serial buffer size. Only then a
   *        seamless transfer of data chunks can be performed, each chunk being the
   *        size of the serial buffer (512 bytes).
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Data a pointer to a data buffer
   *  @param DataLengthReceived a pointer to a value for returning actual data read
   *  @param DataLengthMax the maximal data length to be read (1..512)
   *  @param Timeout a time period [s] after which the function returns if no data was received
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getUARTBuffer(Camera_handle hCamera,
                       unsigned char *Data,
                       int *DataLengthReceived,
                       int DataLengthMax,
                       float Timeout);

  /** setUARTBaud.
   *  The baud rate of the camera's UART will be set to one out of a set of
   *  pre-defined baud rates. Alternatively, any baud rate can be provided
   *  as integer which would not have to comply with any pre-defined value.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BaudRate the baud rate to be assigned to the UART
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setUARTBaud(Camera_handle hCamera,
                     SVGigE_BaudRate BaudRate);

  /** getUARTBaud.
   *  The currently set baud rate in the camera's UART will be returned.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BaudRate the UART's currently assigned baud rate
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getUARTBaud(Camera_handle hCamera,
                     SVGigE_BaudRate *BaudRate);



//------------------------------------------------------------------------------
// 25 - Controlling camera: Direct register and memory access
//------------------------------------------------------------------------------

  /** Set GigE camera register.
      *  A register of a SVS GigE camera will be directly written to
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param RegisterAddress a valid address of a SVS GigE camera register
   *  @param RegisterValue a value that has to be written to selected register
   *  @param GigECameraAccessKey a valid key for directly accessing a GigE camera
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setGigECameraRegister(Camera_handle hCamera,
                               unsigned int RegisterAddress,
                               unsigned int RegisterValue,
                               unsigned int GigECameraAccessKey);

  /** Get GigE camera register.
   *  A value from a SVS GigE camera register will be directly read out
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param RegisterAddress a valid address of a GigE camera register
   *  @param ProgammedRegisterValue the current programmed value will be returned
   *  @param GigECameraAccessKey a valid key for directly accessing a GigE camera
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getGigECameraRegister(Camera_handle hCamera,
                               unsigned int RegisterAddress,
                               unsigned int *ProgammedRegisterValue,
                               unsigned int GigECameraAccessKey);

  /** Write GigE camera memory.
   *  A block of data will be written to the memory of a SVS GigE camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MemoryAddress a valid memory address in a SVS GigE camera
   *  @param DataBlock a block of data that has to be written to selected memory
   *  @param DataLength the length of the specified DataBlock
   *  @param GigECameraAccessKey a valid key for directly accessing a GigE camera
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_writeGigECameraMemory(Camera_handle hCamera,
                               unsigned int  MemoryAddress,
                               unsigned char *DataBlock,
                               unsigned int  DataLength,
                               unsigned int  GigECameraAccessKey);

  /** Read GigE camera memory.
   *  A block of data will be read from the memory of a SVS GigE camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MemoryAddress a valid memory address in a SVS GigE camera
   *  @param DataBlock an address where the data from selected memory will be written to
   *  @param DataLength the data length to be read from the camera's memory
   *  @param GigECameraAccessKey a valid key for directly accessing a GigE camera
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_readGigECameraMemory(Camera_handle hCamera,
                              unsigned int  MemoryAddress,
                              unsigned char *DataBlock,
                              unsigned int  DataLength,
                              unsigned int  GigECameraAccessKey);

//------------------------------------------------------------------------------
// 26 - Controlling camera: Persistent settings and recovery
//------------------------------------------------------------------------------

  /** Write EEPROM defaults.
   *  The current settings will be made the EEPROM defaults that will be
   *  restored on each camera start-up
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_writeEEPROM(Camera_handle hCamera);

  /** Read EEPROM defaults.
   *  The EEPROM content will be moved to the appropriate camera registers.
   *  This operation will restore the camera settings that were active when
   *  the EEPROM write function was performed
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_readEEPROM(Camera_handle hCamera);

  /** Restore factory defaults.
   *  The camera's registers will be restored to the factory defaults and at
   *  the same time those settings will be written as default to EEPROM
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_restoreFactoryDefaults(Camera_handle hCamera);

 /** Load settings from XML
  *  New camera settings will be loaded from a XML file.
  *  The  XML file content will be moved to the appropriate camera registers.
  *  In this operation the XML file will be used instead of the EEPROM.
  *
  *  @see CameraContainer_getCamera()
  *  @param hCamera a camera handle received from CameraContainer_getCamera()
  *  @param Filename a complete path and filename where to load the settings from
  *  @return success or error code
  */
  __usrdllexport__ SVGigE_RETURN
  Camera_loadSettingsFromXml(Camera_handle hCamera,
                             const char *Filename);

 /** Save settings to XML
  *  The current settings will be stored in a XML file
  *  In this operation the XML file will be used instead of the EEPROM.
  *
  *  @see CameraContainer_getCamera()
  *  @param hCamera a camera handle received from CameraContainer_getCamera()
  *  @param Filename a complete path and filename where to write the new settings.
  *  @return success or error code
  */
  __usrdllexport__ SVGigE_RETURN
  Camera_SaveSettingsToXml(Camera_handle hCamera,
                           const char *Filename);

//------------------------------------------------------------------------------
// 27 - General functions
//------------------------------------------------------------------------------

  /** Estimate white balance.
   *  Current image will be investigated for a suitable white balance setting
   *
   *  @param BufferRGB a buffer with current RGB image
   *  @param BufferLength the length of the RGB buffer
   *  @param Red new value for red color
   *  @param Green new value for green color
   *  @param Blue new value for blue color
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  SVGigE_estimateWhiteBalance(unsigned char *BufferRGB,
                              int BufferLength,
                              float *Red,
                              float *Green ,
                              float *Blue);

  /** Estimate white balance with and without using a gray card.
   *  Current image will be investigated for a suitable white balance
   *
   *  @param BufferRGB a buffer with current RGB image
   *  @param PixelNumber of the Current image
   *  @param Red new value for red color
   *  @param Green new value for green color
   *  @param Blue new value for blue color
   *  @param Whitebalance_Art white balance estimation methode used
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  SVGigE_estimateWhiteBalanceExtended(unsigned char *BufferRGB,
													int PixelNumber,
													int &Red,
													int &Green,
													int &Blue,
													SVGIGE_Whitebalance_SELECT  Whitebalance_Art );

  /** Write image as a bitmap file to disk
   *  An image given by image data, geometry and type will be written to a
   *  specified location on disk.
   *
   *  @param Filename a path and filename for the bitmap file
   *  @param Data a pointer to image data
   *  @param SizeX the width of the image
   *  @param SizeY the height of the image
   *  @param PixelType either GVSP_PIX_MONO8 or GVSP_PIX_RGB24
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  SVGigE_writeImageToBitmapFile(const char *Filename, unsigned char *Data, int SizeX, int SizeY, GVSP_PIXEL_TYPE PixelType);

  /** Install a filter driver
   *  A filter driver will be installaed automatically for current system platform
   *  which may be WinXP x86/x64 or Win7 x86/64.
   *  The location of the driver package can be provided by function argument.
   *  If not, a default location will be supposed for particular driver files.
   *  If the driver location is provided then that location has to contain the
   *  platform folders for WinXP x86/x64 and Win7 x86/x64 except when the name
   *  of the .inf file is also provided. In the latter case the path has to
   *  point to the folder where the .inf file can be found.
   *
   *  @param PathToDriverPackage (optional) the full path to driver packages (excluding platform folders)
   *  @param FilenameINF (optional) the name of an INF file to install for the protocol edge
   *  @param FilenameINF_m (optional) the name of an INF file to install for the miniport edge
   *  @return a success or failure code
   */
  __usrdllexport__ SVGigE_RETURN
  SVGigE_installFilterDriver(const char *PathToDriverPackage, const char *FilenameINF, const char *FilenameINF_m);

  /** De-install filter driver
   *  A SVGigE filter driver component will be located in the system and if it
   *  is present then it will be uninstalled.
   *
   *  @return a success or failure code
   */
  __usrdllexport__ SVGigE_RETURN
  SVGigE_uninstallFilterDriver();

//------------------------------------------------------------------------------
// 28 - Diagnostics
//------------------------------------------------------------------------------

  /**
   *  Error get message.
   *  If the provided function return code represents an error condition then
   *  a message will be mapped to the return code which will explain it.
   *
   *  @param ReturnCode a valid function return code
   *  @return a string which will explain the return code
   */
  __usrdllexport__ const char *
  Error_getMessage(SVGigE_RETURN ReturnCode);

  /** Register for log messages.
   *  Log messages can be requested for various log levels:
   *  0 - logging off
   *  1 - CRITICAL errors that prevent from further operation
   *  2 - ERRORs that prevent from proper functioning
   *  3 - WARNINGs which usually do not affect proper work
   *  4 - INFO for listing camera communication (default)
   *  5 - DIAGNOSTICS for investigating image callbacks
   *  6 - DEBUG for receiving multiple parameters for image callbacks
   *  7 - DETAIL for receiving multiple signals for each image callback
   *
   *  Resulting log messages can be either written into a log file
   *  respectively they can be received by a callback and further
   *  processed by an application.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LogLevel one of the above log levels
   *  @param LogFilename a file name where all log messages will be written to or NULL
   *  @param LogCallback a callback function that will receive all log messages or NULL
   *  @param MessageContext a context that will be returned to application with each callback or NULL
   *  @return success or error code
   */
  typedef void (__stdcall *LogMessageCallback)(char *LogMessage, void *MessageContext);

  __usrdllexport__ SVGigE_RETURN
  Camera_registerForLogMessages(Camera_handle hCamera,
                                int LogLevel =4,
                                const char *LogFilename =NULL,
                                LogMessageCallback LogCallback =NULL,
                                void *MessageContext =NULL);


//------------------------------------------------------------------------------
// 29 - Special control: Lens control
//------------------------------------------------------------------------------


/** Is Lens available.
   *The camera will be queried whether a MFT Lens is available or not.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isAvailable a flag indicating whether Objective is availabel or not
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
  Camera_isLensAvailable(Camera_handle hCamera, bool *isAvailable);



/** Get lens name.
   *  The lens name that is obtained from the lens firmware will be
   *  returned
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ const char*
  Camera_getLensName(Camera_handle hCamera);



/** setLensFocalLenght.
   *
   *  A provided focal length  will be transferred to lens.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocalLenght the focal length to be assigned to the Lens (granularity: 1/10 mm, ex. 350 corresponds to 35 mm).
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
	Camera_setLensFocalLenght(Camera_handle hCamera, unsigned int FocalLenght);


/** getLensFocalLenght.

   *  The currently set focal length  of the lens will be returned.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocalLenght the currently assigned focal length (granularity: 1/10 mm, ex. 350 corresponds to 35 mm).
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
	Camera_getLensFocalLenght(Camera_handle hCamera, unsigned int *FocalLenght);

/** getLensFocalLenghtMin.

   *  Get the minimal focal length that can be used.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocalLenghtMin the minimal focal length setting(granularity: 1/10 mm, ex. 140 corresponds to 14 mm).
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
	Camera_getLensFocalLenghtMin(Camera_handle hCamera, unsigned int *FocalLenghtMin);

  /** getLensFocalLenghtMax.
   *  Get the maximal focal length that can be used.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocalLenghtMax the maximal focal length setting (granularity: 1/10 mm, ex. 420 corresponds to 42 mm).
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
	Camera_getLensFocalLenghtMax(Camera_handle hCamera, unsigned int *FocalLenghtMax);


  /** set focus unit
   *  A selected focus unit will be transferred to lens.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Selected_unit the focus unit( mm or 1/10 mm) to be assigned to Lens
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
	Camera_setLensFocusUnit(Camera_handle hCamera, FOCUS_UNIT Selected_unit );


  /** get focus unit
   *  The currently focus unit will be returned.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Selected_unit the currently used focus unit.
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
  Camera_getLensFocusUnit(Camera_handle hCamera, FOCUS_UNIT *Selected_unit );


/** setLensFocus.
   *  A provided focus will be transferred to lens.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Focus the focus in the selected unit to be assigned to lens (default unit is mm)
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
	Camera_setLensFocus(Camera_handle hCamera, unsigned int Focus );


/** getLensFocus.
   *  The currently set focus of the lens will be returned.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Focus the currently assigned focus in the selected unit(default unit is mm)
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
  Camera_getLensFocus(Camera_handle hCamera, unsigned int *Focus);

/** getLensFocusMin.
   *  Get the minimal focus that can be used.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocusMin the minimal focus setting in the selected unit(default unit is mm)
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
   Camera_getLensFocusMin(Camera_handle hCamera, unsigned int *FocusMin );


/** getLensFocusMax.
   *  Get the maximal focus that can be used.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocusMax the maximal focus setting in the selected unit(default unit is mm)
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
  Camera_getLensFocusMax(Camera_handle hCamera, unsigned int *FocusMax);



/** setLensAperture.
   *  A provided aperture will be transferred to lens.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Aperture the aperture to be assigned to the Lens (granularity: 1/10 , ex. 90 corresponds to 9)
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
	Camera_setLensAperture(Camera_handle hCamera, unsigned int Aperture);


/** getLensAperture.
   *  The currently set aperture of the lens will be returned.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Aperture the currently assigned aperture (granularity: 1/10 , ex. 90 corresponds to 9)
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
	Camera_getLensAperture(Camera_handle hCamera, unsigned int *Aperture);

/** getLensApertureMin.
   *  Get the minimal aperture that can be used.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ApertureMin the minimal aperture setting (granularity: 1/10 , ex. 35 corresponds to 3.5)
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
	Camera_getLensApertureMin(Camera_handle hCamera, unsigned int *ApertureMin);

/** getLensApertureMax.
   *  Get the maximal aperture that can be used.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ApertureMax the maximal aperture setting (granularity: 1/10 , ex. 90 corresponds to 9)
   *  @return success or error code
   */
 __usrdllexport__ SVGigE_RETURN
	Camera_getLensApertureMax(Camera_handle hCamera, unsigned int *ApertureMax);



//------------------------------------------------------------------------------
// 99 - Deprecated functions
//------------------------------------------------------------------------------

  /** Start acquisition cycle.
   *  2009-05-05: DEPRECATED, please use Camera_softwareTrigger()
   *
   *  The camera will be triggered for starting a new acquisition cycle.
   *  A mandatory pre-requisition for successfully starting an acquisition
   *  cycle by software is to have the camera set to ACQUISITION_MODE_SOFTWARE_TIGGER
   *  before
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_startAcquisitionCycle(Camera_handle hCamera);

  /** Set tap calibration
   *  2009-03-10: DEPRECATED, please use Camera_setTapBalance()
   *
   *  The provided tab calibration values will be written to camera. The calibration
   *  values apply to a particular tap. Thus adjusting all taps requires running this
   *  function multiple, once for each tap.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TapID an identification number (0, 1) of the tap
   *  @param Gain the gain value for the ADC as integer
   *  @param Offset the offset value for the ADC as integer
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setTapCalibration(Camera_handle hCamera,
                           unsigned int TapID,
                           unsigned int Gain,
                           unsigned int Offset);

  /** Get tap calibration
   *  2009-03-10: DEPRECATED, please use Camera_getTapBalance()
   *
   *  Current tab calibration values will be retrieved from camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TapID an identification number (0, 1) of the tap
   *  @param Gain the gain value of the ADC will be returned
   *  @param Offset the offset value of the ADC will be returned
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getTapCalibration(Camera_handle hCamera,
                           unsigned int TapID,
                           unsigned int *Gain,
                           unsigned int *Offset);

	/** 2009-02-19: Re-implemented for backward compatibility */
  __usrdllexport__ SVGigE_RETURN
  Camera_setLUTMode(Camera_handle hCamera,
                    LUT_MODE LUTMode);

	/** 2009-02-19: Re-implemented for backward compatibility */
  __usrdllexport__ SVGigE_RETURN
  Camera_getLUTMode(Camera_handle hCamera,
                    LUT_MODE *LUTMode);

  /** Create look-up table.
   *
   *  DEACTIVATED !
   *  2006-12-20: White balance has been re-implemented by the Camera_setWhiteBalance()
   *              function and a separate lookup table can be uploaded by setLookupTable().
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_createLUTwhiteBalance(Camera_handle hCamera, float Red, float Green, float Blue);

  /** Stamp timestamp. (2008: functionality removed)
   *  A hardware timestamp will be written into the selected camera register.
   *  The timestamp's actual value can be read out by Camera_getTimestamp()
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TimestampIndex the index of the timestamp to be set by hardware
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_stampTimestamp(Camera_handle hCamera,
                        int TimestampIndex);

  /** Get timestamp. (2008: functionality removed)
   *  The value of a selected hardware timestamp will be returned. The index
   *  provided to the function is valid in the range between 0 and 8
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TimestampIndex the index of the timestamp to be returned
   *  @param Timestamp the timestamp's value in seconds and part of a second
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getTimestamp(Camera_handle hCamera,
                      int TimestampIndex,
                      double *Timestamp);

  /** Camera_getDebugInfo
   *  The value of a selected debug information will be returned.
	 *  This function is for internal use only.
   *
   *  @see CameraContainer_getCamera()
   *  @param hImage an image handle that was received from the callback function
   *  @param Index the index of the debug info to be returned
   *  @return success or error code
   */
  __usrdllexport__ unsigned
  Image_getDebugInfo(Image_handle hImage,
                     int Index);

  /** Get image gray
	 *
	 *  OBSOLETE. Please obtain a raw image and use a suitable conversion algorithm
	 *            in order to get from a raw image of a bayer coded sensor to a
	 *            gray image of intended quality.
	 *            Please refer to: http://en.wikipedia.org/wiki/Grayscale
	 *
   *  A bayer coded image will be converted into a BW image. The caller needs to
   *  provide a sufficient 8-bit buffer.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @param Buffer8bit a buffer for 8-bit image data
   *  @param BufferLength the length of the image buffer
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Image_getImageGray(Image_handle hImage,
                    unsigned char *Buffer8bit,
                    int BufferLength);

  /** Force open connection to camera.
   *
   *  INTERNAL USE ONLY, please do not use in customer applications.
   *
   *  A TCP/IP control channel will be established.
   *  The connection will be established independently whether the camera firmware
   *  matches a minimal build number or not. In case of a low build number the
   *  application may run into error conditions with SDK functions. Therefore
   *  this function can be used only by those applications that need to access
   *  cameras with an older firmware build. The application has to deal with all
   *  problem situations in this case. Usually an application needs to do direct
   *  register access in order to operate a camera with an older build number
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Timeout the time without traffic or heartbeat after which a camera drops a connection (default: 3.0 sec.)
   *                NOTE: Values between 0.0 to 0.5 sec. will be mapped to the default value (3.0 sec.)
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_forceOpenConnection(Camera_handle hCamera, float Timeout);

  /** Set tap balance
   *  2011-08-18 - DEPRECATED. Please use setTapGain() instead.
	 *
	 *  A provided tap balance in [dB] will be transferred to camera.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TapBalance the new value for tap balance to be activated
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setTapBalance(Camera_handle hCamera,
                       float TapBalance);

  /** Get tap balance
   *  2011-08-18 - DEPRECATED. Please use getTapGain() instead.
	 *
   *  Currently adjusted tap balance in [dB] will be retrieved from camera.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TapBalance a pointer to a tap balance value
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getTapBalance(Camera_handle hCamera,
                       float *TapBalance);

	/** 2012-01-18: Re-implemented for backward compatibility */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_createMultiStream(StreamingChannel_handle *hStreamingChannel,
                                     CameraContainerClient_handle hCameraContainer,
                                     GVSP_PIXEL_TYPE PixelType,
                                     void *StreamParams,
                                     int StreamCount,
                                     int BufferCount,
                                     StreamCallback CallbackFunction,
                                     void *Context);


 /** Set channel timeout(functionality removed).
   *  An overall channel timeout will be set that is applied to the period from receiving a first
   *  data packet in an image stream and the completion of the image. When the specified
   *  timeout is reached the streaming channel will send a NULL pointer to application in a
   *  registered before image callback function. Possible image data from any camera arriving
   *  after timeout will be rejected. The channel will be ready for receiving a subsequent image.
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param ChannelTimeout the overall channel timeout between first data packet and image completion
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_setChannelTimeout(StreamingChannel_handle hStreamingChannel,
                                     float ChannelTimeout);

  /** Get channel timeout(functionality removed).
   *  A channel's timeout will be queried and be returned to application.
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param ChannelTimeout the programmed overall channel timeout between first data packet and image completion
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  StreamingChannel_getChannelTimeout(StreamingChannel_handle hStreamingChannel,
                                     float *ChannelTimeout);

 /** Set tap user settings(functionality removed).
   *  A provided tap gain in [dB] will be transferred to camera.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TapSelect one of the defined tap selectors
   *  @param TapUserGain the new value for tap user gain
   *  @param TapUserOffset the new value for tap user offset
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_setTapUserSettings(Camera_handle hCamera,
										SVGIGE_TAP_SELECT TapSelect,
                    float TapUserGain,
                    float TapUserOffset);

  /** Get tap user settings(functionality removed).
   *  Currently adjusted tap gain in [dB] will be queried from camera.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TapSelect one of the defined tap selectors
   *  @param TapUserGain the currently value for tap user gain
   *  @param TapUserOffset the currently value for tap user offset
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_getTapUserSettings(Camera_handle hCamera,
										SVGIGE_TAP_SELECT TapSelect,
                    float *TapUserGain,
                    float *TapUserOffset);

/** Save tap balance settings( no longer available)
   *  Current settings for tap balance will be saved into a XML file. Usually the
   *  tap balance is adjusted during camera production. Whenever a need exists for
   *  changing those factory settings, e.g. for balancing image sensor characteristics
   *  dependent on gain, particular settings can be determined, saved to files and
   *  later loaded on demand.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Filename a complete path and filename where to save the settings
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_saveTapBalanceSettings(Camera_handle hCamera,
                                const char *Filename);

  /** Load tap balance settings(no longer available )
   *  New settings for tap balance will be loaded from a XML file.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Filename a complete path and filename where to load the settings from
   *  @return success or error code
   */
  __usrdllexport__ SVGigE_RETURN
  Camera_loadTapBalanceSettings(Camera_handle hCamera,
                                const char *Filename);



} // extern "C"

#ifdef __BORLANDC__
#pragma option pop
#endif

#endif


	/** 1394-Based Digital Camera Control Library
	 *  Bayer pattern decoding functions
	 *  Copyright (C) Damien Douxchamps <ddouxchamps@users.sf.net>
	 *
	 *  Written by Damien Douxchamps and Frederic Devernay
	 *
	 *  This library is free software; you can redistribute it and/or
	 *  modify it under the terms of the GNU Lesser General Public
	 *  License as published by the Free Software Foundation; either
	 *  version 2.1 of the License, or (at your option) any later version.
	 *
	 *  This library is distributed in the hope that it will be useful,
	 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
	 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	 *  Lesser General Public License for more details.
	 *
	 *  You should have received a copy of the GNU Lesser General Public
	 *  License along with this library; if not, write to the Free Software
	 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
	 */

#ifndef BAYERH
#define BAYERH

	enum {
	 DC1394_SUCCESS = 0,
	 DC1394_INVALID_BAYER_METHOD
	};

	enum {
	  DC1394_BYTE_ORDER_UYVY=0,
	  DC1394_BYTE_ORDER_YUYV
	};

	enum {
	  DC1394_BAYER_METHOD_NONE  = -1,
	  DC1394_BAYER_METHOD_NEAREST = 0,
	  DC1394_BAYER_METHOD_SIMPLE,
	  DC1394_BAYER_METHOD_BILINEAR,
	  DC1394_BAYER_METHOD_HQLINEAR,
	  DC1394_BAYER_METHOD_EDGESENSE,
	  DC1394_BAYER_METHOD_DOWNSAMPLE,
	};

	#define DC1394_BAYER_METHOD_MIN      DC1394_BAYER_METHOD_NEAREST
	#define DC1394_BAYER_METHOD_MAX      DC1394_BAYER_METHOD_EDGESENSE
	#define DC1394_BAYER_METHOD_NUM     (DC1394_BAYER_METHOD_MAX-DC1394_BAYER_METHOD_MIN+1)

	enum {
	  DC1394_COLOR_FILTER_RGGB = 512,
	  DC1394_COLOR_FILTER_GBRG,
	  DC1394_COLOR_FILTER_GRBG,
	  DC1394_COLOR_FILTER_BGGR
	};

	typedef unsigned short uint16_t;
	typedef unsigned int uint_t;

	int
	dc1394_bayer_decoding_8bit(const unsigned char *bayer, unsigned char *rgb, unsigned int sx, unsigned int sy, unsigned int tile, unsigned int method);

	int
	dc1394_bayer_decoding_16bit(const uint16_t *bayer, uint16_t *rgb, uint_t sx, uint_t sy, uint_t tile, uint_t bits, uint_t method);

#endif
