===================
NerianSceneScanPro
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`NerianSceneScanPro`
**Type**:       :plugintype:`NerianSceneScanPro`
**License**:    :pluginlicense:`NerianSceneScanPro`
**Platforms**:  Windows
**Devices**:    Cameras from company *Nerian* (tested with SceneScanPro)
**Author**:     :pluginauthor:`NerianSceneScanPro`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: NerianSceneScanPro

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: NerianSceneScanPro

		
Parameters
==========
**autoExposureGainControlROI**: {int rect [x0,y0,width,height]}
    ROI for automatic exposure and gain control [x,y,width,height]. X and y are the offset of the ROI from the image center. A value of 0 means the ROI is horizontally centered
**autoIntensityDelta**: {float}
    minimum intensity change that is required for adjusting the camera settings.Intensity values are relatively to the target intensity. A value of 0.01 represents a change of 1 %.
**autoMaxExposureTime**: {float}
    maximum exposure time that can be selected automatically in microseconds.
**autoMaxGain**: {float}
    maximum gain that can be selected automatically in dB.
**autoROI**: {int}
    1 if an ROI for automatic exposure and gain control is enabled.
**autoRecalibration**: {int}
    1 auto re-calibration is enabled.
**autoSkippedFrames**: {int}
    current interval at which the automatic exposure and gain control is run. The value indicates the number of skipped frames between each adjustment.Typically a value > 0 is desired to give the cameras enough time to react to the new setting.
**autoTargetFrame**: {int}
    target frame for automatic exposure and gain control. 0: left frame, 1: right frame, 2 both frames.
**autoTargetIntensity**: {float}
    target image intensity (from 0.0 to 1.0) of the automatic exposure and gain control.
**bpp**: {int}, read-only
    Bit depth of the output data from camera in bpp (can differ from sensor bit depth).
**consistencyCheck**: {int}
    1 if the consistency check is enabled
**consistencyCheckSensitivity**: {int}
    sensitivity value for the consistency check
**disparityOffset**: {int}
    current offset of the evaluated disparity range
**exposureGainMode**: {int}
    0: auto exposure and gain, 1: auto exposure manual gain, 2: manual exposure auto gain, 3: manual exposure and gain
**gapInterpolation**: {int}
    1 if the texture gap interpolation is enabled
**integrationTime**: {float}
    Integrationtime of CCD [0..1] (no unit)
**manualExposureTime**: {float}
    manually selected exposure time in microseconds. This parameter is only relevant if the auto mode is set to manual exposure.
**manualGain**: {float}
    manually selected gain in dB. This parameter is only relevant if the auto mode is set to manual gain.
**maskBorderPixels**: {int}
    1 if border pixels are removed from the computed
**maxFrameTimeDifference**: {int}
    maximum allowed time difference between two corresponding frames.
**name**: {str}, read-only
    
**noiseReduction**: {int}
    1 if the noise reduction filter is enabled
**operationMode**: {int}
    0: Pass trhough, 1: rectify, 2: stereo matching
**roi**: {int rect [x0,y0,width,height]}, read-only
    ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]
**saveAutoRecalibration**: {int}
    1 persistent storage of auto re-calibration results.
**sizex**: {int}, read-only
    width of ROI (x-direction)
**sizey**: {int}, read-only
    height of ROI (y-direction)
**sizez**: {int}, read-only
    number of planes of returned dataObject
**speckleFilterIterations**: {int}
    set the number of speckle reduction iterations
**stereoMatchingEdgeSensitivity**: {int}
    edge sensitivity of the SGM algorithm
**stereoMatchingP1Edge**: {int}
    SGM penalty P1 for small disparity changes at image edges
**stereoMatchingP1NoEdge**: {int}
    SGM penalty P1 for small disparity changes outside image edges
**stereoMatchingP2Edge**: {int}
    SGM penalty P2 for small disparity changes at image edges
**stereoMatchingP2NoEdge**: {int}
    SGM penalty P2 for small disparity changes outside image edges
**textureFilter**: {int}
    1 if the texture filter is enabled
**textureFilterSensitivity**: {int}
    sensitivity value for the texture filter
**trigger0**: {int}
    1 if trigger signal 0 is enabled.
**trigger0PulseWidth**: {float}
    trigger 0 pulse width in milliseconds.
**trigger1**: {int}
    1 if trigger signal 1 is enabled.
**trigger1Offset**: {float}
    time offset between trigger signal 1 and signal 0 in milliseconds.
**trigger1PulseWidth**: {float}
    trigger 1 pulse width in milliseconds.
**triggerFrequency**: {float}
    frequency of the trigger signal in Hz.
**uniquenessCheck**: {int}
    1 if the uniqueness check is enabled
**uniquenessCheckSensitivity**: {int}
    sensitivity value for the uniqueness check
    

Installation
============

*Windows:*

Install the visiontransfer software on your computer and add the visiontransfer.dll directory to your path variables. In visiontransfer 7.0 there is unfortunately a bug in the DLL, so the application can only be used in release (to be fixed in later versions). Since both debug and release dll have the same name and are not compatible, the dll must be added to the paths depending on the compilation mode.
 
    
Changelog
=========

* itom setup 3.2.1: This plugin has been compiled using the NerianSceneScanPro API 7.0
* itom setup 4.1.0: This plugin has been compiled using the NerianSceneScanPro API 7.0