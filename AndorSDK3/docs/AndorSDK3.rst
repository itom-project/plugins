===================
 Andor SDK3
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`AndorSDK3`
**Type**:       :plugintype:`AndorSDK3`
**License**:    :pluginlicense:`AndorSDK3`
**Platforms**:  Windows, Linux ready but not tested
**Devices**:    Andor cameras supported by their SDK3 (Zyla, Neo)
**Author**:     :pluginauthor:`AndorSDK3`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: AndorSDK3

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: AndorSDK3

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    AVTVimba
**integration_time**: {float}
    Exposure time of chip (in seconds)
**binning**: {int}
    Horizontal and vertical binning, depending on camera ability. 104 means a 1x binning in horizontal and 4x binning in vertical direction. (only symmetric binning is allowed; if read only binning is not supported)
**gain**: {float}, read-only
    Gain (normalized value 0..1)
**sizex**: {int}, read-only
    Pixelsize in x (cols)
**sizey**: {int}, read-only
    Pixelsize in y (rows)
**roi**: {int seq.}
    ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]
**bpp**: {int}
    Bitdepth of each pixel
**timeout**: {float}
    acquisition timeout in secs
**frame_rate**: {float}
    frame rate in Hz
**interface**: {str}, read-only
    camera interface type
**firmware_version**: {str}, read-only
    firmware version
**camera_model**: {str}, read-only
    model name of camera
**camera_name**: {str}, read-only
    name of camera
**serial_number**: {str}, read-only
    serial number of camera
**sdk_version**: {str}, read-only
    Andor SDK3 version
**trigger_mode**: {str}
    camera trigger (Internal, Software, External, External Start, External Exposure)
**fan_speed**: {str}
    fan speed (Off, Low, On - not all values are available for every camera)
**pixel_readout_rate**: {str}
    pixel readout rate in MHz ('100 MHz', '200 MHz', '280 MHz', '550 MHz'; not all options are available for all cameras)
**electronic_shuttering_mode**: {int}
    0: rolling shutter (for highest frame rate, best noise performance, default), 1: global shutter (for pulsed, fast moving images)
**full_aoi_control**: {int}, read-only
    indicates if full AOI control is available (usually yes, for some Neo cameras it isn't and you can only apply certain ROI sizes (see camera manual))
**readout_time**: {float}, read-only
    time to readout data from the sensor in the current configuration (0.0 if not implemented)
**sensor_temperature**: {float}, read-only
    current temperature of sensor in °C (inf if not implemented)
**sensor_cooling**: {int}
    state of the sensor cooling. Cooling is disabled (0) by default at power up and must be enabled (1) for the camera to achieve its target temperature

Installation
============

*Windows:*

Go to the Andor download page (e.g. https://andor.oxinst.com).
Register as a user and Download the ANDOR SDK3.

Enter your product details comprising the Product Model and Serial Number to request access to the SDK:

The Andor SDK should be found via the default installation path (e.g. C:/Program Files/Andor SDK3).
If not, pease the environmental variable **ANDOR_SDK_ROOT** accordingly.


Changelog
==========

* itom setup 3.1.0: This plugin has been compiled using the Andor SDK 3.08.30007.0
* itom setup 3.2.1: This plugin has been compiled using the Andor SDK 3.08.30007.0
* itom setup 4.0.0: This plugin has been compiled using the Andor SDK 3.08.30007.0
* itom setup 4.1.0: This plugin has been compiled using the Andor SDK 3.08.30007.0
* itom setup 4.3.0: This plugin has been compiled using the Andor SDK 3.15.30084.0
