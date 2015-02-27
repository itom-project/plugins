===================
 PGRFlyCapture
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PGRFlyCapture`
**Type**:       :plugintype:`PGRFlyCapture`
**License**:    :pluginlicense:`PGRFlyCapture`
**Platforms**:  Windows, Linux
**Devices**:    Point Grey cameras (USB3) supported by Fly Capture driver. (GigE possible, but yet not implemented)
**Author**:     :pluginauthor:`PGRFlyCapture`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: PGRFlyCapture
    
This plugin has mainly been tested with cameras of type Flea3 and Grasshopper3.

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: PGRFlyCapture

Parameters
==========

These parameters are available and can be used to configure the **PGRFlyCapture** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*. If a parameter is read-only, it is not writeable or supported by the specific camera.

**name**: {str}, read-only
    name of the plugin (*PGRFlyCapture*)
**integration_time**: {double}
    integration time in seconds. The range of the integration is limited if *extended_shutter* is False. Enable extended shutter to set bigger values for the integration time.
**frame_time**: {double}
    frame time in seconds, inverse of frame rate in fps. The frame time is a constant time step at which images can be acquired. The frame time is only writeable if extended shutter is False. If the extended shutter is enabled, the maximum acquisition speed is given by the integration time only.
**extended_shutter**: {int}
    0: extended shutter is disabled, 1 (default): extended shutter is enabled. See frame_time and integration_time for more information.
**gain**: {double}
    The normalized gain of the camera [0,1]. Read-only if not available.
**offset**: {double}
    The normalized offset of the camera[0,1]. Read-only if not available. Corresponds to Point Grey property Brightness.
**exposureEV**: {int}
    Camera brightness control in absolute values. Read-only if not available.
**sharpness**: {int}
    Camera sharpness control in absolute values. Read-only if not available.
**gamma**: {int}
    Camera gamma control in absolute values. Read-only if not available.
**sizex**, **sizey**: {int}, read-only
    Current width and height in pixel of the region of interest. Adjustable via x0, x1, y0, y1
**x0**, **x1**: {int}
    Use these zero-based values to set the left and right margin of the region of interest. 0 <= x0 < x1 < width of camera chip.
**y0**, **y1**: {int}
    Use these zero-based values to set the top and bottom margin of the region of interest. 0 <= y0 < y1 < height of camera chip.
**bpp**: {int}
    Bitdepth of the returned camera image (e.g. 8, 12 or 16 bits per pixel). Please consider that some cameras having an 12bit A/D converter
    allow to obtain 16bit values. This possible however results in a bigger data volume that needs to be transmitted. Consider to use 12bit instead
    that is also mapped to an uint16 data object. At startup, the highest possible value is set, it can be limited by the initialization parameter **bppLimit**.
**timeout**: {double}
    Timeout in seconds when acquiring an image and waiting for the resulting data.
**camSerialNumber**, **camModel**, **camVendor**, **camSensor**, **camResolution**: {str}
    Information about the connected camera
**camFirmwareVersion**, **camFirmwareBuildTime**, **camInterface**: {str}
    Further information about the connected camera
**timestamp**: {double}
    Time in ms since last image acquisition

Changelog
==========

* itom setup 1.2.0: This plugin has been compiled using the FlyCapture 2.6.3.4
* itom setup 1.3.0: This plugin has been compiled using the FlyCapture 2.6.3.4
* itom setup 1.4.0: This plugin has been compiled using the FlyCapture 2.6.3.4

Linux
======

For linux, please consider the document "Using Linux with USB 3.0", published by Point Grey as technical application note TAN2012007. Starting the camera crashed (or came to a timeout when
calling *startDevice* for cameras with an image size bigger than 2MB if the notes in section **Configuration USBFS** are not considered.
