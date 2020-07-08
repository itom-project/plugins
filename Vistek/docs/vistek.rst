===================
 Vistek
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`Vistek`
**Type**:       :plugintype:`Vistek`
**License**:    :pluginlicense:`Vistek`
**Platforms**:  Windows
**Devices**:    Cameras from company Vistek
**Author**:     :pluginauthor:`Vistek`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: Vistek

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: Vistek

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    Vistek
**cameraModel**: {str}, read-only
    Camera Model ID
**cameraManufacturer**: {str}, read-only
    Camera manufacturer
**cameraVersion**: {str}, read-only
    Camera firmware version
**cameraSerialNo**: {str}, read-only
    Serial number of the camera (see camera housing)
**cameraIP**: {str}, read-only
    IP adress of the camera
**camnum**: {int}
    Camera Number
**exposure**: {float}
    Exposure time in [s] (deprecated: use integration_time instead; this is an alias for integration_time only)
**integration_time**: {float}
    Exposure time in [s]
**gain**: {float}
    Gain [0..18 dB]
**offset**: {float}
    Offset [0.0..1.0]
**binning**: {int}
    Binning mode (OFF = 0 [default], HORIZONTAL = 1, VERTICAL = 2,  2x2 = 3, 3x3 = 4, 4x4 = 5).
    In order to be compatible with the default binning definition of itom plugins, the following values can be set, too:
    OFF = 0, HORIZONTAL = 102, VERTICAL = 201, 2x2=202, 3x3 = 303, 4x4 = 404. getParam returns the values 0-5.
**sizex**: {int}, read-only
    Width of current camera frame
**sizey**: {int}, read-only
    Height of current camera frame
**bpp**: {int}
    bit-depth for camera buffer
**timestamp**: {float}, read-only
    Time in ms since last image (end of exposure)
**streamingPacketSize**: {int}, read-only
    Used streaming packet size (in bytes, more than 1500 usually only possible if you enable jumbo-frames at your network adapter)
**logLevel**: {int}
    Log level. The logfile is Vistek_SVGigE.log in the current directory. 0 - logging off (default),  1 - CRITICAL errors that prevent from further operation, 2 - ERRORs that prevent from proper functioning, 3 - WARNINGs which usually do not affect proper work, 4 - INFO for listing camera communication (default), 5 - DIAGNOSTICS for investigating image callbacks, 6 - DEBUG for receiving multiple parameters for image callbacks, 7 - DETAIL for receiving multiple signals for each image callback

Please consider, that 12bit images have been delivered in a range between 16 and 65535 with an increment of 16 for Vistek library version <= 1.0.0.
From version 1.1.0, this range has been adapted to the default range [0,4096).

Disclaimer
==========

The company *Vistek* is not responsible for this plugin and therefore does not provide any support.

Changelog
==========

* itom setup 1.2.0: This plugin has been compiled using the SVCam GigE SDK 1.4.24.59
* itom setup 1.3.0: This plugin has been compiled using the SVCam GigE SDK 1.4.24.59
* itom setup 1.4.0: This plugin has been compiled using the SVCam GigE SDK 1.4.24.59
* itom setup 2.0.0: This plugin has been compiled using the SVCam GigE SDK 1.4.24.59
* itom setup 2.1.0: This plugin has been compiled using the SVCam GigE SDK 1.4.24.59
* itom setup 2.2.0: This plugin has been compiled using the SVCam GigE SDK 1.4.24.59
* itom setup 3.0.0: This plugin has been compiled using the SVCam GigE SDK 1.5.2.251
* itom setup 3.1.0: This plugin has been compiled using the SVCam GigE SDK 1.5.2.251
* itom setup 3.2.1: This plugin has been compiled using the SVCam GigE SDK 1.5.2.251
* itom setup 4.0.0: This plugin has been compiled using the SVCam GigE SDK 1.5.2.251
