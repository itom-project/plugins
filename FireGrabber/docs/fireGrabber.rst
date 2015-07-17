===================
 FireGrabber
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`FireGrabber`
**Type**:       :plugintype:`FireGrabber`
**License**:    :pluginlicense:`FireGrabber`
**Platforms**:  Windows, Linux
**Devices**:    Cameras supported by FirePackage from Allied Vision
**Author**:     :pluginauthor:`FireGrabber`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: FireGrabber

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: FireGrabber

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    name of plugin
**vendor**: {int}, read-only
    vendor ID of the camera
**cameraID**: {int}, read-only
    camera ID of the camera
**vendorName**: {str}, read-only
    vendor name of the camera [if connected]
**modelName**: {str}, read-only
    model name of the camera [if connected]
**integration_time**: {float}
    Integrationtime of CCD programmed in s
**frame_time**: {float}, read-only
    Transmission time per frame in s
**bpp**: {int}
    bit depth of camera
**brightness**: {float}
    Brightness value (if supported)
**sharpness**: {float}
    Sharpness value (if supported)
**gamma**: {int}
    Gamma correction (0: off, 1: on, default: off)
**gain**: {float}
    Virtual gain
**offset**: {float}, read-only
    Offset not used here
**x0**: {int}
    Startvalue for ROI
**y0**: {int}
    Startvalue for ROI
**x1**: {int}
    Stopvalue for ROI
**y1**: {int}
    Stopvalue for ROI
**sizex**: {int}, read-only
    Pixelsize in x (cols)
**sizey**: {int}, read-only
    Pixelsize in y (rows)
	