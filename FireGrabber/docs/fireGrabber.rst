===================
 FireGrabber
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`FireGrabber`
**Type**:       :plugintype:`FireGrabber`
**License**:    :pluginlicense:`FireGrabber`
**Platforms**:  Windows, Linux
**Devices**:    Cameras supported by FirePackage from Allied Vision and remotely controlled via Itom LibUSB Plugin.
**Author**:     :pluginauthor:`FireGrabber`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: FireGrabber

For cameras from Allied Vision it is recommended to use the new Vimba interface and the itom plugin AVTVimba. This
supports more types of cameras and features than the deprecated FirePackage driver from Allied Vision.


Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: FireGrabber


Dependencies
============

FireGrabber plugin depends on LibUSB.


Parameters
===========

An instance of this plugin has the following internal parameters:

**bpp**: {int}
    bit depth of camera
**brightness**: {float}
    Brightness value (if supported)
**cameraID**: {int}, read-only
    camera ID of the camera
**frame_time**: {float}, read-only
    Transmission time per frame in s
**gain**: {float}
    Virtual gain
**gamma**: {int}
    Gamma correction (0: off, 1: on, default: off)
**integration_time**: {float}
    Integrationtime of CCD programmed in s
**modelName**: {str}, read-only
    model name of the camera [if connected]
**name**: {str}, read-only
    name of plugin
**offset**: {float}, read-only
    Offset not used here.
**sizex**: {int}, read-only
    Pixelsize in x (cols)
**sizey**: {int}, read-only
    Pixelsize in y (rows)
**timebase**: {int}, read-only
    timebase (step width of integration_time) in µs
**vendorID**: {int}, read-only
    vendor ID of the camera
**vendorName**: {str}, read-only
    vendor name of the camera [if connected]
**x0**: {int}
    Startvalue for ROI
**x1**: {int}
    Stopvalue for ROI
**y0**: {int}
    Startvalue for ROI
**y1**: {int}
    Stopvalue for ROI


Changelog
==========

* itom setup 1.2.0: Release
