===================
 PCOCamera
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PCOCamera`
**Type**:       :plugintype:`PCOCamera`
**License**:    :pluginlicense:`PCOCamera`
**Platforms**:  Windows
**Devices**:    PCO Cameras supported by the pco.sdk
**Author**:     :pluginauthor:`PCOCamera`
=============== ========================================================================================================

Overview
========

The PCOCamera is a plugin to access PCO.XXXX, e.g. PCO.1300 or PCO.2000, with itom. It uses the SDK pco.sdk from PCO AG, Germany.
The plugin has mainly been developed and tested using the cameras PCO.1200s, PCO.1300, PCO.2000 and PCO.edge USB3.

The camera is always operated in a software trigger mode with a standard image size (no extended image size).

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: PCOCamera

Parameters
==========

These parameters are available and can be used to configure the **PCOCamera** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**name**: {str}, read-only
    name of the plugin (*PCOCamera*)
**camera_name**: {str}, read-only
    device name of camera
**interface**: {str}, read-only
    name of the interface (eithernet, USB, cameralink...)
**x0**, **x1**: {int}
    first and last column (zero-based) of a software region of interest. The real camera image can be cropped to a region of interest, the
    corresponding horizontal boundaries are given by **x0** and **x1**, whereas the current width is obtained by **sizex**.
**y0**, **y1**: {int}
    the same than above but first and last row (related to the height of the ROI).
**sizex**, **sizey**: {int}, read-only
    width and height of the region of interest or the full camera size (default)
**bpp**: {int}
    bit depth, bits per pixel (usually not adjustable)
**temperatures**: {double}, read-only
    list containing the current CCD, camera and power supply temperatures in degree celcius
**coolingSetPointTemperature**: {int}
    set point for the CCD cooling control in degree celcius (only available if supported with this camera)
**IRSensitivity**: {bool} [0,1]
    enables (True, 1) or disables (False, 0) the IR sensitivity of the image sensor, parameter is set to read-only if not available for the specific camera
**pixelrate**: {int}
    Transer pixelrate for data from the camera in MHz.
**conversionFactor**: {double}
    conversion factor in electrons/count
**binning**: {int}
    Horizontal and vertical binning. The value is obtained by *horizontal * 100 + vertical*. Therefore, no binning corresponds to 101. Some cameras accepts a linear range of binning values [1,2,...max], others only allow a binary range [1,2,4,8,..max]. If the binning is changed, the region of interest is adapted as well to a suitable value.
**gain**: {double}, read-only
    not available
**offset**: {double}, read-only
    not available

Most parameters not only have a minimum and maximum value but also a step size.

.. note::

    Please consider that the parameters defining the region of interest may change if the binning is changed, since an increased binning value decreases the available image size.

Compilation
============

For compiling this plugin, download the latest pco.sdk (pco Software-Development-Toolkit) from http://www.pco.de and install it on your computer.
Then set the CMake variable **PCO_SDK_DIR** or environment variable **PCO_SDK_ROOT** the to the base directory of the pco.sdk.
In addition to the SDK from PCO, you need to install necessary drivers for operating your framegrabber board, the GigE connection etc.
If you can open the camera in the tool CamWare from PCO, you should also be able to open it in itom.
For GigE cameras you also need to install the PCO GigE driver and make sure that the connection is properly configured.

Changelog
==========

* itom setup 1.2.0: This plugin has been compiled using pco.sdk 1.17.0
* itom setup 1.3.0: This plugin has been compiled using pco.sdk 1.17.0
* itom setup 1.4.0: This plugin has been compiled using pco.sdk 1.17.0
* itom setup 2.0.0: This plugin has been compiled using pco.sdk 1.17.0
* itom setup 2.1.0: This plugin has been compiled using pco.sdk 1.17.0
* itom setup 2.2.0: This plugin has been compiled using pco.sdk 1.18.0
* itom setup 3.0.0: This plugin has been compiled using pco.sdk 1.18.0
* itom setup 3.1.0: This plugin has been compiled using pco.sdk 1.23.0
* itom setup 3.2.1: This plugin has been compiled using pco.sdk 1.24.0
* itom setup 4.0.0: This plugin has been compiled using pco.sdk 1.25.0
* itom setup 4.1.0: This plugin has been compiled using pco.sdk 1.25.0
* itom setup 4.3.0: This plugin has been compiled using pco.sdk 1.27.0
