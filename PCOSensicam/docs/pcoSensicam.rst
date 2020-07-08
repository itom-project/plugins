===================
 PCOSensicam
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PCOSensicam`
**Type**:       :plugintype:`PCOSensicam`
**License**:    :pluginlicense:`PCOSensicam`
**Platforms**:  Windows
**Devices**:    PCO Cameras supported by the pco.sensicam SDK
**Author**:     :pluginauthor:`PCOSensicam`
=============== ========================================================================================================
 
Overview
========

The PCOSensicam is a plugin to access PCO.Sensicam, with itom. It uses the SDK pco.sensicam SDK from PCO AG, Germany.
The plugin has mainly been developed and tested using the camera PCO.Sensicam QE. Dicam-cameras are not tested.

This camera is tested with software triggering as well as an external trigger. Use 'acquire' to prepare the camera
for the next image acquisition, finally started by a rising or falling edge of the trigger input.

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: PCOSensicam

Parameters
==========

These parameters are available and can be used to configure the **PCOSensicam** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**name**: {str}, read-only
    name of the plugin (*PCOCamera*)
**integration_time**: {float}
    exposure time in seconds
**delay_time**: {float}
    only used for triggered image acquisitions: Delay between trigger signal and start of acquisition (in sec)
**x0**, **x1**: {int}
    first and last column (zero-based) of a software region of interest. The real camera image can be cropped to a region of interest, the
    corresponding horizontal boundaries are given by **x0** and **x1**, whereas the current width is obtained by **sizex**.
**y0**, **y1**: {int}
    the same than above but first and last row (related to the height of the ROI).
**sizex**, **sizey**: {int}, read-only
    width and height of the region of interest or the full camera size (default)
**bpp**: {int}
    bit depth, bits per pixel (usually not adjustable)
**trigger**: {int}
    software trigger (0, default), external rising edge (1), external falling edge (2)
**binning**: {int}
    Horizontal and vertical binning. The value is obtained by *horizontal * 100 + vertical*. Therefore, no binning corresponds to 101. Some cameras accepts a linear range of binning values [1,2,...max], others only allow a binary range [1,2,4,8,..max]. If the binning is changed, the region of interest is adapted as well to a suitable value.
**gain**: {double}, read-only
    not available
**offset**: {double}, read-only
    not available 

Most parameters not only have a minimum and maximum value but also a step size.

.. note::
    
    Please consider that the parameters defining the region of interest may change if the binning is changed, since an increased binning value decreases the available image size.
    
Non supported features or cameras
==================================

Currently the following features or cameras are not supported:

* sensicam double shutter or qe double shutter: the double shutter acquisition is not implemented.

Installation
=============

If the plugin can not be loaded with itom, make sure that you have the PCO board driver (e.g. for pco525 boards) installed.
This driver puts the required library sen_cam.dll into the C:/Windows/System32 directory.

Compilation
============

For compiling this plugin, download the latest pco.sensicam SDK (pco Software-Development-Toolkit) from http://www.pco.de and install it on your computer. Then set the CMake
variable *PCO_SENSICAM_SDK_DIR** to the base directory of the pco.sensicam. In addition to the SDK from PCO, you need to install necessary drivers for operating your framegrabber board etc. If you can open the camera in the tool CamWare from PCO, you should also be able to open it in itom.

Changelog
==========

* plugin inserted after the release of itom 1.4.0
* itom setup 2.1.0: This plugin has been compiled using pco.sensicam SDK V601_04
* itom setup 2.2.0: This plugin has been compiled using pco.sensicam SDK V601_04
* itom setup 3.0.0: This plugin has been compiled using pco.sensicam SDK V601_04
* itom setup 3.1.0: This plugin has been compiled using pco.sensicam SDK V601_04
* itom setup 3.2.1: This plugin has been compiled using pco.sensicam SDK V601_04
* itom setup 4.0.0: This plugin has been compiled using pco.sensicam SDK V601_04

