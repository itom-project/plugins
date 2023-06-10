.. |mus| unicode:: U+00B5 s

===================
 NITWidySWIR
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`NITWidySWIR`
**Type**:       :plugintype:`NITWidySWIR`
**License**:    :pluginlicense:`NITWidySWIR`
**Platforms**:  Windows only
**Devices**:    Cameras from company *NIT* (tested with USB2 WidySWIR 640U-S)
**Author**:     :pluginauthor:`NITWidySWIR`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: NITWidySWIR

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: NITWidySWIR

Parameters
==========

**bpp**: {int}
    Bits per pixel bpp (8,14).
**enablePixelCorrection**: {int}
    enables pixel correction by using NIT NUC files. By changing the integration_time the NUC file is changed, too.
**firmware_version**: {float}, read-only
    Firmware version of connectecd device.
**framerate**: {int}
    Framerate of image acquisition (0..1000) in [fps].
**gain**: {float}
    Gain of camera (0..7.5) in [no unit].
**histogram_threshold**: {float}
    Histogram threshold of image acquisition (0.025, 0.1, 0.4, 1.6) in [%].
**integration_time**: {float}
    Integrationtime of connected device (0.1..25.6) in [s].
**model_id**: {str}, read-only
    Model ID of connected device.
**name**: {str}, read-only
    Name of plugin.
**nucFilePath**: {str}
    Path of the NUC pixel correction files.
**offset**: {float}
    Offset of image acquisition (0..100) in [%].
**pixel_clock**: {float}, read-only
    Pixel clock of device (12.5, 16.66, 20, 25, 33.33, 40, 50, 66.66, 80) in [MHz].
**roi**: {int rect [x0,y0,width,height]}, read-only
    Region of Interest ROI [x0, y0, width, height].
**serial_number**: {int}, read-only
    Serial number of connected device.
**shutter_mode**: {str}
    Shutter Mode of connected device (Global Shutter, Rolling).
**sizex**: {int}, read-only
    Width of ROI (x-direction).
**sizey**: {int}, read-only
    Height of ROI (y-direction).
**trigger_mode**: {str}
    Trigger Mode of connected device (Disabled, Input, Output). Use Disabled for software trigger.

Image Acquisition
==================

With this camera an image is acquired by the standard **itom** image acquisition syntax:

    .. code-block:: python

        cam = dataIO("NITWidySWIR") # get instance of plugin
        cam.startDevice() # start device
        obj = dataObject() # define dataObject
        cam.acquire() # acquire an image
        cam.getVal(obj) # collect the image in a dataObject

The camera can be initialized with two optional parameters (**printManual, printParameterValues**).

    .. code-block:: python

        cam = dataIO("NITWidySWIR", printManual = 1, printParameterValues = 1)

**printParameterValues** print all values of the parameters in the itom shelf.
**printManual** prints the Manual, given by the camera link in the following:

    +-----------------------------+------------------------------------------------------------------------------------------+
    | | Manual: ------------------+ |                                                                                        |
    | | Connector                 | | USB 2.0                                                                                |
    | | Model                     | | NSC1201                                                                                |
    | | Firmware Version          | | 8.0                                                                                    |
    | | Serial Number             | | 24a8c                                                                                  |
    | | --------------------------| |                                                                                        |
    | | First Column :            | | { 0, 4, ... , 628, 632 }                                                               |
    | | Number of Column :        | | { 8, 16, ... , 632, 640 }                                                              |
    | | First Line :              | | { 0, 4, ... , 500, 504 }                                                               |
    | | Number of Line :          | | { 8, 16, ... , 504, 512 }                                                              |
    | | Pixel Depth :             | | { 8bits, 14bits }                                                                      |
    | | Trigger Mode :            | | { Output, Input, Disabled }                                                            |
    | | Mode :                    | | { Rolling, Global Shutter }                                                            |
    | | Pixel Clock :             | | { 12.5MHz, 16.66MHz, ... , 66.66MHz, 80MHz }                                           |
    | | Gain :                    | | { AGC, 0.25, ... , 7.50, 7.75 }                                                        |
    | | Offset :                  | | { 0.00% full scale, 0.39% full scale, ... , 99.06% full scale, 99.45% full scale }     |
    | | Exposure Time :           | | { 100 |mus|, 200 |mus|, ... , 25500 |mus|, 25600 |mus| }                               |
    | | Histogram Threshold :     | | { 1.6%, 0.4%, 0.1%, 0.025% }                                                           |
    +-----------------------------+------------------------------------------------------------------------------------------+

Installation
============

*Windows:*

* The USB2 **NITWidySWIR** driver is installed automatically by Windows after the first connection.

If not please download the NIT SDK from https://new-imaging-technologies.com/software-log-in/#sdk
and set the environment variable **NIT_SDK_ROOT** to the installation diretcory (e.g. C:\NIT-SDK).

Compilation
===========

In order to compile the **NITWidySWIR** plugin, get the **SDK C++**, which will be deliverd together with the camera on a USB stick or contact the company (http://www.new-imaging-technologies.com).
Then set the CMake variable **NITLIBRARY_INCLUDE_DIR** to the directory: **...\\SDK C++\\NITLIBRARY 1.5\\include++**.
CMake will find all necessary files.

Changelog
=========

* itom setup 2.2.1: This plugin has been compiled using the NITLibrary 1.5
* itom setup 3.0.0: This plugin has been compiled using the NITLibrary 1.5
* itom setup 3.1.0: This plugin has been compiled using the NITLibrary 1.5
* itom setup 3.2.1: This plugin has been compiled using the NITLibrary 1.5
* itom setup 4.0.0: This plugin has been compiled using the NITLibrary 1.5
* itom setup 4.1.0: This plugin has been compiled using the NITLibrary 1.5
* itom setup 4.3.0: This plugin has been compiled using the NITLibrary 1.5
