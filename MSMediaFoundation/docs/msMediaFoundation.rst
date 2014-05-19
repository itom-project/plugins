===================
 MSMediaFoundation
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`MSMediaFoundation`
**Type**:       :plugintype:`MSMediaFoundation`
**License**:    :pluginlicense:`MSMediaFoundation`
**Platforms**:  Windows (Vista, 7, 8)
**Devices**:    USB Plug&Play Cameras
**Author**:     :pluginauthor:`MSMediaFoundation`
**Version**:    :pluginversion:`MSMediaFoundation`
**Available**:  This plugin will be part of the itom setup from itom 1.2.0 on.
=============== ========================================================================================================
 
Overview
========

This plugin uses the Microsoft Media Foundation framework (Windows Vista, 7, 8) for capturing supported camera devices (e.g. ordinary USB or integrated cameras).

This driver detects an interal list of connected cameras. The parameter *cameraNumber* indicates the device to open (until now, there is no mechanism to open the next
not yet opened device!). The camera can either be used as colored camera, as gray valued camera or it is also possible to only select one color channel that is mapped
to the gray output.

Any detected and supported device can offer multiple framerates and sizes. Use the parameter *mediaTypeID* to select the right value. Open your device with *mediaTypeID* = -1
to let the plugin print a list of supported formats (the plugin initialization then stops with a desired error).

Usage
======

This plugin is able to connect to any USB Plug&Play Cameras as well as other devices that are readable via the Microsoft Media Foundation framework (Windows Vista or
higher). Each connected camera is accessible by an auto-increment camera number, starting with 0. Additionally it is possible to choose between different colorModes:

* "auto" opens a color camera in color mode (rgba32) and grayscale cameras in a 8bit grayscale mode
* "gray" forces cameras to be opened in 8bit grayscale mode, hence, color images are automatically converted to this format
* "color" forces the camera to be opened in a color mode
* "red", "green", "blue" only uses the desired channel, stored in a 8bit grayscale image

Usually cameras can deliver data in different formats (image size and data format). The desired media type can be choosen using the initialization parameter *mediaTypeID*.
Once the *mediaTypeID* is set to -1 at initialization, a list of all available media types is printed and the initialization aborts afterwards with a warning.
Currently, only cameras with the media type *RGB24* or *YUY2* are supported.

Some camera images are vertically flipped (y-direction). If this is the case, the initialization parameter *flipImage* can be set to 1.

An example for opening such a camera device is:

.. code-block:: python
    
    cam = dataIO("MSMediaFoundation", cameraNumber = 0, colorMode = "auto", mediaTypeID = 0)

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: MSMediaFoundation
        
Parameters
==========

These parameters are available and can be used to configure the **MSMediaFoundation** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**name**: {str}, read-only
    name of the plugin (*MSMediaFoundation*)
**deviceName**: {str}, read-only
    name of the connected device
**x0**, **x1**: {int}
    first and last column (zero-based) of a software region of interest. The real camera image can be cropped to a region of interest, the
    corresponding horizontal boundaries are given by **x0** and **x1**, whereas the current width is obtained by **sizex**.
**y0**, **y1**: {int}
    the same than above but first and last row (related to the height of the ROI).
**sizex**, **sizey**: {int}, read-only
    width and height of the region of interest or the full camera size (default)
**bpp**: {int}
    bit depth, bits per pixel [here: 8]
**colorMode**: {str} ("auto", "color", "red", "green", "blue", "gray")
    color mode (see initialization parameters)

The remaining parameters are depending on the connected device and always consist of the real parameter (double, value mapped between [0,1]) and
an integer parameter ("auto"). If the auto-parameter is 0, the corresponding parameter is used to really set the setting, if auto is 1, the camera
automatically adjusts the real parameters.

Possible parameters are:

* **integrationTime**
* **brightness**
* **contrast**
* **saturation**
* **sharpness**
* **hue**
* **focus**
* **gamma**
* **iris**
* **zoom**
* **backlightCompensation**.

Each parameter not only has a minimum and maximum value but also a step size. All this is considered by their parameter, also having a specific step size.

Compilation
============

For compiling this plugin, Windows Vista or later is required as well as an installed Windows SDK. From the Microsoft
Windows SDK is it mainly necessary to install "Windows Headers and Libraries". If the installation fails, one reason is, that
you need to uninstall the Redistributable Packages (Microsoft Visual C++) at first, that will be installed with the SDK afterwards.

The location of the installed Windows SDK is then automatically detected by CMake.

Affiliation
============

This plugin internally uses a modified version of VideoInput, proposed by Evgeny Pereguda and published under http://www.codeproject.com/Articles/559437/Capturing-video-from-web-camera-on-Windows-7-and-8 (Code Project Open License).

