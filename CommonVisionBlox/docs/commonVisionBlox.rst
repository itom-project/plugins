=========================================
 GenICam cameras via Common Vision Blox
=========================================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`CommonVisionBlox`
**Type**:       :plugintype:`CommonVisionBlox`
**License**:    :pluginlicense:`CommonVisionBlox`
**Platforms**:  Windows
**Devices**:    GenICam via Common Vision Blox
**Author**:     :pluginauthor:`CommonVisionBlox`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: CommonVisionBlox

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: CommonVisionBlox

Compilation
===========

Download the latest CommonVisionBlox SDK from https://www.commonvisionblox.com/en/.

Install this software and set the CMake variable **CVB_DIR** or the environment variable **CVB_ROOT**
to the install directory of the CommonVisionBlox SDK (e.g. C:\Program Files\STEMMER IMAGING\Common Vision Blox\).


Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    name of plugin
**integration_time**: {float}
    Exposure time of chip (in seconds)
**heartbeat_timeout**: {int}
    Heartbeat timeout of GigE Vision Transport Layer
**acquisition_mode**: {str}
    'snap' is a single image acquisition (only possible in trigger_mode 'off'), 'grab' is a continuous acquisition
**trigger_mode**: {str}
    'off': camera is not explicitly triggered but operated in freerun mode. The next acquired image is provided upon acquire, 'software' sets trigger mode to On and fires a software trigger at acquire (only possible in acquisition_mode 'grab')
**sizex**: {int}, read-only
    Pixelsize in x (cols)
**sizey**: {int}, read-only
    Pixelsize in y (rows)
**bpp**: {int}, read-only
    bit depth
**raw**: {str}
    use raw:paramname to set internal paramname of camera to value. paramname is the original GenICam parameter name.
**vendor_name**: {str}, read-only
    vendor name
**model_name**: {str}, read-only
    model name
**roi**: {int seq.}
    ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]


Usage
=====

For Windows use open the command line interface with administrator privileges.
Use **setx** to permanently change the System **PATH** Variable (eg. "setx / M PATH "%PATH;C:\Program Files\STEMMER IMAGING\Common Vision Blox").

Until now, only monochrome cameras with pixel format Mono8, Mono10, .. Mono16 are supported. Only the parameter integration_time
is created as plugin parameter (redirected to ExposureTime or ExposureTimeAbs with an assumed time base of micro seconds, integration_time
is still in seconds). All other parameters can be directly accessed via the string parameter 'raw', e.g.

cam.getParam("raw:ExposureMode") or
cam.setParam("raw:ExposureMode", "Timed")

The getParam command will also print out more information about the parameter. If 'raw' is obtained without suffix, all parameters
are printed to the command line.

Acquisition
===========

You can obtain images either by setting the camera in a continuous image acquisition mode (parameter **acquisition_mode** = **grab**) or by acquiring single
images upon a call to acquire (**acquisition_mode** = **snap**). You should try the method which gives better performance for your camera. Try to decrease the acquisition
rate if you have packet losts. In mode **grab** you can additionally trigger the next acquired image by setting **trigger_mode** to **software** instead of **off**. In the latter
case, acquire decards all old images and obtains the next acquired image.

Hints
======
Try to enable jumbo frames in your network adapter and set the packet size in Common Vision Blox to the highest rate. Save the configuration before
loading the camera in itom. If you want to operate the camera with more than 8bit, make sure to set the CVB Color Format to Mono16 in Common Vision Blox (not auto)
and save the configuration as well.

Usually you need to configure the camera and its communication first in CommonVisionBlox before using the camera in itom. Open CommonVisionBlox and configure the camera.
Then safe the configuration (stored in %CVBDATA%/Drivers/GenICam.ini where %CVBDATA% is an environment variable created by CommonVisionBlox) by clicking the corresponding button.

If the camera is loaded in itom, the specific camera and configuration is obtained by this file (if **scanForCameras** is set to False). If you set **scanForCameras** to True,
CommonVisionBlox will be forced to scan for newly connected cameras and the configuration file is automatically reset to its default.

Things to configure are for example:

* color format (bit depth...)
* packet size

Changelog
=========

* itom setup 3.1.0: This plugin has been compiled using the common vision blox 13.01.006
* itom setup 3.2.1: This plugin has been compiled using the common vision blox 13.01.006
* itom setup 4.0.0: This plugin has been compiled using the common vision blox 13.01.006
* itom setup 4.1.0: This plugin has been compiled using the common vision blox 13.01.006
* itom setup 4.2.0: This plugin has been compiled using the common vision blox 13.01.006
* itom setup 4.3.0: This plugin has been compiled using the common vision blox 14.00.010
