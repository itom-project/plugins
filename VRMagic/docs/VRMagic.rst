===================
 VRMagic
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`VRMagic`
**Type**:       :plugintype:`VRMagic`
**License**:    :pluginlicense:`VRMagic`
**Platforms**:  Windows, (Linux possible but not implemented yet)
**Devices**:    USB Cameras from company *VRMagic* (tested with framegrabber VRmAVC-2)
**Author**:     :pluginauthor:`VRMagic`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: VRMagic

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: VRMagic
        
Parameters
============

**bpp**: {int}
    Bit depth of the output data from camera in bpp (can differ from sensor bit depth).

**brightness**: {int}
    Brightness of camera / framegrabber (0 to 255)

**contrast**: {int}
    Contrast of camera / framegrabber (0 to 255)

**format**: {str}, read-only
    Format of the image. See documentation of SDK for explanation.

**group_ID**: {int}, read-only
    Group ID of device (gid).

**hue**: {int}
    Hue of camera / framegrabber (0 to 255)

**name**: {str}, read-only
    name of the camera

**product_ID**: {int}, read-only
    Product ID of device (pid).

**saturation**: {int}
    Saturation of camera / framegrabber (0 to 255)

**serial_number**: {str}, read-only
    Serial number of device.

**signal_source**: {str}
    Signal source of the grabber [svideo, composite, yc].

**sizex**: {int}, read-only
    Width of ROI (number of columns).

**sizey**: {int}, read-only
    Height of ROI (number of rows).

**vendor_ID**: {int}, read-only
    Vendor ID of device (vid).
    
Installation
=============

Install the VRmagic SDK, named VRmUsbCam DevKit (https://www.vrmagic.com/de/imaging/downloads/). For compiling the
plugin set the CMake variable VRMagic_INCLUDE_DIR to the directory that contains the header files of the SDK. Under
Windows this is often found under *%CommonProgramFiles%\VRmagic\VRmUsbCamSDK\include*.

Parameters
===========

An instance of this plugin has the following internal parameters:


    
Changelog
==========

* itom setup 2.1.0: This plugin has been compiled using the VRmagic SDK 4.5.0