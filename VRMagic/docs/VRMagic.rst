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
