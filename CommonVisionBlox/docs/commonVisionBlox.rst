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
        
Usage
=====

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

