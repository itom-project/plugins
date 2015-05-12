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

The getParam command will also print out more information about the parameter. If 'raw' is obtained without suffix, all non-readonly parameters
are printed to the command line.
        
Hints
======


