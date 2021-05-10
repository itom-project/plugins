===================
 ThorlabsFF
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsFF`
**Type**:       :plugintype:`ThorlabsFF`
**License**:    :pluginlicense:`ThorlabsFF`
**Platforms**:  Windows
**Devices**:    Filter Flipper (MFF101)
**Author**:     :pluginauthor:`ThorlabsFF`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: ThorlabsFF

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: ThorlabsFF

Parameters
==========

These parameters are available and can be used to configure the **ThorlabsKCubeIM** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**deviceName**: {str}, read-only
    description of the device
**firmwareVersion**: {int}, read-only
    firmware version of the connected device
**name**: {str}, read-only
    name of the plugin
**pollingInterval**: {int}, read-only
    device polling interval in ms
**position**: {int}
    position of the device (position1: 1, position2: 2)
**serialNumber**: {str}, read-only
    serial number of the device
**softwareVersion**: {int}, read-only
    software version of the connected device
**transitTime**: {int}
    transit time of the device in ms

Installation
============

Install the Thorlabs Kinesis software and USB Drivers. 

Usage
============

This example shows how to initalized the device in **itom** and change the position:

    .. code-block:: python
        
        flipper = dataIO("ThorlabsFF") # get instance of plugin
        flipper.setParam("transitTime", 300) # set the transit time to 300 ms
        flipper.setParam("position", 1) # flip to position 1
        flipper.setParam("position", 2) # flip to position 2


Compilation
===========

To compile this plugin, install the Thorlabs KINESIS driver package in the same bit-version than itom (32/64bit).
Then set the CMake variable **THORLABS_KINESIS_DIRECTORY** to the base directory of Kinesis (e.g. C:/Program Files/Thorlabs/Kinesis).
The required libraries from Kinesis will automatically be copied to the *lib* folder of itom. Do not use Kinesis 1.6.0 or below for compiling this plugin.

Kinesis 1.7.0 requires the Microsoft C++ Redistributable 2012.

Changelog
=========

* itom setup 4.1.0: This plugin has been compiled with Thorlabs Kinesis 1.14.25.