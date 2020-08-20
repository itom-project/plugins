================================
 Thorlabs KCube Inertial Motor
================================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsKCubeIM`
**Type**:       :plugintype:`ThorlabsKCubeIM`
**License**:    :pluginlicense:`ThorlabsKCubeIM`
**Platforms**:  Windows
**Devices**:    K-Cube Controller for Piezo Inertia Stages and Actuators, e.g. KIM101
**Author**:     :pluginauthor:`ThorlabsKCubeIM`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: ThorlabsKCubeIM

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: ThorlabsKCubeIM

Parameters
===========

These parameters are available and can be used to configure the **ThorlabsKCubeIM** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**acceleration**: {seq. of int}
    acceleration Steps/s?
**async**: {int}
    asychronous (1) or sychronous (0) mode
**deviceName**: {str}, read-only
    Description of the device
**firmwareVersion**: {str}, read-only
    Firmware version of the device
**lockFrontPanel**: {int}, read-only
    1 to lock the front panel, else 0
**maxVoltage**: {seq. of int}
    maximum voltage of axis
**name**: {str}, read-only
    Name of the plugin
**numaxis**: {int}, read-only
    number of axes (channels), default 4
**serialNumber**: {str}, read-only
    Serial number of the device
**softwareVersion**: {str}, read-only
    Software version of the device
**stepRate**: {seq. of int}
    step rate in Steps/s
**timeout**: {float}
    timeout for move operations in sec


Usage
============

This example shows how to initalized the device in **itom** and change the position:

    .. code-block:: python
        
        mot = dataIO("ThorlabsFF") # get instance of plugin. Optional give the serialnumber of the device
        mot.calib(0) # set the current position of axis 0 to position 0
        mot.calib(0, 1, 2, 3) # set the current positions of axis 0 - 3 to position 0
        mot.setParam("maxVoltage", [100, 100, 100, 100]) # set the maximum voltage of the 4 axes
        mot.setParam("stepRate", [2000, 2000, 2000, 2000]) # set the steprate of the 4 axes
        mot.setParam("acceleration", [100000, 100000, 100000, 100000])
        mot.setPosAbs(0, 100) # set the axis 0 to the absolute position 100
        mot.setPosRel(0, 100) # move the axis 0 relative by 100
        mot.setPosAbs(0, 100, 1, 100, 2, 100, 3, 100) # set all 4 axis to the absolute position 100
        mot.setPosRel(0, 100, 1, 100, 2, 100, 3, 100) # move all 4 axis relative by 100
        mot.getPos(0) # get the position of axis 0
        mot.getPos(0, 1, 2, 3) # get the positions of all 4 axis
        
        

Compilation
===========

To compile this plugin, install the Thorlabs KINESIS driver package in the same bit-version than itom (32/64bit).
Then set the CMake variable **THORLABS_KINESIS_DIRECTORY** to the base directory of Kinesis (e.g. C:/Program Files/Thorlabs/Kinesis).
The required libraries from Kinesis will automatically be copied to the *lib* folder of itom. Do not use Kinesis 1.6.0 or below for compiling this plugin.

Kinesis 1.7.0 requires the Microsoft C++ Redistributable 2012.

Changelog
==========

