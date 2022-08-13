===========================
 Thorlabs Benchtop DC Servo
===========================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsBDCServo`
**Type**:       :plugintype:`ThorlabsBDCServo`
**License**:    :pluginlicense:`ThorlabsBDCServo`
**Platforms**:  Windows
**Devices**:    Thorlabs 30 mm XY Stage with Integrated Controller, Brushed DC Servo Motors
**Author**:     :pluginauthor:`ThorlabsBDCServo`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: ThorlabsBDCServo

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: ThorlabsBDCServo

Parameters
===========

These parameters are available and can be used to configure the **ThorlabsBPDCServo** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**acceleration**: Sequence[float]
    Acceleration values for each axis in mm/s^2.
    
    *2 values required, Value range: [0, 5], Default: [5, 5]*
**async**: int
    asychronous (1) or synchronous (0) mode
    
    *Value range: [0, 1], Default: 0*
**backlash**: Sequence[float]
    Backlash distance setting in mm (used to control hysteresis).
    
    *2 values required, Value range: [0, 5], Default: [0, 0]*
**channel**: Sequence[int], read-only
    Channel number of each axis.
**deviceName**: str, read-only
    Description of the device
**enabled**: Sequence[int]
    If 1, the axis is enabled and power is applied to the motor. 0: disabled, the motor can
    be turned by hand.
    
    *Allowed number of values: 0 - 2, Value range: [0, 1], Default: [1, 1]*
**firmwareVersion**: int, read-only
    Firmware version of the device
**homed**: Sequence[int], read-only
    If 0, the axis is not homed. 1: homed.
    
    *Allowed number of values: 0 - 2, Value range: [0, 1], Default: [1, 1]*
**maximumTravelPosition**: Sequence[float], read-only
    Maximum travel position for each axis in mm.
    
    *2 values required, Value range: [15, 15], Default: [15, 15]*
**minimumTravelPosition**: Sequence[float], read-only
    Minimum travel position for each axis in mm.
    
    *2 values required, Value range: [-15, -15], Default: [-15, -15]*
**name**: str, read-only
    name of plugin
**numaxis**: int, read-only
    number of axes (channels)
    
    *Value range: [0, 100], Default: 2*
**serialNumber**: str, read-only
    Serial number of the device
**timeout**: float
    Timeout for positioning in seconds.
    
    *Value range: [0, 200], Default: 5*
**velocity**: Sequence[float]
    Velocity values for each axis in mm/s.
    
    *2 values required, Value range: [0, 2.6], Default: [2.6, 2.6]*

Compilation
===========

To compile this plugin, install the Thorlabs KINESIS driver package in the same bit-version than itom (32/64bit). 
It has been implemented using KINESIS version 1.14.32.
Then set the CMake variable **THORLABS_KINESIS_DIRECTORY** to the base directory of Kinesis (e.g. C:/Program Files/Thorlabs/Kinesis).
The required libraries from Kinesis will automatically be copied to the *lib* folder of itom. Do not use Kinesis 1.6.0 or below for compiling this plugin.

Kinesis 1.7.0 requires the Microsoft C++ Redistributable 2012.

Changelog
==========
