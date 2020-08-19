===================
 Thorlabs KCube Inertial Motor
===================

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
    :plugin: ThorlabsISM

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



Compilation
===========

To compile this plugin, install the Thorlabs KINESIS driver package in the same bit-version than itom (32/64bit).
Then set the CMake variable **THORLABS_KINESIS_DIRECTORY** to the base directory of Kinesis (e.g. C:/Program Files/Thorlabs/Kinesis).
The required libraries from Kinesis will automatically be copied to the *lib* folder of itom. Do not use Kinesis 1.6.0 or below for compiling this plugin.

Kinesis 1.7.0 requires the Microsoft C++ Redistributable 2012.

Changelog
==========

