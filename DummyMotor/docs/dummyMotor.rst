===================
 DummyMotor
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`DummyMotor`
**Type**:       :plugintype:`DummyMotor`
**License**:    :pluginlicense:`DummyMotor`
**Platforms**:  Windows, Linux
**Devices**:    Virtual dummy motor with
**Author**:     :pluginauthor:`DummyMotor`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: DummyMotor

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: DummyMotor

Parameters
===========

An instance of this plugin has the following parameters:

**accel**: {float}
    Acceleration in mm/s^2, currently not implemented
**async**: {int}
    Toggles if motor has to wait until end of movement (0:sync) or not (1:async)
**name**: {str}, read-only
    name of the plugin
**numaxis**: {int}, read-only
    Number of Axis attached to this stage
**speed**: {float}
    Speed of the axis between 0.1 and 100000 mm/s
