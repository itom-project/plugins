===================
 STANDA 8SMC4-USB
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ST8SMC4USB`
**Type**:       :plugintype:`ST8SMC4USB`
**License**:    :pluginlicense:`ST8SMC4USB`
**Platforms**:  Windows, Linux
**Devices**:    Motor-Controller STANDA 8SMC4-USB-B8-1
**Author**:     :pluginauthor:`ST8SMC4USB`
=============== ========================================================================================================
 
Overview
========

The plugin implements the controller 8SMC4-USB from STANDA.

.. pluginsummaryextended::
    :plugin: ST8SMC4USB

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: ST8SMC4USB

Parameters
==========

These parameters are available and can be used to configure the **STANDA 8SMC4-USB** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**name**: {str}, read-only
    name of the plugin
**deviceID**: {str}, read-only
    name of controller
**unitsPerStep**: {double}, read-only
    units (deg or mm) per step of axis, e.g. full step resolution of data sheet of actuator
**deviceNum**: {int}, read-only
    The current number of this specific device, if there are more than one devices connected. (0 = first device)
**devicePort**: {int}, read-only
    Serial port of device
**unit**: {int} [0,1], read-only
    unit of axis
    0: degree (default)
    1: mm
**microSteps**: {int}
    micro steps for motor [1,2,4,8,16,32,64,128,256]
**async**: {int} [0,1]
    Defines if the code continues while a controller is processing a task.
**accel**: {int}
    Motor shaft acceleration, steps/s^2 (stepper motor) or RPM/s(DC)
    Range: 0..65535
**speed**: {int}
    Target speed(for stepper motor: steps / c, for DC: rpm)
    Range: 0..1000000
**microStepSpeed**: {int}
    Target speed in 1/256 microsteps/s
    Range: 0..255

Installation
============

For installation you need Microsoft Visual C++ 2008 Redistributable pack.
