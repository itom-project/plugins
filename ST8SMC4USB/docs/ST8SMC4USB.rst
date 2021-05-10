===================
 STANDA 8SMC4-USB
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`Standa8SMC4USB`
**Type**:       :plugintype:`Standa8SMC4USB`
**License**:    :pluginlicense:`Standa8SMC4USB`
**Platforms**:  Windows
**Devices**:    Motor-Controller STANDA 8SMC4-USB-B8-1
**Author**:     :pluginauthor:`Standa8SMC4USB`
=============== ========================================================================================================
 
Overview
========

The plugin implements the controller 8SMC4-USB from STANDA.

.. pluginsummaryextended::
    :plugin: Standa8SMC4USB

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: Standa8SMC4USB

Parameters
==========

These parameters are available and can be used to configure the **STANDA 8SMC4-USB** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**name**: {str}, read-only
    name of the plugin
**device_id**: {str}, read-only
    name of controller
**units_per_step**: {double}, read-only
    units (deg or mm) per step of axis, e.g. full step resolution of data sheet of actuator
**device_num**: {int}, read-only
    The current number of this specific device, if there are more than one devices connected. (0 = first device)
**device_port**: {int}, read-only
    Serial port of device
**unit**: {int} [0,1], read-only
    unit of axis
    0: degree (default)
    1: mm
**micro_steps**: {int}
    micro steps for motor [1,2,4,8,16,32,64,128,256]
**async**: {int} [0,1]
    Defines if the code continues while a controller is processing a task.
**accel**: {double}
    Motor shaft acceleration, mm or degree/s^2 (stepper motor) or RPM/s(DC)
    Range: 0..65535 (in steps/s^2)
**decel**: {double}
    Motor shaft deceleration, mm or degree/s^2 (stepper motor) or RPM/s(DC)
    Range: 0..65535 (in steps/s^2)
**speed**: {double}
    Target speed(for stepper motor: mm or degree/s, for DC: rpm)
    Range: 0..1000000 (in steps/s)

Installation
============

Before using the motor you need to install the communcation driver that is requested once you plug the motor controller to the USB host.
You get this driver either by installing the software XILab from Standa or by directly downloading the driver's inf file (Windows only). For more information see
https://en.xisupport.com/projects/enxisupport/wiki/Software. The driver might require the Microsoft Visual C++ 2008 Redistributable package.

Changelog
=========

* 2015-06-15: The XiLab SDK currently delivered with the source files of the itom plugin has the version 2.3.2.
* itom setup 4.1.0: plugin uses the XiLab SDK in version 2.3.2.