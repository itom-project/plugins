===================
 PIPiezoCtrl
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PIPiezoCtrl`
**Type**:       :plugintype:`PIPiezoCtrl`
**License**:    :pluginlicense:`PIPiezoCtrl`
**Platforms**:  Windows, Linux
**Devices**:    Piezo controller E-662, E-816, E-621, E-625, E-665, C-663 from Physik Instrumente
**Author**:     :pluginauthor:`PIPiezoCtrl`
**Requires**:   Plugin SerialIO
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: PIPiezoCtrl

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: PIPiezoCtrl

Parameters
==========

These parameters are available and can be used to configure the **PIPiezoCtrl** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**PI_CMD**: {str}
    use this parameter followed by :YourCommand in order to read/write value from/to device
    (e.g. PI_CMD:ERR?)
**async**: {int}
    asynchronous (1) or synchronous (0) mode
**checkFlags**: {int}
    Check flags (or-combination possible): 0x01: check position boundaries before
    positioning and actualize current position after positioning (default: on), 0x02: check
    for errors when positioning (default: off), 0x04: if device has a on-target flag, it is
    used for checking if the device is on target (default: on), else a simple time gap is
    used that lets the driver sleep after positioning
**comPort**: {int}, read-only
    The current com-port ID of this specific device. -1 means undefined
**ctrlName**: {str}, read-only
    device information string
**ctrlType**: {str}, read-only
    Current type of controller, e.g. E-662, E-665, E-753...
**delayOffset**: {float}
    offset delay [s] per movement (independent on step size)
**delayProp**: {float}
    delay [s] per step size [mm] (e.g. value of 1 means that a delay of 100ms is set for a
    step of 100mu)
**hasLocalRemote**: {int}, read-only
    defines whether the device has the ability to switch between local and remote control
    (1), else (0)
**hasOnTargetFlag**: {int}, read-only
    defines whether the device has the ability to check the 'on-target'-status (1), else (0)
**local**: {int}
    defines whether system is in local (1) or remote (0) mode.
**name**: {str}, read-only
    name of the plugin
**numaxis**: {int}, read-only
    Number of axes (here always 1)
**piezoName**: {str}, read-only
    piezo information string
**posLimitHigh**: {float}
    higher position limit [m] of piezo (this can be supported by the device or by this
    plugin)
**posLimitLow**: {float}
    lower position limit [m] of piezo (this can be supported by the device or by this
    plugin)
**velocity**: {float}, read-only
    velocity of the stage for the controller type C663 in mm per s
