===================
 PI GCS2
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PI_GCS2`
**Type**:       :plugintype:`PI_GCS2`
**License**:    :pluginlicense:`PI_GCS2`
**Platforms**:  Windows, Linux
**Devices**:    Piezo controllers from Physik Instrumente that can be controlled using the GCS2 command set (e.g. E753)
**Author**:     :pluginauthor:`PI_GCS2`
**Requires**:   Plugin SerialIO
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: PI_GCS2

Plugin to interface with Physik Instrumente (PI) GCS Controllers for X/Y and Z motion.

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: PI_GCS2

Parameters
===========

An instance of this plugin has the following internal parameters:

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
    higher position limit [mm] of piezo (this can be supported by the device or by this
    plugin)
**posLimitLow**: {float}
    lower position limit [mm] of piezo (this can be supported by the device or by this
    plugin)
**referenced**: {int}, read-only
    Axis is referenced (1), not referenced (0), idle (-1)
**velocity**: {float}
    velocity of the stage for the controller type C663 in mm per s
