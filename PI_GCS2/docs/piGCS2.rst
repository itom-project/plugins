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

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: PI_GCS2

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    PI GCS2
**ctrlType**: {str}, read-only
    Current type of controller, e.g. E-662, E-665, ...
**ctrlName**: {str}, read-only
    device information string
**piezoName**: {str}, read-only
    piezo information string
**comPort**: {int}, read-only
    port number of used COM port.
**numaxis**: {int}, read-only
    return the number of axes, here always 1
**posLimitLow**, **posLimitHigh**: {double}
    lower and upper position limit [mm] of the piezo. Movements out of this range is blocked either by the plugin or by controllers that internally support
    this feature. If the feature is available in the controller, the real ranges are set to these parameters at startup.
**delayProp**, **delayOffset**: {double}
    If the controller has no possibility to check whether the stage already is on target (param 'hasOnTargetFlag') or if these checks are disabled (param 'checkFlags'), a virtual
    sleep is inserted after each movement in synchronous mode. The positioning command will then only return after this delay time. The virtual delay is
    calculated by 'delayOffset' (in sec) + 'delayProp' (in sec/mm) * distance to drive (in mm).
**hasOnTargetFlag**: {int} [0,1]
    defines whether the device has the ability to check the 'on-target'-status (1), else (0)
**local**: {int} [0,1]
    defines if the system is in local mode (1) or in remote mode (0). It can only be controlled by this plugin in remote mode.
**hasLocalRemote**: {int} [0,1]
    returns if the piezo can switch between a local (0) and remote control (1).
**async**: {int} [0,1]
    defines if the piezo should be moved in a synchronous or asynchronous mode (default: synchronous (0)). In asynchronous mode, the positioning commands will immediately
    return after starting the movement without waiting for the end of the movement.
**checkFlags**: {int}, [0,7]
    Bitmask that is used to set if more of less checks should be executed during the positioning. More checks result in more information but need a little bit more
    time during the positioning. These possible values can be combined using an or-combination: 0x0001: check position boundaries before positioning and actualize current 
    position after positioning (default: on), 0x0010: check for errors when positioning (default: off), 0x1000: if device has a on-target flag, it is used for checking if 
    the device is on target (default: on), else a simple time gap is used that lets the driver sleep after positioning
**PI_CMD**: {str}
    Special parameter that can be used to directly set or read parameters from the PI device. Use this parameter followed by :YourCommand in order to read/write value 
    from/to device (e.g. PI_CMD:ERR?)