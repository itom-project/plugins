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

ITOM Plugin to be used for interaction with the K-Cube Controller for Piezo Inertia Stages and Actuators (e.g. KIM101).

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
    acceleration Steps/s
**async**: {int}
    synchronous (0, default) or asynchronous (1) mode
**enableAxes**: {int}
    disable (0, default) or enable axis (1)
**deviceName**: {str}, read-only
    Description of the device
**dualChannel**: {int}
    single Channel mode (0, default) or dual Channel mode (1)
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

This example shows how to initialized the device in **itom** and change the position:

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

**Dual Channel Mode** (recommended)

In the case that two axis are used together, the dual channel mode recommended (e. g. two axis are attached to a mirror and are used to move the mirror in both dimensions).
Channel 0 and 1 or channel 2 and 3 are move at once. This reduces the moving time a lot.

    .. code-block:: python

        mot.setParam("dualChannel", 1)  # dual channel is activated by the parameter *dualChannel*
        mot.setPosAbs(0, 50, 1, 200)  # The moving commands must contain the positions of both dual channel axis or the positions of all four axes.
        mot.setPosAbs(2, 50, 3, 200)
        mot.setPosAbs(0, 25, 1, 50, 2, 100, 3, 150)


Compilation
===========


To compile this plugin, install the Thorlabs KINESIS from
https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=Motion_Control&viewtab=0
driver package in the same bit-version than itom (32/64bit).
Then set the CMake variable **THORLABS_KINESIS_DIRECTORY** or the environment variable **THORLABS_KINESIS_ROOT**
to the base directory of Kinesis (e.g. C:/Program Files/Thorlabs/Kinesis).
The required libraries from Kinesis will automatically be copied to the *lib* folder of itom.

Kinesis 1.7.0 requires the Microsoft C++ Redistributable 2012.

Changelog
==========

* itom setup 4.1.0: This plugin has been compiled with Thorlabs Kinesis 1.14.25.
* itom setup 4.2.0: This plugin has been compiled with Thorlabs Kinesis 1.14.28.
* itom setup 4.3.0: This plugin has been compiled with Thorlabs Kinesis 1.14.47.
