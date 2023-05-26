================================
 Thorlabs KCube DC Servo Drive
================================

=============== ================================================================
**Summary**:    :pluginsummary:`ThorlabsKCubeDCServo`
**Type**:       :plugintype:`ThorlabsKCubeDCServo`
**License**:    :pluginlicense:`ThorlabsKCubeDCServo`
**Platforms**:  Windows
**Devices**:    K-Cube Controller for Brushed DC Servo Motors, e.g. KDC101
**Author**:     :pluginauthor:`ThorlabsKCubeDCServo`
=============== ================================================================

Overview
========

ITOM Plugin to be used for interaction with the K-Cube Controller for Brushed DC Servo Motors(e.g. KDC101).

.. pluginsummaryextended::
    :plugin: ThorlabsKCubeDCServo

Initialization
==============

The following parameters are mandatory or optional for initializing an instance
of this plugin:

    .. plugininitparams::
        :plugin: ThorlabsKCubeDCServo

Parameters
===========

These parameters are available and can be used to configure the
**ThorlabsKCubeDCServo** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of
these parameters is obtained by the method *getParam*, writeable parameters can
be changed using *setParam*.

**acceleration**: {float}
    acceleration in real world units (e.g. mm/s^2)
**async**: {int}
    synchronous (0, default) or asychronous (1) mode
**deviceName**: {str}, read-only
    Description of the device
**enableAxis**: {int}
    If enabled (1, default), power is applied to the motor so it is fixed in position. Else
    (0), the motor can be freely moved.
**firmwareVersion**: {str}, read-only
    Firmware version of the device
**hardwareVersion**: {str}, read-only
    Hardware version of the device
**homingAvailable**: {int}, read-only
    1 if actuator supports a home drive, else 0
**lockFrontPanel**: {int}, read-only
    1 to lock the front panel, else 0 (default)
**name**: {str}, read-only
    Name of the plugin
**numaxis**: {int}, read-only
    Number of axes
**serialNumber**: {str}, read-only
    Serial number of the device
**speed**: {float}
    speed in real world units (e.g. mm/s)
**timeout**: {float}
    timeout for move operations in sec


Usage
============

This example shows how to initalized the device in **itom** and change the position:

    .. code-block:: python

        serialNo = 2700001
        mot = dataIO("ThorlabsKCubeDCServo", serialNo = serialNo)

        # execute a home drive to get the zero position
        mot.calib(0)

        mot.setParam("speed", 2.5) # 2.5 mm/s

        mot.setPosAbs(0, 25)  # move to the absolute position 25 mm
        mot.setOrigin(0)  # sets the current position to be '0'.
        mot.setPosAbs(0, -25)  # moves back to the original position
        mot.setPosRel(0, 0.01)  # moves by 10 micrometer
        print(mot.getPos(0))  # returns the position of the axis

Simulation
===========

It is possible to test this plugin based on a simulated controller. To do this,
start the Kinesis Simulator first and add a KDC101 DC Servo Drive device. Then,
start the itom plugin and set the initial parameter ``connectToKinesisSimulator``.

Usually, modern Thorlabs controllers can automatically request necessary hardware parameters
from the connected stage. However, if the load-setting command returns with
a warning at startup, this failed and all units are in device units only (not
in physical units). In order to overcome this, especially in simulated environments,
you have to open the simulated stage (using Kinesis Simulator) at least once
with the real Kinesis Software. This software is also able to connect to the
simulator, and there, you can bind a real stage to the controller with a certain
serial ID. After having done this one time on the computer, the itom plugin
should properly load the controller and device with the same serial number.

Compilation
===========

To compile this plugin, install the Thorlabs KINESIS from
https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=Motion_Control&viewtab=0
driver package in the same bit-version than itom (32/64bit).
It has been implemented using KINESIS version 1.14.32.
Then set the CMake variable **THORLABS_KINESIS_DIRECTORY** or the environment variable **THORLABS_KINESIS_ROOT**
to the base directory of Kinesis (e.g. C:/Program Files/Thorlabs/Kinesis).
The required libraries from Kinesis will automatically be copied to the *lib* folder of itom.

Kinesis 1.7.0 requires the Microsoft C++ Redistributable 2012.

Changelog
==========

* itom setup 4.2.0: This plugin has been compiled with Thorlabs Kinesis 1.14.28.
* itom setup 4.3.0: This plugin has been compiled with Thorlabs Kinesis 1.14.35.
