=========================
 Thorlabs Benchtop Piezo
=========================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsBP`
**Type**:       :plugintype:`ThorlabsBP`
**License**:    :pluginlicense:`ThorlabsBP`
**Platforms**:  Windows
**Devices**:    One or multi-axis piezo controllers of type Thorlabs Benchtop Piezo
**Author**:     :pluginauthor:`ThorlabsBP`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: ThorlabsBP

ITOM Plugin to be used for interaction with the Thorlabs Benchtop Piezo Driver Series.

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: ThorlabsBP

Parameters
===========

These parameters are available and can be used to configure the **ThorlabsBP** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**async**: {int}
    asynchronous (1) or synchronous (0) mode
**channel**: {seq. of int}, read-only
    Channel number of each axis.
**controlMode**: {seq. of int}
    Open loop (0), closed loop (1)
**deviceName**: {str}, read-only
    Description of the device
**enabled**: {seq. of int}
    If 1, the axis is enabled and power is applied to the motor. 0: disabled, the motor can be turned by hand.
**hasFeedback**: {seq. of int}, read-only
    If 0, the axis is not equipped with a strain gauge feedback, else 1.
**maximumTravelRange**: {seq. of float}, read-only
    Maximum travel range for each axis in mm. This requires an actuator with built in position sensing. These values might not be correct if the motor is in open loop mode.
**maximumVoltage**: {seq. of int}
    Maximum output voltage (75, 100 or 150 V).
**name**: {str}, read-only
    name of plugin
**numaxis**: {int}, read-only
    number of axes (channels)
**serialNumber**: {str}, read-only
    Serial number of the device
**slewRateClosedLoop**: {seq. of float}
    Speed limit (slew rate) of each axis in V/ms. 0 (default) disables the limit and let the controller decide to move with the ideal speed.
**slewRateOpenLoop**: {seq. of float}
    Speed limit (slew rate) of each axis in V/ms. 0 (default) disables the limit and let the controller decide to move with the ideal speed.
**timeout**: {float}
    Timeout for positioning in seconds.
**zeroed**: {seq. of int}, read-only
    If 0, the axis is not zeroed. 1: zeroed. If the axis is not zeroed, it is possible that position values at the edge of the valid range can not be reached.

.. note::

    the 'maximumTravelRange' can only be determined if the corresponding axis is in closed loop mode. The closed loop mode can only be set for axis with
    a position feedback.

Many parameters are sequences. The length of the sequence must always correspond to the number of axes, such that the first value in the sequence determines
the parameter for the first connected axis, the second is responsible for the 2nd axis...

Control mode
=============

Each axis can be set to a closed-loop or open-loop control mode (see parameter *controlMode*). If the axis is operated in closed-loop (only possible if a feedback line
is connected to the controller) the position values are given in 'mm'. In open-loop, the set-point and hence target position is always given as voltage value ('V'), such
that the commands **getPos**, **setPosAbs** and **setPosRel** all given or except values in 'V'.

Target position
===============

The controller itself does not provide information if the set-point position has been reached. Therefore, this decision is taken dependent on the control mode
of each axis in the following way:

* Closed loop: The target position is considered to be reached if the difference of the current and the desired target position is below 50nm AND the difference between two
    current position values (registered with a delay of approximately 120ms) is below 10nm.
* Open loop: The target position is considered to be reached if the difference of the current and the desired target voltage is below 0.05V AND the difference between two
    current voltage values (registered with a delay of approximately 120ms) is below 0.01V.

Compilation
===========

To compile this plugin, install the Thorlabs KINESIS from
https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=Motion_Control&viewtab=0
driver package in the same bit-version than itom (32/64bit).
It has been implemented using KINESIS version 1.14.32.
Then set the CMake variable **THORLABS_KINESIS_DIRECTORY** or the environment variable **THORLABS_KINESIS_ROOT**
to the base directory of Kinesis (e.g. C:/Program Files/Thorlabs/Kinesis).
The required libraries from Kinesis will automatically be copied to the *lib* folder of itom.
Do not use Kinesis 1.6.0 or below for compiling this plugin.

Kinesis 1.7.0 requires the Microsoft C++ Redistributable 2012.

Changelog
==========

* This plugin is part of itom with version > 2.1.0.
* itom setup 2.2.0: This plugin has been compiled with Thorlabs Kinesis 1.7.0; it requires the Microsoft C++ Redistributable 2012
* itom setup 3.0.0: This plugin has been compiled with Thorlabs Kinesis 1.9.3; it requires the Microsoft C++ Redistributable 2012
* itom setup 3.1.0: This plugin has been compiled with Thorlabs Kinesis 1.14.5; it requires the Microsoft C++ Redistributable 2012
* itom setup 3.2.1: This plugin has been compiled with Thorlabs Kinesis 1.14.15; it requires the Microsoft C++ Redistributable 2012
* itom setup 4.0.0: This plugin has been compiled with Thorlabs Kinesis 1.14.23;
* itom setup 4.1.0: This plugin has been compiled with Thorlabs Kinesis 1.14.25.
* itom setup 4.2.0: This plugin has been compiled with Thorlabs Kinesis 1.14.28.
* itom setup 4.3.0: This plugin has been compiled with Thorlabs Kinesis 1.14.47.
