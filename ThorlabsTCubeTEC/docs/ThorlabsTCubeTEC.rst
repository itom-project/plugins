===================
 ThorlabsTCubeTEC
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsTCubeTEC`
**Type**:       :plugintype:`ThorlabsTCubeTEC`
**License**:    :pluginlicense:`ThorlabsTCubeTEC`
**Platforms**:  Windows
**Devices**:    TCube TEC (temperature controller)
**Author**:     :pluginauthor:`ThorlabsTCubeTEC`
=============== ========================================================================================================

Overview
========

ITOM Plugin to be used for interaction with the Thorlabs T-Cube TEC Controller Series.


.. pluginsummaryextended::
    :plugin: ThorlabsTCubeTEC

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: ThorlabsTCubeTEC

Parameters
==========

These parameters are available and can be used to configure the **ThorlabsTCubeTEC** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**currentLimit**: {float}
    The maximum current limit in mA.

    *Value range: [0, 2000], Default: 1000*
**currentTemperature**: {float}, read-only
    The current temperature in °C or kOhm, depending on the sensor type

    *Value range: [-45, 145], Default: -87.39*
**derivativeGain**: {float}
    The derivative gain term for the temperature loop parameters.

    *Value range: [0, 100], Default: 0.183111*
**deviceName**: {str}, read-only
    description of the device
**firmwareVersion**: {int}, read-only
    firmware version of the connected device
**integralGain**: {float}
    The integral gain term for the temperature loop parameters.

    *Value range: [0, 100], Default: 0.0122074*
**name**: {str}, read-only
    name of the plugin
**pollingInterval**: {int}, read-only
    device polling interval in ms

    *Value range: [1, 10000], Default: 200*
**proportionalGain**: {float}
    The proportional gain term for the temperature loop parameters.

    *Value range: [1, 100], Default: 0.0457778*
**sensorType**: {str}, read-only
    Connected sensor type.
**serialNumber**: {str}, read-only
    serial number of the device
**softwareVersion**: {int}, read-only
    software version of the connected device
**targetTemperature**: {float}
    The target temperature in °C or kOhm, depending on the sensor type.

    *Value range: [-45, 145], Default: -87.39*
**enableControl**: {int}
    Enable (1) or disable (0) cube for computer control.

    *Value range: [0, 1], Default: 1*

Installation
============

Install the Thorlabs Kinesis software and USB Drivers.

Usage
============

This example shows how to initialized the device in **itom** and change the position:

    .. code-block:: python

        # create a new instance of the device
        tec = dataIO("ThorlabsTCubeTEC", serialNo="87000001", sensorType="Transducer")
        tec.setParam("targetTemperature", 30.0) # set the target to 30°C


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
=========

* itom setup 3.2.1: This plugin has been compiled with Thorlabs Kinesis 1.14.15; it requires the Microsoft C++ Redistributable 2012
* itom setup 4.0.0: This plugin has been compiled with Thorlabs Kinesis 1.14.23;
* itom setup 4.1.0: This plugin has been compiled with Thorlabs Kinesis 1.14.25.
* itom setup 4.2.0: This plugin has been compiled with Thorlabs Kinesis 1.14.28.
* itom setup 4.3.0: This plugin has been compiled with Thorlabs Kinesis 1.14.47.
