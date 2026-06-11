===================
 Thorlabs ISM
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsISM`
**Type**:       :plugintype:`ThorlabsISM`
**License**:    :pluginlicense:`ThorlabsISM`
**Platforms**:  Windows
**Devices**:    One axis motor controllers of type Thorlabs Integrated Stepper Motors, e.g. K10CR1 Rotation Stage
**Author**:     :pluginauthor:`ThorlabsISM`
=============== ========================================================================================================

Overview
========

ITOM Plugin to be used for interaction with the Thorlabs Integrated Stepper Motors Device Series.

.. pluginsummaryextended::
    :plugin: ThorlabsISM

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: ThorlabsISM

Parameters
===========

These parameters are available and can be used to configure the **ThorlabsISM** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**accel**: {float}
    Target acceleration in °/s^2 (travelMode == 2) or mm/s^2 (travelMode == 1)
**async**: {int}
    asynchronous (1) or synchronous (0) mode
**deviceName**: {str}, read-only
    Description of the device
**enabled**: {int}
    If 1, the axis is enabled and power is applied to the motor. 0: disabled, the motor can
    be turned by hand.
**homed**: {int}, read-only
    1 if actuator is 'homed', else 0
**homingAvailable**: {int}, read-only
    1 if actuator supports a home drive, else 0
**moveCurrent**: {int}
    Percentage of full power to give while moving.
**name**: {str}, read-only
    Name of the plugin
**numaxis**: {int}, read-only
    number of axes (channels)
**restCurrent**: {int}
    Percentage of full power to give while not moving.
**serialNumber**: {str}, read-only
    Serial number of the device
**speed**: {float}
    Target speed in °/s (travelMode == 2) or mm/s (travelMode == 1)
**stagePosMax**: {float}, read-only
    Maximum stage position in mm (travelMode == 1) or ° (travelMode == 2). For °, given
    positions will be wrapped by 360° for absolute moves.
**stagePosMin**: {float}, read-only
    Minimum stage position in mm (travelMode == 1) or ° (travelMode == 2). For °, given
    positions will be wrapped by 360° for absolute moves.
**timeout**: {float}
    timeout for move operations in sec
**travelMode**: {int}, read-only
    travel mode: linear (1), rotational (2), undefined (0)

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

* This plugin is part of itom with version > 2.1.0.
* itom setup 2.2.0: This plugin has been compiled with Thorlabs Kinesis 1.7.0; it requires the Microsoft C++ Redistributable 2012
* itom setup 3.0.0: This plugin has been compiled with Thorlabs Kinesis 1.9.3; it requires the Microsoft C++ Redistributable 2012
* itom setup 3.1.0: This plugin has been compiled with Thorlabs Kinesis 1.14.5; it requires the Microsoft C++ Redistributable 2012
* itom setup 3.2.1: This plugin has been compiled with Thorlabs Kinesis 1.14.15; it requires the Microsoft C++ Redistributable 2012
* itom setup 4.0.0: This plugin has been compiled with Thorlabs Kinesis 1.14.23;
* itom setup 4.1.0: This plugin has been compiled with Thorlabs Kinesis 1.14.25.
* itom setup 4.2.0: This plugin has been compiled with Thorlabs Kinesis 1.14.28.
* itom setup 4.3.0: This plugin has been compiled with Thorlabs Kinesis 1.14.47.
