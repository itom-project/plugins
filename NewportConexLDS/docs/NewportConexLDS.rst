===================
 NewportConexLDS
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`NewportConexLDS`
**Type**:       :plugintype:`NewportConexLDS`
**License**:    :pluginlicense:`NewportConexLDS`
**Platforms**:  Some words about supported operating systems
**Devices**:    Some words about supported devices
**Author**:     :pluginauthor:`NewportConexLDS`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: NewportConexLDS

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: NewportConexLDS

Parameters
==========

These parameters are available and can be used to configure the ``NewportConexLDS`` instance. Many of them are directly initialized by the parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable parameters can be changed using *setParam*.

**calibrationCoefficients**: Sequence[float]
    Calibration coefficients of x and y axis.

    *Allowed number of values: 0 - 2, step: 2, Value range: [0, inf], Default: [5325.69,
    5393]*
**commandError**: str, read-only
    Command error string.

    *Match: "", Default: "No error"*
**configurationState**: str
    Configuration state (MEASURE, READY, CONFIGURATION).

    *Match: ["", "MEASURE", "READY", "CONFIGURATION"], Default: "CONFIGURATION"*
**deviceName**: str, read-only
    Device name.

    *Match: "", Default: "CONEX-LDS"*
**factoryCalibrationState**: str, read-only
    Factory calibration information.

    *Match: "", Default: "14-214-
    009;02/05/2022;02/05/2023;5325.69;5393.00;16.0;19.0;31.0;29.0;-62.00;26.00;2000.0"*
**frequency**: float
    Low pass filter frequency.

    *Value range: [0:0.2:inf], Default: 0.2*
**gain**: Sequence[float]
    Gain of x and y axis.

    *Allowed number of values: 0 - 18446744073709551615, Value range: [0, 200], Default:
    [200, 4]*
**highLevelPowerThreshold**: int
    High level power threshold for valid measurement.

    *Value range: [0, 2000], Default: 95*
**laserPowerState**: int
    Enable/Disable laser power (0==OFF, 1==ON).

    *Value range: [0, 1], Default: 0*
**lowLevelPowerThreshold**: int
    Low level power threshold for valid measurement.

    *Value range: [0, 2000], Default: 10*
**name**: str, read-only

**offset**: Sequence[float]
    Offset values of x and y axis.

    *Allowed number of values: 0 - 18446744073709551615, All values allowed, Default: [1, -5]*
**range**: int
    Value range.

    *Value range: [0, inf], Default: 2000*
**requestTimeout**: int
    Request timeout in ms for the SerialIO interface.

    *Value range: [0, inf], Default: 5000*
**unit**: str
    Measurement unit.

    *Match: "", Default: "URAD"*
**version**: str, read-only
    Controller version.

    *Match: "", Default: "1.1.1"*

Additional functions (exec functions)
=======================================

By using the following execFunctions you set the channels parameter by giving a list of channel number and a list of parameter values of same list length.
The plugin execFunctions are:

.. py:function::  instance.exec('getPositionAndPower', )

    Measure the position and laser power.

    :return: positionAndPower - Positions of x and y axis.
    :rtype: Sequence[float]
    :return: timeStemp - Timestemp of measurement.
    :rtype: str

.. py:function::  instance.exec('getPositionAndPowerMeasurement', data [,interval])

    Measure the position and laser power. It will fill the input dataObject with positions, laser power and timestemps.
... Please note that this function blocks itom until the entire measurement has been carried out.

    :param data: Measruement data X, Y, position and laser power.
    :type data: itom.dataObject
    :param interval: Interval between measruement points in ms.
    :type interval: int - optional
    :return: timeStemps - Timestemps corresponding to the measruement data.
    :rtype: Sequence[str]

Exemplary usage from Python
=======================================


Changelog
==========
