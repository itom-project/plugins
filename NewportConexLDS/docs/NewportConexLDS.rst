===================
 NewportConexLDS
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`NewportConexLDS`
**Type**:       :plugintype:`NewportConexLDS`
**License**:    :pluginlicense:`NewportConexLDS`
**Platforms**:  Windows, Linux
**Devices**:    Newport Conex-LDS
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
**configurationState**: str, read-only
    Configuration state (MEASURE, READY, CONFIGURATION).

    *Match: ["", "MEASURE", "READY", "CONFIGURATION"], Default: "MEASURE"*
**deviceName**: str, read-only
    Device name.

    *Match: "", Default: "CONEX-LDS"*
**enableConfiguration**: int
    Enable/Disable configuration (0==OFF, 1==ON). Laser is disabled when it is on.

    *Value range: [0, 1], Default: 0*
**factoryCalibrationState**: str, read-only
    Factory calibration information.

    *Match: "", Default: "14-214-
    009;02/05/2022;02/05/2023;5325.69;5393.00;16.0;19.0;31.0;29.0;-62.00;26.00;2000.0"*
**frequency**: float
    Low pass filter frequency as response time before ouputing measurement that is inversely
    proportional to the low pass filter frequency. Following frequencies [Hz] corresponds to
    a resolution [µrad] (RMS noise): 1 == 0.03, 20 == 0.013, 50 == 0.021, 100 == 0.030, 200
    == 0.042, 500 == 0.067, 1000 == 0.095, 2000 == 0.134.

    *Value range: [0:0.2:inf], Default: 0.2*
**gain**: Sequence[float]
    Gain of x and y axis.

    *Allowed number of values: 0 - 2, step: 2, Value range: [0, 200], Default: [1, 1]*
**highLevelPowerThreshold**: int
    High level power threshold for valid measurement.

    *Value range: [0, 2000], Default: 100*
**laserPowerState**: int
    Enable/Disable laser power (0==OFF, 1==ON).

    *Value range: [0, 1], Default: 1*
**lowLevelPowerThreshold**: int
    Low level power threshold for valid measurement.

    *Value range: [0, 2000], Default: 10*
**name**: str, read-only
    Plugin name.
**offset**: Sequence[float]
    Offset values of x and y axis.

    *Allowed number of values: 0 - 2, step: 2, All values allowed, Default: [-4.3, 5.4]*
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
    :return: timeStamp - Timestamp of measurement.
    :rtype: str

.. py:function::  instance.exec('getPositionAndPowerMeasurement', data [,interval])

    Measure the position and laser power. It will fill the input dataObject with positions, laser power and timestamps.
    ... Please note that this function blocks itom until the entire measurement has been carried out.

    :param data: Measruement data X, Y, position and laser power.
    :type data: itom.dataObject
    :param interval: Interval between measruement points in ms.
    :type interval: int - optional
    :return: timestamps - Timestamps corresponding to the measruement data.
    :rtype: Sequence[str]

Exemplary usage from Python
=======================================

In the following examples, it is shown how to use this Plugin.
First an instance must be initialized using the ``SerialIO`` Plugin.
COM port must be adapted.

.. code-block:: python

    from datetime import datetime
    from itom import dataIO
    import numpy as np

    comPort = 5
    try:
        conex
    except NameError:
        serial = dataIO("SerialIO", comPort, 921600, "\r\n", enableDebug=True)
        conex = dataIO("NewportConexLDS", serial)

    conex.getParamListInfo()

Laser must be disabled before parameter can be set using ``setParam``.
Before changing parameter the ``configurationState`` must be enabled using ``enableConfiguration``.

.. code-block:: python

    # Disable laser
    conex.setParam("laserPowerState", 0)

    # To set parameter the configuration state must be 'CONFIGURATION'
    conex.setParam("enableConfiguration", 1)

    conex.setParam("frequency", 20.0)
    conex.setParam("offset", (-4.3, 5.4))
    conex.setParam("gain", (1.0, 1.0))
    conex.setParam("lowLevelPowerThreshold", 10)
    conex.setParam("highLevelPowerThreshold", 100)
    conex.setParam("frequency", 2000)
    conex.setParam("range", 2000)

    # get parameters
    conex.getParam("configurationState")
    conex.getParam("frequency")
    conex.getParam("offset")
    conex.getParam("gain")
    conex.getParam("lowLevelPowerThreshold")
    conex.getParam("highLevelPowerThreshold")
    conex.getParam("range")
    conex.getParam("unit")

Before the laser can be turn ON ``configurationState`` must be disabled.

.. code-block:: python

    # Leave configuration mode. Configuration must be OFF before laser can be set ON.
    conex.setParam("enableConfiguration", 0)

    # Get all parameter information
    conex.getParamListInfo()

    # Enable laser
    conex.setParam("laserPowerState", 1)


A measurement is performed using the execFunction ``getPositionAndPower``.
In this example 100 data points are acquired.

.. code-block:: python

    # Perform a measurement of 10 data points
    data = []
    timeStamps = []
    for _idx in range(100):
        values, timeStamp = conex.exec("getPositionAndPower")
        data.append(values)
        timeStamps.append(timeStamp)

Create plot with measured data over time.

.. code-block:: python

    # convert to dataObject and set metainformation
    data = dataObject(np.array(data).transpose())
    data.setTag("legendTitle0", "x axis")
    data.setTag("legendTitle1", "y axis")
    data.setTag("legendTitle2", "level")

    # convert timeStamps string into datetime
    timeStamps = [datetime.strptime(ts, "%Y-%m-%d %H:%M:%S.%f") for ts in timeStamps]
    dateScale = dataObject([1, len(timeStamps)], "datetime", data=timeStamps)

    # plot Measurement over time
    [i, h] = plot1(data, dateScale, properties={"legendPosition": "Right", "lineWidth": 2, "grid": "GridMajorXY"})

Create 2 dimensional plot of data. First data points below a laser power level of 10% are deleted which are no valid.

.. code-block:: python

    # filter data by laser levels below 1%
    position = np.array(data.copy())
    columsToRemove = np.where(position[2] <= 10)[0]
    position = np.delete(position, columsToRemove, axis=1)

    x = position[0, :]
    y = position[1, :]
    plot1(y, x, properties={"lineSymbol": "Ellipse", "lineStyle": "NoPen", "grid": "GridMajorXY"})


Changelog
==========

* itom setup 4.3.0 - v1.1.0: Renamed timeStemp to timeStamp
