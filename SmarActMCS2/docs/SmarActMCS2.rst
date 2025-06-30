===================
 SmarActMCS2
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`SmarActMCS2`
**Type**:       :plugintype:`SmarActMCS2`
**License**:    :pluginlicense:`SmarActMCS2`
**Platforms**:  Windows
**Devices**:    SmarAct MCS2 Controller
**Author**:     :pluginauthor:`SmarActMCS2`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: SmarActMCS2

.. note::
    Please note that the System is only tested and developed with ETHERNET connection.

.. note::
    When changing the position of the axes on the controller, itom will not update the position (e.g. in the dock-widget).

Initialization
==============

To use this Plugin, following software is needed: **"MCS2 Software"** (Request it from SmarAct)

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: SmarActMCS2

Parameters
==========

These parameters are available and can be used to configure the ``SmarActMCS2`` instance.
Many of them are directly initialized by the parameters of the constructor.
During the runtime of an instance, the value of these parameters is obtained by
the method ``getParam``, writeable parameters can be changed using ``setParam``.

**acceleration**: Sequence[float]
    Acceleration of each axis, default 100 mm/s^2.
    
    *Allowed number of values: number of channels, Value range: [0:0.001:100], Default: [100, 100, 100, ...]*
**async**: int
    asynchronous move (1), synchronous (0) [default]
    
    *Value range: [0, 1], Default: 0*
**baseUnit**: Sequence[int], read-only
    Baseunit of Channel: (0) for none, (1) for millimeter and (2) for degree.
    
    *Allowed number of values: number of channels, Value range: [0, 1], Default: [0, 0, 0, ...]*
**deviceName**: str, read-only
    Device Name.
    
    *Match: "", Default: "MCS2-00011725"*
**interfaceType**: str, read-only
    Interface Type (USB or ETHERNET).
    
    *Match: "", Default: "ETHERNET"*
**limitLower**: Sequence[float]
    Lower limits of axes.
    
    *Allowed number of values: number of channels, Value range: [-inf:0.001:inf], Default: [-inf, -inf, -inf, ...]*
**limitUpper**: Sequence[float]
    Upper limits of axes.
    
    *Allowed number of values: number of channels, Value range: [-inf:0.001:inf], Default: [inf, inf, inf, ...]*
**name**: str, read-only
    Plugin Name.
    
    *Match: "", Default: "SmarActMCS2"*
**noOfBusModules**: int, read-only
    Number of Bus Modules.
    
    *Value range: [0, inf], Default: 1*
**noOfChannels**: int, read-only
    Number of Channels.
    
    *Value range: [0, inf], Default: 3*
**positionerType**: Sequence[int]
    Positionertype number of Channel according to Smaract Manual.
    
    *Allowed number of values: number of channels, Value range: [0, 10000], Default: [0, 0, 0, ...]*
**positionerTypeName**: Sequence[str], read-only
    Positionertype name of Channel.
**sensorPresent**: Sequence[int], read-only
    Show if sensor is present (1) or not (0).
    
    *Allowed number of values: number of channels, Value range: [0, 1], Default: [0, 0, 0, ...]*
**serialNumber**: str, read-only
    Serial number.
    
    *Match: "", Default: "network:sn:MCS2-00011725"*
**useLimits**: Sequence[int]
    Use axes limits of axis (1) or not (0).
    
    *Allowed number of values: number of channels, Value range: [0, 1], Default: [0, 0, 0, ...]*
**velocity**: Sequence[float]
    Velocity of each axis, default 10 mm/s.
    
    *Allowed number of values: number of channels, Value range: [0:0.001:100], Default: [10, 10, 10, ...]*



Additional functions (exec functions)
=====================================

.. py:function::  instance.exec('SmaractCalibrate' [,axis])

    Perform the SmarAct calibration function.

    :param axis: axis to perform SmarAct calibration
    :type axis: int - optional


Exemplary usage from Python
===========================

In the following examples, it is shown how to use this plugin.
First an instance must be initialized. The plugin will search for all ``SmarActMCS2`` deformable mirrors and will select the first device.

.. code-block:: python

    from itom import actuator

    mot = actuator("SmarActMCS2")

If you want to connect to a specific device, you can indicate the serial number.

.. code-block:: python

    from itom import actuator

    mot = actuator("SmarActMCS2", "---SerialNo.--- (e.g. network:sn:MCS2-00012345)")

The ``position`` of the segments can be set by using the ``setPosAbs``.
In this example the ``position`` of axis [0, 1, 5, 23] are set to [30, 20, 180, 40].
The unit (``mm`` or ``degree``) depends on the type of stage.

.. code-block:: python

    mot.setPosAbs(0, 30, 1, 20, 5, 180, 23, 40)

The current ``position`` of axis 0, 1, 5, 23 can be shown by using ``getPos``.

.. code-block:: python

    mot.getPos(0, 1, 5, 23)

Increment the ``position`` of axis 35 relative about 20:

.. code-block:: python

    mot.getPos(35)
    mot.setPosRel(35, 20)
    mot.getPos(35)

Perform calibration of axis 1:

.. code-block:: python

    mot.calib(1)

Perform SmaractCalibrate execFunction for all axes and for axis 0:

.. code-block:: python

    mot.exec("SmaractCalibrate")
    mot.exec("SmaractCalibrate", 0)

Use Limit for Axis 0 and 1 and set lower and upper limit. Here 3 stages are connected (numper of channels).

.. code-block:: python

    mot.setParam("useLimits", [1, 1, 0])
    mot.setParam("limitLower", [-1.0, 2.1, 0])
    mot.setParam("limitUpper", [1.05, 5.23, 0])

Show the types of all channels of the controller and adapt them to specific values accourding to the manual (MCS2 Positioner Types)

.. code-block:: python

    mot.getParam("positionerType")
    mot.getParam("positionerTypeName")

    mot.setParam("positionerType", [502, 312, 301])


Changelog
==========
