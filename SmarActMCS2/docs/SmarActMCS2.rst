===================
 MyActuator
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`MyActuator`
**Type**:       :plugintype:`MyActuator`
**License**:    :pluginlicense:`MyActuator`
**Platforms**:  Some words about supported operating systems
**Devices**:    Some words about supported devices
**Author**:     :pluginauthor:`MyActuator`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: SmarActMCS2

.. note::
    Please note that the System is only tested and developed with ETHERNET connection.

Initialization
==============

To use this Plugin, following software is needed: **"MCS2 Software"** (Request it from SmarAct)

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: MyActuator

Parameters
==========

These parameters are available and can be used to configure the ``SmarActMCS2`` instance.
Many of them are directly initialized by the parameters of the constructor.
During the runtime of an instance, the value of these parameters is obtained by
the method ``getParam``, writeable parameters can be changed using ``setParam``.

**async**: int, read-only
    Toggles if motor has to wait until end of movement (0: sync) or not (1: async). Only
    sync supported.

    *Value range: [0, 1], Default: 0*

**serialNumber**: String, read-only
    Serial number of device (e.g.: network:sn:MCS2-00012345")

    *Default: unknown*

**deviceName**: String, read-only
    Name of connected device (e.g.: MCS2-00012345")

    *Default: unknown*

**interfaceType**: String, read-only
    Interface type of connected device (USB or ETHERNET)
    (Please note that the System is only tested and developed with ETHERNET connection)

    *Default: unknown*

**noOfBusModules**: int, read-only
    Number of Bus-Modules of connected Controller.

    *Default: 0*

**noOfChannels**: int, read-only
    Number of Channels of all Bus-Modules of connected Controller.
    (is equal to the number of axes. Please note that the SmarAct MCS2 uses the number of channels for all available channels of the bus modules. If you want to know whether a stage is connected or not, use the parameter ``sensorPresent``)

    *Default: 0*

**velocity**: DoubleArray
    Velocity of each axis (in mm/s).

    *Value range: [0, 100], Default: 10*

**acceleration**: DoubleArray
    Acceleration of each axis (in mm/s^2).

    *Value range: [0, 100], Default: 100*

**baseUnit**: IntArray, read-only
    Baseunit of each axis: (0) for none, (1) for millimeter and (2) for degree.

    *Value range: [0, 2], Default: 0*

**sensorPresent**: IntArray, read-only
    Shows fow each axis if sensor (stage) is present (1) or not (0).

    *Value range: [0, 1], Default: 0*

**useLimits**: IntArray
    Use axes limits of axis (1) or not (0).

    *Value range: [0, 1], Default: 0*

**limitUpper**: DoubleArray
    Upper limits of axes.

    *Default: 0*

**limitLower**: DoubleArray
    Lower limits of axes.

    *Default: 0*



Additional functions (exec functions)
=====================================

.. py:function::  instance.exec('SmaractCalibrate', axis)

    Perform the SmarAct calibration function.

    :param axis: axis to perform SmarAct calibration (optional)
    :type axis: int


Exemplary usage from Python
===========================

In the following examples, it is shown how to use this plugin.
First an instance must be initialized. The plugin will search for all ``SmarActMCS2`` deformable mirrors and will select the first device.

.. code-block:: python

    from itom import actuator

    mot = mot = actuator("SmarActMCS2")

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

    mot.setParam("useLimits", [1, 1, 0])
    mot.setParam("limitLower", [-1.0, 2.1, 0])
    mot.setParam("limitUpper", [1.05, 5.23, 0])

Changelog
==========
