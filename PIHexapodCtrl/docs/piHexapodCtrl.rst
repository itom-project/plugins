===============
 PIHexapodCtrl
===============

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PIHexapodCtrl`
**Type**:       :plugintype:`PIHexapodCtrl`
**License**:    :pluginlicense:`PIHexapodCtrl`
**Platforms**:  Windows, Linux
**Devices**:    6-axis Hexapod from Physik Instrumente
**Author**:     :pluginauthor:`PIHexapodCtrl`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: PIHexapodCtrl


Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: PIHexapodCtrl

Parameters
==========

These parameters are available and can be used to configure the **PIHexapodCtrl** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**PI_CMD**: {str}
    use this parameter followed by :YourCommand in order to read/write value from/to device (e.g. PI_CMD:ERR?)
**async**: {int}
    asychronous (1.0) or sychronous (0.0) mode
**axesNames**: {str}, read-only
    semicolon-separated list of the names of all connected axes
**identifier**: {str}, read-only
    Identifier string of device (\*idn?)
**name**: {str}, read-only
    PIHexapodCtrl
**numaxis**: {int}, read-only
    Number of axes attached to this stage (see axesNames for names of axes)
**pivotPoint**: {seq. of float}
    current pivot point (x,y,z)
**speed**: {float}
    speed of every axis in physical units / s

Additional functions
====================

.. py:function::  instance.exec('getPivotPoint')

    Get the system Pivot-Point (origin of rotation)

    :return: xPosition - Position of the Pivot-Point in x
    :rtype: float
    :return: yPosition - Position of the Pivot-Point in y
    :rtype: float
    :return: zPosition - Position of the Pivot-Point in z
    :rtype: float

.. py:function::  instance.exec('setPivotPoint', xPosition, yPosition, zPosition)

    Set the system Pivot-Point (origin of rotation)

    :param xPosition: Position of the Pivot-Point in x
    :type xPosition: float
    :param yPosition: Position of the Pivot-Point in y
    :type yPosition: float
    :param zPosition: Position of the Pivot-Point in z
    :type zPosition: float

.. py:function::  instance.exec('beFunny', cycles, amplitude, timeconstant)

    Print the current positions of the specified axis to the command line

    :param cycles: Cycle to iterate
    :type cycles: int
    :param amplitude: Amplitude in mm
    :type amplitude: float
    :param timeconstant: Wait between to command in seconds
    :type timeconstant: float

Example
========

.. code-block:: python

    if not "mot" in globals():
        mot = actuator("PIHexapodCtrl", IP="129.69.65.1", Port=50000)

    mot.setParam("speed", 1.5)
    print("speed:", mot.getParam("speed"))

    mot.setPosAbs(1, 5, 2, -5) #move y to 5mm and z to -5mm
    print("current position of axes x, y, z:", mot.getPos(0,1,2))

    #in order to set pivot point, tilts must be zero:
    mot.setPosAbs(3,0,4,0,5,0)
    print("pivot point(x,y,z):", mot.getParam("pivotPoint"))
    print("set pivot point to (2,2,2):")
    mot.setParam("pivotPoint", (2,2,2))

    print("set pivot point to (0,0,0):")
    mot.setParam("pivotPoint", (0,0,0))

    print("ask for last question (via PI GCS):")
    print(mot.getParam("PI_CMD:ERR?"))

    print("get identifier (via PI GCS):")
    print(mot.getParam("PI_CMD:*IDN?"))



Changelog
===========

* itom > 3.0.0: PIHexapodCtrl published for the first time
