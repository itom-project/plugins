===================
ThorlabsElliptec
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsElliptec`
**Type**:       :plugintype:`ThorlabsElliptec`
**License**:    :pluginlicense:`ThorlabsElliptec`
**Platforms**:  Windows, Linux
**Devices**:    Thorlabs Elliptec devices
**Author**:     :pluginauthor:`ThorlabsElliptec`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: ThorlabsElliptec

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: ThorlabsElliptec


Parameters
==========

These parameters are available and can be used to configure the ``ThorlabsElliptec`` instance.
Many of them are directly initialized by the parameters of the constructor.
During the runtime of an instance, the value of these parameters is obtained by
the method *getParam*, writeable parameters can be changed using *setParam*.

**address**: int
    Address of stage, 0x0 - 0xF. The address can be changed by setting the parameter later
    on.
    
    *Value range: [0x0, 0xf], Default: 0x0*
**async**: int
    Toggles if motor has to wait until end of movement (0:sync) or not (1:async)
    
    *Value range: [0, 1], Default: 0*
**autoSaveSettings**: int
    If 1, motor frequency settings (search, optimization, manual adjustment...) are
    automatically stored to the device. See also the 'saveUserData' exec function.
    
    *Value range: [0, 1], Default: 1*
**axisType**: int, read-only
    Axis type: -1: indexed, 0: rotatory, 1: linear
    
    *Value range: [-1, 1], Default: 0*
**backwardFrequency1**: int
    The backward frequency for the first motor in Hz
    
    *Value range: [4, inf], Unit: Hz, Default: 99595*
**backwardFrequency2**: int
    The backward frequency for the second motor in Hz (if available)
    
    *Value range: [4, inf], Unit: Hz, Default: 98926*
**calibDirection**: int, read-only
    The direction for the calib / homeing operation. 0: clockwise, 1: counter-clockwise.
    Only relevant for rotary stages.
    
    *Value range: [0, 1], Default: 0*
**comPort**: int, read-only
    
    *Value range: [1, inf], Default: 11*
**description**: str, read-only
    
    *Match: "General", Default: "Rotation Stage"*
**forwardFrequency1**: int
    The forward frequency for the first motor in Hz
    
    *Value range: [4, inf], Unit: Hz, Default: 80989*
**forwardFrequency2**: int
    The forward frequency for the second motor in Hz (if available)
    
    *Value range: [4, inf], Unit: Hz, Default: 81889*
**model**: str, read-only
    
    *Match: "General", Default: "ELL18"*
**name**: str, read-only
    
    *Match: "General", Default: "ThorlabsElliptec"*
**numMotors**: int, read-only
    number of piezo actuators to move the stage
    
    *Value range: [0, 3], Default: 2*
**numaxis**: int, read-only
    Number of axes attached to this stage: Here always 1. Multiple axes, connected to one
    bus driver, must init multiple objects with the same serial object.
    
    *Value range: [1, 1], Default: 1*
**pulsesPerUnit**: int, read-only
    Pulses per unit (mm or deg) / -1 for indexed position devices
    
    *Value range: [-1, inf], Default: 398*
**serial**: str, read-only
    
    *Match: "Communication", Default: "11800036"*
**travelRange**: int, read-only
    Travel range of the axis
    
    *All values allowed, Unit: ° or mm, Default: 360*


Additional functions (exec functions)
=====================================

.. py:function::  instance.exec('cleanMechanics')

    Cleans the mechanics by applying forward and backward runs of the full travel range. This operation might take several minutes (e.g. 30min). It can be interrupted by the Keyboard Interrupt.


.. py:function::  instance.exec('optimizeMotors')

    Fine tunes the frequency search for forward and backward direction. At first applies a search frequency run for coarse optimization, then starts the fine tuning. This operation might take several 
... minutes (e.g. 30min). It can be interrupted by the Keyboard Interrupt.


.. py:function::  instance.exec('resetDefaults')

    Reset all frequencies to their default values.


.. py:function::  instance.exec('saveUserData')

    Save motor parameters like forward or backward frequency.


.. py:function::  instance.exec('searchFrequencies', motorIndex)

    Requests a frequency search to optimize the operating frequencies for backward and forward movement of the indicated motor.

    :param motorIndex: Motor Index (0 or 1)
    :type motorIndex: int

Exemplary usage from Python
===========================

.. code-block:: python
    
    # initialization of the device at its internal address 0x0.
    # Address can be in the range 0x0-0xF.
    serial = dataIO("SerialIO", port=1, baud=9600, endline="\r\n")
    elliptec = actuator("ThorlabsElliptec", serial, address=0x0)

    # to home the device, call the calib method for axis 0.
    elliptec.calib(0)

    # get the position (either in ° or mm, depending on the type of device.)
    print("The current position is", elliptec.getPos(0))

    # move the device relatively or absoluty
    elliptec.setPosRel(0, 15)
    elliptec.setPosAbs(0, 42)

    # for tuning the frequencies there are the following parameters:
    # * forwardFrequency1 in Hz
    # * backwardFrequency1 in Hz
    # * for stages with two motors:
    # * forwardFrequency2 and backwardFrequency2
    elliptec.setParam("forwardFrequency1", 89000)

    if elliptec.getParam("numMotors") > 1:
        elliptec.setParam("forwardFrequency2", 89000)

    # the frequency will be adjusted to the next allowed value when it is changed.

    # Depending on the type of stage, several additional methods are available 
    # as 'exec' functions. They can mainly be used to clean the mechanics or 
    # automatically tune the frequencies. Cleaning the motor of optimizing the 
    # frequencies might take very long (20-40min):

    # reset all frequencies to their defaults
    elliptec.exec("resetDefaults")

    # search a good set of frequency values for motor index 0 or 1:
    elliptec.exec("searchFrequencies", 0) # or 1

    # cleaning the mechanics or use a cleaning run to finally optimize the frequencies
    elliptec.exec("cleanMechanics")
    elliptec.exec("optimizeMotors")

    # interrupting the cleaning and optimization runs can be done by triggering 
    # the KeyboardInterrupt of Python.

Changelog
==========

* itom setup 5.0.0 - v1.0.0: Initial version
