========================
 Nanotec Stepper Motor
========================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`NanotecStepMotor`
**Type**:       :plugintype:`NanotecStepMotor`
**License**:    :pluginlicense:`NanotecStepMotor`
**Platforms**:  Windows
**Devices**:    Motor-Controller Nanotec SMCP
**Author**:     :pluginauthor:`NanotecStepMotor`
=============== ========================================================================================================

Overview
========

The plugin implements the controllers SMCP from Nanotec.

.. pluginsummaryextended::
    :plugin: NanotecStepMotor

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: NanotecStepMotor

Parameters
==========

These parameters are available and can be used to configure the **NanotecStepMotor** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**accel**: {seq. of int}
    motor shaft acceleration, range: 1..65.535 [default 2.364]
**async**: {int}
    asychronous (1) or sychronous (0) [default] mode
**axisID**: {seq. of int}
    internal ID of axis
**axisSteps**: {seq. of float}, read-only
    number of full steps per unit (deg or mm) of axis, 0: axis not connected [default]
**coilCurrent**: {seq. of int}
    coil current if motor is running, range: 0..100
**coilCurrentRest**: {seq. of int}
    coil current if motor is in rest, range: 0..100
**decel**: {seq. of int}
    motor shaft deceleration, range: 1..65.535 [default 2.364]
**deviceID**: {str}, read-only
    Name of controller
**devicePort**: {int}, read-only
    Serial port of device
**microSteps**: {seq. of int}
    micro steps for motor [1, 2, 4, 5, 8, 10, 16, 32, 64]
**name**: {str}, read-only
    NanotecStepMotor
**numaxis**: {int}, read-only
    Number of axis
**speed**: {seq. of int}
    target speed range: 1..1.000.000 [default 1.000]
**units**: {seq. of int}, read-only
    unit of axis, 0: degree [default], 1: mm

Usage
======

In order to open an instance of **NanotecStepMotor**, create an instance of **serialIO** first. Then pass this
instance to the constructor of **NanotecStepMotor**. The motor plugin configures the serial connection with the
right parameter set.

At the time of the initialization you have to indicate which Nanotec axis IDs should be controlled by this
plugin. For each connected axis, you further have to indicate the number of full steps per unit, whereas the unit
can be either in mm or degree.

In the following example, the axes 1 and 4 are connected (controllable by the plugin axis numbers 0 and 1 respectively).
They both run at 10 full steps per mm (1) (hence: linear axes):

ser = dataIO("SerialIO",4,115200,"\r",8,1)
mot = actuator("NanotecStepMotor",ser,[1,4],[10,10],[1,1])
