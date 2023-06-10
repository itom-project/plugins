===================
 DummyMotor
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`DummyMotor`
**Type**:       :plugintype:`DummyMotor`
**License**:    :pluginlicense:`DummyMotor`
**Platforms**:  Windows, Linux
**Devices**:    Virtual dummy motor with up to 6 axes.
**Author**:     :pluginauthor:`DummyMotor`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: DummyMotor

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: DummyMotor

Parameters
===========

An instance of this plugin has the following parameters:

**accel**: float
    Acceleration in mm/s^2, currently not implemented

    *Value range: [1, 10], Unit: mm/s^2, Default: 1*
**async**: int
    Toggles if motor has to wait until end of movement (0:sync) or not (1:async)

    *Value range: [0, 1], Default: 0*
**limitNeg**: Sequence[float]
    negative limits of axes

    *1 values required, Value range: [-1e+208, 1e+208], Default: [-1000]*
**limitPos**: Sequence[float]
    positive limits of axes

    *1 values required, Value range: [-1e+208, 1e+208], Default: [1000]*
**name**: str, read-only
    name of the plugin

    *Match: "General", Default: "DummyMotor"*
**numaxis**: int, read-only
    Number of axes attached to this stage

    *Value range: [1, 6], Default: 1*
**speed**: float
    Speed of the axis between 0.1 and 100000 mm/s

    *Value range: [0.1, 100000], Unit: mm/s, Default: 1*
**useLimits**: Sequence[int]
    Use axes limits and limit switches

    *1 values required, Value range: [0, 1], Default: [0]*
