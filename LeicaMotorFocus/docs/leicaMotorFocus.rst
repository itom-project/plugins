===================
 LeicaMotorFocus
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`LeicaMotorFocus`
**Type**:       :plugintype:`LeicaMotorFocus`
**License**:    :pluginlicense:`LeicaMotorFocus`
**Platforms**:  Windows, Linux
**Devices**:    z-stage controller of Leica MZ12 or MZ12.5
**Author**:     :pluginauthor:`LeicaMotorFocus`
**Requires**:   Plugin SerialIO
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: LeicaMotorFocus

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}
    LeicaMotorFocus
**speed**: {float}
    Speed in m/s (Default=Maximum: 23,33 mm/s)
**ratio**: {int}
    Sensitivity of Handwheel. From 1 (fine) to 32 (coarse) (Default: 8)
**inverseAxis**: {int}
    0: actuator moves upwards for positive relative position, 1: actuator moves downwards. (default: 0)
**inverseRefSwitch**: {int}
    0: actuator uses upper reference switch, 1: actuator uses lower reference switch for calibration. (default: 0)
**numaxis**: {int}, read-only
    Number of axis @ device in ito-version is always 1

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: LeicaMotorFocus
