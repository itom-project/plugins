===================
 GWInstekPSP
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`GWInstekPSP`
**Type**:       :plugintype:`GWInstekPSP`
**License**:    :pluginlicense:`GWInstekPSP`
**Platforms**:  Windows, Linux
**Devices**:    Power supplies PSP-405, PSP-603, PSP-2010 of company GWInstek
**Author**:     :pluginauthor:`GWInstekPSP`
**Requires**:   Plugin SerialIO
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: GWInstekPSP

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: GWInstekPSP

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    GWInstekPSP
**status**: {str}, read-only
    Current status string of controller
**voltage**: {float}
    Output voltage; the unit: V
**current**: {float}, read-only
    Output current; the unit: A
**load**: {float}, read-only
    Output load; the unit: W
**voltage_limit**: {float}
    Output voltage limit; the unit: V
**current_limit**: {float}
    Output current limit; the unit: A
**load_limit**: {float}
    Output load limit; the unit: W
**save**: {int}
    Save the present status to the EEPROM on exit
**relay**: {int}
    Relay status 0: off, 1: on
**temperature**: {int}, read-only
    Temperature status 0: normal, 1: overheat
**wheel**: {int}
    Wheel knob 0: normal, 1: fine
**wheel_lock**: {int}, read-only
    Wheel knob 0: lock, 1: unlock
**remote**: {int}, read-only
    Remote status 0: normal, 1: remote
**lock**: {int}, read-only
    Lock status 0: lock, 1: unlock
