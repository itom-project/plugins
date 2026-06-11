===================
 Aerotech A3200
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`AerotechA3200`
**Type**:       :plugintype:`AerotechA3200`
**License**:    :pluginlicense:`AerotechA3200`
**Platforms**:  Windows 32bit
**Devices**:    Aerotech A3200
**Author**:     :pluginauthor:`AerotechA3200`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: AerotechA3200

Parameters
==========

These parameters are available and can be used to configure the **AerotechA3200** instance.

**name**: {str}, read-only
    name of the plugin (*AerotechA3200*)
**numAxis**: {int}, read-only
    number of connected axes
**async**: {int}
    asynchronous move (1) or synchronous (0) [default]. In synchronous mode, the move command in Python waits until the movement has been finished
**scaleFactor**: {double}, read-only
    scale factor of connected controller, counts per metric unit
**clearoffset**: {double}, read-only
    reset the offsets to zero, back to absolute coordinate (offset is introduced when executing setOrigin at a certain position)
**finished**: {int}, read-only
    check if the motion of every axis is finished
**xenabled**, **yenabled**, **zenabled**: {int}
    enables/disables the x,y or z axis
**speed**: {double-array}, 3 values
    speed of every axis
**acknowledge**: {int}
    set this parameter to 0 to acknowledge the errors of connected controller
**stop**: {int}
    set this parameter to 0 to immediately stop all moving axes

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: AerotechA3200

Changelog
=========

* itom setup 3.1.0: This plugin has been compiled
* itom setup 4.1.0: This plugin has been compiled
* itom setup 4.2.0: This plugin has been compiled
* itom setup 4.3.0: This plugin has been compiled
