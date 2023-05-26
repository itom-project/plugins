===========================
 Uhl stages (text based)
===========================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`UhlText`
**Type**:       :plugintype:`UhlText`
**License**:    :pluginlicense:`UhlText`
**Platforms**:  Windows, Linux
**Devices**:    Stages from Uhl (also compatible with Lang and Merzhaeuser)
**Author**:     :pluginauthor:`UhlText`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: UhlText

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: UhlText

Parameters
===========

**accel**: {float}

**async**: {int}

**comPort**: {int}, read-only
    The current com-port ID of this specific device. -1 means undefined

**inversex**: {int}

**inversey**: {int}

**inversez**: {int}

**joyenabled**: {int}
    Enabled/disabled Joystick. Default: enabled

**name**: {str}, read-only

**numaxis**: {int}, read-only

**speed**: {float}

**timeout**: {float}
    timeout for axes movements in seconds
