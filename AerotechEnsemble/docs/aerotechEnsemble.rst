===================
 AerotechEnsemble
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`AerotechEnsemble`
**Type**:       :plugintype:`AerotechEnsemble`
**License**:    :pluginlicense:`AerotechEnsemble`
**Platforms**:  Windows
**Devices**:    Axes from company Aerotech that can be driven using the Aerotech Ensemble interface
**Author**:     :pluginauthor:`AerotechEnsemble`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: AerotechEnsemble

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: AerotechEnsemble

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    AerotechEnsemble
**controller**: {str}, read-only
    name of the connected controller
**communication**: {str}, read-only
    type of the communication (USB, Ethernet)
**libraryVersion**: {str}, read-only
    Version of the Ensemble C library
**async**: {int}
    asynchronous move (1), synchronous (0) [default]
**numAxis**: {int}, read-only
    number of connected axes
**speed**: {float seq.}
    speed of every axis

Installation
============

For using this plugin you have to install Microsoft Visual C++ 2008 Redistributable.

Changelog
==========

* itom setup 1.2.0: This plugin has been compiled using the Aerotech Ensemble SDK 4.6.1.10
* itom setup 2.0.0: This plugin has been compiled using the Aerotech Ensemble SDK 4.6.1.10
* itom setup 2.1.0: This plugin has been compiled using the Aerotech Ensemble SDK 4.6.1.10
* itom setup 2.2.0: This plugin has been compiled using the Aerotech Ensemble SDK 4.6.1.10
* itom setup 3.1.0: This plugin has been compiled using the Aerotech Ensemble SDK 4.6.1.10
* itom setup 3.2.1: This plugin has been compiled using the Aerotech Ensemble SDK 4.6.1.10
* itom setup 4.0.0: This plugin has been compiled using the Aerotech Ensemble SDK 4.6.1.10
* itom setup 4.1.0: This plugin has been compiled using the Aerotech Ensemble SDK 4.6.1.10
