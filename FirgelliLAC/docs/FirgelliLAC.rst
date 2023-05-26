===================
 Firgelli LAC
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`FirgelliLAC`
**Type**:       :plugintype:`FirgelliLAC`
**License**:    :pluginlicense:`FirgelliLAC`
**Platforms**:  Windows
**Devices**:    One axis motor controller FirgelliLAC
**Author**:     :pluginauthor:`FirgelliLAC`
=============== ========================================================================================================

Overview
========

The plugin implements the controller LAC from Firgelli (http://www.firgelli.com/LAC_Board_p/lac.htm). It is mainly
tested with the product version PQ12-SS-GG-VV-P.

.. pluginsummaryextended::
    :plugin: FirgelliLAC

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: FirgelliLAC

Parameters
==========

These parameters are available and can be used to configure the **Firgelli LAC** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**accuracy**: {float}
    Accuracy in %; range: 0.0..100.0 %
**async**: {int}
    asychronous (1) or sychronous (0) mode
**device_id**: {str}, read-only
    Name of controller
**device_num**: {int}, read-only
    The current number of this specific device, if there are more than one devices connected. (0 = first device)
**dll_version**: {str}, read-only
    Version of DLL file
**name**: {str}, read-only
    Name of the plugin
**speed**: {float}
    Target speed in %; range: 0.0..100.0 %
**spoolMax**: {float}, read-only
    Maximum length of spool (mm)

Installation
============

Before using this motor install the required driver, e.g. by installing the LAC USB configuration program from http://www.firgelli.com/LAC_Board_p/lac.htm.

Changelog
==========

* 2016-04-07: This plugin will be delivered with itom 2.2.0 for the first time
* 2016-04-07: The required library MPUSBAPI is delivered with the sources of this plugin in version 1.1.0.
* itom 4.1.0: This plugin has been compiled
