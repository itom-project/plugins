===================
 HidApi
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`HidApi`
**Type**:       :plugintype:`HidApi`
**License**:    :pluginlicense:`HidApi`
**Platforms**:  Windows, Linux, Mac
**Devices**:    Any generic HID device
**Author**:     :pluginauthor:`HidApi`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: HidApi

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: HidApi
        
Parameters
===========

An instance of this plugin has the following internal parameters:

**debug**: {int}
    If true, all out and inputs are written to dockingWidget
**manufacturer**: {str}, read-only
    manufacturer string
**name**: {str}, read-only
    name of device
**product**: {str}, read-only
    product string
**serial_number**: {str}, read-only
    serial number string
**use_feature_report_not_output**: {int}
    if true, getVal and setVal will operate on feature reports, else on the output buffer (default)