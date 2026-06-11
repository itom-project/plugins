===================
 HidApi
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`HidApi`
**Type**:       :plugintype:`HidApi`
**License**:    :pluginlicense:`HidApi`
**Platforms**:  Windows, Linux, Mac
**Devices**:    Any generic HID device remotely controlled via the Itom LibUSB Plugin.
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


Dependencies
============

HidApi plugin depends on LibUSB.


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

Usage
======

In order to check all connected HID devices, open an instance of HidApi with *vendor_id* = 0, *product_id* = 0 and *print_info_about_all_devices* = 1 (True).
Then, a list of all detected devices is print to the command line.

Once, the right device is detected (e.g. by its vendor and product id and optionally by its serial number), open an instance with the right
parameters.

A command is sent by the method **setVal**, the answer can be read by **getVal**. Usually, all commands are sent and answers read via the default output
and input lines of the HID communication. In order to start a communication via the feature port, set the parameter **use_feature_report_not_output** to 1 (instead of 0).
The communication is done via **bytearray** in Python. Usually the first element in a bytearray must be set to the report id (this depends on your device).

An example is::

    trans = [0x00, 0x05, 0x10, 0x00, 0x00, 0x00]
    hidDevice.setVal(bytearray(trans))
    answer = bytearray(50) #init buffer with 50 characters
    numSignsRead = hidDevice.getVal(answer)

Changelog
==========

* itom 2.1.0: initial version of HidApi, compiled with hidapi-0.7.0
* itom 2.2.0: compiled with hidapi-0.7.0
* itom 3.0.0: compiled with hidapi-0.7.0
* itom 3.1.0: compiled with hidapi-0.7.0
* itom 3.2.1: compiled with hidapi-0.7.0
* itom 4.0.0: compiled with hidapi-0.7.0
* itom 4.1.0: compiled with hidapi-0.7.0
* itom 4.2.0: compiled with hidapi-0.7.0
* itom 4.3.0: compiled with hidapi-0.7.0
