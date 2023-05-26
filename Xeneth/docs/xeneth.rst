===================
 Xeneth cameras
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`Xeneth`
**Type**:       :plugintype:`Xeneth`
**License**:    :pluginlicense:`Xeneth`
**Platforms**:  Windows, Linux
**Devices**:    Xeneth cameras, family Bobcat
**Author**:     :pluginauthor:`Xeneth`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: Xeneth

ITOM Plugin to be used for interaction with the Xeneth Bobcat family of NIR Cameras.


Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: Xeneth

Compilation
===========

The Xenics Xeneth Software Development Kit is publicaly not available.
Please contact Xenics to get access to the SDK https://www.xenics.com/

Download the software and run the installation.

The library should be fined via the CMAKE default settings, if not please set the CMake variable
**XENETH_SDK_DIR** or the environment variable **XENETH_ROOT**
to the directory that contains the header files of the SDK (e.g. D:\Program Files\Xeneth\Sdk).

Changelog
==========

* itom setup 4.3.0: This plugin has been compiled with Xeneth SDK 2.7.0.
