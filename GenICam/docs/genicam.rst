===================
 GenICam
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`GenICam`
**Type**:       :plugintype:`GenICam`
**License**:    :pluginlicense:`GenICam`
**Platforms**:  Windows, Linux
**Devices**:    Camera control of devices that support the GenICam standard
**Author**:     :pluginauthor:`GenICam`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: GenICam

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: GenICam
        
Compilation
===========

In order to compile this plugin, download the latest GenICam(TM) GenApi reference implementation from http://www.emva.org/standards-technology/genicam/genicam-downloads/.
Unpack the corresponding archive (under Windows: Development and Runtime.zip) in one folder and set the CMake variable GENICAM_ROOT to this base folder.
After re-configuring CMake the other variables (e.g. GENICAM_GCBASE_LIBRARY...) should be found automatically.