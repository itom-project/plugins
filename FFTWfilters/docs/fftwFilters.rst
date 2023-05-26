===================
 FFTW-Filter
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`FFTW-Filter`
**Type**:       :plugintype:`FFTW-Filter`
**License**:    :pluginlicense:`FFTW-Filter`
**Platforms**:  Windows, Linux
**Author**:     :pluginauthor:`FFTW-Filter`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: FFTW-Filter

These filters are defined in the plugin:

.. pluginfilterlist::
    :plugin: FFTW-Filter
    :overviewonly:

Usage
======

For Windows:
------------

1. Download from fftw-3.3.5-dll64.zip from http://www.fftw.org/install/windows.html
2. Unzip the file.
3. Create the import library (.lib file):
  3.1 Open the Visual Studio Developer Command prompt:
     On my machine the location is C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat
4. Navigate to the unzip location and type:
	lib /machine:x64 /def:libfftw3-3.def
	lib /machine:x64 /def:libfftw3f-3.def
	lib /machine:x64 /def:libfftw3l-3.def

5. Set the CMake variable FFTW_DIR to this new folder or define a windows environment variable FFTW_ROOT.


For Ubuntu:
-----------

> sudo apt-get update -y
> sudo apt-get install -y fftw-dev

Filters
==============

Detailed overview about all defined filters:

.. pluginfilterlist::
    :plugin: FFTW-Filter

Changelog
=========

* itom setup 2.2.1: This plugin has been compiled using the FFTW 3.3.5
* itom setup 3.0.0: This plugin has been compiled using the FFTW 3.3.5
* itom setup 3.1.0: This plugin has been compiled using the FFTW 3.3.5
* itom setup 3.2.1: This plugin has been compiled using the FFTW 3.3.5
* itom setup 4.0.0: This plugin has been compiled using the FFTW 3.3.5
