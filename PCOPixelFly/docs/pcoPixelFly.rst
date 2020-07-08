===================
 PCOPixelFly
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PCOPixelFly`
**Type**:       :plugintype:`PCOPixelFly`
**License**:    :pluginlicense:`PCOPixelFly`
**Platforms**:  Windows
**Devices**:    PCO Pixelfly qet cameras with frame grabber board
**Author**:     :pluginauthor:`PCOPixelFly`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: PCOPixelFly

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: PCOPixelFly

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    PCOPixelFly
**integration_time**: {float}
    Integrationtime of CCD programmed in s
**gain**: {float}
    Standard light mode (0, default), low light mode (1)
**x0**: {int}
    Startvalue for ROI
**y0**: {int}
    Startvalue for ROI
**x1**: {int}
    Stopvalue for ROI
**y1**: {int}
    Stopvalue for ROI
**roi**: {int seq.}
    ROI (x,y,width,height)
**sizex**: {int}, read-only
    ROI-Size in x
**sizey**: {int}, read-only
    ROI-Size in y
**bpp**: {int}
    bit depth in bits per pixel
**binning**: {int}
    Activate 2x2 binning
**trigger_mode**: {int}, read-only
    Set Triggermode, currently not implemented
**shift_bits**: {int}
    In 8 bit, select a number of bits (0..5) that the 12bit acquired values should be shifted before transport with 8 bit precision. (only in 8bit bpp mode)
**board_number**: {int}, read-only
    Number of this board
**driver_version**: {str}, read-only
    driver version

Changelog
=========

* itom setup 3.1.0: This plugin has been compiled using the pco pixelfly SDK 2.01.03
* itom setup 3.2.1: This plugin has been compiled using the pco pixelfly SDK 2.01.03
* itom setup 4.0.0: This plugin has been compiled using the pco pixelfly SDK 2.01.03