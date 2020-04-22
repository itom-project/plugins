===================
 QCam
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`QCam`
**Type**:       :plugintype:`QCam`
**License**:    :pluginlicense:`QCam`
**Platforms**:  Windows, (Linux possible but yet not implemented)
**Devices**:    Cameras from company QImaging
**Author**:     :pluginauthor:`QCam`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: QCam

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: QCam

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    QCam
**integration_time**: {float}
    Integrationtime of CCD programmed in s
**gain**: {float}
    Gain
**offset**: {float}
    Offset
**x0**: {int}
    first pixel of ROI in x-direction
**y0**: {int}
    first pixel of ROI in y-direction
**x1**: {int}
    last pixel of ROI in x-direction
**y1**: {int}
    last pixel of ROI in y-direction
**sizex**: {int}, read-only
    width of ROI
**sizey**: {int}, read-only
    height of ROI
**bpp**: {int}
    bit depth per pixel
**cooled**: {int}
    CCD cooler
    
Changelog
==========

* itom setup 1.2.0: This plugin has been compiled using the QImaging 2.0.13.1
* itom setup 1.3.0: This plugin has been compiled using the QImaging 2.0.13.1
* itom setup 1.4.0: This plugin has been compiled using the QImaging 2.0.13.1
* itom setup 2.0.0: This plugin has been compiled using the QImaging 2.0.13.1
* itom setup 2.1.0: This plugin has been compiled using the QImaging 2.0.13.1
* itom setup 2.2.0: This plugin has been compiled using the QImaging 2.0.13.1
* itom setup 3.0.0: This plugin has been compiled using the QImaging 2.0.13.1
* itom setup 3.1.0: This plugin has been compiled using the QImaging 2.0.13.1
* itom setup 3.2.1: This plugin has been compiled using the QImaging 2.0.13.1
* itom setup 4.0.0: This plugin has been compiled using the QImaging 2.0.13.1
