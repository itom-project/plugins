===================
 DummyMultiChannelGrabber
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`DummyMultiChannelGrabber`
**Type**:       :plugintype:`DummyMultiChannelGrabber`
**License**:    :pluginlicense:`DummyMultiChannelGrabber`
**Platforms**:  Windows, Linux
**Devices**:    Virtual camera providing random noise images
**Author**:     :pluginauthor:`DummyMultiChannelGrabber`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: DummyMultiChannelGrabber

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: DummyMultiChannelGrabber
        
Parameters
===========

An instance of this plugin has the following parameters:

**binning**: {int}
    Binning of different pixel, binning = x-factor * 100 + y-factor (binning in y only possible if height of sensor >= 4.
**bpp**: {int}
    bitdepth of images
**frame_time**: {float}
    Minimum time between the start of two consecutive acquisitions [s], default: 0.0.
**gain**: {float}
    Virtual gain
**integration_time**: {float}
    Minimum integration time for on acquisition [s], default: 0.0 (as fast as possible).
**name**: {str}, read-only
    GrabberName
**offset**: {float}
    Virtual offset
**roi**: {int rect [x0,y0,width,height]}
    ROI (x,y,width,height)
**sizex**: {int}, read-only
    size in x (cols) [px]
**sizey**: {int}, read-only
    size in y (rows) [px]

