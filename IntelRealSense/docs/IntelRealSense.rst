===================
 IntelRealSense
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`IntelRealSense`
**Type**:       :plugintype:`IntelRealSense`
**License**:    :pluginlicense:`IntelRealSense`
**Platforms**:  Windows, Linux, NvidiaJetson, Raspi3
**Devices**:    Cameras from company *INTEL* of type D400, D300, ST300, sim. (Tested with D435)
**Author**:     :pluginauthor:`INTEL; Faulhaber, Andreas`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: IntelRealSense

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: IntelRealSense

Parameters
==========

**api_version**: {str}, read-only
    IntelRealSense API version

**binning**: {int}
    1x1 (101), 2x2 (202) or 4x4 (404) binning if available. See param 'binning_type' for setting the way binning is executed.
**bpp**: {int}
    Bit depth of the output data from camera in bpp (can differ from sensor bit depth). For color cameras set bpp to 32 in order to obtain the color data.
**cam_number**: {int}, read-only
    Index of the camera device.
**color_camera**: {int}, read-only
    0: monochrome camera, 1: color camera - set bpp to 32 to obtain color image.
**framerate**: {float}, read-only
    Framerate of image acquisition (in fps). This parameter reflects the current framerate. If timing_mode is in XI_ACQ_TIMING_MODE_FREE_RUN (0, default), the framerate is readonly and fixed to the highest possible rate. For xiQ cameras only, timing_mode can be set to XI_ACQ_TIMING_MODE_FRAME_RATE (1) and the framerate is adjustable to a fixed value.
**integration_time**: {float}
    Exposure time (in seconds).
**max_sensor_bitdepth**: {int}, read-only
    maximum bitdepth of the sensor.
**name**: {str}, read-only
    name of the camera
**roi**: {int rect [x0,y0,width,height]}
    ROI (x, y, width, height) [this replaces the values x0, x1, y0, y1].
**sensor_type**: {str}, read-only
    Sensor type of the attached camera
**sizex**: {int}, read-only
    Width of ROI (number of columns).
**sizey**: {int}, read-only
    Height of ROI (number of rows).
**x0**: {int}
    First horizontal index within current ROI (deprecated, use parameter 'roi' instead).
**x1**: {int}
    Last horizontal index within current ROI (deprecated, use parameter 'roi' instead).
**y0**: {int}
    First vertical index within current ROI (deprecated, use parameter 'roi' instead).
**y1**: {int}
    Last vertical index within current ROI (deprecated, use parameter 'roi' instead).

Installation
============

*Windows:*

Install the IntelRealSense SDK (https://www.intelrealsense.com/, currently tested with version 2.0) and check that
your camera runs with the internal RealSenseViewer.exe. If this is the case, the camera should also run with itom.

*Linux:*

See https://www.intelrealsense.com/developers/ for installation instructions.


Changelog
=========