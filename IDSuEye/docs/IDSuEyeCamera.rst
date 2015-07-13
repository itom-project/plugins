===================
 IDS uEye
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`IDSuEye`
**Type**:       :plugintype:`IDSuEye`
**License**:    :pluginlicense:`IDSuEye`
**Platforms**:  Windows, Linux
**Devices**:    IDS Imaging cameras
**Author**:     :pluginauthor:`IDSuEye`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: IDSuEye

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: IDSuEye

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    name of the plugin
**integration_time**: {float}
	Exposure time of chip (in seconds)
**long_integration_time_enabled**: {int}
    If long exposure time is available, this parameter let you enable this. If this value is changed, the range and value of integration_time might change, too
**pixel_clock**: {int}
    Pixel clock in MHz. If the pixel clock is too high, data packages might be lost. A change of the pixel clock might influence the exposure time
**binning**: {int}
    Horizontal and vertical binning, depending on camera ability. 104 means a 1x binning in horizontal and 4x binning in vertical direction. (values up to 1x, 2x, 3x, 4x, 5x, 6x, 8x, 12x are valid; if read only binning is not supported)
**frame_rate**: {float}, read-only
    frame rate in fps. This value is always set to the minimum value in order to allow huge exposure times. Since the camera is not run in free-run mode, the frame rate is not important.
**gain**: {float}
    Gain (normalized value 0..1)
**gain_rgb**: {float seq.}
	RGB-gain values (normalized value 0..1)
**gain_boost_enabled**: {int}
    enables / disables an additional analog hardware gain (gain boost). Readonly if not supported
**offset**: {float}
	Offset (leads to blacklevel offset) (normalized value 0..1). Readonly if not adjustable
**auto_blacklevel_enabled**: {int}
    If the camera supports an auto blacklevel correction (auto offset in addition to offset), this feature can be enabled / disabled by this parameter
**sizex**: {int}, read-only
    Pixelsize in x (cols)
**sizey**: {int}, read-only
    Pixelsize in y (rows)
**roi**: {int seq.}
    ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]
**x0**: {int}
    Index of left boundary pixel within ROI
**y0**: {int}
    Index of top boundary pixel within ROI
**x1**: {int}
    Index of right boundary pixel within ROI
**y1**: {int}
    Index of bottom boundary pixel within ROI
**trigger_mode**: {str}
    trigger modes for starting a new image acquisition
**cam_model**: {str}, read-only
    Model identifier of the attached camera
**cam_id**: {int}, read-only
    ID of the camera
**sensor_type**: {str}, read-only
    Sensor type of the attached camera
**color_mode**: {str}
    color_mode: 'gray' (default) or 'color' if color camera
**bpp**: {int}
    Bitdepth of each pixel
**timeout**: {float}
    Timeout for acquiring images in seconds