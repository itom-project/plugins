========================
 Thorlabs DCx cameras
========================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsDCxCam`
**Type**:       :plugintype:`ThorlabsDCxCam`
**License**:    :pluginlicense:`ThorlabsDCxCam`
**Platforms**:  Windows
**Devices**:    Thorlabs DCx cameras
**Author**:     :pluginauthor:`ThorlabsDCxCam`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: ThorlabsDCxCam

Parameters
===========

An instance of this plugin has the following internal parameters:

**auto_blacklevel_enabled**: {int}, read-only
    If the camera supports an auto blacklevel correction (auto offset in addition to offset), this feature can be enabled / disabled by this parameter.
**binning**: {int}
    Horizontal and vertical binning, depending on camera ability. 104 means a 1x binning in horizontal and 4x binning in vertical direction. (values up to 1x, 2x, 3x, 4x, 5x, 6x, 8x, 12x are valid; if read-only binning is not supported; some cameras only support certain combinations of binnings.)
**bpp**: {int}
    Bitdepth of each pixel
**cam_id**: {int}, read-only
    ID of the camera
**cam_model**: {str}, read-only
    Model identifier of the attached camera
**color_mode**: {str}
    color_mode: 'gray' (default) or 'color' if color camera
**fps**: {float}, read-only
    current fps reported by camera
**frame_rate**: {float}
    frame rate in fps (will affect the allowed range of the integration_time, this frame_rate is only considered if trigger_mode == 'off'.
**gain**: {float}
    Gain (normalized value 0..1)
**gain_boost_enabled**: {int}
    enables / disables an additional analog hardware gain (gain boost). Readonly if not supported.
**gain_rgb**: {seq. of float}, read-only
    RGB-gain values (normalized value 0..1)
**integration_time**: {float}
    Exposure time of chip (in seconds).
**long_integration_time_enabled**: {int}, read-only
    If long exposure time is available, this parameter let you enable this. If this value is changed, the range and value of integration_time might change, too.
**name**: {str}, read-only
    GrabberName
**offset**: {float}
    Offset (leads to blacklevel offset) (normalized value 0..1). Readonly if not adjustable.
**pixel_clock**: {int}
    Pixel clock in MHz. If the pixel clock is too high, data packages might be lost. A change of the pixel clock might influence the exposure time.
**roi**: {int rect [x0,y0,width,height]}
    ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]
**sensor_type**: {str}, read-only
    Sensor type of the attached camera
**serial_number**: {str}, read-only
    Serial number of camera
**sizex**: {int}, read-only
    Pixelsize in x (cols)
**sizey**: {int}, read-only
    Pixelsize in y (rows)
**timeout**: {float}
    Timeout for acquiring images in seconds
**trigger_mode**: {str}
    trigger modes for starting a new image acquisition, depending on the camera the following modes are supported: 'off' (fixed frame_rate), without fixed frame_rate: 'software', 'hi_lo', 'lo_hi', 'pre_hi_lo', 'pre_lo_hi'

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: ThorlabsDCxCam

Compilation
===========

Please install the ThorCam software package from http://www.thorlabs.de/software_pages/ViewSoftwarePage.cfm?Code=ThorCam and install it.
Then set the CMAKE variable **THORLABS_DCxCAMERASUPPORT_DEVELOP_DIRECTORY** or environment variable **THORLABS_DCXCAM_ROOT**
of the **Develop** subdirectory of the Thorlabs installation path (e.g. C:\Program Files\Thorlabs\Scientific Imaging\DCx Camera Support\Develop).
At runtime of itom, the Thorlabs drivers must be installed for the DCx camera series such that the library
**uc480_64.dll** or **uc480.dll** can be found in the Windows System32 directory.

Known problems
===============

Sometimes, the camera raises an acquisition error right after a change of the trigger mode. In this case, make an idle-grab (with a possible try-except) before starting
with the right acquisition parameters.

Changelog
==========

* itom setup 2.2.0: This plugin has been compiled using the Thorlabs DCx USB camera driver 4.20
* itom setup 3.0.0: This plugin has been compiled using the Thorlabs DCx USB camera driver 4.20
* itom setup 3.1.0: This plugin has been compiled using the Thorlabs DCx USB camera driver 4.20
* itom setup 4.3.0: This plugin has been compiled using the ThorCamlabs Software 3.70
