===================
 Ximea
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`Ximea`
**Type**:       :plugintype:`Ximea`
**License**:    :pluginlicense:`Ximea`
**Platforms**:  Windows, Linux
**Devices**:    Cameras from company *Ximea* (tested with various xiQ USB3 cameras, monochrome and color)
**Author**:     :pluginauthor:`Ximea`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: Ximea

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: Ximea
		
Parameters
==========

**aeag**: {int}, read-only
    Enable / Disable Automatic exposure / gain. AEAG is disabled in the current implementation.
**api_version**: {str}, read-only
    XIMEA API version
**bad_pixel**: {int}
    Enable bad pixel correction.
**binning**: {int}
    1x1 (101), 2x2 (202) or 4x4 (404) binning if available. See param 'binning_type' for setting the way binning is executed.
**binning_type**: {int}
    Type of binning if binning is enabled. 0: pixels are interpolated, 1: pixels are skipped (faster).
**bpp**: {int}
    Bit depth of the output data from camera in bpp (can differ from sensor bit depth). For color cameras set bpp to 32 in order to obtain the color data.
**buffers_queue_size**: {int}, read-only
    Number of buffers in the queue.
**cam_number**: {int}, read-only
    Index of the camera device.
**color_camera**: {int}, read-only
    0: monochrome camera, 1: color camera - set bpp to 32 to obtain color image.
**device_driver**: {str}, read-only
    Current device driver version
**device_type**: {str}, read-only
    Device type (1394, USB2.0, CURRERA, ...)
**frame_burst_count**: {int}
    Define and set the number of frames in a burst (trigger_mode2 should be set to XI_TRG_SEL_FRAME_BURST_START.
**framerate**: {float}, read-only
    Framerate of image acquisition (in fps). This parameter reflects the current framerate. If timing_mode is in XI_ACQ_TIMING_MODE_FREE_RUN (0, default), the framerate is readonly and fixed to the highest possible rate. For xiQ cameras only, timing_mode can be set to XI_ACQ_TIMING_MODE_FRAME_RATE (1) and the framerate is adjustable to a fixed value.
**gain**: {float}
    Gain in % (the percentage is mapped to the dB-values).
**gamma**: {float}
    Luminosity gamma value in %.
**gammaColor**: {float}
    Chromaticity gamma value in %. Only for color cameras.
**gpi_level**: {seq. of int}, read-only
    Current level of all available gpi pins. (0: low level, 1: high level)
**gpi_mode**: {seq. of int}
    Set the input pin modes for all available gpi pins. This is a list whose lengths corresponds to the number of available pins. Use gpo_mode[i] to access the i-th pin. 0: Off, 1: trigger, 2: external signal input (not implemented by Ximea api)
**gpo_mode**: {seq. of int}
    Set the output pin modes for all available gpo pins. This is a list whose lengths corresponds to the number of available pins. Use gpo_mode[i] to access the i-th pin.
**hdr_enable**: {int}, read-only
    Enable HDR mode. default is OFF (not supported by all devices).
**hdr_it1**: {int}, read-only
    Exposure time of first slope (in % of exposure time - not supported by all devices).
**hdr_it2**: {int}, read-only
    Exposure time of second slope (in % of exposure time - not supported by all devices).
**hdr_knee1**: {int}, read-only
    First kneepoint (% of sensor saturation - not supported by all devices).
**hdr_knee2**: {int}, read-only
    Second kneepoint (% of sensor saturation - not supported by all devices).
**integration_time**: {float}
    Exposure time (in seconds).
**lensAvialable**: {int}
    Toggle if lens settings are avialable.
**max_sensor_bitdepth**: {int}, read-only
    maximum bitdepth of the sensor.
**name**: {str}, read-only
    name of the camera
**offset**: {float}, read-only
    Currently not used.
**roi**: {int rect [x0,y0,width,height]}
    ROI (x, y, width, height) [this replaces the values x0, x1, y0, y1].
**sensor_type**: {str}, read-only
    Sensor type of the attached camera
**serial_number**: {str}, read-only
    Serial number of device.
**sharpness**: {float}
    Sharpness strength in %.
**sizex**: {int}, read-only
    Width of ROI (number of columns).
**sizey**: {int}, read-only
    Height of ROI (number of rows).
**timeout**: {float}
    Acquisition timeout in s.
**timing_mode**: {int}
    Acquisition timing: 0: free run (default), 1: by frame rate.
**trigger_mode**: {int}
    Set triggermode, 0: free run, 1: ext. rising edge, 2: ext. falling edge, 3: software.
**trigger_selector**: {int}
    Set trigger selector, 0: Exposure Frame Start, 1: Exposure Frame duration, 2: Frame Burst Start, 3: Frame Burst duration (this parameter was called trigger_mode2 in a previous version of this plugin).
**x0**: {int}
    First horizontal index within current ROI (deprecated, use parameter 'roi' instead).
**x1**: {int}
    Last horizontal index within current ROI (deprecated, use parameter 'roi' instead).
**y0**: {int}
    First vertical index within current ROI (deprecated, use parameter 'roi' instead).
**y1**: {int}
    Last vertical index within current ROI (deprecated, use parameter 'roi' instead).

Additional functions (exec functions)
=====================================

.. py:function::  ximeaCam.exec('update_shading', illumination)
    :noindex:
    
    Change value of the shading correction

    :param illumination: Current intensity value
    :type illumination: int


.. py:function::  ximeaCam.exec('initialize_shading', dark_image, white_image, x0, y0)
    :noindex:
    
    Initialize pixel shading correction. At the moment you can only use one set of data which will be rescaled each time

    :param dark_image: Dark Image, if null, empty image will be generated
    :type dark_image: dataObject
    :param white_image: White Image, if null, empty image will be generated
    :type white_image: dataObject
    :param x0: Position of ROI in x
    :type x0: int
    :param y0: Position of ROI in y
    :type y0: int


.. py:function::  ximeaCam.exec('shading_correction_values', integration_time, shading_correction_factor)
    :noindex:
    
    Change value of the shading correction

    :param integration_time: Integrationtime of CCD programmed in s
    :type integration_time: float
    :param shading_correction_factor: Corresponding values for shading correction
    :type shading_correction_factor: seq. of float


Image Acquisition and Frame Burst
=================================

If you acquire an image, the obtained data object has some tags defined:

.. code-block:: python
    
    obj = dataObject()
    cam.acquire() #cam must be started before
    cam.getVal(obj)
    print(obj.tags)

The tags are:

* timestamp: timestamp of image acquisition in seconds (not MU family)
* frame_counter: continuous number of frame
* roi_x0: left offset of ROI (only for Ximea API > 4.0.0.5)
* roi_y0: top offset of ROI (only for Ximea API > 4.0.0.5)

If you change *trigger_mode* to anything else than *Off* and set *trigger_selector* to *frame_burst_start (2)*, it is possible
to acquire a serie of frames after the software or hardware trigger impulse. This can be adjusted using the parameter *frame_burst_count*.

If this is set, the acquired data object is not two-dimensional but three-dimensional, where the first (z-) dimension 
corresponds to the number of acquired frames. If this is the case, the tags are:

* timestamp0, timestamp1, timestamp2, ... (for each sub-frame, not MU family)
* frame_counter0, frame_counter1, ...

Installation
============

*Windows:*

Install the XIMEA API (http://www.ximea.com/support/documents/4, currently tested with version 4.10.0.0) and check that
your camera runs with the internal XiViewer from XIMEA. If this is the case, the camera should also run with itom.

*Linux:*

Install the XIMEA driver from the ximea website and use the commands described there, too, in oder to install the driver.
Then point the CMAKE variable XIMEA_APIDIR to the include directory of the Ximea package. This must contain the file m3api.h.
Like under Windows, the library itself is dynamically loaded at runtime. It is usually loaded from /usr/lib.

If you want to externally trigger the camera, make sure that you check if your GPIO pins require a 5V or 24V signal. Some cameras
only support 24V, modern camera devices support both. This is written at the housing (at least for xiQ USB3 cameras).

    
Changelog
=========

* itom setup 1.2.0: This plugin has been compiled using the Ximea API 4.0.0.5
* itom setup 1.3.0: This plugin has been compiled using the Ximea API 4.0.0.5
* itom setup 1.4.0: This plugin has been compiled using the Ximea API 4.0.0.5
* itom setup 2.0.0: This plugin has been compiled using the Ximea API 4.4.0
* itom setup 2.1.0: This plugin has been compiled using the Ximea API 4.4.0
* itom setup 2.2.0: This plugin has been compiled using the Ximea API 4.10.0
* itom setup 3.0.0: This plugin has been compiled using the Ximea API 4.10.2
* itom setup 3.1.0: This plugin has been compiled using the Ximea API 4.10.2
* itom setup 3.2.1: This plugin has been compiled using the Ximea API 4.16
* itom setup 4.0.0: This plugin has been compiled using the Ximea API 4.18.04
* itom setup 4.1.0: This plugin has been compiled using the Ximea API 4.18.04
