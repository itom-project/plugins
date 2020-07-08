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
        
Usually, the next free camera is opened by *camera_id* = 0. If you want to select a specific camera, make sure to set unique camera IDs in the IDS camera manager tool.
Then set *camera_id* to the desired camera ID (range 1..254).

In the changelog below, you can see which uEye driver has been used to compile this plugin for several setup versions of itom. In order to
guarantee a full working plugin, you should have the same driver installed on your computer. Else, a warning will be displayed at startup telling
that some faults may occur. Usually, the compatibility is given, if the major version number and the ten-digit of the minor is equal, e.g. 4.80 and 4.81 are compatible.

Support for Thorlabs cameras
============================

This plugin can also be used to operate Thorlabs cameras of the **DCC** or **DCU** series (e.g. https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=4024 or https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=2916).
These cameras are OEM versions from IDS. This plugin has for instance been tested with the version DCC1545M-GL.
    
Parameters
===========

An instance of this plugin has the following internal parameters:

**auto_blacklevel_enabled**: {int}
    If the camera supports an auto blacklevel correction (auto offset in addition to offset), this feature can be enabled / disabled by this parameter.
**binning**: {int}, read-only
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
**num_buffer**: {int}
    Number of Buffers used for acquisition. Note that if this is > 1, a sequence of images will be acquired.
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
    timeout in seconds when waiting for the next image. For sequences x1000.
**trigger_mode**: {str}
    trigger modes for starting a new image acquisition, depending on the camera the following modes are supported: 'off' (fixed frame_rate), without fixed frame_rate: 'software', 'hi_lo', 'lo_hi', 'pre_hi_lo', 'pre_lo_hi'
**x0**: {int}
    Index of left boundary pixel within ROI
**x1**: {int}
    Index of right boundary pixel within ROI
**y0**: {int}
    Index of top boundary pixel within ROI
**y1**: {int}
    Index of bottom boundary pixel within ROI


        
Compilation
===========

With the sources of this plugin, the header and library files of the uEye SDK in the version denoted in the changelog are directly included. Hence, the plugin compiles as it is. 
Nevertheless, you need to have the camera drivers installed on your computer that fit to the uEye SDK of the plugin. However, you can also install the full SDK in any version
on your computer and set the CMake variable IDS_DEVELOP_DIRECTORY to the develop-subfolder of the SDK (this folder contains the include and Lib subfolder). If you indicated this,
please delete IDS_HEADER_FILE and IDS_LIBRARY in CMake and press configure again. Then, the plugin will be compiled with your individual SDK.

Please install the 32bit/64bit version of IDS uEye SDK that corresponds to your operating system, not to the type of itom. If you decide to configure the SDK installer, you don't
need to install any DirectShow or ActiveX components as well as additional drivers if you only want to use the camera with itom.

Under linux, simply install the drivers from the IDS website. After an successful installation, the header file *ueye.h* and the library file is installed
to the default directories under linux. The CMakeLists.txt file of this itom plugin will then automatically detect these files and compile the plugin.

Acquisiton of image sequences
==============================

If the number of buffers (num_buffer) is set to a value greater than 1, a sequence of images will be acquired with the acquire()-function (without frameloss). The sequence
is retrieved by getVal() as usual, but the obtained dataObject will be 3D. The number of planes corresponds to the amount of images in the sequence (i.e. the number of buffers).
Please be aware that the necessary memory for the sequence will be allocated and there is no dynamic limitation of the parameter. Choosing a high value for the number of buffers
might crash the program, if the sytems memory is insufficient.

Known problems
===============

Sometimes, the camera raises an acquisition error right after a change of the trigger mode. In this case, make an idle-grab (with a possible try-except) before starting
with the right acquisition parameters.

Changelog
==========

* itom setup 2.0.0: This plugin has been compiled using the uEye SDK 4.61
* itom setup 2.1.0: This plugin has been compiled using the uEye SDK 4.61
* itom setup 2.2.0, 2.2.1: This plugin has been compiled using the uEye SDK 4.80 (In order to use newer USB3 cameras with the USB2 port, install the 4.81 driver)
* itom setup 3.0.0: This plugin has been compiled using the uEye SDK 4.81
* itom setup 3.1.0: This plugin has been compiled using the uEye SDK 4.90.3
* itom setup 3.2.1: This plugin has been compiled using the uEye SDK 4.91.0
* itom setup 4.0.0: This plugin has been compiled using the uEye SDK 4.93.0
