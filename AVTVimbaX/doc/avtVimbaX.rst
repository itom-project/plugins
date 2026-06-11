===================
 AVT Vimba X
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`AVTVimbaX`
**Type**:       :plugintype:`AVTVimbaX`
**License**:    :pluginlicense:`AVTVimbaX`
**Platforms**:  Windows (GigE + USB3 [+CSI2]), Linux only supports GigE (not tested yet)
**Devices**:    AVT Cameras driven by Vimba interface
**Author**:     :pluginauthor:`AVTVimbaX`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: AVTVimbaX


Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: AVTVimbaX

Parameters
===========

An instance of this plugin has the following internal parameters:

**binning**: {int}, read-only
    binning (horizontal_factor * 100 + vertical_factor)
**bpp**: {int}
    Bit depth of sensor
**camera_number**: {int}
    Camera Number
**gain**: {float}
    Gain of AD in dB, set it to 0.0 for best image quality.
**gain_auto**: {int}
    auto-controlled gain (0: off, 1: continuously varies the gain; gain will be read-only then)
**gamma**: {float}
    Gamma value
**integration_time**: {float}
    Integrationtime of CCD [s]
**interface**: {str}, read-only
    Interface type (Firewire, GigE)
**name**: {str}, read-only
    name of plugin
**offset**: {float}
    Offset as physical value that is a DC offset applied to the video signal. This values changes the blacklevel.
**roi**: {int rect [x0,y0,width,height]}
    ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]
**serial_no**: {str}, read-only
    Serial number
**sizex**: {int}, read-only
    width of ROI
**sizey**: {int}, read-only
    height of ROI
**timeout**: {float}
    timeout for image acquisition in sec
**trigger_activation**: {str}
    trigger activation (RisingEdge, FallingEdge, AnyEdge, LevelHigh, LevelLow). Not all values are supported for all cameras.
**trigger_mode**: {int}
    trigger mode (0: Off, 1: On)
**trigger_source**: {str}
    trigger source (Freerun, Line1, Line2, Line3, Line4, FixedRate, Software, InputLines). Not all values are supported for all cameras.

Usage
=============

In order to use this plugin, please install the VimbaX SDK from Allied Vision (see https://www.alliedvision.com/de/support/software-downloads/).
During the installation, the necessary VimbaX DLLs (VmbC.dll and VmbCPP.dll) should be copied into the system directory of Windows, such that no further steps need to be
done using the AVT cameras within **itom**. If this is not the case, copy both libraries from the VimbaX installation folder to the lib-folder of itom (make sure that you use
the 64bit versions for 64bit itom and vice versa).

If you want to build this plugin from the sources, you need to make sure that you installed the C++ component of the VimbaX SDK (select this component during the install process).
In CMake enable the checkbox of the variable **PLUGIN_AVTVimbaX** and re-configure CMake. Then set the variable **AVTVimbaX_INCLUDE_DIR** or the Environment Variable **AVTVimbaX_ROOT**
to anything similar like **C:\Program Files\Allied Vision\Vimba X\api\include** (subfolder of the VimbaX installation path) and generate your CMake project again.
The variables **AVTVimbaX_API_DIR**, **AVTVimbaX_LIBRARY**... should now be found automatically if you indicated a right VimbaX installation directory in the step before.

Known issues
==============

Some cameras raise a timeout error when acquiring images. If so, try to set the parameter 'trigger_mode' to 0 (off)::

    cam.setParam("trigger_mode", 0)

Changelog
==========

* itom setup 4.3.0: This plugin has been compiled using AVT VimbaX v1.0.5
