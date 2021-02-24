===================
 AVT Vimba
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`AVTVimba`
**Type**:       :plugintype:`AVTVimba`
**License**:    :pluginlicense:`AVTVimba`
**Platforms**:  Windows (Firewire + GigE + USB3), Linux only supports GigE (not tested yet)
**Devices**:    AVT Cameras driven by Vimba interface
**Author**:     :pluginauthor:`AVTVimba`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: AVTVimba


Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: AVTVimba
        
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

In order to use this plugin, please install the Vimba SDK from Allied Vision in version 2.0 or higher (see http://www.alliedvisiontec.com/de/produkte/software.html).
During the installation, the necessary Vimba DLLs (VimbaC.dll and VimbaCPP.dll) should be copied into the system directory of Windows, such that no further steps need to be
done using the AVT cameras within **itom**. If this is not the case, copy both libraries from the Vimba installation folder to the lib-folder of itom (make sure that you use
the 64bit versions for 64bit itom and vice versa).

If you want to build this plugin from the sources, you need to make sure that you installed the C++ component of the Vimba SDK (select this component during the install process).
In CMake enable the checkbox of the variable **PLUGIN_AVTVimba** and re-configure CMake. Then set the variable **AVTVIMBA_INCLUDE_DIR** to anything similar like *C:/Program Files/Allied Vision Technologies/Vimba_2.0/VimbaCPP/Include* (subfolder of the Vimba installation path) and generate your CMake project again. The variables **AVTVIMBA_API_DIR**, AVTVIMBA_LIBRARY**... should now
be found automatically if you indicated a right Vimba installation directory in the step before.

Known issues
==============

Some cameras raise a timeout error when acquiring images. If so, try to set the parameter 'trigger_mode' to 0 (off)::
    
    cam.setParam("trigger_mode", 0)

Changelog
==========

* itom setup 1.4.0: This plugin has been compiled using AVT Vimba 1.3.0
* itom setup 2.0.0: This plugin has been compiled using AVT Vimba 1.3.0
* itom setup 2.1.0: This plugin has been compiled using AVT Vimba 1.4.0
* itom setup 2.2.0: This plugin has been compiled using AVT Vimba 2.0
* itom setup 3.0.0: This plugin has been compiled using AVT Vimba 2.0
* itom setup 3.1.0: This plugin has been compiled using AVT Vimba 2.1
* itom setup 3.2.1: This plugin has been compiled using AVT Vimba 2.1
* itom setup 4.0.0: This plugin has been compiled using AVT Vimba 3.1.0
* itom setup 4.1.0: This plugin has been compiled using AVT Vimba 3.1.0