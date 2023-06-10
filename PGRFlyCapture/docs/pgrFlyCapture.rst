===================
 PGRFlyCapture
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PGRFlyCapture`
**Type**:       :plugintype:`PGRFlyCapture`
**License**:    :pluginlicense:`PGRFlyCapture`
**Platforms**:  Windows, Linux
**Devices**:    Point Grey cameras (USB3) supported by Fly Capture driver.
                (GigE possible, but not implemented yet)
**Author**:     :pluginauthor:`PGRFlyCapture`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: PGRFlyCapture

This plugin has mainly been tested with cameras of type Flea3 and Grasshopper3.

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: PGRFlyCapture

Parameters
==========

These parameters are available and can be used to configure the **PGRFlyCapture**
instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of
these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*. If a parameter is read-only, it is
not writeable or supported by the specific camera.

**bpp**: {int}
    bitdepth of each pixel
**cam_firmware_build_time**: {str}, read-only
    Built time of the firmware used in the connected camera
**cam_firmware_version**: {str}, read-only
    Serial number of the firmware used in the connected camera
**cam_interface**: {str}, read-only
    Interface of camera
**cam_model**: {str}, read-only
    Model identifier of the connected camera
**cam_register**: {int}
    Direct read/write of registers, use the hex-number of the register as
    suffix to read/write a specific register (e.g. getParam('cam_register:0xA01F'))
**cam_resolution**: {str}, read-only
    Resolution of the chip in connected camera
**cam_sensor**: {str}, read-only
    Identifier of the chip in connected camera
**cam_serial_number**: {int}, read-only
    Serial number of the connected camera
**cam_vendor**: {str}, read-only
    Name of the camera vendor
**color_mode**: {str}, read-only
    colorMode: 'gray' (default) or 'color' if color camera
**exposure_ev**: {int}
    Camera brightness control (EV)
**extended_shutter**: {int}
    1 (default): extended shutter is on (long integration times are supported
    and frame_time becomes invalid), 0: frames are only acquired in the pulse
    given by frame_time.
**frame_time**: {float}, read-only
    Frame rate in seconds. This is only considered if the camera is not in an
    extended shutter mode. The frame_time might influence the integration time.
**gain**: {float}
    gain (normalized value 0..1)
**gamma**: {int}
    Gamma adjustment
**integration_time**: {float}
    Integrationtime of CCD programmed in seconds.
**metadata**: {int}
    If 1 (default), the timestamp, frame counter and roi position (depending on
    the camera model) will be acquired and added into the first pixels of the
    image (available as tag of the data object as well), 0: metadata disabled
**name**: {str}, read-only
    name of the camera
**num_idle_grabs_after_param_change**: {int}
    With some cameras, parameter changes like the exposure time or gain will only
    take effect x images after the change. If this parameter is set to > 0, the
    given number of images are acquired after changing any parameter in order to
    delete the intermediate images.
**offset**: {float}
    offset (normalized value 0..1, mapped to PG-parameter BRIGHTNESS)
**packetsize**: {int}
    Packet size of current image settings
**roi**: {int rect [x0,y0,width,height]}
    region of interest, ROI (x,y,width,height)
**sharpness**: {int}
    Sharpness
**sizex**: {int}, read-only
    Pixelsize in x (cols)
**sizey**: {int}, read-only
    Pixelsize in y (rows)
**start_delay**: {float}
    On some computers, a blue screen sometimes occurs if the time gap between
    starting the camera and acquiring an image is too short. In this case, try
    to increase this parameter (in s).
**timeout**: {float}
    Timeout for acquiring images in seconds
**video_mode**: {int}, read-only
    Current video mode, default is Mode7

Additional functions (exec functions)
=====================================

.. py:function::  ptGreyCam.exec('printParameterInfo')
    :noindex:

    print all current parameters of the camera for internal checks.

    :param funcName: Name of the function, must be "printParameterInfo")
    :type funcName: str
    :return: None
    :rtype: None

.. py:function::  ptGreyCam.exec('setStrobeMode', source, onOff = 1, polarity = 0, delay = 0.0, duration = 0.0)
    :noindex:

    configures the strobe mode for the given GPIO pin (source).

    :param funcName: Name of the function, must be "setStrobeMode")
    :type funcName: str
    :param source: the GPIO pin to be edited (mandatory)
    :type source: int
    :param onOff: ON or OFF this function; 0: OFF, 1 : ON, optional
    :type onOff: int
    :param polarity: 0 = active low; 1 = active high, optional
    :type polarity: int
    :param delay: delay after start of exposure until the strobe signal asserts in ms.
                 This value must be in the range [0.0-100.0], optional
    :type delay: float
    :param duration: duration of the strobe signal in ms, a value of 0 means
                     de-assert at the end of exposure, if required.
                     This value must be in the range [0.0-100.0], optional
    :type duration: float
    :return: None
    :rtype: None

.. py:function::  [onOff, polarity, delay, duration] = ptGreyCam.exec('getStrobeMode', source)
    :noindex:

    returns the configuration of the strobe mode for the given GPIO pin (source).

    :param funcName: Name of the function, must be "getStrobeMode")
    :type funcName: str
    :return: Tuple with the current values for onOff (int), polarity (int), delay (float) and duration (float).
             For the meaning of these values see the description of the exec-function 'setStrobeMode'.
    :rtype: None


Compilation
=============

Download the FlyCapture2 SDK from https://www.flir.de/.

FlyCapture version 2.13.3.31 uses libiomp5md.dll in version 5.0.2011.1108.
Libiomp5md.dll is also used by numpy (mkl version), but often in another version
that is not compatible (there is an error when opening itom). Since it is
impossible to load two dlls of the same name in one process, you may need to edit
the FlyCapture2_vXXX.dll.
A useful tool for this is the software "CFF Explorer". This program allows you
to change the import table of the DLL.

    * open FlyCapture2_vXXX.dll loaded by the plugin using **CFF Explorer**.
      The File should be located at *...\itom\build\itom\libs*
    * change the name libiomp5md.dll under Import Directory to a different name
      (e.g. libiomp5mm.dll)
    * save the changes of the changed FlyCapture2_vXXX.dll in **CFF Explorer**
    * rename the libiomp5md.dll in *...\itom\build\itom\libs* (as well as the
      source of the FlyCapture SDK if necessary) to the new name, too
      (e.g. libiomp5mm.dll)

The result is, that FlyCapture2_vXXX.dll does not depend any more on the
old *libiomp5md.dll* but on the renamed version with the same content.
This might resolve the name conflict with the same file (different version)
shipped with Numpy+MKL.

Then set the CMake variable *FLYCAPTURE_PGRFLYCAP_INCLUDE_DIR** or the environment path vairable **FLIR_SDK_ROOT**
to the base directory of the pco.sensicam (e.g. C:\Program Files\Point Grey Research\FlyCapture2).


Image Acquisition
===================

If you acquire an image, the obtained data object has some tags defined if the
parameter 'metadata' is set to 1::

    obj = dataObject()
    cam.acquire() #cam must be started before
    cam.getVal(obj)

    print(obj.tags)

The tags are:

* timestamp: timestamp of image acquisition in seconds
* frame_counter: continuous number of frame (if camera does not run in any trigger mode,
  this number can increase more than by one from one acquired image to the next one)
* roi_x0: left offset of ROI
* roi_y0: top offset of ROI

If 'metadata' is 0 or if the camera model does not support this additional information,
no tags are appended to each data object.
Please consider, that the image information is embedded in the first pixels of
each image (see https://www.ptgrey.com/tan/10563).

When camera property settings take effect
===========================================

In the technical documentations of PointGrey cameras, there is a section
about **when camera property settings take effect**. This
section gives hints after how many acquired images changes in properties like
integration_time, gain, etc. will be *visible* in the
next image. With respect to this documentation, most changes will be applied to
the "after next" image, if the camera is in trigger-mode.
If the camera is in free-run mode (trigger_mode = -1), it sometimes needs up
to 4 frames until changes become visible!

Usually, the plugin does not acquire any idle grabs after having changed any
parameter. However, if you set the parameter 'num_idle_grabs_after_param_change'
to any value bigger than zero, the number of images are acquired. This happens
at the next call of **startDevice** if the camera is currently stopped
or immediately at the end of the **setParam** command.

Changelog
==========

* itom setup 1.2.0: This plugin has been compiled using the FlyCapture 2.6.3.4
* itom setup 1.3.0: This plugin has been compiled using the FlyCapture 2.6.3.4
* itom setup 1.4.0: This plugin has been compiled using the FlyCapture 2.6.3.4
* itom setup 2.0.0: This plugin has been compiled using the FlyCapture 2.7.3.18
* itom setup 2.1.0: This plugin has been compiled using the FlyCapture 2.7.3.18
* itom setup 2.2.0: This plugin has been compiled using the FlyCapture 2.7.3.18,
  under Windows it requires the Microsoft C++ Redistributable 2012
* itom setup 3.0.0: This plugin has been compiled using the FlyCapture 2.7.3.18,
  under Windows it requires the Microsoft C++ Redistributable 2012
* itom setup 3.1.0: This plugin has been compiled using the FlyCapture 2.11.3.425,
  under Windows it requires the Microsoft C++ Redistributable 2012
* itom setup 3.1.0: This plugin will be compiled using the FlyCapture 2.13.3.31,
  under Windows it requires the Microsoft C++ Redistributable 2012
* itom setup 3.2.1: This plugin will be compiled using the FlyCapture 2.13.3.31,
  under Windows it requires the Microsoft C++ Redistributable 2012
* itom setup 4.0.0: This plugin will be compiled using the FlyCapture 2.13.3.61,
  under Windows it requires the Microsoft C++ Redistributable 2012
* itom setup 4.1.0: This plugin will be compiled using the FlyCapture 2.13.3.61,
  under Windows it requires the Microsoft C++ Redistributable 2012
* itom setup 4.3.0: This plugin will be compiled using the FlyCapture 2.13.3.61,
  under Windows it requires the Microsoft C++ Redistributable 2012

Linux
======

For linux, please consider the document "Using Linux with USB 3.0", published by
Point Grey as technical application note TAN2012007. Starting the camera crashed
(or came to a timeout when
calling *startDevice* for cameras with an image size bigger than 2MB if the notes
in section **Configuration USBFS** are not considered.
