===================
 PGRFlyCapture
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PGRFlyCapture`
**Type**:       :plugintype:`PGRFlyCapture`
**License**:    :pluginlicense:`PGRFlyCapture`
**Platforms**:  Windows, Linux
**Devices**:    Point Grey cameras (USB3) supported by Fly Capture driver. (GigE possible, but not implemented yet)
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

These parameters are available and can be used to configure the **PGRFlyCapture** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*. If a parameter is read-only, it is not writeable or supported by the specific camera.
	
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
    Direct read/write of registers, use the hex-number of the register as suffix to read/write a specific register (e.g. getParam('cam_register:0xA01F'))
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
    1 (default): extended shutter is on (long integration times are supported and frame_time becomes invalid), 0: frames are only acquired in the pulse given by frame_time.
**frame_time**: {float}, read-only
    Frame rate in seconds. This is only considered if the camera is not in an extended shutter mode. The frame_time might influence the integration time.
**gain**: {float}
    gain (normalized value 0..1)
**gamma**: {int}
    Gamma adjustment
**integration_time**: {float}
    Integrationtime of CCD programmed in seconds.
**metadata**: {int}
    If 1 (default), the timestamp, frame counter and roi position (depending on the camera model) will be acquired and added into the first pixels of the image (available as tag of the data object as well), 0: metadata disabled
**name**: {str}, read-only
    name of the camera
**offset**: {float}
    offset (normalized value 0..1, mapped to PG-parameter BRIGHTNESS)
**packetsize**: {int}
    Packet size of current image settings
**roi**: {int rect [x0,y0,width,height]}
    region of interest, ROI (x,y,width,height)
**sharpness**: {int}, read-only
    Sharpness
**sizex**: {int}, read-only
    Pixelsize in x (cols)
**sizey**: {int}, read-only
    Pixelsize in y (rows)
**timeout**: {float}
    Timeout for acquiring images in seconds
**trigger_mode**: {int}
    -1: Complete free run, 0: enable standard external trigger (PtGrey mode 0), 1: Software Trigger (PtGrey mode 0, Software Source), 2: Bulb shutter external trigger (PtGrey mode 1), 3: Overlapped external trigger (PtGrey mode 14)
**trigger_polarity**: {int}
    For hardware trigger only: Set the polarity of the trigger (0: trigger active low, 1: trigger active high)
**video_mode**: {int}, read-only
    Current video mode, default is Mode7
	
Image Acquisition
===================

If you acquire an image, the obtained data object has some tags defined if the parameter 'metadata' is set to 1::
    
    obj = dataObject()
    cam.acquire() #cam must be started before
    cam.getVal(obj)
    
    print(obj.tags)
    
The tags are:

* timestamp: timestamp of image acquisition in seconds
* frame_counter: continuous number of frame (if camera does not run in any trigger mode, this number can increase more than by one from one acquired image to the next one)
* roi_x0: left offset of ROI
* roi_y0: top offset of ROI

If 'metadata' is 0 or if the camera model does not support this additional information, no tags are appended to each data object.
Please consider, that the image information is embedded in the first pixels of each image (see https://www.ptgrey.com/tan/10563).

Changelog
==========

* itom setup 1.2.0: This plugin has been compiled using the FlyCapture 2.6.3.4
* itom setup 1.3.0: This plugin has been compiled using the FlyCapture 2.6.3.4
* itom setup 1.4.0: This plugin has been compiled using the FlyCapture 2.6.3.4
* itom setup 2.0.0: This plugin has been compiled using the FlyCapture 2.7.3.18
* itom setup 2.1.0: This plugin has been compiled using the FlyCapture 2.7.3.18
* itom setup 2.2.0: This plugin has been compiled using the FlyCapture 2.7.3.18, under Windows it requires the Microsoft C++ Redistributable 2012
* itom setup 3.0.0: This plugin has been compiled using the FlyCapture 2.7.3.18, under Windows it requires the Microsoft C++ Redistributable 2012

Linux
======

For linux, please consider the document "Using Linux with USB 3.0", published by Point Grey as technical application note TAN2012007. Starting the camera crashed (or came to a timeout when
calling *startDevice* for cameras with an image size bigger than 2MB if the notes in section **Configuration USBFS** are not considered.
