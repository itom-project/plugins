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
    Bitdepth of the returned camera image (e.g. 8, 12 or 16 bits per pixel). Please consider that some cameras having an 12bit A/D converter
    allow to obtain 16bit values. This possible however results in a bigger data volume that needs to be transmitted. Consider to use 12bit instead
    that is also mapped to an uint16 data object. At startup, the highest possible value is set, it can be limited by the initialization parameter **bppLimit**.
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
    Camera brightness control (EV) in absolute values. Read-only if not available.
**extended_shutter**: {int}
    1 (default): extended shutter is on (long integration times are supported and frame_time becomes invalid), 0: frames are only acquired in the pulse given by frame_time.
**frame_time**: {float}, read-only
    frame time in seconds, inverse of frame rate in fps. The frame time is a constant time step at which images can be acquired. The frame time is only writeable if extended shutter is False. If the extended shutter is enabled, the maximum acquisition speed is given by the integration time only.
**gain**: {float}
    The normalized gain of the camera [0,1]. Read-only if not available.
**gamma**: {int}
    Camera gamma control in absolute values. Read-only if not available.
**integration_time**: {float}
    integration time in seconds. The range of the integration is limited if *extended_shutter* is False. Enable extended shutter to set bigger values for the integration time.
**name**: {str}, read-only
    name of the camera
**metadata**: {int}
	If 1 (default), the timestamp, frame counter and roi position (depending on the camera model) will be acquired and added into the first pixels of the image (available as tag of the data object as well), 0: metadata disabled
**offset**: {float}
    The normalized offset of the camera[0,1]. Read-only if not available. Corresponds to Point Grey property Brightness.
**packetsize**: {int}
    Packet size of current image settings
**roi**: {int rect [x0,y0,width,height]}
    Current region of interest of camera. Format (x0,y0,width,height). Change one of those parameters by using *setParam("roi[1]",16)*.
**sharpness**: {int}
    Camera sharpness control in absolute values. Read-only if not available.
**sizex**: {int}, read-only
    Pixelsize in x (cols)
**sizey**: {int}, read-only
    Pixelsize in y (rows)
**timeout**: {float}
    Timeout in seconds when acquiring an image and waiting for the resulting data.
**trigger_mode**: {int}
    -1: Complete free run, 0: Disable trigger, 1: enable trigger mode, 2: enable software-trigger
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

Linux
======

For linux, please consider the document "Using Linux with USB 3.0", published by Point Grey as technical application note TAN2012007. Starting the camera crashed (or came to a timeout when
calling *startDevice* for cameras with an image size bigger than 2MB if the notes in section **Configuration USBFS** are not considered.
