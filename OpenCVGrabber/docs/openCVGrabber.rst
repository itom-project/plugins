===================
 OpenCVGrabber
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`OpenCVGrabber`
**Type**:       :plugintype:`OpenCVGrabber`
**License**:    :pluginlicense:`OpenCVGrabber`
**Platforms**:  Windows, Linux
**Devices**:    USB Plug&Play Cameras and further cameras supported by OpenCV
**Author**:     :pluginauthor:`OpenCVGrabber`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: OpenCVGrabber

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: OpenCVGrabber

Parameters
===========

An instance of this plugin has the following parameters:

**backlightCompensation**: float
    backlightCompensation [0..1]
    
    *Value range: [0:0.5:1], Default: 0.5*
**backlightCompensationAuto**: int
    auto-controlled backlightCompensation (on:1, off:0)
    
    *Value range: [0, 1], Default: 0*
**bpp**: int
    bpp
    
    *Value range: [8, 8], Default: 8*
**brightness**: float
    brightness [0..1]
    
    *Value range: [0:0.00392157:1], Default: 0.501961*
**brightnessAuto**: int
    auto-controlled brightness (on:1, off:0)
    
    *Value range: [0, 1], Default: 0*
**colorMode**: str
    color mode of camera (auto|color|red|green|blue|gray, default: auto -> color or gray)
    
    *Match: ["auto", "color", "red", "green", "blue", "gray"], Default: "auto"*
**contrast**: float
    contrast [0..1]
    
    *Value range: [0:0.00392157:1], Default: 0.12549*
**contrastAuto**: int
    auto-controlled contrast (on:1, off:0)
    
    *Value range: [0, 1], Default: 0*
**deviceName**: str, read-only
    name of device
**gamma**: float
    gamma [0..1]
    
    *Value range: [0:0.0166667:1], Default: 0.5*
**gammaAuto**: int
    auto-controlled gamma (on:1, off:0)
    
    *Value range: [0, 1], Default: 0*
**hue**: float
    hue [0..1]
    
    *Value range: [0:0.00277778:1], Default: 0.5*
**hueAuto**: int
    auto-controlled hue (on:1, off:0)
    
    *Value range: [0, 1], Default: 0*
**integrationTime**: float
    Integrationtime of CCD in seconds
    
    *Value range: [0.000244141, 0.125], Default: 0.015625*
**integrationTimeAuto**: int
    auto-controlled integration time of CCD (on:1, off:0)
    
    *Value range: [0, 1], Default: 1*
**name**: str, read-only
    name of the plugin
**roi**: Tuple[int,int,int,int] (rect [x0,y0,width,height])
    ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]
    
    *Height: Value range: [0, 719], Default: [0, 0, 1280, 720]*
**saturation**: float
    saturation [0..1]
    
    *Value range: [0:0.01:1], Default: 0.64*
**saturationAuto**: int
    auto-controlled saturation (on:1, off:0)
    
    *Value range: [0, 1], Default: 0*
**sharpness**: float
    sharpness [0..1]
    
    *Value range: [0:0.142857:1], Default: 0*
**sharpnessAuto**: int
    auto-controlled sharpness (on:1, off:0)
    
    *Value range: [0, 1], Default: 0*
**sizex**: int, read-only
    width of ROI (x-direction)
    
    *Value range: [1, 1280], Default: 1280*
**sizey**: int, read-only
    height of ROI (y-direction)
    
    *Value range: [1, 720], Default: 720*
**whiteBalance**: float
    whiteBalance [0..1]
    
    *Value range: [0:0.0027027:1], Default: 0.486486*
**whiteBalanceAuto**: int
    auto-controlled whiteBalance (on:1, off:0)
    
    *Value range: [0, 1], Default: 1*
**x0**: int
    first pixel index in ROI (x-direction)
    
    *Value range: [0, 1279], Default: 0*
**x1**: int
    last pixel index in ROI (x-direction)
    
    *Value range: [0, 1279], Default: 1279*
**y0**: int
    first pixel index in ROI (y-direction)
    
    *Value range: [0, 719], Default: 0*
**y1**: int
    last pixel index in ROI (y-direction)
    
    *Value range: [0, 719], Default: 719*
**zoom**: float
    zoom [0..1]
    
    *Value range: [0:0.0333333:1], Default: 0*
**zoomAuto**: int
    auto-controlled zoom (on:1, off:0)
    
    *Value range: [0, 1], Default: 0*