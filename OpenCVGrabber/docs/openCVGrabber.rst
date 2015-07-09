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

**bpp**: {int}
    bpp
**brightness**: {float}
    Brightness, this is a device dependent property directly redirected to the driver without unified physical unit (not available for all plugins)
**color_mode**: {str}
    color mode of camera (auto|color|red|green|blue|gray, default: auto -> color or gray)
**contrast**: {float}
    Contrast, this is a device dependent property directly redirected to the driver without unified physical unit (not available for all plugins)
**dump_grabs**: {int}
    number of useless images acquired before each real acquisition. This might be necessary since some devices have an internal buffer and deliver older images than the time of acquisition
**exposure**: {float}
    Exposure time, this is a device dependent property directly redirected to the driver without unified physical unit (not available for all plugins)
**gain**: {float}
    Gain, this is a device dependent property directly redirected to the driver without unified physical unit (not available for all plugins)
**gamma**: {float}
    Gamma, this is a device dependent property directly redirected to the driver without unified physical unit (not available for all plugins)
**hue**: {float}
    Hue, this is a device dependent property directly redirected to the driver without unified physical unit (not available for all plugins)
**name**: {str}, read-only
    name of the plugin
**native_parameter**: {float}, read-only
    use 'native_parameter:idx' to request the value of a native OpenCV parameter. idx is the enumeration value for enum items starting with CV_CAP_PROP...
**roi**: {int rect [x0,y0,width,height]}
    ROI (x,y,width,height)
**saturation**: {float}
    Saturation, this is a device dependent property directly redirected to the driver without unified physical unit (not available for all plugins)
**sharpness**: {float}
    Sharpness, this is a device dependent property directly redirected to the driver without unified physical unit (not available for all plugins)
**sizex**: {int}, read-only
    width of ROI (x-direction)
**sizey**: {int}, read-only
    height of ROI (y-direction)