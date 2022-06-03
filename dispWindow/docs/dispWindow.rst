===================
 DispWindow
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`DispWindow`
**Type**:       :plugintype:`DispWindow`
**License**:    :pluginlicense:`DispWindow`
**Platforms**:  Windows, Linux
**Devices**:    OpenGL based widget to show cosine and graycode sequences
**Author**:     :pluginauthor:`DispWindow`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: DispWindow

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: DispWindow

Parameters
============

Every instance of this plugin has the following parameters:

**color**: int
    0: Red, 1: Green, 2: Blue, 3: White
    
    *Value range: [0, 3], Default: 3*
**dObj**: dataObject
    DataObject with pixel values to display.
**gamma**: int
    0: disable gamma correction, 1: enable gamma correction; default disable (see also
    'lut')
    
    *Value range: [0, 1], Default: 0*
**lut**: Sequence[int] (char)
    Lookup table for a gamma correction with 256 values. The gamma correction itself is
    en-/disabled via parameter 'gamma'. If enabled, the value to display is modified by
    lut[value]. Per default the lut is a 1:1 relation.
**name**: str, read-only
    name of the plugin
**numgraybits**: int, read-only
    Number of different images: Phaseshift + GrayCode + 2
    
    *Value range: [0, 80], Default: 5*
**numimg**: int
    Number of current image (phase images, dark image, bright image, graycode images)
    
    *Value range: [-1, 10], Default: 0*
**orientation**: int
    0: vertical, 1: horizontal; default: vertical
    
    *Value range: [0, 1], Default: 0*
**period**: int
    Cosine period in pixel. This must be a multiple of 2 and the number of 'phaseshift'.
    
    *Value range: [4:2:4096], Default: 12*
**phaseshift**: int
    Count of phase shifts. If this value is changed and the 'period' does not fit to the new
    value, the 'period' is adapted to the next possible value.
    
    *Value range: [3, 8], Default: 4*
**x0**: int
    x0 position of display window [px]
    
    *Value range: [0, 1920], Default: 1820*
**xsize**: int
    width of window [px]
    
    *Value range: [3, 1920], Default: 100*
**y0**: int
    y0 position of display window [px]
    
    *Value range: [0, 1080], Default: 0*
**ysize**: int
    height of window [px]
    
    *Value range: [3, 1080], Default: 100*

Gamma correction
=================

If the parameter *gamma* is set to 0, no further gamma correction is enabled. This means, that the gray values of the cosine fringes are
directly displayed as they are using a 8bit resolution::

    screen_pixel = round(255 * [2*cos(alpha) + 1])
    
However, if your camera and projector has a non-linear behaviour between displayed and detected gray value, a gamma correction can and should
be enabled. Then, the displayed gray value is::
    
    screen_pixel = lut[round(255 * [2*cos(alpha) + 1])]
    
The lookup table (lut) consists of 256 values. In order to register the lookup table, you can project a uniform background with uniformly distributed
gray-values (at least 64 different values) between 0 and full resolution (e.g. 255). The values should be sorted beginning with the darkest one until 'white'::
    
    for i in range(0,256,4):
        projector.exec("projectGrayValue", i)
    
.. note::
    
    displaying any other image (graycode or cosine fringe) will clear the gray valued image

Then, record every uniform projection with the camera and determine the mean gray value. You then get a list of gray values, like the following one::

    g = [25, 28, 32, 38, 50, 90, 92, 96, ... 198]
    
Call the **exec** function **calcLut** and pass *g* in order to let the plugin calculate the *lut* (based on a linear interpolation if *g* contains less values
than 256)::
    
    projector.exec("calcLut", g)
    
Then, you need to enable the gamma correction::
    
    projector.setParam("gamma", 1)
    
Finally, the *lut* (parameter *lut*) contains values, such that the recorded gray values applying the same uniformly distributed background images are linearly distributed.

Save current view to image file
================================

Use the exec function **grabFramebuffer** to save the currently displayed view in the same size to an image file (the file type is determined by the file's ending (file type must be
supported by Qt's QImage)::
    
    projector.exec("grabFramebuffer", "D:/test.pgm") #or
    projector.exec("grabFramebuffer", "D:/test.jpg")
