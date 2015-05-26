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