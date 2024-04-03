===================
 GLDisplay
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`GLDisplay`
**Type**:       :plugintype:`GLDisplay`
**License**:    :pluginlicense:`GLDisplay`
**Platforms**:  Windows, Linux
**Devices**:    OpenGL based widget to show any textures given by dataObjects
**Author**:     :pluginauthor:`GLDisplay`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: GLDisplay

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: GLDisplay

Parameters
===========

An instance of this plugin has the following parameters:

**color**: {int}
    0: Red, 1: Green, 2: Blue, 3: White
**currentIdx**: {int}
    Index of currently displayed image [0..)
**gamma**: {int}
    0: disable gamma correction, 1: enable gamma correction; default disable (see also 'lut')
**lut**: {seq. of int (char)}
    Lookup table for a gamma correction with 256 values. The gamma correction itself is en-/disabled via parameter 'gamma'. If enabled, the value to display is modified by lut[value]. Per default the lut is a 1:1 relation.
**name**: {str}, read-only
    name of the plugin
**numImages**: {int}, read-only
    Number of different images
**x0**: {int}
    x0 position of display window [px]
**xsize**: {int}
    width of window [px]
**y0**: {int}
    y0 position of display window [px]
**ysize**: {int}
    height of window [px]

Example
========

The following example shows how to project different vertical and horizontal sine textures as well as random maps that
are either horizontally or vertically repeated or spread to the real size of the display:

.. code-block:: python

    import numpy as np

    gl = dataIO("GLDisplay")

    #4-phase shifted sine (8px width), repeated
    s1 = [127*(1+np.sin(j*2*np.pi/8)) for j in range(0,8)]
    s2 = [127*(1+np.sin(np.pi/4 + j*2*np.pi/8)) for j in range(0,8)]
    s3 = [127*(1+np.sin(np.pi/2 + j*2*np.pi/8)) for j in range(0,8)]
    s4 = [127*(1+np.sin(3*np.pi/4 + j*2*np.pi/8)) for j in range(0,8)]
    sine = dataObject([4,1,8],'uint8',data=s1+s2+s3+s4)
    sine.setTag("wrapT","GL_REPEAT")
    sine.setTag("wrapS","GL_REPEAT")
    gl.exec("addTextures",sine)

    #4-phase shifted sine (8px width), scaled
    sine2 = dataObject([4,8,1],'uint8',data=s1+s2+s3+s4)
    sine2.setTag("wrapT","SCALED")
    sine2.setTag("wrapS","SCALED")
    gl.exec("addTextures",sine2)

    #random value, repeated
    ra = dataObject.randN([4,1024,768],'uint8')
    ra.setTag("wrapT","SCALED")
    ra.setTag("wrapS","SCALED")
    gl.exec("addTextures",ra)

It is also possible to display coloured dataObjects (type: rgba32). If so, make sure that the gamma correction parameter is
set to False. Else, the red-channel of the coloured data object is used for lookup in the gamma correction LUT:

.. code-block:: python

    a = dataObject([10,10],'rgba32')
    a[0:3,:] = rgba(255,0,0) #first three lines are red
    a[3:6,:] = rgba(0,255,0) #next three lines are green
    a[6:9,:] = rgba(0,0,255) #three lines blue
    a[9:10,:] = rgba(255,255,255) #...and one line in white
    gl.exec("addTextures",a)

Another feature is to edit one or a sequence of texture(s). This work in the same way than adding texture, however by the
function 'editTextures'. This function requires the texture data object as first argument and the texture index that is replaced (zero-based).
If the given data object is 3D and has more than one plane, the following textures are replaced as well:

.. code-block:: python

    gl.exec("editTextures",dataObject.randN([100,100]), firstTextureIndex=0)
