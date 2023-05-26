==============
 FileGrabber
==============

=============== ========================================================================================================
**Summary**:    :pluginsummary:`FileGrabber`
**Type**:       :plugintype:`FileGrabber`
**License**:    :pluginlicense:`FileGrabber`
**Platforms**:  Windows, Linux
**Devices**:    Virtual file camera (grabbing from image files)
**Author**:     :pluginauthor:`FileGrabber`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: FileGrabber

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: FileGrabber

Parameters
===========

An instance of this plugin has the following internal parameters:

**binning**: {int}, read-only
    Binning of different pixel
**bpp**: {int}, read-only
    Grabdepth of the images
**current_image**: {int}
    The current shown image
**frame_time**: {float}, read-only
    Time between two frames
**gain**: {float}, read-only
    Virtual gain
**integration_time**: {float}
    Integrationtime of CCD programmed in s
**name**: {str}
    GrabberName
**number_of_images**: {int}, read-only
    The maximal number if images
**offset**: {float}, read-only
    Virtual offset
**sizex**: {int}, read-only
    Pixelsize in x (cols)
**sizey**: {int}, read-only
    Pixelsize in y (rows)
**time_out**: {float}, read-only
    Timeout for acquiring images
**x0**: {int}, read-only
    Pixelsize in x (cols)
**x1**: {int}, read-only
    Pixelsize in x (cols)
**y0**: {int}, read-only
    Pixelsize in y (rows)
**y1**: {int}, read-only
    Pixelsize in y (rows)
