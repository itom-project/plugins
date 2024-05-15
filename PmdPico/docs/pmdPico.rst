===================
 PmdPico
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PmdPico`
**Type**:       :plugintype:`PmdPico`
**License**:    :pluginlicense:`PmdPico`
**Platforms**:  Windows
**Devices**:    pico flexx, pico maxx, pico monster
**Author**:     :pluginauthor:`PmdPico`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: PmdPico

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

**acquisition_mode**: {int}
    Indicates which data should be recorded. 0: depth data, 1: gray value, 2: confidence of depth, 3: all
**auto_exposure**: {int}
    Indicates if the integration time is set automatically (1) or Manual (0)
**bpp**: {int}, read-only
    bpp of gray value image
**cam_number**: {int}, read-only
    index of the camera device
**data_mode**: {int}
    Indicates whether depth data (0), gray value (1) or confidence map (2) is transferred when using copyVal, getVal or the live image
**framerate**: {int}
    framerate of image acquisition (in fps). This parameter reflects the current framerate.
**integration_time**: {float}
    integration time of [sec]
**name**: {str}, read-only
    PmdPico
**sensor_name**: {str}, read-only
    sensor name of the attached camera
**serial_number**: {str}, read-only
    serial number of device.
**sizex**: {int}, read-only
    width (x-direction)
**sizey**: {int}, read-only
    height (y-direction)
**use_case**: {str}
    current use case of camera. To get a list of all available use cases call the getUseCases exec function. Note the mixed modes are not supported yet.
**x0**: {int}, read-only
    first pixel index (x-direction)
**x1**: {int}, read-only
    last pixel index (x-direction)
**y0**: {int}, read-only
    first pixel index (y-direction)
**y1**: {int}, read-only
    last pixel index (y-direction)

Additional functions (exec functions)
=====================================

.. py:function::  instance.exec('getCoordinates', xCoordinate, yCoordinate)

    returns the coordinates map of the acquired image.

    :param xCoordinate: x-coordinate map of the same shape as the current image
    :type xCoordinate: dataObject
    :param yCoordinate: y-coordinate map of the same shape as the current image
    :type yCoordinate: dataObject

.. py:function::  instance.exec('getUseCases', )

    prints out the available use cases.

Installation
============

For using this plugin, please install the Royale software that is shipped with your PMD device.
Or got to the website and request a download link from the vendor:
https://pmdtec.com/

Make sure that the path to the royale.dll is added to your system path variables.

Set the CMAKE variable **PmdPico_ROYALE_DIR** or the environment variable **PMD_ROYALE_ROOT**
to the PMD Royale SDK installation folder (e.g. C:\Program Files\royale\4.24.0.1201)

Changelog
=========

* itom setup 3.1.0: plugin uses the Royale SDK in version 3.12.0
* itom setup 3.2.1: plugin uses the Royale SDK in version 3.21.1
* itom setup 4.0.0: plugin uses the Royale SDK in version 3.21.1
* itom setup 4.1.0: plugin uses the Royale SDK in version 3.21.1
* itom setup 4.3.0: plugin uses the Royale SDK in version 4.24.0
