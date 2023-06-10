================================
 Avantes AvaSpec Spectrometer
================================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`AvantesAvaSpec`
**Type**:       :plugintype:`AvantesAvaSpec`
**License**:    :pluginlicense:`AvantesAvaSpec`
**Platforms**:  Windows, Linux ready but not tested
**Devices**:    Avantes AvaSpec Spectrometer
**Author**:     :pluginauthor:`AvantesAvaSpec`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: AvantesAvaSpec

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: AvantesAvaSpec

Parameters
===========

An instance of this plugin has the following internal parameters:

**average**: {int}
    Number of averages for every frame
**bpp**: {int}, read-only
    Bit depth. 16 (uint16), if single acquisition. 32 (float32), if averaging.
**integration_time**: {float}
    Integration time of CCD programmed in s, some devices do not accept the full range of allowed values (see AvaSpec for real minimum value of your device).
**lambda_coeffs**: {seq. of float}, read-only
    Coefficients for polynom that determines lambda_table (lambda_table[idx] = c[0] + c[1]*idx + c[2]*idx^2 + ... + c[4]*idx^4)
**lambda_table**: {seq. of float}, read-only
    Vector with the wavelength of all active pixels
**name**: {str}, read-only
    plugin name
**roi**: {int rect [x0,y0,width,height]}
    ROI (x,y,width,height)
**serial_number**: {str}, read-only
    Serial number of spectrometer. Same as identifier.
**sizex**: {int}, read-only
    current width of ROI
**sizey**: {int}, read-only
    current height
**detector_name**: {str}, read-only
    Name of the detector.
**dark_correction**: {int}
    Dark correction mode. See section below.

Timestamp
=========

Every acquired image will have a tag 'timestamp' defined. It contains the timestamp of the acquisition in seconds since the
startup of the spectrometer.

Vendor and Product ID
======================

If the vendor or product ID is unknown, open the libUsb plugin and select the optional parameter "print info of all devices". This will give you a list of all usb devices connectable with libUSB.

Dark correction
================

Some spectrometers have more pixels on the sensor than are used for the signal generation. These, covered and hence dark pixels, can be used for a dark
correction. There are three types of dark correction (parameter dark_correction):

* Off (0): No dark correction is applied, if the sensor is recognized to have such dark pixels, a tag 'dark' is created that contains the mean value of all dark pixels.
  If no averaging is enabled, the output format of the dataObject is uint16, else float32. Sensors, that don't have dark pixels can only be operated in this mode.
* Static (1): The mean value of all dark pixels is subtracted from all pixels. The output format is float32 always (negative values might occur).
* Dynamic (2): Only choose this mode, if the software AvaSpec provides dynamic dark correction for the sensor (see sensor configuration >> checkbox 'dynamic dark correction'
  must be enabled. In this case, odd and even pixels have different dark correction values (they are probably read by different electronics). Choose this mode to
  subtract different mean values for even and odd pixels. The tag 'dark' still contains the mean of both mean values (0.5 * (mean_even + mean_odd)).

Changelog
=========

* itom setup 3.1.0: This plugin has been compiled using the libUSB Plugin
* itom setup 4.1.0: This plugin has been compiled using the libUSB Plugin
