================================
 OceanOptics Spectrometers
================================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`OceanOpticsSpec`
**Type**:       :plugintype:`OceanOpticsSpec`
**License**:    :pluginlicense:`OceanOpticsSpec`
**Platforms**:  Windows, Linux ready but not tested
**Devices**:    Ocean Optics Spectrometers
**Author**:     :pluginauthor:`OceanOpticsSpec`
=============== ========================================================================================================
 
Overview
========


.. pluginsummaryextended::
    :plugin: OceanOpticsSpec

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: OceanOpticsSpec

Parameters
===========

An instance of this plugin has the following internal parameters:

**average**: {int}
    Number of averages for every frame
**bpp**: {int}, read-only
    Bit depth. The output object is float32 for all cases but uint16 only if no averaging is
    enabled and the dark_correction is disabled or no dark correction pixels are available
    for this sensor.
**detector_name**: {str}, read-only
    Name of the detector.
**integration_time**: {float}
    Integration time of CCD programmed in s, some devices do not accept the full range of
    allowed values (see AvaSpec for real minimum value of your device).
**lambda_coeffs**: {seq. of float}, read-only
    Coefficients for polynom that determines lambda_table (lambda_table[idx] = c[0] +
    c[1]*idx + c[2]*idx^2 + c[3]*idx^3)
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
    
Timestamp
=========

Every acquired image will have a tag 'timestamp' defined. It contains the timestamp of the acquisition in seconds based on QDateTime::currentMSecsSinceEpoch(). If data is averaged, the timestamp of the latest acquisition is used.


  
Changelog
=========

* itom setup 4.1.0: This plugin has been compiled using the libUSB Plugin
* ONLY fully implemented for STS series, other spectrometers using OBP won't fully work.