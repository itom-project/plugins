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
**frame_time**: {float}, read-only
    Shortest time between two frames of CCD in s
**integration_time**: {float}
    Integrationtime of CCD programmed in s
**lambda_coeffs**: {seq. of float}, read-only
    Coefficients for polynom that determines lambda_table (lambda_table[idx] = c[0] + c[1]*idx + c[2]*idx^2 + ... + c[4]*idx^4)
**lambda_table**: {seq. of float}, read-only
    Vector with the wavelength of all active pixels
**name**: {str}, read-only
    PlugIn-Name
**roi**: {int rect [x0,y0,width,height]}
    ROI (x,y,width,height)
**serial_number**: {str}, read-only
    Serial number of spectrometer. Same as identifier.
**sizex**: {int}, read-only
    current width of ROI
**sizey**: {int}, read-only
    current height
**time_out**: {float}
    Timeout for grabbing in s
**trigger_enable**: {int}, read-only
    Enable triggermode
**trigger_mode**: {int}
    Set Triggermode