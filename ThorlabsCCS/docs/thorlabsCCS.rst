===================
 Thorlabs CCS
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsCCS`
**Type**:       :plugintype:`ThorlabsCCS`
**License**:    :pluginlicense:`ThorlabsCCS`
**Platforms**:  Some words about supported operating systems
**Devices**:    Some words about supported devices
**Author**:     :pluginauthor:`ThorlabsCCS`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: ThorlabsCCS

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: ThorlabsCCS
        
Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    ThorlabsCCS
**roi**: {int seq.}
    ROI (x,y,width,height)
**sizex**: {int}, read-only
    width of ROI (x-direction)
**sizey**: {int}, read-only
    height of ROI (y-direction)
**bpp**: {int}
    bpp
**integration_time**: {float}
    integration time of ccd in sec
**manufacturer_name**: {str}, read-only
    manufacturer name
**device_name**: {str}, read-only
    device name
**serial_number**: {str}, read-only
    serial number
**firmware_revision**: {str}, read-only
    firmware revision
**instrument_driver_revision**: {str}, read-only
    instrument driver revision
**wavelength_data**: {float seq.}, read-only
    wavelength in nm (air) for each pixel
	
Changelog
=========

* itom setup 3.1.0: This plugin has been compiled using the ThorlabsOSASW SDK 2.80
* itom setup 3.2.1: This plugin has been compiled using the ThorlabsOSASW SDK 2.85
* itom setup 4.0.0: This plugin has been compiled using the ThorlabsOSASW SDK 2.90