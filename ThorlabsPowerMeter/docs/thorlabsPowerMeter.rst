=======================
 Thorlabs Power Meter
=======================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsPowerMeter`
**Type**:       :plugintype:`ThorlabsPowerMeter`
**License**:    :pluginlicense:`ThorlabsPowerMeter`
**Platforms**:  Windows
**Devices**:    Thorlabs Power and Energy Meter Consoles PM100x
**Author**:     :pluginauthor:`ThorlabsPowerMeter`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: ThorlabsPowerMeter

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: ThorlabsPowerMeter
     
Parameters
==========
**attenuation**: {float}
    attenuation [db]
**auto_range**: {int}
     shows if the auto power range is wether on (1) or off(2) 
**average_number**: {int}
    defines the number of measurements to be averaged
**bandwidth**: {int}
    defines if the input filter state is whether High (0) or Low (1)
**calibration_message**: {str}, read-only
    calibration message
**dark_offset**: {float}, read-only
    setted dark offset [unknown]
**device_name**: {str}, read-only
    device name
**firmware_revision**: {str}, read-only
    firmware revision
**line_frequency**: {int}
     line frequency of 50Hz or 60Hz
**manufacturer_name**: {str}, read-only
    manufacturer name
**measurement_mode**: {str}
    absolute or relative
**name**: {str}, read-only
    name of the device
**power_range**: {float}
    power range [W]; will be set to the next bigger possible value
**reference_power**: {float}
    reference power for relative measurements
**serial_number**: {str}, read-only
    serial number
**wavelength**: {float}
    wavelength [nm]
    
Additional functions (exec functions)
=====================================

.. py:function::  instance.exec('zero_device', )

    function to set the zero value of the device    
Installation
============

For using this plugin, please install the PM100x_Instrument_Driver that is shipped with the Thorlabs power meter.
        
Changelog
=========

itom 3.0.0: plugin uses the driver PM100x_Instrument_Driver in version 3.0.2
