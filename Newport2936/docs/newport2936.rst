===================
 Newport2936
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`Newport2936`
**Type**:       :plugintype:`Newport2936`
**License**:    :pluginlicense:`Newport2936`
**Platforms**:  Windows
**Devices**:    Newport 1931-C, Newport 1936-R and Newport 2936-R
**Author**:     :pluginauthor:`Newport2936`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: MyGrabber
    

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: MyGrabber
        
Parameters
==========

**attenuatorA**: {int}
    Attenuator enabled (1) or disabled (0) for channel A
**attenuatorB**: {int}
    Attenuator enabled (1) or disabled (0) for channel B
**bpp**: {int}
    bpp
**channels**: {int}, read-only
    Number of channels (single: 1, dual: 2)
**controller_serial**: {str}, read-only
    Controller serial number
**filterTypeA**: {int}
    Filter type of channel A: (0) no filtering, (1) analog filter, (2) digital filter, (3) analog and digital filter
**filterTypeB**: {int}
    Filter type of channel B: (0) no filtering, (1) analog filter, (2) digital filter, (3) analog and digital filter
**firmware_date**: {str}, read-only
    Firmware date (mm/dd/yy)
**firmware_version**: {str}, read-only
    Firmware version #
**integrationTime**: {float}
    Integrationtime of CCD [0..1] (no unit)
**manufacturer_name**: {str}, read-only
    Manufacturer name
**model_name**: {str}, read-only
    Model name
**name**: {str}, read-only
    
**offsetValueA**: {float}
    Offset for zero value channel A [W]
**offsetValueB**: {float}
    Offset for zero value channel 2 [W]
**wavelengthA**: {int}
    Wavelength [nm] for channel A
**wavelengthB**: {int}
    Wavelength [nm] for channel B
    
Additional functions (exec functions)
=====================================

.. py:function::  instance.exec('zero_device', channel)

    Function to set the zero value of the device to the current Value

    :param channel: Channel
    :type channel: int

.. py:function::  instance.exec('zero_device_to', value, channel)

    Zero device to a specific value

    :param value: Zero Value of Device in Watts
    :type value: float
    :param channel: Channel
    :type channel: int

Changelog
=========

* itom setup 3.1.0: This plugin has been compiled using Newport USB Driver 5.0.8
* itom setup 3.2.1: This plugin has been compiled using Newport USB Driver 5.0.8
* itom setup 4.0.0: This plugin has been compiled using Newport USB Driver 5.0.8
* itom setup 4.1.0: This plugin has been compiled using Newport USB Driver 5.0.8