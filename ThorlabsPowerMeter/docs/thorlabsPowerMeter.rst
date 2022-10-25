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

Compilation
===========

It is possible to use the Thorlabs power meter with the legacy Thorlabs software **ThorlabsPowerMeter_1.0.2**
(see https://www.thorlabs.de/software_pages/ViewSoftwarePage.cfm?Code=PM100x). Install this software and set the
following CMake variables:

* THORLABS_POWER_METER_API_VERSION: 1.02
* THORLABS_PM100D_VISA_DIR: <path to the install directory of Thorlabs PM100D Visa, e.g. C:/Program Files/IVI Foundation/VISA/Win64>

Alternatively you can use the Thorlabs Optical Power Meter Monitor Software **Thorlabs.OpticalPowerMonitor.1.1**
(see https://www.thorlabs.de/software_pages/ViewSoftwarePage.cfm?Code=OPM). Install this software and set the 
CMake variables to the following values:

* THORLABS_POWER_METER_API_VERSION: 1.1
* THORLABS_PM100D_VISA_DIR: <path to the install directory of Thorlabs PM100D Visa, e.g. C:/Program Files/IVI Foundation/VISA/Win64>

.. note::
    
    If you want to change the version of an existing configuration, please remove all related THORLABS_PM100D... variables, set the
    new version and press configure.


Exemplary usage from Python
============================

In the following script, the first detectable power meter is connected and a oscilloscope-like
plot is opened that displays a moving graph of recent intensity values:

.. code-block:: python

    if not "pmXXX" in globals():
        pmXXX = dataIO("ThorlabsPowerMeter", "")

    numPoints = 1000
    image = dataObject.zeros([1,numPoints],'float64')
    [i,plot_handle] = plot1(image)

    def timeout():
        global timer_id
        d = dataObject()
        pmXXX.acquire() #acquire new intensity value
        
        image[0,0:numPoints-1] = image[0,1:] #shift pixels to the left by one...
        
        pmXXX.getVal(d) #get the recently acquired value
        image.copyMetaInfo(d)
        image[0,numPoints-1] = d[0,0] #...append new value to the end of image
        
        if plot_handle.exists():
            try:
                plot_handle["source"] = image #update the displayed image
            except:
                pass
        else:
            print("Figure has been closed. Stop acquisition...")
            timer_id.stop()
            del timer_id

    timer_id = timer(50, timeout) #call timeout every 50ms


Changelog
=========

* itom 3.0.0: plugin uses the driver PM100x_Instrument_Driver in version 3.0.2
* itom 3.1.0: plugin uses the driver PM100x_Instrument_Driver in version 1.0.2 (Thorlabs has changed the major version number again)
* Due to the chaotic version handling of Thorlabs PowerMeter, the source code is changed such that only version 3.0.2 is no longer supported. 
* itom 3.2.1: plugin uses the driver PM100x_Instrument_Driver in version 1.1.2317.102 
* itom 4.0.0: plugin uses the driver PM100x_Instrument_Driver in version 2.2
* itom 4.1.0: plugin uses the driver PM100x_Instrument_Driver in version 2.2