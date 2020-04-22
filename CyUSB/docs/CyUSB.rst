===================
 CyUSB
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`CyUSB`
**Type**:       :plugintype:`CyUSB`
**License**:    :pluginlicense:`CyUSB`
**Platforms**:  Windows 
**Devices**:    Any generic USB devices
**Author**:     :pluginauthor:`CyUSB`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: CyUSB

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: CyUSB
        
Parameters
===========

An instance of this plugin has the following internal parameters:

**debug**: {int}
    If true, all out and inputs are written to the dockWidget.
**endpoint_read**: {int}
    Endpoint index for reading operations. (default: first detected input endpoint.)
**endpoint_write**: {int}
    Endpoint index for writing operations. (default: first detected output endpoint.)
**name**: {str}
    name of device
**number_of_devices**: {int}
    maximum number of detected devices
**timeout**: {float}
    Timeout for reading commands in [s].
    
Usage
======

Any USB device is defined by its vendor and product ID. Both are usually hex-values. If the connection is established via the GUI dialog, the corresponding integer
values need to be given. If you have no idea about the vendor and product ID, set the optional initialization parameter *printInfoAboutAllDevices* to 1 (True). Then,
information about all connected USB devices including its name and vendor / product ID is printed to the console.

Once the device is identified, you need to indicate a so called endpoint ID. The read- and write-communication with the device is done via several endpoints. All reading endpoints
are in the range 128..255 while the writing endpoints are between 0..127. For many devices the corresponding read/write endpoints have the same offset between its limit 0 and 128.
Therefore, the initialization only contains the parameter 'endpoint', such that *endpoint_read* is set to 128+endpoint and *endpoint_write* to 'endpoint'. You can always change
the endpoints using the specific parameters.

Again, if you set *printInfoAboutAllDevices* to 1 at initialization, the available endpoints are also print to the console for the selected device. If the list can not resolve further
information for your selected device, this device can not be used by this generic plugin. 

Once the USB-device is opened and the endpoints are configured, you can send and read data in the same way than via a serial connection (see help for plugin **SerialIO**).::

    #send values:
    myCommand = bytearray(128)
    usbDevice.setVal(myCommand)
    
    #read values:
    deviceAnswer = bytearray(128) #buffer
    usbDevice.getVal(deviceAnswer)
    print(deviceAnswer) #number of characters read
    
The command **getVal** only reads the number of characters that arrived at the current endpoint at the moment of its call. Analyze the return value and probably call **getVal**
again if you expect more characters to arrive. This is also the same behaviour than for serial connections.

This plugin is also used by other hardware plugins to communicate with further devices.
        
Compilation
===========
In order to compile CyUSB, get the Cypress Seminconductor SDK from: http://www.cypress.com/file/135301?finished=1 (e.g. CY3684Setup.exe). Install the SDK (select typical as 
setup type to install the SDK components; you can quit installing the 3rd party softwares uVision2 and GPIF Designer). Then set the CMake variable CyAPI_INCLUDE_DIR to a
directory similar than **C:\\Program Files\\Cypress\\USB\\CY3684_EZ-USB_FX2LP_DVK\\1.1\\Windows Applications\\library\\cpp\\inc**.

Run plugin
==========
In order to run a device via the CyUSB plugin, you also need to install the drivers from cypress.com. This is already done if you installed the SDK like stated above.

Changelog
=========

* itom setup 2.2.0: This plugin has been compiled using the Cypress CyAPI 1.1
* itom setup 2.2.1: This plugin has been compiled using the Cypress CyAPI 1.3.3
* itom setup 3.0.0: This plugin has been compiled using the Cypress CyAPI 1.3.3
* itom setup 3.1.0: This plugin has been compiled using the Cypress CyAPI 1.3.3
* itom setup 3.2.1: This plugin has been compiled using the Cypress CyAPI 1.3.3
* itom setup 4.0.0: This plugin has been compiled using the Cypress CyAPI 1.3.4

