===================
 OphirPowermeter
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`OphirPowermeter`
**Type**:       :plugintype:`OphirPowermeter`
**License**:    :pluginlicense:`OphirPowermeter`
**Platforms**:  Windows, (Linux possible but yet not implemented)
**Devices**:    Powermeter VEGA from company *Ophir*
**Author**:     :pluginauthor:`OphirPowermeter`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: OphirPowermeter
    
Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: OphirPowermeter

Parameters
==========

**ROMVersion**: {str}, read-only
    Version of ROM software.
**battery**: {int}, read-only
    1 if battery is OK, 0 if battery is low.
**calibrationDueDate**: {str}, read-only
    Calibration due date.
**comPort**: {int}, read-only
    The current com-port ID of this specific device. -1 means undefined.
**connection**: {str}, read-only
    type of the connection ('RS232', 'USB').
**deviceType**: {str}, read-only
    Device type (NOVA, VEGA, LASERSTAR-S (single channel), LASERSTAR-D (dual channel), Nova-
    II).
**headName**: {str}, read-only
    Head name connected to the device.
**headSerialNumber**: {str}, read-only
    Head serial number connected to the device.
**headType**: {str}, read-only
    Head type (thermopile, BC20, temperature probe, photodiode, CIE head, RP head,
    pyroelectric, nanoJoule meter, no head connected.
**measurementType**: {str}
    Measurement type (energy or power).
**name**: {str}, read-only
    Name of plugin.
**range**: {int}
    Measurement range (0: AUTO, 1: 150W, 2: 50.0W, 3: 5.00W).
**serialNumber**: {str}, read-only
    Serial number of the device shown on display.
**timeout**: {int}, read-only
    Request timeout, default 1000 ms.
**unit**: {str}
    Unit of device
**wavelength**: {str}
    Available discrete wavelengths: 10.6 .8-6 <.8u.
**wavelengthSet**: {str}, read-only
    Setting of the measurement wavelength (DISCRETE or CONTINUOUS).



Exemplary usage from Python
============================

In the following script, the first detectable power meter is connected and a oscilloscope-like
plot is opened that displays a moving graph of recent intensity values:

.. code-block:: python
    
    connectionType = "USB"

    if connectionType == "USB":  # connects the USB powermeter type
        adda = dataIO("OphirPowermeter", connection=connectionType)  # connect a USB Powermeter
    elif connectionType == "RS232":  # connects the RS232 powermeter type with the additional SerialIO instance
        port = 4
        baud = 9600
        endline = '\n\r'
        adda = dataIO("OphirPowermeter", connection=connectionType, serial=dataIO("SerialIO", port, baud, endline))

    numPoints = 1000
    image = dataObject.zeros([1, numPoints], 'float64')
    [i, plotHandle] = plot1(image)


    def timeout():
        global timerId
        d = dataObject()
        adda.acquire()  # acquire new intensity value

        image[0, 0: numPoints - 1] = image[0, 1:]  # shift pixels to the left by one...

        adda.getVal(d)  # get the recently acquired value
        image.copyMetaInfo(d)
        image[0, numPoints - 1] = d[0, 0]  # ...append new value to the end of image

        if plotHandle.exists():
            try:
                plotHandle["source"] = image  # update the displayed image
            except:
                pass
        else:
            print("Figure has been closed. Stop acquisition...")
            timerId.stop()
            del timerId


    timerId = timer(50, timeout)  # call timeout every 50ms

Changelog
=========

* itom setup 4.1.0: This plugin has been compiled using the StarLab 3.60.0