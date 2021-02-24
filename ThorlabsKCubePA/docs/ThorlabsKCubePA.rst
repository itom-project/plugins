=================================
 Thorlabs KCube Position Aligner
=================================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsKCubePA`
**Type**:       :plugintype:`ThorlabsKCubePA`
**License**:    :pluginlicense:`ThorlabsKCubePA`
**Platforms**:  Windows
**Devices**:    Plugin to control a KCube Position Aligner control unit for PSD devices
**Author**:     :pluginauthor:`ThorlabsKCubePA`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: ThorlabsKCubePA

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: ThorlabsKCubePA

Parameters
===========

These parameters are available and can be used to configure the **ThorlabsKCubePA** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**deviceName**: {str}, read-only
    Description of the device
**serialNumber**: {str}
    serial number of the device
**name**: {str}, read-only
    ThorlabsKCubePA

Usage example
==============

The following example continuously acquires the X/Y position differences of two PSDs, plots them in an endless monitor and logs the position in a csv log file:

.. code-block:: python
    
    import time
    import gc

    if "psd1" in globals():
        del psd1
        gc.collect()
    if "psd2" in globals():
        del psd2
        gc.collect()

    psd1 = dataIO("ThorlabsKCubePA", "69250785")
    psd2 = dataIO("ThorlabsKCubePA", "69250726")

    psd1.startDevice()
    psd2.startDevice()

    delay = 0.2 #seconds

    #open plot
    buffer = 1000 #length of plot
    buf1 = dataObject() #temporary buffer for data retrieval
    buf2 = dataObject()

    data = dataObject.nans([4,buffer], 'float64')
    data.setTag("title", "Position differences")
    data.setTag("legendTitle0", "PSD1, dx")
    data.setTag("legendTitle1", "PSD1, dy")
    data.setTag("legendTitle2", "PSD2, dx")
    data.setTag("legendTitle3", "PSD2, dy")

    close('all')
    [_,h] = plot1(data, properties = {"legendPosition":"Right"})

    #open logfile
    with open("log.csv", "wt") as fp:
        fp.write("timestamp;psd1_x;psd1_y;psd2_x;psd2_y\n")
        
        counter = 0
        try:
            while True:
                psd1.acquire()
                psd2.acquire()
                psd1.getVal(buf1)
                psd2.getVal(buf2)
                
                logtext = "%.5f;%.4f;%.4f;%.4f;%.4f\n" % (time.time(), buf1[0,0], buf1[1,0], buf2[0,0], buf2[1,0])
                fp.write(logtext)
                
                if counter % 50 == 0:
                    fp.flush() #flush the file from time to time
                counter += 1
                
                #update plot
                data[:,0:buffer-1] = data[:,1:buffer]
                data[0:2,buffer-1] = buf1
                data[2:4,buffer-1] = buf2
                h["source"] = data
                time.sleep(delay)
        except KeyboardInterrupt:
            print("quit the acquisition")
            
    psd1.stopDevice()
    psd2.stopDevice()
    del psd1
    del psd2
            
        

Compilation
===========

To compile this plugin, install the Thorlabs KINESIS driver package in the same bit-version than itom (32/64bit).
Then set the CMake variable **THORLABS_KINESIS_DIRECTORY** to the base directory of Kinesis (e.g. C:/Program Files/Thorlabs/Kinesis).
The required libraries from Kinesis will automatically be copied to the *lib* folder of itom. Do not use Kinesis 1.14.9 or below for compiling this plugin.

Kinesis 1.14.10 requires the Microsoft C++ Redistributable 2012.

Changelog
==========

* itom setup 3.2.1: This plugin has been compiled with Thorlabs Kinesis 1.14.15; it requires the Microsoft C++ Redistributable 2012
* itom setup 4.0.0: This plugin has been compiled with Thorlabs Kinesis 1.14.23;
* itom setup 4.1.0: This plugin has been compiled with Thorlabs Kinesis 1.14.25.