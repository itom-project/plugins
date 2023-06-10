===================
 SuperlumBL
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`SuperlumBL`
**Type**:       :plugintype:`SuperlumBL`
**License**:    :pluginlicense:`SuperlumBL`
**Platforms**:  Windows, (Linux possible but yet not implemented)
**Devices**:    Lightsource from company *Superlum*
**Author**:     :pluginauthor:`SuperlumBL`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: SuperlumBL

Applications:

-   Optical coherence tomography
-	Characterization of optical components
-	Optical fiber metrology
-	Optical measurement


Features(S840-B-I-20):

-	50nm Bandwidth
-	840nm Center wavelength
-	15mW ex fiber (single-mode)
-	Built-in optical isolator
-  	Powered directly from a wall outlet.
-  	RS-232 remote control capability.


Description (from www.superlumdiodes.com):

S-series Broadlighters are high-power, high-stability, low-coherence, broadband AC powered SLD light sources. They are based on Superlum's high-power (HP) SLD modules. The most high-power Broadlighters are equipped with appropriate polarization-insensitive fiber-optic isolators that protect the SLD modules from being damaged by optical feedback. All the light sources have two operating modes: the High Power mode and the Low Power mode. During normal operation, the High Power mode of the Broadlighter is always employed, whereas the Low Power mode is intended for safe alignment of optical components and circuits in the customer's optical system during the set-up process. The devices are developed for optical test and measurement applications.

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: SuperlumBL

To communicate with the device, you need to perform the following settings for the serial port communication using the Plugin **SerialIO**.

The parameter are as follow:

    =========================== ============
    Baud rate (bits per second) 57600
    Data bits                   8
    Parity                      none
    Stop bits                   1
    Flow control                none
    Data type                   ASCII string
    =========================== ============

Then create a new instance of the plugin **SuperlumBL** using the instance of the **SerialIO** plugin.

.. code-block:: python

    serial = dataIO("SerialIO", COM-Port, Baudrate, endline="\r\n")
    bs = dataIO("SuperlumBL", serial, deviceName)

After the initialization of the plugin **SuperlumBL** the remote communication is set. The plugin works only, if the **remote access** is available.
If the instance of **SuperlumBL** is deleted, the remote access is switched to the local mode.

Parameters
==========

These parameters are available and can be used to configure the **SuperlumBL** instance. All of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**comPort**: {int}, read-only
    The current com-port ID of this specific device. -1 means undefined.
**local**: {int}
    ( 0 ) local or ( 1 ) remote mode.
**name**: {str}, read-only
    Name of plugin.
**optical_output**: {int}
    ( 0 ) optical output is disabeld, ( 1 ) optical output is enabled.
**power_mode**: {int}
    ( 0 ) LOW Power mode, ( 1 ) HIGH Power mode.
**serial_number**: {str}, read-only
    Serial number of device.

Usage
=====

First open the serial port and assign it to the variable **serial**. For example COM Port 1, Baud rate 57600, endline = "\\r\\n".

.. code-block:: python

    serial = dataIO("SerialIO", 1, 57600, endline="\r\n")

Then create a new instance of the acuator plugin **SuperlumBL**. Mandatory parameters are the serialIO instance and **deviceName**. Assign it to the variable **bl**.

.. code-block:: python

    bl = actuator("SuperlumBL", serial, deviceName)

All the parameters can be changed by using the function **setParam**. This example shows how to set output power mode. 0 means "low", 1 means "high".

.. code-block:: python

    bs.setParam("power_mode", 1)

The optical output of the Broadlighter is enabled by setting the parameter **optical_output** to 1 or disabled by setting it to 0.

.. code-block:: python

    bs.setParam("optical_output", 1)

The parameters can be queried by using the function **getParam**.

.. code-block:: python

	>>bs.getParam("optical_output")
	1
	>>
