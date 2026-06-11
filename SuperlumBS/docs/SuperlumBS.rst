===================
 SuperlumBS
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`SuperlumBS`
**Type**:       :plugintype:`SuperlumBS`
**License**:    :pluginlicense:`SuperlumBS`
**Platforms**:  Windows, (Linux possible but yet not implemented)
**Devices**:    Lightsource from company *SuperlumBS*
**Author**:     :pluginauthor:`SuperlumBS`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: SuperlumBS

Applications:

-   biomedical imaging
-   optical coherence tomography
-   spectroscopy
-   optical metrology

Features:

-  3-mW output power ex fiber.
-  Tuning/sweeping range adjustable up to 50 nm.
-  Instantaneous linewidth of less than 0.05 nm.
-  200 Hz sweep rate over 50 nm.
-  2 kHz sweep rate over 5 nm.
-  1 kHz wavelength alternation rate for any two selected wavelengths within full tuning range.
-  PM- or SM-fiber output.
-  Powered directly from a wall outlet.
-  RS-232 remote control capability.

Description:

Broadsweeper BS840 is a widely tunable external cavity laser based on a broadband SOA in a 840-nm band.
Fast and narrow-band Acousto-Optic Tunable Filter (AOTF) isused as a selective intracavity element for tuning
and sweeping the laser wavelength. Active temperature control of AOTF ensures excellent stability of the
wavelength. The absence of moving parts in external cavity ensures high accuracy of wavelength setting and
excellent sweep-to-sweep reproducibility of an instantaneous wavelength. A specially designed power control loop
allows the Broadsweeper to demonstrate a flat-top tuning characteristic even at the maximum tuning rate.
The following modes of operation are available for the standard version:
-  Operation at any selected wavelength within the full tuning range.
-  Alternation of the two desired wavelengths (any two wavelengths from the full tuning range may be chosen).
-  Linear sweep of wavelength with a rate of 200 Hz overthe full tuning range (50 nm), and of 2 kHz over any
5-nm interval within the full tuning range.
Broadsweeper's external cavity is based on a PM-fiber. The standard version is shipped with FC/APC-terminated
SM-fiber cable for connection to the optical adapter on the front panel. Output power is measured at the end of this
cable. PM-PANDA-fiber cable with main polarization aligned to the key of the connector is optionally available upon
request.

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: SuperlumBS

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

Then create a new instance of the plugin **SuperlumBS** using the instance of the **SerialIO** plugin.

.. code-block:: python

    serial = dataIO("SerialIO", COM-Port, Baudrate, endline="\r\n")
    bs = DataIO("SuperlumBS", serial)

After the initialization of the plugin **SuperlumBS** the remote communication is set. The plugin works only, if the **remote access** is available, **Master-Key** is in position I and the **remote Interlock** is closed.
If the instance of **SuperlumBS** is deleted, the remote access is switched to the local mode.

Parameters
==========

These parameters are available and can be used to configure the **SuperlumBS** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**comPort**: {int}, read-only
    The current com-port ID of this specific device. -1 means undefined.
**full_tuning_range_HIGH_end**: {float}, read-only
    FULL spectral tuning range of sweeping in AUTOmatic OR EXTernal sweep mode in LOW power mode.
**full_tuning_range_HIGH_start**: {float}, read-only
    FULL spectral tuning range of sweeping in AUTOmatic OR EXTernal sweep mode in HIGH power mode.
**full_tuning_range_LOW_end**: {float}, read-only
    FULL spectral tuning range of sweeping in AUTOmatic OR EXTernal sweep mode in LOW power mode.
**full_tuning_range_LOW_start**: {float}, read-only
    FULL spectral tuning range of sweeping in AUTOmatic OR EXTernal sweep mode in HIGH power mode.
**local**: {int}
    ( 0 ) local or ( 1 ) remote mode.
**master_key**: {int}, read-only
    Master Key is in position O (0) or position I (1).
**modification_end_wavelength**: {float}
    end modification wavelength in AUTOmatic or EXTernal sweep mode.
**modification_start_wavelength**: {float}
    start modification wavelength in AUTOmatic or EXTernal sweep mode.
**modulation_frequency**: {float}
    Modulation frequency in Two-Wavelength MODulation mode.
**name**: {str}, read-only
    Name of plugin.
**operation_booster**: {int}, read-only
    ( -1 ) booster module is not installed, ( 0 ) optical output of booster is disabled, ( 1 ) optical output of booster is enabled.
**operation_mode**: {int}
    ( 1 ) MANual, ( 2 ) AUTOmatic, ( 3 ) EXTernal, ( 4 ) MODulation.
**optical_output**: {int}
    ( 0 ) optical output is disabled, ( 1 ) optical output is enabled.
**power_mode**: {int}
    ( 0 ) LOW Power mode, ( 1 ) HIGH Power mode.
**remote_interlock**: {int}, read-only
    Remote Interlock is in open (0) or is closed (1).
**serial_number**: {str}, read-only
    Serial number of device.
**sweep_speed**: {int}
    sweep speed in AUTOmatic or EXTernal mode between 2nm/s - 10000nm/s. Increment: 1nm/s.
**wavelength**: {float}
    operation wavelength [nm] in MANual Mode. Increment: 0.05nm.
**wavelength_first**: {float}
    first wavelength in Two-Wavelength MODulation mode.
**wavelength_second**: {float}
    second wavelength in Two-Wavelength MODulation mode.

Usage
=====

First open the serial port and assign it to the variable **serial**. For example COM Port 1, Baud rate 57600, endline = "\\r\\n".

.. code-block:: python

    serial = dataIO("SerialIO", 1, 57600, endline="\r\n")

Then create a new instance of the acuator plugin **SuperlumBS**. A Mandatory parameter is the serialIO instance. Assign it to the variable **bs**.

.. code-block:: python

    bs = DataIO("SuperlumBS", serial)

All the parameter can be changed by using the function **setParam**. This example shows how the wavelength in the MANual Mode can be changed.

.. code-block:: python

    bs.setParam("wavelength", 850.00)

The light of the Broadsweeper is enabled by setting the parameter **optical_output** to 1 or disabled by setting it to 0.

.. code-block:: python

    bs.setParam("optical_output", 1)

The parameters can be queried by using the function **getParam**. This example shows how to get the Operation Mode of the BroadSweeper device.
The return integer value will be 1 for MANual, 2 for AUTOmatic, 3 for EXTneral and 4 for MODulation Mode.

.. code-block:: python

    >>bs.getParam("operation_mode")
    1
    >>

Changelog
=========

* plugin type changed to dataIO
