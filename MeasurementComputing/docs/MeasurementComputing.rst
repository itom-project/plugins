=====================
 MeasurementComputing
=====================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`MeasurementComputing`
**Type**:       :plugintype:`MeasurementComputing`
**License**:    :pluginlicense:`MeasurementComputing`
**Platforms**:  Windows
**Devices**:    *Measurement Computing Data Acquisition Devices*
**Author**:     :pluginauthor:`MeasurementComputing`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: MeasurementComputing

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: MeasurementComputing

To create a new instance using the following python code:

.. code-block:: python

    instance = dataIO("MeasurementComputing", board_number)

Parameters
==========

These parameters are available and can be used to configure the **MeasurementComputing** instance. Many of them are directly initialized by the parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable parameters can be changed using *setParam*.

	**analog_high_input_channel**: {int}
		last analog input channel (See pin description of your device).
	**analog_high_output_channel**: {int}
		last analog output channel (See pin descirption of your device).
	**analog_input_bpp**: {int}, read-only
		bit resolution of the analog input.
	**analog_low_input_channel**: {int}
		first analog input channel (See pin description of your device).
	**analog_low_output_channel**: {int}
		first analog output channel (See pin descirption of your device).
	**analog_number_inputs**: {int}, read-only
		number of input channels of this device.
	**analog_number_outputs**: {int}, read-only
		number of output channels of this device.
	**analog_output_bpp**: {int}, read-only
		bit resolution of the analog output.
	**analog_voltage_input**: {int}
		if parameter is set to 1, the A/D value is returned as a voltage value.
	**analog_voltage_output**: {int}
		if parameter is set to 1, the D/A value is set as a voltage value.
	**clock_frequency**: {int}
		clock frequency in megahertz (MHz) (40, 10, 8, 6, 5, 4, 3, 2, 1) or 0 for not supported.
	**device_name**: {str}, read-only
		name of connected device.
	**digital_number_ports**: {int}, read-only
		number of digital I/O ports of this device.
	**digital_port_mode**: {int}
		sets the digital port defined by the parameter digtal_devices_type as input (2) or output(1).
	**digital_port_name**: {str}
		digital devices type.
	**input_range_code**: {str}
		A/D range code, if board has a programmable gain. Refer to board specific information for a list of the supported A/D ranges.
	**input_samples_per_second**: {int}
		analog input samples per second. The samples are distributed over all channels. The effective rate per channel is this parameter divided by the number of channels. USB1208LS: 100 Hz - 1200 Hz, for higher rates a fast acquisition with 8000 Hz is executed where only 4096 samples can be acquired in one run.
	**name**: {str}, read-only
		name of itom plugin.
	**number_counter_channel**: {int}, read-only
		number of counter channels of this device.
	**number_temperature_channel**: {int}, read-only
		number of temperature channels of this device.
	**output_range_code**: {str}
		D/A range code, if board has a programmable gain. Refer to board specific information for a list of the supported D/A ranges.
	**samples_per_input_channel**: {int}
		number of samples that are acquired per channel after each acquisition.
	**serial_number**: {str}, read-only
		serial number of connected device.
	**temperature_scale**: {str}
		scale value of the temperature input. Coises are CELSIUS, FAHRENHEIT, KELVIN, VOLTS and NOSCALE. default = CELSIUS.

Range codes
===========

The table below shows the range codes for the **input_range_code** and **output_range_code** parameters and the voltage range, which can be detected. Valid range for your hardware are listed in the Universal Library User's Guide.

	+------------------------------------+---------------------------------+
	|             BIPOLAR                |            UNIPOLAR             |
	+================+===================+================+================+
	|range code      |range [Volts]      |range code      |range [Volts]   |
	+----------------+-------------------+----------------+----------------+
	|BIT60VOLTS      |-60 to 60          |UNI10VOLTS      |0 to 10         |
	+----------------+-------------------+----------------+----------------+
	|BIP30VOLTS      |-30 to +30         |UNI5VOLTS       |0 to 5          |
	+----------------+-------------------+----------------+----------------+
	|BIP20VOLTS      |-20 to +20         |UNI4VOLTS       |0 to 4          |
	+----------------+-------------------+----------------+----------------+
	|BIP15VOLTS      |-15 to +15         |UNI2PT5VOLTS    |0 to 2.5        |
	+----------------+-------------------+----------------+----------------+
	|BIP10VOLTS      |-10 to +10         |UNI2VOLTS       |0 to 2          |
	+----------------+-------------------+----------------+----------------+
	|BIP5VOLTS       |-5 to +5           |UNI1PT67VOLTS   |0 to 1.67       |
	+----------------+-------------------+----------------+----------------+
	|BIP4VOLTS       |-4 to +4           |UNI1PT25VOLTS   |0 to 1.25       |
	+----------------+-------------------+----------------+----------------+
	|BIP2PT5VOLTS    |-2.5 to +2.5       |UNI1VOLTS       |0 to 1          |
	+----------------+-------------------+----------------+----------------+
	|BIP2VOLTS       |-2.0 to +2.0       |UNIPT5VOLTS     |0 to 0.5        |
	+----------------+-------------------+----------------+----------------+
	|BIP1PT25VOLTS   |-1.25 to +1.25     |UNIPT25VOLTS    |0 to 0.25       |
	+----------------+-------------------+----------------+----------------+
	|BIP1VOLTS       |-1 to +1           |UNIPT2VOLTS     |0 to 0.2        |
	+----------------+-------------------+----------------+----------------+
	|BIPPT625VOLTS   |-0.625 to +0.625   |UNIPT1VOLTS     |0 to 0.1        |
	+----------------+-------------------+----------------+----------------+
	|BIPPT5VOLTS     |-0.5 to +0.5       |UNIPT05VOLTS    |0 to 0.05       |
	+----------------+-------------------+----------------+----------------+
	|BIPPT25VOLTS    |-0.25 to +0.25     |UNIPT02VOLTS    |0 to 0.02       |
	+----------------+-------------------+----------------+----------------+
	|BIPPT2VOLTS     |-0.2 to +0.2       |UNIPT01VOLTS    |0 to 0.01       |
	+----------------+-------------------+----------------+----------------+
	|BIPPT1VOLTS     |-0.1 to +0.1       |                                 |
	+----------------+-------------------+----------------+----------------+
	|BIPPT05VOLTS    |-0.05 to +0.05     |                                 |
	+----------------+-------------------+----------------+----------------+
	|BIPPT01VOLTS    |-0.01 to +0.01     |                                 |
	+----------------+-------------------+----------------+----------------+
	|BIPPT005VOLTS   |-0.005 to +0.005   |                                 |
	+----------------+-------------------+----------------+----------------+
	|BIP1PT67VOLTS   |-1.67 to +1.67     |                                 |
	+----------------+-------------------+----------------+----------------+
	|BIPPT312VOLTS   |-0.312 to +0.312   |                                 |
	+----------------+-------------------+----------------+----------------+
	|BIPPT156VOLTS   |-0.156 to +0.156   |                                 |
	+----------------+-------------------+----------------+----------------+
	|BIPPT125VOLTS   |-0.125 to +0.125   |                                 |
	+----------------+-------------------+----------------+----------------+
	|BIPPT078VOLTS   |-0.078 to +0.078   |                                 |
	+----------------+-------------------+----------------+----------------+

Additional functions (exec functions)
=======================================

The plugin execFunctions are:

	=============	==============================================================================================================================================================================================
	Name			Descirption
	=============	==============================================================================================================================================================================================
	**getBitIn**   	reads a single bit of the specified I/O port. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a input port.
	**getCIn**     	reads and returns the current count of the specified counter input channel. Use the parameter counter_set_value to reset the counter.
	**getDIn**     	reads the digital I/O port value. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a input port.
	**getTIn**     	reads and returns the temperature value of the specific input channel defined by the temperature_input_channel. Use the parameter temperature_scale the define the temperature scaling value.
	**getVIn**     	reads and returns the voltage value of the specified input channel in the parameter voltage_input_channel.
	**setBitOut**  	sets a single bit of the specified I/O port. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a output port.
	**setDOut**    	sets the digital I/O port value. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a output port.
	**setVOut**    	set the voltage value at the specific analog output channel defined by the parameter voltage_output_channel.
	=============	==============================================================================================================================================================================================

.. py:function::  instance.exec('getVIn', voltage_input_channel)

    reads and returns the voltage value of the specified input channel in the parameter voltage_input_channel.

    :param voltage_input_channel: voltage input channel
    :type voltage_input_channel: int
    :return: voltage_input - voltage value of defined input channel
    :rtype: float

.. py:function::  instance.exec('setVOut', voltage_output_channel, voltage_output)

    set the voltage value at the specific analog output channel defined by the parameter voltage_output_channel.

    :param voltage_output_channel: voltage output channel
    :type voltage_output_channel: int
    :param voltage_output: voltage value to set at the output channel
    :type voltage_output: float

.. py:function::  instance.exec('getTIn', temperature_input_channel)

    reads and returns the temperature value of the specific input channel defined by the temperature_input_channel. Use the parameter temperature_scale the define the temperature scaling value.

    :param temperature_input_channel: temperature input channel
    :type temperature_input_channel: int
    :return: temperature_input - temperature value of defined input channel
    :rtype: float

.. py:function::  instance.exec('getBitIn', digital_port_number, digital_port_bit_number)

    reads a single bit of the specified I/O port. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a input port.

    :param digital_port_number: digital I/O port to read
    :type digital_port_number: str
    :param digital_port_bit_number: digital port bit number of the specific I/O port
    :type digital_port_bit_number: int
    :return: digital_port_value - digital input value of the specific I/O port-bit
    :rtype: int

.. py:function::  instance.exec('setDOut', digital_port_number, digital_port_value)

    sets the digital I/O port value. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a output port.

    :param digital_port_number: digital I/O port to read
    :type digital_port_number: str
    :param digital_port_value: digital output value of the specific I/O port
    :type digital_port_value: int

.. py:function::  instance.exec('setBitOut', digital_port_number, digital_port_bit_number, digital_port_value)

    sets a single bit of the specified I/O port. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a output port.

    :param digital_port_number: digital I/O port to read
    :type digital_port_number: str
    :param digital_port_bit_number: digital port bit number of the specific I/O port
    :type digital_port_bit_number: int
    :param digital_port_value: digital output value of the specific I/O port-bit
    :type digital_port_value: int

.. py:function::  instance.exec('getCIn', counter_input_channel [,counter_set_value])

    reads and returns the current count of the specified counter input channel. Use the parameter counter_set_value to reset the counter.

    :param counter_input_channel: counter input channel
    :type counter_input_channel: int
    :param counter_set_value: counter value to load into the counter's register. To reset the counter, load the value zero
    :type counter_set_value: int - optional
    :return: counter_input - counter value of defined input channel
    :rtype: int

.. py:function::  instance.exec('getDIn', digital_port_number)

    reads the digital I/O port value. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a input port.

    :param digital_port_number: digital I/O port to read
    :type digital_port_number: str
    :return: digital_port_value - digital input value of the specific I/O port
    :rtype: int

Usage
=====

Then create a new instance of the analog-digital converter plugin **MeasurementComputing**. A Mandatory parameter is the board number, defined by the software 'InstaCal'.

.. code-block:: python

    instance = dataIO("MeasurementComputing", board_number)

Plugin parameter can be canged by using the function **setParam**. This examples shows how the analog input channels are configured. The devices will acquire the data from the **analog_low_input_channel** to the **analog_high_input_channel**.

.. code-block:: python

	high_channel = 3
	low_channel = 0
	instance.setParam("analog_high_input_channel", high_channel)
	instance.setParam("analog_low_input_channel", low_channel)

This examples shows how the range code is defined (see the available ranges in the table above):

.. code-block:: python

	range_code = "BIP5VOLTS"
	instance.setParam("input_range_code", range_code)

The analog input signals can be acquired by using following example code. The data are saved in the dataobject of size [m x n], where **m** is the number of input channels (the number of channels is equal to (**analog_high_input_channel** - **analog_low_input_channel** + **1**)) and **n** is the number of acquired input samples (definded by the parameter **samples_per_input_channel**).
**analog_voltage_input** parameter can be used to save the data in voltage values.

.. code-block:: python

	instance.acquire()
	d = dataObject()
	instance.getVal(d)

Output values of the analog output ports are used by the following example. First you must define a dataObject with the output values you want to set by the analog output channel. Use the parameter **analog_voltage_output** to define, if you want to used voltage values, otherwise your maximum digital value is definded by the **analog_output_bpp**.

.. code-block:: python

	# set the analog output by digital values
	numberChannels = 2
	numberSamples = 1
	range_code = "UNI5VOLTS"
	instance.setParam("analog_voltage_output", 0)
	instance.setParam("output_range_code", range_code)
	outValues = dataObject([numberChannels, numberSamples], 'int16')
	outValues[:,:] = 1023 # 5V analog output in case of 10bit output channel resolution
	instance.setVal(outValues)

	#set the analog output by voltage values
	instance.setParam("analog_voltage_output", 1)
	outValues = dataObject([numberChannels, numberSamples], 'float32')
	outValues[:,:] = 5.0
	instance.setVal(outValues)

The digital port can be used like in the following example.

.. code-block:: python

	# set the digital port as input to use it as a input channel
	instance.setParam("digital_port_mode", 2)
	instance.exec("getDIn", "FIRSTPORTA")

	# set the digital port as output to use it for output reasons
	instance.setParam("digital_port_mode", 1)
	outValues = 255 	# all port pin connections to high
	instance.exec("setDOut", "FIRSTPORTA", outValues)


One single bit of the digital port is read by using the execFunction **getBitIn**.

.. code-block:: python

	# set the digital port as input
	instance.setParam("digital_port_mode", 2)
	instance.exec("getBitIn", "FIRSTPORTA", 0)

	# set the digital port as output
	instance.setParam("digital_port_mode", 1)
	outValues = 255
	bitNumber = 0
	instance.exec("setBitOut", "FIRSTPORTA", bitNumber, outValues)

The temperature channel can be read by the execFunction **getTIn**. Use the temperature_scale parameter to define the value unit you want to get the data.

.. code-block:: python

	channel = 0
	instance.setParam("temperature_scale", "CELSIUS")
	instance.exec("getTIn", channel)

The counter input is used by the execFunction **getCIn**. With the optional parameter **counter_set_value** the counter can be reset.

.. code-block:: python

	channel = 0
	instance.exec("getCIn", channel)
	# reset the counter
	resetvalue = 0
	instance.exec("getCIn", channel, resetvalue)

Installation
=============

You have to install the MC DAQ Software from https://www.mccdaq.com/Software-Downloads,
namely the tool "InstaCal and Universal Library for Windows". Then, indicate the following variables in CMake to
properly configure the build of this plugin:

Cmake should detect the correct directories to access the MC DAQ Software Suite if the default installation folder
has been chosen. If not set the evironment variale MCDAQ_ROOT to the installation folder (e.g. By C:/Program Files (x86)/Measurement Computing/DAQ).

Check if the following variables are set appropriately:

* MeasurementComputing_DAQ_BINARY: e.g. C:/Program Files (x86)/Measurement Computing/DAQ/cbw64.dll (or cbw32.dll for 32bit itom)
* MeasurementComputing_DAQ_SDK_DIR: e.g. C:/Users/Public/Documents/Measurement Computing/DAQ/C

Changelog
==========

* 2016-01-18: This plugin was added to the public repository and will be part of setups > itom 2.0.0
* itom setup 2.1.0: This plugin has been compiled using CBW library version 1.83
* itom setup 2.2.0: This plugin has been compiled using CBW library version 1.83
* itom setup 2.2.1: This plugin has been compiled using CBW library version 1.89
* itom setup 3.0.0: This plugin has been compiled using CBW library version 1.89
* itom setup 3.1.0: This plugin has been compiled using CBW library version 1.89
* itom setup 3.2.1: This plugin has been compiled using CBW library version 1.89
* itom setup 4.0.0: This plugin has been compiled using CBW library version 1.89
* itom setup 4.1.0: This plugin has been compiled using CBW library version 1.89
* itom setup 4.3.0: This plugin has been compiled using MCC DAQ Software library version 6.73
