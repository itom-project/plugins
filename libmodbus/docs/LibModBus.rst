============
 LibModBus
============

=============== ========================================================================================================
**Summary**:    :pluginsummary:`LibModBus`
**Type**:       :plugintype:`LibModBus`
**License**:    :pluginlicense:`LibModBus`
**Platforms**:  Windows, Linux
**Devices**:    Modbus communication over TCP/IP and RTU
**Author**:     :pluginauthor:`LibModBus`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: LibModBus

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: LibModBus


An example for opening a *LibModBus* instance at local host *127.0.0.1*; port *502* is:

.. code-block:: python

    lmb = dataIO("LibModBus", '127.0.0.1', 502)

Parameters
==========

These parameters are available and can be used to configure the **LibModBus** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**name**: {str}, read-only
    name of the plugin (*LibModBus*)

**target**: {str}, read-only
    IP-Address of the connected Modbus-TCP device or serial port for Modbus-RTU connection. Under Windows COM1-COM9 is supported, unix-based systems should use /dev/ttyS0.. or /dev/ttyUSB0..

**port**: {int}, read-only
    port used for IP communication or device ID for ModbusRTU

**baud**: {int}, read-only
    The baudrate of the port for RTU communication

**parity**: {str}, read-only
    Parity for RTU communication (N->None, E->Even, O->Odd)

**databit**: {int}, read-only
    Number of bits to be written in line for RTU communication

**stopbit**: {int}, read-only
    Stop bits after every n bits for RTU communication

**output_mode**: {bool}
    if output_mode is true, multiple outputs will be sent to command line, e.g. register values of getVal function. Default is false

**registers**: {str}
    fallback addressing for modbus registers. This value will be used, if a dataObject without 'registers'-tag is sent to the getVal- or setVal-function.
    *registers* needs to be stored with address and number of consecutive registers seperated by ',' and different registers seperated by ';' i.e.: '10,2;34,1;77,4' to address registers 10,11;34;77..80. Number 1 of consecutive registers can be left out i.e.:'10,2;34;77,4'


Usage
=====

Values can be read or written via modbus communication using the *getVal(dObj)* and *setVal(dObj)* functions. The dataObject *dObj* needs to be two dimensional with the first dimension set to 1,
the second dimension has to have the exact size of the numbers of written or read registers. As modbus uses 16bit integer values, dObj must be initialized as 'uint16' for modbus register values.
To read or write coils, dObj must be initialized as 'uint8' as the decision for using read/write coil or read/write register only depends on the data type of the input data object.

.. code-block:: python

    obj = dataObject([1,10],'uint16')

To address the requested registers, either parameter *registers* can be used, or obj can be given a *registers*-tag. The *registers*-tag has to have the same structure as the *registers* parameter.
The parameter is useful if the same registers need to be read/written multiple times while the tag should be used for changing registers.

.. code-block:: python

    lmb = dataIO("LibModBus", '127.0.0.1', 502)
    lmb.setParam('registers','10,2;34,1;77,4;100,1;101,1;102,1')
    obj = dataObject([1,10],'uint16')           #initializes register-object
    coilObj = dataObject([1,10],'uint8')        #initializes coil-object
    lmb.getVal(obj)                             #reads registers 10,11,34,77..80,100..102 and saves them to obj consecutive
    lmb.getVal(coilObj)                         #reads coils 10,11,34,77..80,100..102 and saves them to obj consecutive
    obj.setTag('registers','105,4;200,4;204,2') #sets registers-tag
    lmb.setVal(obj)                             #writes previously read values to registers 105..108,200..203,204,205
    obj.setTag('registers','105,4;200,4')
    lmb.setVal(obj)                             #produces error, obj is of size [1,10] but only 8 registers (105..108,200..203) are requested

The number of consecutive registers is generally used to read/write values that are bigger than 16bit (2 registers for 32bit, 4 registers for 64bit)and should be used that way. Please refer to the documentation of the modbus slave you will be using.
