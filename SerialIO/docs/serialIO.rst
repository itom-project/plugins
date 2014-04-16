==========
 SerialIO
==========

=============== ========================================================================================================
**Summary**:    :pluginsummary:`SerialIO`
**Type**:       :plugintype:`SerialIO`
**License**:    :pluginlicense:`SerialIO`
**Platforms**:  Windows, Linux
**Devices**:    COM-Ports (Windows), ttySx and ttyUSBx (Linux)
**Author**:     :pluginauthor:`SerialIO`
=============== ========================================================================================================

Overview
========

This plugin encapsulates the access to the serial port. It can be used on both
Windows and linux based operating systems.

In **itom**, this plugin is also interally used by other plugins (e.g. *PIPiezoCtrl*, *UhlActuator*, *LeicaMotorFocus* or *PiezosystemJena*).

Features are:

- Access to the port settings by its internal parameters and the initialization.
- Support for different baudrates, stop bits, parity and flow control.
- Endline characters can automatically appended to each sent string and are split from received strings.
- Any string can be sent with a certain delay in between each character (useful for older devices).
- A debugging parameter allows displaying the entire data transfer in the optional toolbox.
- The configuration dialog can be used to send user-defined strings via the opened connection.

Initialization
==============
  
For a connection to a serial port, create a new instance of this plugin using:

.. py:function:: dataIO("SerialIO", port, baud, endline [, bits, stopbits, parity, flow, sendDelay, timeout, debug])
    :noindex:
    
    This is the necessary constructor of the class *dataIO* of *itom* in order to create a new instance of the plugin **serialIO**.

    The parameter are as follows:

    ============ =============== ===================================================================================================
    port         int             COM-port number (e.g. 1)
    baud         int             Baudrate (see *baudrate* in the parameter description below)
    endline      str             endline character (see *endline* in the parameter description below)
    bits         int, optional   number of bits to be written in line [5,8], default: 8
    stopbits     int, optional   stop bits after every n bits [1,2], default: 1
    parity       int, optional   0: no parity [default], 1: odd parity, 2: even parity
    flow         int, optional   bitmask for the flow control (see *flow* in the parameter description below) [0, 127], default: 0
    sendDelay    int, optional   0: write output buffer in one block, else: delay in ms after each character (same for input)
    timeout      float, optional Timeout for reading the current input buffer of the serial port in [s], [0,64], default: 4s
    enableDebug  int, optional   0: no debug output [default], 1: all data transfer is printed to the toolbox
    ============ =============== ===================================================================================================

An example for opening port **COM 1** with 9600 bauds is:

.. code-block:: python
    
    serial = dataIO("SerialIO", 1, 9600, endline="\n")

Parameters
==========

These paramaters are available and can be used to configure the **serialIO** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**name**: {str}, read-only
    name of the plugin (*SerialIO*)
**port**: {int}, read-only
    connected COM port number (defined by initialization)
**baud**: {int}
    current baudrate in symbols per second (approximately bits/sec). Allowed baudrates are::
    
        50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400,
        4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800,
        500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000,
        2500000, 3000000, 3500000, 4000000
    
    However not all baudrates are supported on each operating system.
**bits**: {int}
    Number of bits to be written in line [5,8]
**stopbits**: {int}
    Stop bits after every n-th bit [1,2]
**parity**: {int}
    Parity check
    0: no parity
    1: odd parity
    2: even parity
**flow**: {int}
    bitmask for the flow control. This mask is an or-combination of the following values (add values for resulting flow value)::
        
        Xon,Xoff: Xoff (0, default), Xon (1); 1. bit
        rts control: disabled (0, default), enabled (2), handshake (4 or 4+2); 2. and 3. bit
        cts control: disabled (0, default), enabled (8); 4. bit
        dtr control: disabled (0, default), enabled (16), handshake (32 or 32+16); 5. and 6. bit
        dsr control: disabled (0, default), enabled (64); 7. bit
    
    Example: Xon, rts handshake, dsr enabled is 1 + 4 + 64 = 69 for the flow value
        
**endline**: {str}
    If a string is put to the output buffer (and send via the opened port), the endline-string is automatically appended to the string.
    Typical values are::
        
        "\n","\n\r","\r" or ""
    
    Use the empty endline character string ("") if you want to have full control.
**sendDelay**: {str}
    This value represents a delay (in ms) after each character that is send and received
**timeout**: {double}
    Timeout in seconds. If the incoming buffer of the serial port cannot be read within this time, the call returns. [0,65]
**enableDebug**: {int}
    Set this value to 1 if you want to read the entire data transfer in the toolbox of an instance (disabled: 0).
    
Usage
=====

Lets assume a serial port connection should be established with the following properties:

* COM 1
* 9600 baud
* 8 bits
* 1 stopbit
* no parity
* every command should finish with "\\n"

Then open the serial port and assign it to the variable *serial*

.. code-block:: python  
    
    serial = dataIO("SerialIO",1,9600,endline="\n",bits=8,stopbits=1,parity=0)

If you have a scenario that you need to ask for the position of an actuator. Maybe the string to send in order to ask
for the current position is **POS?\n**, then use the **setVal** method to send this string (*\\n* is automatically appended):

.. code-block:: python
    
    serial.setVal("POS?")
    
Then it is necessary to get the result. Therefore create a bytearray with enough space and pass this array to the **getVal** method:

.. code-block:: python
    
    ba = bytearray(9) #array with nine elements
    len = serial.getVal(ba)

*len* finally contains the number of characters that have been filled by the serial port, of course, *len* cannot be bigger than
the size of the allocated buffer *ba*. If the serial port does not respond at all within the given timeout time, an error is raised.
*getVal* does not wait until the entire buffer is filled or the timeout occurs but returns immediately with the content of the buffer that
has been filled until this time. In order to get the full answer, it is also possible to repeatedly call *getVal*.

Clear input or output buffer
============================

Sometimes, it is necessary to immediately clear all characters inside of the input buffer (obtained by *getVal*) and/or output buffer (send by *setVal*). This can be done using specific **exec**-functions:

.. code-block:: python
    
    serial.exec("clearInputBuffer") #clear input buffer
    serial.exec("clearOutputBuffer") #clear output buffer
    
    #alternative:
    serial.exec("clearBuffer", 0) #clear input buffer
    serial.exec("clearBuffer", 1) #clear output buffer

Installation
============

For using this plugin no further 3rd party libraries or drivers are necessary.

If you are using linux to open a serial connection and you are running **itom** without root privileges, which is recommended, you have to add
your user to the **dialout** group, logout once and login again.

.. code-block:: bash

    sudo adduser USERNAME dialout

where USERNAME ist the username under which you are running **itom**

