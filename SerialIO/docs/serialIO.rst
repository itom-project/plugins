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

  Use the checkBox **enable $(ascii-code) statement parsing** to enable ASCII code parsing to *$(ascii-code) statement*.

Initialization
==============

For a connection to a serial port, create a new instance of this plugin using:

.. py:function:: dataIO("SerialIO", port, baud, endline [, bits, stopbits, parity, flow, sendDelay, timeout, debug])
    :noindex:

    This is the necessary constructor of the class *dataIO* of *itom* in order to create a new instance of the plugin **serialIO**.

    The parameter are as follows:

    ============ =============== ===================================================================================================
    port         int             Windows: COM-port number (e.g. 1), Linux: ttySx, ttyUSBx or ttyACMx (x is port, see infos below)
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

For linux, the port number may start with zero. If port numbers < 1000 are given, at first the serial port **ttySx** is checked, where
x is the given port number. If **ttySx** does not exist, **ttyUSBx** is searched. If a port number in the range [1000,1999] is given,
the device **ttyUSBx** is used, where *x* is (port - 1000). Finally, if port is in the range [2000,2999], the device **ttyACMx** is
searched, where *x* is (port - 2000).

Parameters
==========

These parameters are available and can be used to configure the **serialIO** instance. Many of them are directly initialized by the
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
**endlineRead**: {str}
    Same behaviour like **endline**, however it determines the delimiter for incoming strings and is only
    used or evaluated in **readline** is 1. The user is referred to **readline**.
**readline**: {int}
    Per default, **readline** is set to 0. This means that the **getVal** command returns the values that are currently
    available at the input buffer of the computer. If you call **getVal** too fast, it might be, that the full answer
    is not available yet. Then you need to recall **getVal** again.
    If you set **readline** to 1, **getVal** collects values from the input buffer and checks if **endlineRead** is contained in the string. If so, **getVal** writes
    all characters also those behind the first appearance of **endlineRead** into the bytearray. Neverteless the number of obtained signs returned by **getVal**
    just counts the signs to the first apperance of *endlineRead*.
    Remaining characters are recognized at the next call to **getVal**. If no endline characters is detected within *timeout* seconds, a timeout (error code: 256)
    is raised.
    The following code example demonstrates how to obtain the bytearray until the **endlineRead** sign.

   .. code-block:: python

    serial.setParam('readline', True)
    b = bytearray(100)
    num = serial.getVal(b)
    signs = b[0:num]
    print(signs)



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

Configure the serialIO plugin to wait for full answers
========================================================

Per default, the user will send a request to a device. The device will then probably answer to this request using a specified string pattern.
If the parameter **readline** is set to 0 (default), a call to *getVal* (see usage above), will only return all characters that fit into the
buffer size and are currently available at the input buffer of the serial port. If the device need more time for the entire answer, the remaining
part of the answer can only be obtained by continuously calling *getVal* until the entire answer is obtained.

Usually, an answer always ends by a certain endline character sequence. If this sequence contains 1 or 2 characters, you can also configure
the serialIO plugin such that *getVal* will continuously check the input buffer of the serial port and collect all characters until the endline
sequence (parameter **endlineRead**) is detected or a timeout occurred. If the endline sequence was found, the characters until but without the
first endline sequence are returned by the passed buffer. Remaining characters are put onto an internal buffer and considered at the next call
to *getVal*. By this configuration you will automatically wait for the full answer without further programming work and without idle delay times.

An example for this alternative approach is:

.. code-block:: python

    serial.setParam("readline", 1)
    serial.setParam("endlineRead", "\n")
    buffer = bytearray(20)
    serial.setVal("POS?")
    num = serial.getVal(buffer)
    print("full answer from device", buffer[0:num])

Clear input or output buffer
============================

Sometimes, it is necessary to immediately clear all characters inside of the input buffer (obtained by *getVal*) and/or output buffer (send by *setVal*). This can be done using specific **exec**-functions:

.. code-block:: python

    serial.exec("clearInputBuffer") #clear input buffer
    serial.exec("clearOutputBuffer") #clear output buffer

    #alternative:
    serial.exec("clearBuffer", 0) #clear input buffer
    serial.exec("clearBuffer", 1) #clear output buffer

Get a list of available COM ports under Windows
===============================================

In order to get a list of all available COM ports under Windows, the following python code snippet can be used. It reads the corresponding registry entries:

.. code-block:: python

    #This script can be used as example for Windows
    #to detect registered COM ports for this computer
    import winreg as wreg

    def DetectCOMPorts():
        try:
            regconn = wreg.ConnectRegistry( None, wreg.HKEY_LOCAL_MACHINE )
            key = wreg.OpenKey( regconn, "HARDWARE\\DEVICEMAP\\SERIALCOMM", wreg.KEY_READ )
            values_count = wreg.QueryInfoKey( key )[1]
            values_list = []
            for i in range( values_count ):
                values_list.append( wreg.EnumValue( key, i ) )
        except ( WindowsError, EnvironmentError ):
            print( "Unable to Connect to the Window Registry and read keys" )
        finally:
            key.Close()
        return values_list

    def NumberOfCOMPorts( values_list ):
        for subkey in iter( values_list ):
            print( "Name : " + subkey[0] )
            print( "Data : " + subkey[1] )

    NumberOfCOMPorts( DetectCOMPorts() )

Installation
============

For using this plugin no further 3rd party libraries or drivers are necessary.

If you are using linux to open a serial connection and you are running **itom** without root privileges, which is recommended, you have to add
your user to the **dialout** group, logout once and login again.

.. code-block:: bash

    sudo adduser USERNAME dialout

where USERNAME ist the username under which you are running **itom**

Changelog
===========

* itom 1.2.0 is shipped with version 0.0.2 of serialIO
* parameters 'readline' and 'endlineRead' are available in serialIO version >= 1.0.0
* serialIO version 1.1.1 bug fix in command history with empty commands
