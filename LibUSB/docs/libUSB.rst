===================
 LibUSB
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`LibUSB`
**Type**:       :plugintype:`LibUSB`
**License**:    :pluginlicense:`LibUSB`
**Platforms**:  Windows (Vista, 7, 8)
**Devices**:    Any generic USB devices
**Author**:     :pluginauthor:`LibUSB`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: LibUSB

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: LibUSB

Parameters
===========

An instance of this plugin has the following internal parameters:

**debug**: {int}
    If true, all out and inputs are written to dockingWidget
**endpoint_read**: {int}
    Endpoint index for reading operations. The used index is LIBUSB_ENDPOINT_IN + endpoint_read, with LIBUSB_ENDPOINT_IN = 128 (default: initialization parameter 'endpoint')
**endpoint_write**: {int}
    Endpoint index for writing operations. The used index is LIBUSB_ENDPOINT_OUT + endpoint_write, with LIBUSB_ENDPOINT_OUT = 0  (default: initialization parameter 'endpoint')
**name**: {str}
    name of the device
**timeout**: {float}
    Timeout for reading commands in [s]

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
information for your selected device, this device can not be used by this generic plugin. This can also be the case for devices that support the VISA-standard. If you want to use
such devices consider to use the Python package *PyVISA*.

Once the USB-device is opened and the endpoints are configured, you can send and read data in the same way than via a serial connection (see help for plugin **SerialIO**).::

    #send values:
    usbDevice.setVal("myCommand")

    #read values:
    b = bytearray(10) #buffer
    r = usbDevice.getVal(b)
    print(r) #number of characters read

The command **getVal** only reads the number of characters that arrived at the current endpoint at the moment of its call. Analyze the return value and probably call **getVal**
again if you expect more characters to arrive. This is also the same behaviour than for serial connections.

This plugin is also used by other hardware plugins to communicate with further devices.

Compilation
===========
In order to compile LibUSB, get the sources or binaries from LibUSB from http://www.libusb.info. Then set LibUSB_DIR to the base
directory of the 3rd party libusb. libusb is statically linked to the libUSB plugin.

Hint: prebuilt versions of libusb for Visual Studio 2015 can also be found here: https://sourceforge.net/projects/itom/files/all-in-one-build-setup/Optional-3rdParty

Possible linker problems with Visual Studio
--------------------------------------------------

If you get a linker error (similar to **unresolved symbol __imp__vsnprintf**, **unresolved symbol **__imp__iob** in libusb-1.0.lib),
then it is likely that the compiled binaries of libusb are not compatible with your version of Visual Studio.

In this case you have to compile LibUSB by yourself using your version of Visual Studio:

1. Download and unpack the sources of **libusb** (https://github.com/libusb/libusb/releases) to any folder
2. Open the project **msvc/libusb_static_XXXX.vcxproj** with your Visual Studio, where XXXX corresponds to the correct version number.
3. Compile the project (libusb-1.0 static) as **Release** in x86 and / or x64.
4. Create a new folder (e.g. libusb_1.0.23_MSVC2015) and copy the following files to that folder ({sources} is the unpacked source folder):

    libusb_1.0.23_MSVC2015

        /include/libusb-1.0 --> this must contain the header file **libusb.h** from {sources}/libusb
        /MS32/static --> this must contain the **libusb-1.0.lib** from {sources}/Win32/static (if 32bit build)
        /MS64/static --> this must contain the newly built **libusb-1.0.lib** from {sources}/Win64/static (if 64bit build)

5. Set the CMake variable LibUSB_DIR to this new folder or define a windows environment variable LibUSB_ROOT.

Changelog
=========

* itom setup 2.1.0: This plugin has been compiled using the libusb 1.0.20
* itom setup 2.2.0: This plugin has been compiled using the libusb 1.0.20
* itom setup 3.0.0: This plugin has been compiled using the libusb 1.0.20
* itom setup 3.1.0: This plugin has been compiled using the libusb 1.0.21
* itom setup 3.2.1: This plugin has been compiled using the libusb 1.0.22
* itom setup 4.0.0: This plugin has been compiled using the libusb 1.0.23
* itom setup 4.1.0: This plugin has been compiled using the libusb 1.0.24
* itom setup 4.3.0: This plugin has been compiled using the libusb 1.0.26
