================================
 Avantes AvaSpec Spectrometer
================================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`AvaSpecSDK`
**Type**:       :plugintype:`AvaSpecSDK`
**License**:    :pluginlicense:`AvaSpecSDK`
**Platforms**:  Windows
**Devices**:    Avantes AvaSpec Spectrometer
**Author**:     :pluginauthor:`AvaSpecSDK`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: AvaSpecSDK

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: AvaSpecSDK

Parameters
===========

An instance of this plugin has the following internal parameters:

**average**: {int}
    Number of averages for every frame
**bpp**: {int}, read-only
    Bit depth. 16 (uint16), if single acquisition. 32 (float32), if averaging.
**integration_time**: {float}
    Integration time of CCD programmed in s, some devices do not accept the full range of allowed values (see AvaSpec for real minimum value of your device).
**lambda_coeffs**: {seq. of float}, read-only
    Coefficients for polynom that determines lambda_table (lambda_table[idx] = c[0] + c[1]*idx + c[2]*idx^2 + ... + c[4]*idx^4)
**lambda_table**: {seq. of float}, read-only
    Vector with the wavelength of all active pixels
**name**: {str}, read-only
    plugin name
**roi**: {int rect [x0,y0,width,height]}
    ROI (x,y,width,height)
**serial_number**: {str}, read-only
    Serial number of spectrometer. Same as identifier.
**sizex**: {int}, read-only
    current width of ROI
**sizey**: {int}, read-only
    current height
**detector_name**: {str}, read-only
    Name of the detector.
**dark_correction**: {int}
    Dark correction mode. See section below.

Timestamp
=========

Every acquired image will have a tag 'timestamp' defined. It contains the timestamp of the acquisition in seconds since the
startup of the spectrometer.

Vendor and Product ID
======================

If the vendor or product ID is unknown, open the libUsb plugin and select the optional parameter "print info of all devices". This will give you a list of all usb devices connectable with libUSB.

Usage
=============

In order to use this plugin, please order the the AvasSpec SDK (see https://www.avantes.com/products/software/interface-packages-and-libraries-for-windows-and-linux/) from
Avantes. Download the **AvaspecX64Dll_9.14.0.0.Setup_64bit.exe** from your Avantes account.
During the installation, the necessary AvasSpec DLLs (avaspecx64.dll) should be copied into the system directory of Windows, such that no further steps need to be
done using the AVT cameras within **itom**. If this is not the case, copy both libraries from the AvasSpec installation folder to the lib-folder of itom (make sure that you use
the 64bit versions for 64bit itom and vice versa).

If you want to build this plugin from the sources, you need to make sure that you installed the C++ component of the AvaSpec SDK (select this component during the install process).
In CMake enable the checkbox of the variable **PLUGIN_AvaSpecSDK** and re-configure CMake. Then set the variable **AVASPEC_DIR** or the Environment Variable **AVASPEC_ROOT**
to anything similar like **C:\AvaSpecX64-DLL_9.14.0.0** (subfolder of the AvasSpec installation path) and generate your CMake project again.
In that case The variables **AVASPEC_DIR** should now be found automatically if you indicated a right AvasSpec installation directory in the step before.

Dark correction
================

Some spectrometers have more pixels on the sensor than are used for the signal generation. These, covered and hence dark pixels, can be used for a dark
correction. There are three types of dark correction (parameter dark_correction):

* Off (0): No dark correction is applied, if the sensor is recognized to have such dark pixels, a tag 'dark' is created that contains the mean value of all dark pixels.
  If no averaging is enabled, the output format of the dataObject is uint16, else float32. Sensors, that don't have dark pixels can only be operated in this mode.
* Static (1): The mean value of all dark pixels is subtracted from all pixels. The output format is float32 always (negative values might occur).
* Dynamic (2): Only choose this mode, if the software AvaSpec provides dynamic dark correction for the sensor (see sensor configuration >> checkbox 'dynamic dark correction'
  must be enabled. In this case, odd and even pixels have different dark correction values (they are probably read by different electronics). Choose this mode to
  subtract different mean values for even and odd pixels. The tag 'dark' still contains the mean of both mean values (0.5 * (mean_even + mean_odd)).

Changelog
=========

* itom setup 4.3.0: This plugin has been compiled using the AvaSpecSDK v9.14.0
