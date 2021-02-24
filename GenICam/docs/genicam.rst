===================
 GenICam
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`GenICam`
**Type**:       :plugintype:`GenICam`
**License**:    :pluginlicense:`GenICam`
**Platforms**:  Windows, Linux
**Devices**:    Camera control of devices that support the GenICam standard
**Author**:     :pluginauthor:`GenICam`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: GenICam

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: GenICam

Starting a device
==================

In order to better understand how a device is opened by this plugin, it is necessary to shortly explain the architecture of
a GenICam compliant device:

* Each camera manufacturer provides access to the parameterization and the image acquisition and transfer between the camera and the computer
  by a so-called GenICam transport layer producer (GenTLProducer). Therefore, the manufacturer has to provide one or multiple transport layer drivers that usually have the file suffix **cti**.
* The technology, how a camera is connected to the computer (e.g. GigE, USB3, CoaxPress, Firewire...), is called an **interface**. Sometimes, every **interface**
  is supported by a distinct **cti** file, sometimes one **cti** file can support multiple **interfaces**.
* Every connected camera is represented by a **device**, that is opened by its corresponding **interface**.
* A device provides different sets of cameras. The entire set of parameters and their descriptions and meta information, is stored in a xml- or zip-file.
  This file can either be stored on the camera itself (default) or on the harddrive. In order to parameterize a camera using a specific xml- or zip-file,
  a certain port is opened to the camera.
* A camera can provide multiple data streams. Usually, a standard camera has exactly one stream that delivers image data. However, it might be possible
  to access devices that deliver different type of output data, e.g. image data or point clouds...

The initialization parameters of a GenICam plugin mostly represent the different items of the connection architecture, as listed above.
It is possible to receive a list of detected transport layers, interfaces or devices if you leave the level, which you are currently interested in, empty.
For instance, if you don't indicate a GenTLProducer, a list of all detected producers is printed to the command line.

If you indicated a certain GenTLProducer, you can leave the 'interface' parameter empty, to get a list of all supported interfaces of this producer. If you
chose 'auto', the first interface with at least one connected camera device will be chosen.

In order to precisely select a device, e.g. based on its name or serial number, indicate the GenTLProducer and the interface, but leave the deviceID empty in order to
get a list of all currently connected devices.

Information about possible ports and streams can be obtained by choosing a high verbosity level as initialization parameter 'verbose'.

.. note:: With some cameras it may be necessary to set the parameter **numBuffers** to a value of 2 (e. g. FLIR AX5).

Parameters
===========

An instance of the GenICam plugin contains both parameters, that are directly redirected to the parameter set of the camera
as well as some parameters, that are always available and are created by the plugin itself. Usually, the original parameters
from the transport layer of the camera start with a capital letter, while plugin parameters have a small letter as first character.

The set of parameters, that is exposed from the device-specific set, announced by the transport layer, depend on the
initialization parameter 'paramVisibilityLevel'. In general, device parameters are categorized into the three visibility categories
'Beginner (0)', 'Expert (1)' or 'Guru (2)'. Depending on the 'paramVisibilityLevel', only parameters are loaded and created in the plugin instance,
whose visibility level is lower or equal than the selected limit. Usually, the 'Guru' level is only necessary for special parameterizations of
the plugin.

The following list contains all parameters, that are related to the GenICam plugin in itom itself:

**name**: {str}, read-only
    name of the plugin: 'GenICam'
**sizex**: {int}, read-only
    Current width of the acquired image; this parameter is sychronized with the standard parameter 'Width' of the GenICam transport layer.
    This parameter is a mandatory parameter for dataIO-instances in itom.
**sizey**: {int}, read-only
    Current height of the acquired image; this parameter is sychronized with the standard parameter 'Height' of the GenICam transport layer.
    This parameter is a mandatory parameter for dataIO-instances in itom.
**bpp**: {int}, read-only
    Current bitdepth per pixel; this parameter is derived from the 'PixelFormat' standard parameter of the GenICam transport layer.
    This parameter is a mandatory parameter for dataIO-instances in itom.
**integration_time**: {float}
    Integration or exposure time in seconds. This parameter is mapped to the standard parameter 'ExposureTime' (in ms).
**roi**: {int rect [x0,y0,width,height]}
    Current region of interest of the image. Changes in this parameter influence the standard parameters 'OffsetX', 'OffsetY', 'Width' and 'Height'.
**timeout**: {float}
    Timeout (in s) for waiting for the arrival of a new image after the acquire command. This parameter is only used within the itom GenICam plugin.
**numBuffers**: {int}, default: 1
    Number of image buffers that should be created in the 'startDevice' command. Some devices indicate a minimum number of image buffers. Use this
    parameter to set this number before starting the device. This parameter is only used within the itom GenICam plugin.
**userDefinedPayloadSize**: {int}, default: 0
    Usually, the ideal size of an image buffer is returned by the transport layer of GenICam or by the standard parameter 'PayloadSize'. However,
    if you set this parameter to a value bigger than 0, a user-defined buffer size in bytes can be selected. Usually, it is not necessary to change this parameter.
    
Verbose level
=============

During the initialization of a GenICam camera instance, it is possible to select a certain verbose level. The higher the number, the more information,
warning or error messages will be printed to the command line of itom. The different verbose levels are:

* 0: nothing is printed to the command line
* 1: error: only severe errors are displayed
* 2: warning: errors and warnings are displayed
* 3: info: errors, warnings and few information are displayed
* 4: debug: this level contains all levels above including detailed information about the startup process as well as detected parameters of the device
* 5: all: all information is printed including details about the state of all image buffers and reported changes in device-specific parameters.

In verbose level 5, both the zipper or unzipped xml configuration file of the camera (and framegrabber, if available) are saved to files on the harddrive.
The filenames are printed to the command line of itom.

CoaXPress or Camera Link
========================

If cameras are connected via CoaXPress or Camera Link to the computer, the image from the camera is transferred to the framegrabber at first. The
framegrabber can then transform the image another time and this GenICam plugin obtains the image from the framegrabber. The real size and format of
the image is then read from the framegrabber.

Both the camera and the framegrabber, which might come from different manufacturers, provide a set of parameters. In order to distinguish between both,
all parameters of the framegrabber will have the prefix **Fg_**. Please remark, that some framegrabbers need to be properly parameterized before starting
the device.

In the example of an Active Silicon CoaXPress framegrabber, you have to set the parameters **Fg_IncomingWidth** to the **Width** of the camera,
**Fg_IncomingHeight** to the **Height** of the camera and **Fg_IncomingPixelFormat** to the current pixel format of the camera. Then adjust the
values **Fg_Width**, **Fg_Height** and **Fg_PixelFormat** to suitable values, since these values are read by itom to configure a proper image acquisition.
        
Compilation
===========

In order to compile this plugin, download the latest GenICam(TM) GenApi reference implementation from http://www.emva.org/standards-technology/genicam/genicam-downloads/.
Unpack the corresponding archive (under Windows: Development and Runtime.zip) in one folder and set the CMake variable GENICAM_ROOT to this base folder.
After re-configuring CMake the other variables (e.g. GENICAM_GCBASE_LIBRARY...) should be found automatically.

GenICam License
================

This plugin uses the official GenICam library to access the camera devices. Here is a copy of the GenICam license file:

GenICam comes in two versions

* a runtime version
* a development version.

The runtime version comes under the following license:

Copyright (c) EMVA and contributors (see source files)
All rights reserved

Redistribution and use in source and binary forms, without  modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the GenICam standard group nor the names of its contributors
  may be used to endorse or promote products derived from this software without
  specific prior written permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

The development version comes under the GenICam license (see below).


GenICam uses the following 3rd party software packages:

==========  ==============  =============================================
Package     License         Internet
==========  ==============  =============================================
Mathparser  LGPL            http://kirya.narod.ru/mathparser.html
--          --              http://www.sama.ru/~despair/ccalc/
Log4Cpp     LGPL            http://log4cpp.sourceforge.net
CppUnit     LGPL            http://cppunit.sourceforge.net
CLSerAll    NI              http://sourceforge.net/projects/clallserial
xs3p        DSTC            http://xml.fiforms.org/xs3p/index.html
xxhash      New BSD         https://code.google.com/p/xxhash/
XSLTProc    MIT license     http://xmlsoft.org/XSLT/xsltproc2.html
XSDe        Proprietary     NA
==========  ==============  =============================================

Note that the XSDe license was purchased by one of the members of the committee but 
allows all members to re-compile the parser as long as only the GenApi XML vocabulary is used.

All license texts come as part of the GenICam distribution in the licenses
subdiretory. If not, you can download them from the internet.

==========  ======================  ================================================
License     File                    Where to find the license texts
==========  ======================  ================================================
LGPL        LGPL.txt                http://www.gnu.org/licenses/lgpl.html
GenICam     GenICam_License.pdf     http://www.genicam.org
CLSerAll    CLSerAll_LICENSE.txt    http://sourceforge.net/projects/clallserial
xs3p        xs3p_License.mht        http://xml.fiforms.org/xs3p/index.html
xxhash      xxhash_License.txt      http://opensource.org/licenses/BSD-3-Clause
XSLTProc    MIT_License.txt         http://opensource.org/licenses/mit-license.html
XSDe        XSDe License.pdf        NA  
==========  ======================  ================================================

Last but not least GenICam redistributes the C/C++ runtime DLLs of the
Microsoft Visual C++ compiler in the version 12.0

Changelog
==========

* itom setup 3.1.0: This plugin has been compiled using GenICam 3.0.2
* itom setup 3.2.1: This plugin has been compiled using GenICam GenAPI 3.1.0
* itom setup 4.0.0: This plugin has been compiled using GenICam GenAPI 3.2.0
* itom setup 4.1.0: This plugin has been compiled using GenICam GenAPI 3.2.0

Workaround
==========

* Vistek, GigE, Windows: It seems that the Camera Link transport layer library (cti-file) has to be loaded by itom before the GigE transport layer is loaded.
  This is implicitely done, if a vistek cti file is loaded. It is also possible to load the CL cti file using a load library command in Python.

