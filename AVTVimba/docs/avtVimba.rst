===================
 AVT Vimba
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`AVTVimba`
**Type**:       :plugintype:`AVTVimba`
**License**:    :pluginlicense:`AVTVimba`
**Platforms**:  Windows (Firewire + GigE), Linux only supports GigE (not tested yet)
**Devices**:    AVT Cameras driven by Vimba interface
**Author**:     :pluginauthor:`AVTVimba`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: AVTVimba


Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: AVTVimba
        
Installation
=============

In order to use this plugin, please install the Vimba SDK from Allied Vision in version 1.3.0 or higher (see http://www.alliedvisiontec.com/de/produkte/software.html).
During the installation, the necessary Vimba DLLs (VimbaC.dll and VimbaCPP.dll) should be copied into the system directory of Windows, such that no further steps need to be
done using the AVT cameras within **itom**. If this is not the case, copy both libraries from the Vimba installation folder to the lib-folder of itom (make sure that you use
the 64bit versions for 64bit itom and vice versa).

If you want to build this plugin from the sources, you need to make sure that you installed the C++ component of the Vimba SDK (select this component during the install process).
In CMake enable the checkbox of the variable **PLUGIN_AVTVimba** and re-configure CMake. Then set the variable **AVTVIMBA_INCLUDE_DIR** to anything similar like *C:/Program Files/Allied Vision Technologies/AVTVimba_1.3/VimbaCPP/Include* (subfolder of the Vimba installation path) and generate your CMake project again. The variables **AVTVIMBA_API_DIR**, AVTVIMBA_LIBRARY**... should now
be found automatically if you indicated a right Vimba installation directory in the step before.

Known issues
==============

Some cameras raise a timeout error when acquiring images. If so, try to set the parameter 'trigger_mode' to 0 (off)::
    
    cam.setParam("trigger_mode", 0)

Changelog
==========

* itom setup 1.4.0: This plugin has been compiled using AVT Vimba 1.3.0
