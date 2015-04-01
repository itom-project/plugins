===================
 X3P Input Output
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`x3pio`
**Type**:       :plugintype:`x3pio`
**License**:    :pluginlicense:`x3pio`
**Platforms**:  Windows, Linux
**Author**:     :pluginauthor:`x3pio`
=============== ========================================================================================================
  
Overview
========

.. pluginsummaryextended::
    :plugin: x3pio

These filters are defined in the plugin:

.. pluginfilterlist::
    :plugin: x3pio
    :overviewonly:

Filters
==============
        
Detailed overview about all defined filters:
    
.. pluginfilterlist::
    :plugin: x3pio
    
Compilation
===============

This plugin requires the 3rd party libary CodeSynthesis XSD 3.3.0. Other versions are not officially supported by the
underlying x3p library (see www.opengps.eu). Please install CodeSynthesis XSD 3.3.0 and indicate XERCESC_ROOT_DIR to the base
directory of CodeSynthesis. XERCESC_INCLUDE must then point to the include directory. The variables XERCESC_LIBRARY, XERCESC_LIBRARY_DEBUG...
should then be automatically found.

After compilation, the following libraries and files are copied into the lib path of itom:

* iso5436-2.xsd
* iso5436-2-xml.dll (x3p library)
* xerces-c_3_1D_vc100.dll (library from CodeSynthesis XSD)

Since the xerces library is copied to the lib folder, CodeSynthesis must not be included into the Path environment variable. Therefore you can uncheck
the corresponding option in the setup of CodeSynthesis.

If there is a problem with the compilation, saying that the option cxx-tree is not available within xsd.exe, then you should check the variable XSD_EXECUTABLE.
This should point to xsd.exe within the bin folder of CodeSynthesis, not to any windows directory.

