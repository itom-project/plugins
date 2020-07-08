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

This plugin provides methods to save and load dataObjects in/from the file format 'x3p'.
This format is specified in ISO 25178 - Geometrical product specification (GPS).

The library ISO 5436-2 XML, that is necessary for this plugin and included in the sources,
is licensed under the LGPL license and uses further libraries. For more information about the license
of the library itself see www.opengps.eu.

For loading a x3p file, this plugin only supports the feature types SUR (surface) and PRF (profile), where profile
is always loaded in a 1xN data object. The x- and y- axis of loaded x3p files must always be incremental, no absolute x- 
or y-axes are supported.

Data objects that should be stored in a x3p file are always stored as feature type SUR, such that 1xN or Mx1 data objects
are also stored as surfaces.

The plugin supports to load multi-layer surface files. The layers are then stored as z-layers.

You can save data in a binary (default) or ascii format. Ascii should only be used for few data points (e.g. < 5000).

Please consider, that the axes and value unit of x3p is always meter. This plugins tries to guess the unit of a data object
by evaluating the axisUnits and valueUnit strings. For axes, the scaling factor to get from the unit string (e.g. µm) to
the x3p unit (m) is considered in the increment of the incremental axes descriptions. For all values, the increment value does
not exist, such that the scaling factor is multiplied to each value before storing it in the x3p file. You need to consider this,
since this might lead to precision loss. If a scaling is required for fixed point data types, the values are stored as double values
in x3p, such that there is enough space for the scaled values.

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
underlying x3p library (see www.opengps.eu), but XSD 4.0 also works (with the bugfix mentioned below). 
Please install CodeSynthesis XSD 3.3.0 or CodeSynthesis XSD 4.0.0 and indicate XERCESC_ROOT_DIR to the base
directory of CodeSynthesis. XERCESC_INCLUDE must then point to the include directory. The variables XERCESC_LIBRARY, XERCESC_LIBRARY_DEBUG...
should then be automatically found.

After compilation, the following libraries and files are copied into the lib path of itom:

* iso5436-2.xsd
* iso5436-2-xml.dll (x3p library)
* xerces-c_3_1_vc100.dll (library from CodeSynthesis XSD)

Since the xerces library is copied to the lib folder, CodeSynthesis must not be included into the Path environment variable. Therefore you can uncheck
the corresponding option in the setup of CodeSynthesis.

If there is a problem with the compilation, saying that the option cxx-tree is not available within xsd.exe, then you should check the variable XSD_EXECUTABLE.
This should point to xsd.exe within the bin folder of CodeSynthesis, not to any windows directory.

If xsd could not be found, set XSD_ROOT_DIR to the base directory of CodeSynthesis, hence the same value than XERCESC_ROOT_DIR.

Bugfix for CodeSynthesis XSD 4.0.0
=====================================

If you get a compiler error telling that DOMDocument is an ambigious symbol (conflict with Windows SDK), then you need to change two lines in xsd/cxx/tree/serialization.txx:

.. code-block:: c++
    
    //old
    DOMDocument& doc (*e.getOwnerDocument ());
    const DOMElement& se (x.dom_content ().get ());
    
    //replace by new:
    xercesc::DOMDocument& doc (*e.getOwnerDocument ());
    const xercesc::DOMElement& se (x.dom_content ().get ());
    
The maintainer from XSD promised in a forum that this bug will be fixed in the 4.1 release.

Changelog
=========

* itom setup 3.1.0: This plugin has been compiled using the XSD 4.0
* itom setup 3.2.1: This plugin has been compiled using the XSD 4.0
* itom setup 4.0.0: This plugin has been compiled using the XSD 4.0

