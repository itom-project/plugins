===================
 ThorlabsDMH
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`ThorlabsDMH`
**Type**:       :plugintype:`ThorlabsDMH`
**License**:    :pluginlicense:`ThorlabsDMH`
**Platforms**:  Windows, Linux
**Devices**:    ThorlabsDMH40/M-P01
**Author**:     :pluginauthor:`ThorlabsDMH`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: ThorlabsDMH

.. hint::
    When manually manipulating the segment voltages, the Zernike coefficients do not match anymore.

Installation
============

Beside the THORLABS **"Deformable Mirrors Software"** (`download here <https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=DMP40>`_)
you also need to install the THORLABS **"Optical Power Monitor"** software (`download here <https://www.thorlabs.com/software_pages/ViewSoftwarePage.cfm?Code=OPM>`_).
This is required because a necessary header file is not included in the **"Deformable Mirrors Software"**.
THORLABS has been informed about this issue and will address it in a future version.


Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: ThorlabsDMH

Parameters
==========

These parameters are available and can be used to configure the ``ThorlabsDMH`` instance.
Many of them are directly initialized by the parameters of the constructor.
During the runtime of an instance, the value of these parameters is obtained by
the method ``getParam``, writeable parameters can be changed using ``setParam``.

**async**: int, read-only
    Toggles if motor has to wait until end of movement (0: sync) or not (1: async). Only
    sync supported.

    *Value range: [0, 1], Default: 0*
**commonVoltageMirror**: float, read-only
    Common voltage of mirror segments.

    *Value range: [0, 300], Default: 0*
**commonVoltageTipTilt**: float, read-only
    Common tip/tilt voltage.

    *Value range: [0, 300], Default: 0*
**extensionDriver**: str, read-only
    Extension driver.

    *Match: "", Default: "not applicable"*
**firmware**: str, read-only
    Firmware.

    *Match: "", Default: "1.0"*
**instrumentName**: str, read-only
    Instrument name.

    *Match: "", Default: "DMH40-P01"*
**manufacturerName**: str, read-only
    Manufacturer name.

    *Match: "", Default: "Thorlabs"*
**maxVoltageMirror**: float, read-only
    Max voltage of mirror segments.

    *Value range: [0, 300], Default: 300*
**maxVoltageTipTilt**: float, read-only
    Max tip/tilt voltage.

    *Value range: [0, 300], Default: 300*
**maxZernikeAmplitude**: float, read-only
    Max Zernike amplitude.

    *Value range: [-1, 1], Default: 1*
**minVoltageMirror**: float, read-only
    Min voltage of mirror segments.

    *Value range: [0, 300], Default: 0*
**minVoltageTipTilt**: float, read-only
    Min tip/tilt voltage.

    *Value range: [0, 300], Default: 0*
**minZernikeAmplitude**: float, read-only
    Min Zernike amplitude.

    *Value range: [-1, 1], Default: -1*
**name**: str, read-only
    Name of the plugin.
**numSegments**: int, read-only
    Number of segments.

    *Value range: [0, 40], Default: 40*
**numTipTilt**: int, read-only
    Number of tip/tilt elements.

    *Value range: [0, 15], Default: 0*
**numaxis**: int, read-only
    Number of axes attached to this stage

    *Value range: [40, 40], Default: 40*
**relaxSteps**: int, read-only
    Relax steps.

    *Value range: [0, 100], Default: 80*
**serialNumber**: str, read-only
    Serial number.

    *Match: "", Default: "M01022475"*
**systemMeasurementSteps**: int, read-only
    System measurement steps.

    *Value range: [0, 100], Default: 33*
**zernikeCount**: int, read-only
    Zernike count.

    *Value range: [0, 15], Default: 12*


Additional functions (exec functions)
=====================================

.. py:function::  instance.exec('relaxMirror', )

    Relax the mirror (hysteresis compensation).


.. py:function::  instance.exec('setZernikes', ZernikeIDs, ZernikeValues)

    Sets a list of Zernike coefficients on the entire mirror surface.

    :param ZernikeIDs: list of zernike IDs
    :type ZernikeIDs: Sequence[int]
    :param ZernikeValues: list of zernike values
    :type ZernikeValues: Sequence[float]


Exemplary usage from Python
===========================

In the following examples, it is shown how to use this plugin.
First an instance must be initialized. The plugin will search for all ``ThorlabsDMH`` deformable mirrors and will select the first device.

.. code-block:: python

    from itom import actuator

    mot = actuator("ThorlabsDMH")

If you want to connect to a specific device, you can indicate the serial number.

.. code-block:: python

    from itom import actuator

    mot = actuator("ThorlabsDMH", "---SerialNo.---")

The ``position`` of the segments can be set by using the ``setPosAbs``.
In this example the ``voltage`` of segment [0, 1, 5, 23] are set to [30V, 20V, 180V, 40V].

.. code-block:: python

    mot.setPosAbs(0, 30, 1, 20, 5, 180, 23, 40)

The current ``voltage`` of segment 0, 1, 5, 23 can be shown by using ``getPos``.

.. code-block:: python

    mot.getPos(0, 1, 5, 23)

Increment the voltage of segment 35 relative about 20V:

.. code-block:: python

    mot.getPos(35)
    mot.setPosRel(35, 20)
    mot.getPos(35)

The hysteresis compensation by relaxing the mirror can be carried out as follows:

.. code-block:: python

    mot.exec("relaxMirror")

The surface can be manipulated using Zernikes. The coefficients 4, 5, 6, 11 and 14 are set here.

.. code-block:: python

    mot.exec("setZernikes", [4, 5, 6, 11, 14], [0.1, -0.6, 0.3, 0.124, -0.426])

Setting all Zernike coefficients to zero can be set with the following command:

.. code-block:: python

    mot.exec("setZernikes", list(range(4, 16)), [0]*12)

Changelog
==========

* itom setup 4.3.0 - v1.0.0: Initial version
