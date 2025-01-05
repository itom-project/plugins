===================
 FaulhaberMCS
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`FaulhaberMCS`
**Type**:       :plugintype:`FaulhaberMCS`
**License**:    :pluginlicense:`FaulhaberMCS`
**Platforms**:  Windows, Linux
**Devices**:    Faulhaber MCS 3242 BX4 ET
**Author**:     :pluginauthor:`FaulhaberMCS`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: FaulhaberMCS

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: FaulhaberMCS


Parameters
==========

These parameters are available and can be used to configure the ``FaulhaberMCS`` instance.
Many of them are directly initialized by the parameters of the constructor.
During the runtime of an instance, the value of these parameters is obtained by
the method *getParam*, writeable parameters can be changed using *setParam*.

**acceleration**: int
    Acceleration in 1/s². Register '0x6083.00'.

    *Value range: [1, 30000], Default: 4250*
**async**: int, read-only
    Asynchronous move (1), synchronous (0) [default]. Only synchronous operation is
    implemented.

    *Value range: [0, 1], Default: 0*
**current**: int, read-only
    Actual value of the current in relative scaling. Register '0x6078.00'.

    *Value range: [-32768, 32767], Default: 10*
**deceleration**: int
    Deceleration in 1/s². Register '0x6084.00'.

    *Value range: [1, 30000], Default: 1280*
**deviceID**: int
    Explicit device ID. Register '0x2400.08'

    *Value range: [0, 65535], Default: 0*
**deviceName**: str, read-only
    Name of device. Register '0x1008.00'.

    *Match: "", Default: "MCS 3242 BX4 ET"*
**fault**: int, read-only
    1: Error present, 0: No error present (Bit 3).

    *Value range: [0, 1], Default: 0*
**firmware**: str, read-only
    Firmware version. Register '0x100A.00'

    *Match: "", Default: "0111.02N"*
**fluxGainControl**: int
    Flux control gain parameter [mOm]. Register '0x2342.01'.

    *Value range: [-1, 0], Default: 1835*
**fluxIntegralTimeControl**: int
    Flux control integral time control parameter [µs]. Register '0x2342.01'.

    *Value range: [150, 2600], Default: 150*
**followingError**: int, read-only
    1: Permissible range for the following error exceeded, 0: The actual position follows
    the instructions without a following error (Bit 13).

    *Value range: [0, 1], Default: 0*
**homed**: int, read-only
    homed (1) or not homed (0).

    *Value range: [0, 1], Default: 0*
**internalLimitActive**: int, read-only
    1: Internal range limit (e.g. limit switch reached), 0: Internal range limit not reached
    (Bit 11).

    *Value range: [0, 1], Default: 0*
**maxMotorSpeed**: int
    Max motor speed in 1/min. Register '0x6080.00'.

    *Value range: [1, 32767], Default: 32767*
**maxTorqueLimit**: int, read-only
    Maximum torque limit in relative scaling. 1000 = motor rated torque. Register
    '0x6072.00'.

    *Value range: [1, 30000], Default: 2000*
**moveTimeout**: int
    Timeout for movement in ms.

    *Value range: [0, inf], Default: 60000*
**name**: str, read-only
    FaulhaberMCS
**netMode**: int
    RS232 net mode. Register '0x2400.05'.

    *Value range: [0, 1], Default: 0*
**nodeID**: int
    Node number. Register '0x2400.03'.

    *Value range: [0, 255], Default: 1*
**operation**: int
    Enable (1) or Disable (0) operation.

    *Value range: [0, 1], Default: 1*
**operationEnabled**: int, read-only
    1: Operation enabled, 0: Operation disabled (Bit 2).

    *Value range: [0, 1], Default: 1*
**operationMode**: int, read-only
    Operation Mode. -4: ATC, -3: AVC, -2: APC, -1: Voltage mode, 0: Controller not
    activated, 1: PP (default), 3: PV, 6: Homing, 8: CSP, 9: CSV, 10: CST. Register
    '0x6060.00'.

    *Value range: [-4, 10], Default: 1*
**positionLimits**: Sequence[int]
    Lower/ upper limit of the position range in userdefined uints. Register lower limit
    '0x607D.01', upper limit '0x607D.02'.

    *Allowed number of values: 0 - 18446744073709551615, All values allowed, Default: [-inf,
    inf]*
**power**: int
    Enable (1) or Disable (0) device power.

    *Value range: [0, 1], Default: 1*
**productCode**: str, read-only
    Product code number. Register '0x1018.02'.

    *Match: "", Default: "11601"*
**profileVelocity**: int
    Profile velocity in 1/min. Register '0x6081.00'.

    *Value range: [1, 32767], Default: 4000*
**quickStop**: int, read-only
    1: Quick stop enabled, Quick stop disabled (Bit 5).

    *Value range: [0, 1], Default: 1*
**quickStopDeceleration**: int
    Quickstop deceleration in 1/s². Register '0x6085.00'.

    *Value range: [1, 32750], Default: 30000*
**readyToSwitchOn**: int, read-only
    1: Ready to switch ON, 0: Not ready to switch ON (Bit 0).

    *Value range: [0, 1], Default: 1*
**revisionNumber**: str, read-only
    Revision number. Register '0x1018.03'

    *Match: "", Default: "14"*
**serialNumber**: str, read-only
    Serial number of device. Register '0x2400.03'.

    *Match: "", Default: "492300001"*
**setPointAcknowledged**: int, read-only
    1: New set-point has been loaded, 0: Previous set-point being changed or already reached
    (Bit 12).

    *Value range: [0, 1], Default: 0*
**switchOnDisabled**: int, read-only
    1: Switch on disabled, 0: Switch on enabled (Bit 6).

    *Value range: [0, 1], Default: 0*
**switchedOn**: int, read-only
    1: Drive is in the 'Switched ON' state, 0: No voltage present (Bit 1).

    *Value range: [0, 1], Default: 1*
**targetReached**: int, read-only
    1: Target has reached, 0: is moving (Bit 10).

    *Value range: [0, 1], Default: 1*
**targetTorque**: int
    Set target value of the torque in relative scaling. Register '0x6071.00'.

    *Value range: [-32768, 32767], Default: 0*
**temperatureCPU**: int, read-only
    CPU temperature in [°C]. Register '0x2326.01'.

    *Value range: [0, 32767], Default: 44*
**temperaturePowerStage**: int, read-only
    Power stage temperature in [°C]. Register '0x2326.02'.

    *Value range: [0, 32767], Default: 30*
**temperatureWinding**: int, read-only
    Winding temperature in [°C]. Register '0x2326.03'.

    *Value range: [0, 32767], Default: 29*
**torque**: int, read-only
    Actual value of the torque in relative scaling. Register '0x6077.00'.

    *Value range: [-32768, 32767], Default: 0*
**torqueGainControl**: int
    Torque control gain parameter [mOm]. Register '0x2342.01'.

    *Value range: [-1, 0], Default: 1835*
**torqueIntegralTimeControl**: int
    Torque control integral time control parameter [µs]. Register '0x2342.01'.

    *Value range: [150, 2600], Default: 150*
**torqueLimits**: Sequence[int]
    Negative/ positive torque limit values in relative scaling. 1000 = motor rated torque.
    Register negative limit '0x60E1.00', positive limit '0x60E0.00'.

    *Allowed number of values: 0 - 18446744073709551615, Value range: [0, 6000], Default:
    [6000, 6000]*
**velocityDeviationThresholdControl**: int
    Velocity deviation threshold control parameter. Register '0x2344.03'.

    *Value range: [0, 65535], Default: 65535*
**velocityDeviationTimeControl**: int
    Velocity deviation time control parameter. Register '0x2344.04'.

    *Value range: [0, 65535], Default: 100*
**velocityGainControl**: int
    Velocity gain control parameter [As 1e-6]. Register '0x2342.01'.

    *Value range: [-1, 0], Default: 1835*
**velocityIntegralPartOption**: int
    Velocity integral part option. Configuration of the speed control loop. '0': integral
    component active, '1': stopped integral component in the position windoed (in PP mode),
    '2': integral component deactivated. Register '0x2344.06'.

    *Value range: [0, 2], Default: 0*
**velocityIntegralTimeControl**: int
    Velocity integral time control parameter [µs]. Register '0x2344.02'.

    *Value range: [0, 65535], Default: 23*
**velocityWarningThresholdControl**: int
    Velocity warning threshold control parameter. Register '0x2344.05'.

    *Value range: [0, 65535], Default: 30000*
**vendorID**: str, read-only
    Vendor ID of device. Register '0x1018.01'.

    *Match: "", Default: "327"*
**voltageEnabled**: int, read-only
    1: Power supply enabled, 0: Power supply disabled (Bit 4).

    *Value range: [0, 1], Default: 0*
**warning**: int, read-only
    1: One of the monitored temperatures has exceeded at least the warning threshold, 0: No
    raised temperatures (Bit 7).

    *Value range: [0, 1], Default: 0*


Additional functions (exec functions)
=====================================

By using the following execFunctions you execute homing according the homing methods.

.. py:function::  instance.exec('homing', method [,offset, switchSeekVelocity, homingSpeed, acceleration, limitCheckDelayTime, torqueLimits])

    In most of the cases before position control is to be used, the drive must perform a reference run to align the position used by the drive to the mechanic setup.

    :param method: Homing method. Methods 1…34: A limit switch or an additional reference switch is used as reference. Method 37: The position is set to 0 without reference run. Methods –1…–4: A mechanical limit stop is set as reference. Register '0x6098.00'.
    :type method: int
    :param offset: Offset of the zero position relative to the position of the reference switch in userdefined units. Register '0x607C.00'.
    :type offset: int - optional
    :param switchSeekVelocity: Speed during search for switch. Register '0x6099.01'.
    :type switchSeekVelocity: int - optional
    :param homingSpeed: Speed during search for zero. Register '0x6099.02'.
    :type homingSpeed: int - optional
    :param acceleration: Speed during search for zero. Register '0x609A.00'.
    :type acceleration: int - optional
    :param limitCheckDelayTime: Delay time until blockage detection [ms]. Register '0x2324.02'.
    :type limitCheckDelayTime: int - optional
    :param torqueLimits: Upper/ lower limit values for the reference run in 1/1000 of the rated motor torque. Register negative limit '0x2350.00', positive limit '0x2351.00'.
    :type torqueLimits: Sequence[int] - optional

Exemplary usage from Python
===========================

In the following examples, it is shown how to use this Plugin.
First an instance must be initialized using the ``SerialIO`` Plugin.

.. code-block:: python

    from itom import actuator, dataIO

    com = dataIO("SerialIO", 6, 115200, "\n")  # adapt COM port number
    mot = actuator("FaulhaberMCS", com, 1)

The current position can be set to zero by using the ``setOrigin`` method of the plugin:

.. code-block:: python

    mot.setOrigin(0)

The relative position can be set to a specific value by using the ``setPosRel`` method of the plugin:

.. code-block:: python

    mot.setPosRel(0, 4096)

The absolute position can be set to a specific value by using the ``setPosAbs`` method of the plugin:

.. code-block:: python

    mot.setPosAbs(0, 4096)

Additional homing methods can be executed by using the ``exec`` method of the plugin:

.. code-block:: python

    mot.exec("homing", -3, torqueLimits=[300, 300]) # mechanical limit stop is set as reference

Parameters can be read by using the ``getParam`` method of the plugin:

.. code-block:: python

    print(mot.getParam("positionLimits"))

Parameters can be set by using the ``setParam`` method of the plugin:

.. code-block:: python

    mot.setParam("profileVelocity", 2000)

Motor can be turned off setting parameter ``operation``:

.. code-block:: python

    mot.setParam("operation", 0) # turn off motor
    mot.setParam("operation", 1) # turn on motor

The motor voltage is turned off setting parameter ``power``:

.. code-block:: python

    mot.setParam("power", 0) # turn off motor voltage

    # To turn it on again, first it must be shut down
    mot.setParam("operation", 0)
    mot.setParam("operation", 1) # turn on motor

For long movement operations, the timeout can be set:

.. code-block:: python

    mot.setParam("moveTimeout", 60000) # set timeout to 60s

The torque control parameter are changed by the plugin parameter:

.. code-block:: python

    mot.setParam("torqueGainControl", 1835)
    mot.setParam("torqueIntegralTimeControl", 150)

The flux control parameter are changed by the plugin parameter:

.. code-block:: python

    mot.setParam("fluxGainControl", 1835)
    mot.setParam("fluxIntegralTimeControl", 150)

The velocity control parameter are changed by the plugin parameter:

.. code-block:: python

    mot.setParam("velocityGainControl", 1835)
    mot.setParam("velocityIntegralTimeControl", 23)
    mot.setParam("velocityDeviationThresholdControl", 65535)
    mot.setParam("velocityDeviationTimeControl", 100)
    mot.setParam("velocityWarningThresholdControl", 30000)
    mot.setParam("velocityIntegralPartOption", 0)

Changelog
==========

* itom setup 4.3.0 - v1.0.0: Initial version
