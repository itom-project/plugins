===================
 QuantumComposer
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`QuantumComposer`
**Type**:       :plugintype:`QuantumComposer`
**License**:    :pluginlicense:`QuantumComposer`
**Platforms**:  Some words about supported operating systems
**Devices**:    Some words about supported devices
**Author**:     :pluginauthor:`QuantumComposer`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: QuantumComposer

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: QuantumComposer


Parameters
==========

These parameters are available and can be used to configure the **QuantumComposer** instance. Many of them are directly initialized by the parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable parameters can be changed using *setParam*.

**amplitudesList**: Sequence[float]
    List of amplitude levels to set to the channels listed in the parameter
    channelIndexList. List must have the same length as the parameter channelIndexList.
    
    *Allowed number of values: 1 - 8, Value range: [2:0.01:20], Default: [-1000, -1000,
    -1000, -1000, -1000, -1000, -1000, -1000]*
**burstCounter**: int
    Number of pulses to generate in the burst mode.
    
    *Value range: [1, 9999999], Default: 4*
**channelBurstCounterList**: Sequence[int]
    List of burst counter values for the given channels (1 - 9999999). List must have the
    same length as the parameter channelIndexList.
    
    *Allowed number of values: 1 - 8, Value range: [1, 9999999], Default: [1, 2, 3, 4, 5, 6,
    7, 8]*
**channelGateLogicList**: Sequence[str]
    List of channel gate logic level (LOW, HIGH).
    
    *Allowed number of values: 1 - 8, Value rules: Match: ["LOW", "HIGH"], Default: [LOW,
    HIGH]*
**channelGateModeList**: Sequence[str]
    List of channel gate modes (DIS = disable, PULS = pulse inhibit, OUTP = output inhibit).
    
    *Allowed number of values: 1 - 8, Value rules: Match: ["DIS", "PULS", "OUTP"], Default:
    [DIS, PULS, OUTP]*
**channelModesList**: Sequence[str]
    List of channel modes which are set to the output for the given channels (NORM = normal,
    SING = single shot, BURS = burst, DCYC = duty cycle).
    
    *Allowed number of values: 1 - 8, Value rules: Match: ["NORM", "SING", "BURS", "DCYC"],
    Default: [NORM, SING, BURS, DCYC]*
**channelOffCounterList**: Sequence[int]
    List of pulse counter values to inhibit during the OFF cycle of the duty cycle mode for
    the given channels (1 - 9999999). List must have the same length as the parameter
    channelIndexList.
    
    *Allowed number of values: 1 - 8, Value range: [1, 9999999], Default: [1, 2, 3, 4, 5, 6,
    7, 8]*
**channelPulseCounterList**: Sequence[int]
    List of pulse counter values to generate during the ON cycle of the duty cycle mode for
    the given channels (1 - 9999999). List must have the same length as the parameter
    channelIndexList.
    
    *Allowed number of values: 1 - 8, Value range: [1, 9999999], Default: [1, 2, 3, 4, 5, 6,
    7, 8]*
**channelPulseWaitCounterList**: Sequence[int]
    List of pulse counter values to wait until enabling output of the duty cycle mode for
    the given channels (0 - 9999999). List must have the same length as the parameter
    channelIndexList.
    
    *Allowed number of values: 1 - 8, Value range: [0, 9999999], Default: [1, 2, 3, 4, 5, 6,
    7, 8]*
**counterCounts**: int, read-only
    Number of counts.
    
    *Value range: [0, inf], Default: 4*
**counterState**: int
    Enables (1), disables(0) the counter function.
    
    *Value range: [0, 1], Default: 0*
**delaysList**: Sequence[float]
    List of delays to set to the channels listed in the parameter channelIndexList. List
    must have the same length as the parameter channelIndexList.
    
    *Allowed number of values: 1 - 8, Value range: [-1000, 1000], Default: [-1000, -1000,
    -1000, -1000, -1000, -1000, -1000, -1000]*
**gateLevel**: float
    Gate threshold in units of V with a range of 0.20V to 15.0V.
    
    *Value range: [0.2:0.01:15], Default: 2.5*
**gateLogic**: str
    Gate logic level (LOW, HIGH).
    
    *Match: ["LOW", "HIGH"], Default: "LOW"*
**gateMode**: str
    Global gate mode of the system output. (DIS: diabled, PULS: pulse inhibit, OUTP: output
    inhibit, CHAN: channel).cycle).
    
    *Match: ["DIS", "PULS", "OUTP", "CHAN"], Default: "DIS"*
**icLock**: str
    Source for the internal rate generator. System clock or external source ranging from
    10MHz to 100MHz (SYS, EXT10, EXT20, EXT25, EXT40, EXT50, EXT80, EXT100).
    
    *Match: ["SYS", "EXT10", "EXT20", "EXT25", "EXT40", "EXT50", "EXT80", "EXT100"], Default:
    "SYS"*
**manufacturer**: str, read-only
    Manufacturer identification.
    
    *Match: "Device parameter", Default: "QC"*
**mode**: str
    Mode of the system output. (NORM: normal, SING: single shot, BURS: burst, DCYC: duty
    cycle).
    
    *Match: ["NORM", "SING", "BURS", "DCYC"], Default: "NORM"*
**model**: str, read-only
    Model identification.
    
    *Match: "Device parameter", Default: "9528"*
**muxsList**: Sequence[int]
    List of timers which are enabled as output for the given channel. List must have the
    same length as the parameter channelIndexList.
    
    *Allowed number of values: 1 - 8, Value range: [0, 255], Default: [1, 2, 3, 4, 5, 6, 7,
    8]*
**name**: str, read-only
    Plugin name.
    
    *Match: "General", Default: "QuantumComposer"*
**ocLock**: str
    External clock output. T0 pulse or 50% duty cycle TTL output from 10MHz to 100MHz (T0,
    10, 11, 12, 14, 16, 20, 25, 33, 50, 100).
    
    *Match: ["T0", "10", "11", "12", "14", "16", "20", "25", "33", "50", "100"], Default:
    "T0"*
**offCounter**: int
    Number of pulses to inhibit output during the off cycle of the Duty Cycle mode.
    
    *Value range: [1, 9999999], Default: 2*
**outputModesList**: Sequence[str]
    List of output modes which are set to the output for the given channels (TTL = TTL/CMOS,
    ADJ = adjustable).
    
    *Allowed number of values: 1 - 8, Value rules: Match: ["TTL", "ADJ"], Default: [TTL, ADJ]*
**period**: float
    T0 period in units of seconds (100ns - 5000s).
    
    *Value range: [6e-08:1e-08:5000], Default: 0.0001*
**polaritiesList**: Sequence[str]
    List of polarities which are set to the output for the given channels (NORM = normal,
    COMP = complement, INV = inverted).
    
    *Allowed number of values: 1 - 8, Value rules: Match: ["NORM", "COMP", "INV"], Default:
    [NORM, COMP, INV]*
**pulseCounter**: int
    Number of pulses to inhibit output during the off cycle of the Duty Cycle mode.
    
    *Value range: [1, 9999999], Default: 3*
**requestTimeout**: int
    Request timeout in ms for the SerialIO interface.
    
    *Value range: [0, inf], Default: 500*
**serialNumber**: str, read-only
    Serial number.
    
    *Match: "Device parameter", Default: "06312"*
**state**: int
    Enables (1), disables (0) the output for all channels. Command is the same as pressing
    the RUN/STOP button.
    
    *Value range: [0, 1], Default: 1*
**statesList**: Sequence[int]
    List of states to enalbe/disable channels listed in the parameter channelIndexList. List
    must have the same length as the parameter channelIndexList.
    
    *Allowed number of values: 1 - 8, Value range: [0, 1], Default: [0, 0, 1, 1, 1, 1, 1, 1]*
**syncsList**: Sequence[int]
    List of channels to sync with the channels listed in the parameter channelIndexList.
    List must have the same length as the parameter channelIndexList.
    
    *Allowed number of values: 1 - 8, Value range: [1, 8], Default: [1, 2, 3, 4, 5, 6, 7, 8]*
**triggerEdge**: str
    Trigger edge to use as the trigger signal (RIS: rising, FALL: falling).
    
    *Match: ["RIS", "FALL"], Default: "RIS"*
**triggerLevel**: float
    Trigger threshold in units of V with a range of 0.20V to 15.0V.
    
    *Value range: [0.2:0.01:15], Default: 2.5*
**triggerMode**: str
    Trigger mode (DIS: disabled, TRIG: triggered, enabled).
    
    *Match: ["DIS", "TRIG"], Default: "DIS"*
**version**: str, read-only
    Version number.
    
    *Match: "Device parameter", Default: "2.4.3-2.0.11"*
**widthsList**: Sequence[float]
    List of widths to set to the channels listed in the parameter channelIndexList. List
    must have the same length as the parameter channelIndexList.
    
    *Allowed number of values: 1 - 8, Value range: [2e-09, 1000], Default: [2e-09, 2e-09,
    2e-09, 2e-09, 2e-09, 2e-09, 2e-09, 2e-09]*


Additional functions (exec functions)
=======================================

By using the following execFunctions you set the channels parameter by giving a list of channel number and a list of parameter values of same list length.
The plugin execFunctions are:

.. py:function::  instance.exec('setChannelAdjustableAmplitude', channelIndexList, amplitudesList)

    Set the adjustable amplitude of channel output level of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param amplitudesList: List of amplitude levels to set to the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.
    :type amplitudesList: Sequence[float]

.. py:function::  instance.exec('setChannelBurstCounter', channelIndexList, channelBurstCounterList)

    Set the channel burst counter for the burst mode of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param channelBurstCounterList: List of burst counter values for the given channels (1 - 9999999). List must have the same length as the parameter channelIndexList.
    :type channelBurstCounterList: Sequence[int]

.. py:function::  instance.exec('setChannelDelays', channelIndexList, delaysList)

    Set the pulse delays of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param delaysList: List of delays to set to the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.
    :type delaysList: Sequence[float]

.. py:function::  instance.exec('setChannelGatesLogicLevel', channelIndexList, channelGateLogicList)

    Set the channel gates logic level of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param channelGateLogicList: List of channel gate logic level (LOW, HIGH).
    :type channelGateLogicList: Sequence[str]

.. py:function::  instance.exec('setChannelGatesModes', channelIndexList, channelGateModeList)

    Set the channel gates mode of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param channelGateModeList: List of channel gate modes (DIS = disable, PULS = pulse inhibit, OUTP = output inhibit).
    :type channelGateModeList: Sequence[str]

.. py:function::  instance.exec('setChannelModes', channelIndexList, channelModesList)

    Set the channel mode of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param channelModesList: List of channel modes which are set to the output for the given channels (NORM = normal, SING = single shot, BURS = burst, DCYC = duty cycle).
    :type channelModesList: Sequence[str]

.. py:function::  instance.exec('setChannelMuxs', channelIndexList, muxsList)

    Set which timers are enabled as output for the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param muxsList: List of timers which are enabled as output for the given channel. List must have the same length as the parameter channelIndexList.
    :type muxsList: Sequence[int]

.. py:function::  instance.exec('setChannelOffCounter', channelIndexList, channelOffCounterList)

    Set the channel pulse counter during the OFF cycles for the duty cycle modes of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param channelOffCounterList: List of pulse counter values to inhibit during the OFF cycle of the duty cycle mode for the given channels (1 - 9999999). List must have the same length as the parame
... ter channelIndexList.
    :type channelOffCounterList: Sequence[int]

.. py:function::  instance.exec('setChannelOutputModes', channelIndexList, outputModesList)

    Set the output amplitude mode of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param outputModesList: List of output modes which are set to the output for the given channels (TTL = TTL/CMOS, ADJ = adjustable).
    :type outputModesList: Sequence[str]

.. py:function::  instance.exec('setChannelOutputState', channelIndexList, statesList)

    Enables/Disables the output state of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param statesList: List of states to enalbe/disable channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.
    :type statesList: Sequence[int]

.. py:function::  instance.exec('setChannelPolarities', channelIndexList, polaritiesList)

    Set the polarity of the pulse for the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param polaritiesList: List of polarities which are set to the output for the given channels (NORM = normal, COMP = complement, INV = inverted).
    :type polaritiesList: Sequence[str]

.. py:function::  instance.exec('setChannelPulseCounter', channelIndexList, channelPulseCounterList)

    Set the channel pulse counter during the ON cycles for the duty cycle modes of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param channelPulseCounterList: List of pulse counter values to generate during the ON cycle of the duty cycle mode for the given channels (1 - 9999999). List must have the same length as the para
... meter channelIndexList.
    :type channelPulseCounterList: Sequence[int]

.. py:function::  instance.exec('setChannelSyncs', channelIndexList, syncsList)

    Set the sync channels of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param syncsList: List of channels to sync with the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.
    :type syncsList: Sequence[int]

.. py:function::  instance.exec('setChannelWaitCounter', channelIndexList, channelPulseWaitCounterList)

    Set the channel pulse counter to wait until enabling output for the duty cycle modes of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param channelPulseWaitCounterList: List of pulse counter values to wait until enabling output of the duty cycle mode for the given channels (0 - 9999999). List must have the same length as the pa
... rameter channelIndexList.
    :type channelPulseWaitCounterList: Sequence[int]

.. py:function::  instance.exec('setChannelWidths', channelIndexList, widthsList)

    Set the pulse width of the given channels.

    :param channelIndexList: List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).
    :type channelIndexList: Sequence[int]
    :param widthsList: List of widths to set to the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.
    :type widthsList: Sequence[float]

Exemplary usage from Python
=======================================

In the following examples, it is shown how to use this Plugin. 

.. code-block:: python

    from itom import dataIO

    serial = dataIO("SerialIO", 5, 38400, "\r\n")  # first create a "SerialIO" instance
    qc = dataIO("QuantumComposer", serial, "USB")  # give it the "QuantumComposer" plugin

The system parameter are set/get by the default ``setParam`` and ``getParam`` methods. 

.. code-block:: python 

    # set
    qc.setParam("mode", "BURS")
    qc.setParam("gateMode", "PULS")
    qc.setParam("triggerEdge", "RIS")
    qc.setParam("triggerLevel", 10.0)
    qc.setParam("gateLogic", "LOW")
    qc.setParam("gateLevel", 15.0)
    qc.setParam("triggerMode", "DIS")
    qc.setParam("state", 1)
    qc.setParam("burstCounter", 100)
    qc.setParam("pulseCounter", 3453)
    qc.setParam("offCounter", 75645)
    qc.setParam("icLock", "SYS")
    qc.setParam("ocLock", "16")
    qc.setParam("period", 6e-7)
    qc.setParam("counterState", 0)
    qc.getParam("counterCounts")

    # get
    qc.getParam("mode")
    # ... 

The channel specific parameter are set by the ``exec`` method of the plugin. 
For each parameter you must give the method a list of ``channels`` you want to change 
and a list of values of same list length. 

.. code-block:: python

    # set
    qc.exec("setChannelWidths", [1, 2], [0.000000002, 0.000000002])
    qc.exec("setChannelDelays", [1], [0.000002])
    qc.exec("setChannelSyncs", [1,2], [5, 6])
    qc.exec("setChannelMuxs", [1,2], [25, 255])
    qc.exec("setChannelPolarities", [1,2, 3], ["NORM","COMP", "INV"])
    qc.exec("setChannelOutputModes", [1,2, 3], ["TTL", "ADJ", "TTL"])
    qc.exec("setChannelAdjustableAmplitude", [1,2, 3], [2.2, 4.3, 13.8])
    qc.exec("setChannelModes", [1,2, 3], ["SING", "BURS", "DCYC"])
    qc.exec("setChannelBurstCounter", [1,2, 3], [99, 99,99])
    qc.exec("setChannelPulseCounter", [1,2, 3], [99, 99,99])
    qc.exec("setChannelOffCounter", [1,2, 3], [99, 99,99])
    qc.exec("setChannelWaitCounter", [1,2, 3], [99, 99,99])
    qc.exec("setChannelGatesModes", [1,2, 3], ["DIS", "PULS", "OUTP"])
    qc.exec("setChannelGatesLogicLevel", [1,2, 3], ["LOW", "HIGH", "LOW"])

The channel specific parameter are get by the ``getParam`` method, too. 


Changelog
==========


