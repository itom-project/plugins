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

**amplitudesList**: {Sequence[float]}
    List of amplitude levels to set to the channels listed in the parameter
    channelIndexList. List must have the same length as the parameter channelIndexList.
**burstCounter**: {int}
    Number of pulses to generate in the burst mode.
**channelBurstCounterList**: {Sequence[int]}
    List of burst counter values for the given channels (1 - 9999999). List must have the
    same length as the parameter channelIndexList.
**channelGateLogicList**: {Sequence[str]}
    List of channel gate logic level (LOW, HIGH).
**channelGateModeList**: {Sequence[str]}
    List of channel gate modes (DIS = disable, PULS = pulse inhibit, OUTP = output inhibit).
**channelModesList**: {Sequence[str]}
    List of channel modes which are set to the output for the given channels (NORM = normal,
    SING = single shot, BURST = burst, DCYC = duty cycle).
**channelOffCounterList**: {Sequence[int]}
    List of pulse counter values to inhibit during the OFF cycle of the duty cycle mode for
    the given channels (1 - 9999999). List must have the same length as the parameter
    channelIndexList.
**channelPulseCounterList**: {Sequence[int]}
    List of pulse counter values to generate during the ON cycle of the duty cycle mode for
    the given channels (1 - 9999999). List must have the same length as the parameter
    channelIndexList.
**channelPulseWaitCounterList**: {Sequence[int]}
    List of pulse counter values to wait until enabling output of the duty cycle mode for
    the given channels (0 - 9999999). List must have the same length as the parameter
    channelIndexList.
**counterCounts**: {int}, read-only
    Number of counts.
**counterState**: {int}
    Enables (1), disables(0) the counter function.
**delaysList**: {Sequence[float]}
    List of delays to set to the channels listed in the parameter channelIndexList. List
    must have the same length as the parameter channelIndexList.
**gateLevel**: {float}
    Gate threshold in units of V with a range of 0.20V to 15.0V.
**gateLogic**: {str}
    Gate logic level (LOW, HIGH).
**gateMode**: {str}
    Global gate mode of the system output. (DIS: diabled, PULS: pulse inhibit, OUTP: output
    inhibit, CHAN: channel).cycle).
**icLock**: {str}
    Source for the internal rate generator. System clock or external source ranging from
    10MHz to 100MHz (SYS, EXT10, EXT20, EXT25, EXT40, EXT50, EXT80, EXT100).
**manufacturer**: {str}, read-only
    Manufacturer identification.
**mode**: {str}
    Mode of the system output. (NORM: normal, SING: single shot, BURST: burst, DCYC: duty
    cycle).
**model**: {str}, read-only
    Model identification.
**muxsList**: {Sequence[int]}
    List of timers which are enabled as output for the given channel. List must have the
    same length as the parameter channelIndexList.
**name**: {str}, read-only
    Plugin name.
**ocLock**: {str}
    External clock output. T0 pulse or 50% duty cycle TTL output from 10MHz to 100MHz (T0,
    10, 11, 12, 14, 16, 20, 25, 33, 50, 100).
**offCounter**: {int}
    Number of pulses to inhibit output during the off cycle of the Duty Cycle mode.
**outputModesList**: {Sequence[str]}
    List of output modes which are set to the output for the given channels (TTL = TTL/CMOS,
    ADJ = adjustable).
**period**: {float}
    T0 period in units of seconds (100ns - 5000s).
**polaritiesList**: {Sequence[str]}
    List of polarities which are set to the output for the given channels (NORM = normal,
    COMP = complement, INV = inverted).
**pulseCounter**: {int}
    Number of pulses to inhibit output during the off cycle of the Duty Cycle mode.
**requestTimeout**: {int}
    Request timeout for the SerialIO interface.
**serialNumber**: {str}, read-only
    Serial number.
**state**: {int}
    Enables (1), disables (0) the output for all channels. Command is the same as pressing
    the RUN/STOP button.
**statesList**: {Sequence[int]}
    List of states to enalbe/disable channels listed in the parameter channelIndexList. List
    must have the same length as the parameter channelIndexList.
**syncsList**: {Sequence[int]}
    List of channels to sync with the channels listed in the parameter channelIndexList.
    List must have the same length as the parameter channelIndexList.
**triggerEdge**: {str}
    Trigger edge to use as the trigger signal (RIS: rising, FALL: falling).
**triggerLevel**: {float}
    Trigger threshold in units of V with a range of 0.20V to 15.0V.
**triggerMode**: {str}
    Trigger mode (DIS: disabled, TRIG: triggered, enabled).
**version**: {str}, read-only
    Version number.
**widthsList**: {Sequence[float]}
    List of widths to set to the channels listed in the parameter channelIndexList. List
    must have the same length as the parameter channelIndexList.


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
    :param channelModesList: List of channel modes which are set to the output for the given channels (NORM = normal, SING = single shot, BURST = burst, DCYC = duty cycle).
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


Changelog
==========

