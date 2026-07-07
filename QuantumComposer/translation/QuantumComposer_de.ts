<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DialogQuantumComposer</name>
    <message>
        <location filename="../dialogQuantumComposer.cpp" line="52"/>
        <source>Configuration Dialog</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location filename="../quantumComposer.cpp" line="49"/>
        <source>QuantumComposer</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="53"/>
        <source>QuantumComposer is an itom-plugin to communicate with the pulse generator 9520 series. 

This plugin has been developed for the 9520 series via a RS232 interface. So you first have to create an instance of the SerialIO plugin 
which is a mandatory input argument of the QuantumComposer plugin. 
The plugin sets the right RS232 parameter during initialization. 

The default parameters are: 

========== ======================================================
Baud Rate  38400 (default for USB), 115200 (default for RS232)
Data Bits  8
Parity     None
Stop bits  1
endline    \r\n
========== ======================================================

.. warning::

    Disable **Echo** of the system settings!</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QuantumComposer</name>
    <message>
        <location filename="../quantumComposer.cpp" line="133"/>
        <source>Plugin name.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="141"/>
        <source>Manufacturer identification.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="149"/>
        <source>Model identification.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="157"/>
        <source>Serial number.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="165"/>
        <source>Version number.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="174"/>
        <source>Request timeout in ms for the SerialIO interface.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="181"/>
        <source>Mode of the system output. (NORM: normal, SING: single shot, BURS: burst, DCYC: duty cycle).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="197"/>
        <source>Enables (1), disables (0) the output for all channels. Command is the same as pressing the RUN/STOP button.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="208"/>
        <source>Number of pulses to generate in the burst mode.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="216"/>
        <location filename="../quantumComposer.cpp" line="226"/>
        <source>Number of pulses to inhibit output during the off cycle of the Duty Cycle mode.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="251"/>
        <source>Gate logic level (LOW, HIGH).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="262"/>
        <source>Gate threshold in units of V with a range of 0.20V to 15.0V.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="269"/>
        <source>Trigger mode (DIS: disabled, TRIG: triggered, enabled).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="279"/>
        <source>Trigger edge to use as the trigger signal (RIS: rising, FALL: falling).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="292"/>
        <source>Trigger threshold in units of V with a range of 0.20V to 15.0V.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="300"/>
        <source>Source for the internal rate generator. System clock or external source ranging from 10MHz to 100MHz (SYS, EXT10, EXT20, EXT25, EXT40, EXT50, EXT80, EXT100).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="319"/>
        <source>External clock output. T0 pulse or 50% duty cycle TTL output from 10MHz to 100MHz (T0, 10, 11, 12, 14, 16, 20, 25, 33, 50, 100).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="342"/>
        <source>T0 period in units of seconds (100ns - 5000s).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="350"/>
        <source>Enables (1), disables(0) the counter function.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="358"/>
        <source>Number of counts.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="373"/>
        <source>List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="235"/>
        <source>Global gate mode of the system output. (DIS: disabled, PULS: pulse inhibit, OUTP: output inhibit, CHAN: channel).cycle).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="394"/>
        <source>List of states to enable/disable channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="406"/>
        <source>Enables/Disables the output state of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="429"/>
        <source>List of widths to set to the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="441"/>
        <source>Set the pulse width of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="465"/>
        <source>List of delays to set to the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="477"/>
        <source>Set the pulse delays of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="491"/>
        <source>List of channels to sync with the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="504"/>
        <source>Set the sync channels of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="518"/>
        <source>List of timers which are enabled as output for the given channel. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="531"/>
        <source>Set which timers are enabled as output for the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="543"/>
        <source>List of polarities which are set to the output for the given channels (NORM = normal, COMP = complement, INV = inverted).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="566"/>
        <source>Set the polarity of the pulse for the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="578"/>
        <source>List of output modes which are set to the output for the given channels (TTL = TTL/CMOS, ADJ = adjustable).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="598"/>
        <source>Set the output amplitude mode of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="611"/>
        <source>List of amplitude levels to set to the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="623"/>
        <source>Set the adjustable amplitude of channel output level of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="637"/>
        <source>List of channel modes which are set to the output for the given channels (NORM = normal, SING = single shot, BURS = burst, DCYC = duty cycle).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="663"/>
        <source>Set the channel mode of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="677"/>
        <source>List of burst counter values for the given channels (1 - 9999999). List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="689"/>
        <source>Set the channel burst counter for the burst mode of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="705"/>
        <source>List of pulse counter values to generate during the ON cycle of the duty cycle mode for the given channels (1 - 9999999). List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="719"/>
        <source>Set the channel pulse counter during the ON cycles for the duty cycle modes of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="736"/>
        <source>List of pulse counter values to inhibit during the OFF cycle of the duty cycle mode for the given channels (1 - 9999999). List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="750"/>
        <source>Set the channel pulse counter during the OFF cycles for the duty cycle modes of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="767"/>
        <source>List of pulse counter values to wait until enabling output of the duty cycle mode for the given channels (0 - 9999999). List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="781"/>
        <source>Set the channel pulse counter to wait until enabling output for the duty cycle modes of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="797"/>
        <source>List of channel gate modes (DIS = disable, PULS = pulse inhibit, OUTP = output inhibit).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="819"/>
        <source>Set the channel gates mode of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="831"/>
        <source>List of channel gate logic level (LOW, HIGH).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="848"/>
        <source>Set the channel gates logic level of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="894"/>
        <source>Input parameter is not a dataIO instance of the SerialIO Plugin!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="956"/>
        <source>Answer of the identification request is not valid!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="1544"/>
        <source>could not read endline parameter from serial port</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="1577"/>
        <source>timeout during read string.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="1614"/>
        <source>Error during SendQuestionWithAnswerDouble, converting %1 to double value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="1643"/>
        <source>Error during SendQuestionWithAnswerInteger, converting %1 to double value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="1886"/>
        <source>The lengths of the channel list (%1) and states list (%2) must be the same.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../quantumComposer.cpp" line="1923"/>
        <location filename="../quantumComposer.cpp" line="1958"/>
        <location filename="../quantumComposer.cpp" line="1993"/>
        <location filename="../quantumComposer.cpp" line="2063"/>
        <location filename="../quantumComposer.cpp" line="2099"/>
        <location filename="../quantumComposer.cpp" line="2132"/>
        <location filename="../quantumComposer.cpp" line="2167"/>
        <location filename="../quantumComposer.cpp" line="2202"/>
        <location filename="../quantumComposer.cpp" line="2238"/>
        <location filename="../quantumComposer.cpp" line="2273"/>
        <location filename="../quantumComposer.cpp" line="2309"/>
        <location filename="../quantumComposer.cpp" line="2344"/>
        <location filename="../quantumComposer.cpp" line="2380"/>
        <location filename="../quantumComposer.cpp" line="2416"/>
        <source>The lengths of the channel list (%1) and widths list (%2) must be the same.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QuantumComposerInterface</name>
    <message>
        <location filename="../quantumComposer.cpp" line="85"/>
        <source>An opened serial port.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
