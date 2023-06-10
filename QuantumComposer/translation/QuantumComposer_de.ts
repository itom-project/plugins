<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DialogQuantumComposer</name>
    <message>
        <source>Configuration Dialog</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <source>licensed under LGPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>QuantumComposer</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
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
        <source>Plugin name.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Manufacturer identification.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Model identification.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Serial number.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Version number.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Request timeout in ms for the SerialIO interface.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Mode of the system output. (NORM: normal, SING: single shot, BURS: burst, DCYC: duty cycle).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Enables (1), disables (0) the output for all channels. Command is the same as pressing the RUN/STOP button.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Number of pulses to generate in the burst mode.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Number of pulses to inhibit output during the off cycle of the Duty Cycle mode.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Global gate mode of the system output. (DIS: diabled, PULS: pulse inhibit, OUTP: output inhibit, CHAN: channel).cycle).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Gate logic level (LOW, HIGH).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Gate threshold in units of V with a range of 0.20V to 15.0V.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Trigger mode (DIS: disabled, TRIG: triggered, enabled).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Trigger edge to use as the trigger signal (RIS: rising, FALL: falling).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Trigger threshold in units of V with a range of 0.20V to 15.0V.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Source for the internal rate generator. System clock or external source ranging from 10MHz to 100MHz (SYS, EXT10, EXT20, EXT25, EXT40, EXT50, EXT80, EXT100).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>External clock output. T0 pulse or 50% duty cycle TTL output from 10MHz to 100MHz (T0, 10, 11, 12, 14, 16, 20, 25, 33, 50, 100).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>T0 period in units of seconds (100ns - 5000s).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Enables (1), disables(0) the counter function.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Number of counts.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of channel indices which output should be enabled/disabled (ChA = 1, ChB = 2, ...).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of states to enalbe/disable channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Enables/Disables the output state of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of widths to set to the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the pulse width of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of delays to set to the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the pulse delays of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of channels to sync with the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the sync channels of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of timers which are enabled as output for the given channel. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set which timers are enabled as output for the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of polarities which are set to the output for the given channels (NORM = normal, COMP = complement, INV = inverted).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the polarity of the pulse for the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of output modes which are set to the output for the given channels (TTL = TTL/CMOS, ADJ = adjustable).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the output amplitude mode of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of amplitude levels to set to the channels listed in the parameter channelIndexList. List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the adjustable amplitude of channel output level of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of channel modes which are set to the output for the given channels (NORM = normal, SING = single shot, BURS = burst, DCYC = duty cycle).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the channel mode of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of burst counter values for the given channels (1 - 9999999). List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the channel burst counter for the burst mode of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of pulse counter values to generate during the ON cycle of the duty cycle mode for the given channels (1 - 9999999). List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the channel pulse counter during the ON cycles for the duty cycle modes of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of pulse counter values to inhibit during the OFF cycle of the duty cycle mode for the given channels (1 - 9999999). List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the channel pulse counter during the OFF cycles for the duty cycle modes of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of pulse counter values to wait until enabling output of the duty cycle mode for the given channels (0 - 9999999). List must have the same length as the parameter channelIndexList.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the channel pulse counter to wait until enabling output for the duty cycle modes of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of channel gate modes (DIS = disable, PULS = pulse inhibit, OUTP = output inhibit).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the channel gates mode of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>List of channel gate logic level (LOW, HIGH).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Set the channel gates logic level of the given channels.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Input parameter is not a dataIO instance of ther SerialIO Plugin!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Answer of the identification request is not valid!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>could not read endline parameter from serial port</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>timeout during read string.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Error during SendQuestionWithAnswerDouble, converting %1 to double value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Error during SendQuestionWithAnswerInteger, converting %1 to double value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The lengths of the channel list (%1) and states list (%2) must be the same.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The lengths of the channel list (%1) and widths list (%2) must be the same.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QuantumComposerInterface</name>
    <message>
        <source>An opened serial port.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
