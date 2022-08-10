<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DialogThorlabsBP</name>
    <message>
        <source>Thorlabs BP</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Information</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Device:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>[unknown]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Serial:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>General settings</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Timeout:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source> s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>asynchronous mode</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Axis settings</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Maximum output voltage:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>75 V</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>100 V</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>150 V</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Maximum travel range:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>0 µm</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Zero this axis (requires few seconds)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Axis enabled</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Closed loop</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Configuration Dialog</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>DockWidgetThorlabsBP</name>
    <message>
        <source>Thorlabs ISM</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>General Information</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Device:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>[Controller]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Serial:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>[unknown]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Axis Control</source>
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
        <source>ThorlabsBP</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>ThorlabsBP is an acutator plugin to control the following integrated devices from Thorlabs: 

* Benchtop Piezo (1 channel) 
* Benchtop Piezo (3 channels) 

It requires the new Kinesis driver package from Thorlabs and implements the interface Thorlabs.MotionControl.Benchtop.Piezo.

Please install the Kinesis driver package in advance with the same bit-version (32/64bit) than itom. 

This plugin has been tested with the Benchtop Piezo with 1 and 3 channels. 

The position values are always in mm if the corresponding axis is in closed-loop mode and if a strain gauge feedback is connected. Else the values are always in volts.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ThorlabsBP</name>
    <message>
        <source>name of plugin</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>number of axes (channels)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Description of the device</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Serial number of the device</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Channel number of each axis.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Timeout for positioning in seconds.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>If 1, the axis is enabled and power is applied to the motor. 0: disabled, the motor can be turned by hand.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>If 0, the axis is not zeroed. 1: zeroed. If the axis is not zeroed, it is possible that position values at the edge of the valid range can not be reached.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>If 0, the axis is not equipped with a strain gauge feedback, else 1.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Open loop (0), closed loop (1)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Maximum travel range for each axis in mm. This requires an actuator with built in position sensing. These values might not be correct if the motor is in open loop mode.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Maximum output voltage (75, 100 or 150 V).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>asychronous (1) or sychronous (0) mode</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Any motor axis is moving. The motor is locked.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>timeout occurred</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ThorlabsBPInterface</name>
    <message>
        <source>Serial number of the device to be loaded, if empty, the first device that can be opened will be opened</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>list of channels to connect to. If an empty list is given, all connected channels are used. The plugin axis indices are then mapped to the channels.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
