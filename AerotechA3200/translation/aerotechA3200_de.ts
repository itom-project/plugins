<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.0" language="de">
<context>
    <name>AerotechA3200</name>
    <message>
        <location filename="../aerotechA3200.cpp" line="+125"/>
        <source>asynchronous move (1), synchronous (0) [default]</source>
        <translation>Asynchrone Fahrt (1), synchrone Fahrt (0) [Voreinstellung]</translation>
    </message>
    <message>
        <location line="+6"/>
        <source>speed of every axis</source>
        <translation>Geschwindigkeit jeder Achse</translation>
    </message>
    <message>
        <location line="+450"/>
        <source>any axis is moving. Parameters cannot be set</source>
        <translation>Eine Achse verfährt gerade. Die Einstellungen konnten nicht geändert werden</translation>
    </message>
    <message>
        <source>not supported yet</source>
        <translation type="obsolete">Momentan nicht unterstützt</translation>
    </message>
    <message>
        <location line="+184"/>
        <source>motor is running. Further action is not possible</source>
        <translation>Motor läuft gerade. Ausführung nicht möglich</translation>
    </message>
    <message>
        <location line="+68"/>
        <source>axis index is out of bound</source>
        <translation>Achsenindex liegt außerhalb des zulässigen Bereichs</translation>
    </message>
    <message>
        <location line="+28"/>
        <source>at least one axis index is out of bound</source>
        <translation>Ein Achsenindex liegt außerhalb des zulässigen Bereichs</translation>
    </message>
    <message>
        <location line="-186"/>
        <location line="+216"/>
        <location line="+88"/>
        <source>Any motor axis is moving. The motor is locked.</source>
        <translation>Ein Motor verfährt gerade. Der Motor ist geblockt.</translation>
    </message>
    <message>
        <location line="-798"/>
        <source>A3200 error %i: %s</source>
        <translation>A3200-Fehler %i: %s</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Unknown A3200 error since the error message was too long</source>
        <translation>Unbekannter A3200-Fehler. Die Fehlermeldung war zu lang</translation>
    </message>
    <message>
        <location line="+46"/>
        <location line="+55"/>
        <source>The axis number %i is not supported. Allowed range [0, 9]</source>
        <translation>Achsennummer %i wird nicht unterstützt. Erlaubter Bereich [0, 9]</translation>
    </message>
    <message>
        <location line="-37"/>
        <source>axis index %i is out of boundary [0, %i]</source>
        <translation>Achsenindex %i liegt außerhalb des aktuellen Bereichs [0, %i]</translation>
    </message>
    <message>
        <location line="+79"/>
        <source>Please make sure that only one controller is configured and connected</source>
        <translation>Bitte stellen Sie sicher, dass nur ein Controller angeschlossen und konfiguriert wurde</translation>
    </message>
    <message>
        <location line="+107"/>
        <source>Not all desired axes are connected to the controller (desired: %i, available: %i)</source>
        <translation>Nicht alle angeforderten Achsen sind mit dem Controller verbunden (angefordert: %i, verfügbar: %i)</translation>
    </message>
    <message>
        <location line="+177"/>
        <source>given index is out of boundary</source>
        <translation>Übergebener Index liegt außerhalb des aktuellen Bereichs</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>given value must be a double if an index is given.</source>
        <translation>Wenn ein Index übergeben wurde, muss der Wert vom Typ Double sein.</translation>
    </message>
    <message>
        <location line="+11"/>
        <source>given value must be a double array</source>
        <translation>Übergebener Wert muss vom Typ Double-Array sein</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>length of given double array must correspond to number of axes</source>
        <translation>Länge des übergebenen Double-Arrays muss mit der Anzahl der Achsen übereinstimmen</translation>
    </message>
    <message>
        <location line="+53"/>
        <location line="+216"/>
        <location line="+88"/>
        <source>Aerotech A3200 Handle is NULL</source>
        <translation>Aerotech A3200-Handle ist NULL</translation>
    </message>
    <message>
        <location line="-215"/>
        <source>axis %i is not enabled</source>
        <translation>Achse %i wurde nicht aktiviert</translation>
    </message>
    <message>
        <location line="+312"/>
        <source>interrupt occurred</source>
        <translation>Unterbrechung aufgetreten</translation>
    </message>
    <message>
        <location line="+42"/>
        <source>timeout occurred</source>
        <translation>Zeitüberschreitung aufgetreten</translation>
    </message>
    <message>
        <location line="+98"/>
        <source>The absolute value of the difference between the position command and the position feedback exceeded the threshold specified by the PositionErrorThreshold parameter.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The average motor current exceeded the threshold specified by the AverageCurrentThreshold and AverageCurrentTime parameters.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The axis encountered the clockwise (positive) end-of-travel limit switch.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The axis encountered the counter-clockwise (negative) end-of-travel limit switch.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The axis was commanded to move beyond the position specified by the SoftwareLimitHigh parameter.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The axis was commanded to move beyond the position specified by the SoftwareLimitLow parameter.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The amplifier for this axis exceeded its maximum current rating or experienced an internal error.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The drive detected a problem with the feedback device specified by the PositionFeedbackType and PositionFeedbackChannel parameters.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The drive detected a problem with the feedback device specified by the VelocityFeedbackType and VelocityFeedbackChannel parameters.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The drive detected an invalid state (all high or all low) for the Hall-effect sensor inputs on this axis.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The commanded velocity is more than the velocity command threshold. Before the axis is homed, this threshold is specified by the VelocityCommandThresholdBeforeHome parameter. After the axis is homed, this threshold is specified by the VelocityCommandThreshold parameter.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The emergency stop sense input, specified by the ESTOPFaultInput parameter, was triggered.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The absolute value of the difference between the velocity command and the velocity feedback exceeded the threshold specified by the VelocityErrorThreshold parameter.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The external fault input, specified by the ExternalFaultAnalogInput or ExternalFaultDigitalInput parameters, was triggered.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The motor thermistor input was triggered, which indicates that the motor exceeded its maximum recommended operating temperature.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The amplifier exceeded its maximum recommended operating temperature.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The encoder fault input on the motor feedback connector was triggered.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>One or more of the drives on the network lost communications with the controller.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The difference between the position feedback and the scaled (adjusted by GainKv) velocity feedback exceeds the threshold specified by the PositionErrorThreshold parameter.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The distance that the axis moved while searching for the marker exceeded the threshold specified by the MarkerSearchThreshold parameter in axis %1 (%2).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Unknown axisFault value: %1 in axis %2 (%3)</source>
        <translation>Unbekannter Fehlerwert %1 bei Achse %2 (%3)</translation>
    </message>
</context>
<context>
    <name>AerotechA3200Interface</name>
    <message>
        <location line="-1213"/>
        <source>list of axes IDs that are enabled (0..9). The first ID then obtains index 0, the second ID index 1... [default: empty list, all available axes are connected]</source>
        <translation>Liste der aktiven Achsen-IDs (0..9). Die erste ID entspricht dem Index 0, die zweite ID Index 1,... [Voreinstellung: Leere Liste, alle möglichen Achsen werden verbunden]</translation>
    </message>
</context>
<context>
    <name>DialogUSBMotion3XIII</name>
    <message>
        <source>General Information</source>
        <translation type="obsolete">Allgemeine Informationen</translation>
    </message>
</context>
<context>
    <name>DockWidgetAerotechA3200</name>
    <message>
        <location filename="../dockWidgetAerotechA3200.ui"/>
        <source>Form</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>General Information</source>
        <translation>Allgemeine Informationen</translation>
    </message>
    <message>
        <location/>
        <source>ID:</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>[ID]</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Axis:</source>
        <translation>Achsen:</translation>
    </message>
    <message>
        <location/>
        <source>[Axis]</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Relative Positioning</source>
        <translation>Relative Positionierung</translation>
    </message>
    <message>
        <location/>
        <source>Step Size</source>
        <translation>Schrittweite</translation>
    </message>
    <message>
        <location/>
        <source>Actual</source>
        <translation>Aktuell</translation>
    </message>
    <message>
        <location/>
        <source>Target</source>
        <translation>Ziel</translation>
    </message>
    <message>
        <location/>
        <source>Start</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Stop</source>
        <translation>Abbruch</translation>
    </message>
    <message>
        <location/>
        <location filename="../dockWidgetAerotechA3200.cpp" line="+104"/>
        <location line="+13"/>
        <source>mm</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Positioning</source>
        <translation>Positionierung</translation>
    </message>
    <message>
        <location/>
        <source>Name 1</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>+</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>-</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Refresh</source>
        <translation>Aktualisierung</translation>
    </message>
    <message>
        <location filename="../dockWidgetAerotechA3200.cpp" line="+173"/>
        <source>unknown error or warning when moving any axis</source>
        <translation>Unbekannter Fehler/Warnung beim Verfahren einer Achse</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Timeout</source>
        <translation>Zeitüberschreitung</translation>
    </message>
    <message>
        <location line="+0"/>
        <source>timeout when moving any axis</source>
        <translation>Zeitüberschreitung beim Verfahren einer Achse</translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location filename="../aerotechA3200.cpp" line="-12"/>
        <source>Plugin for the A3200-controller of Aerotech</source>
        <translation>Plugin des A3200-Controllers von Aerotech</translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Licensed under LGPL, The Aerotech A3200 library belongs to Aerotech under their specific license.</source>
        <translation></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>N.A.</source>
        <translation>K.A.</translation>
    </message>
</context>
<context>
    <name>dialogAerotechA3200</name>
    <message>
        <location filename="../dialogAerotechA3200.ui"/>
        <source>Dialog</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Properties</source>
        <translation>Eigenschaften</translation>
    </message>
    <message>
        <location/>
        <source>Speed X</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source> mm/s</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Axis</source>
        <translation>Achsen</translation>
    </message>
    <message>
        <location/>
        <source>Enable A</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Enable X</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Calibrate</source>
        <translation>Kalibrieren</translation>
    </message>
    <message>
        <location/>
        <source>OK</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Cancel</source>
        <translation>Abbrechen</translation>
    </message>
    <message>
        <location filename="../dialogAerotechA3200.cpp" line="+34"/>
        <source>Speed</source>
        <translation>Geschwindigkeit</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>mm/s</source>
        <translation></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Enable</source>
        <translation>Aktivierung</translation>
    </message>
    <message>
        <location line="+32"/>
        <source>Configuration Dialog</source>
        <translation>Konfigurationsdialog</translation>
    </message>
    <message>
        <location line="+67"/>
        <location line="+4"/>
        <source>Calibration (Homing)</source>
        <translation>Kalibrierung (Nullstellung)</translation>
    </message>
    <message>
        <location line="-4"/>
        <source>Homing successfully executed.</source>
        <translation>Kalibrierung erfolgreich abgeschlossen.</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Timeout while calibrating axes.</source>
        <translation>Zeitüberschreitung beim Achsenkalibrieren.</translation>
    </message>
</context>
<context>
    <name>ito::AddInActuator</name>
    <message>
        <location filename="../../../../build/itom/SDK/include/common/addInInterface.cpp" line="+687"/>
        <source>Constructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Destructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInAlgo</name>
    <message>
        <location line="+92"/>
        <source>Constructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../../../../build/itom/SDK/include/common/addInInterface.h" line="+986"/>
        <source>uninitialized vector for mandatory parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>uninitialized vector for optional parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>uninitialized vector for output parameters!</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInBase</name>
    <message>
        <location filename="../../../../build/itom/SDK/include/common/addInInterface.cpp" line="-577"/>
        <source>function execution unused in this plugin</source>
        <translation>Diese Funktion wurde im Plugin nicht benutzt</translation>
    </message>
    <message>
        <location line="+29"/>
        <source>Toolbox</source>
        <translation></translation>
    </message>
    <message>
        <location line="+147"/>
        <source>Your plugin is supposed to have a configuration dialog, but you did not implement the showConfDialog-method</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInDataIO</name>
    <message>
        <location line="+13"/>
        <source>Constructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Destructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>listener does not have a slot </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>this object already has been registered as listener</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <source>timer could not be set</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+25"/>
        <source>the object could not been removed from the listener list</source>
        <translation type="unfinished">Das Objekt konnte nicht aus der Listener-Liste gelöscht werden</translation>
    </message>
    <message>
        <location line="+94"/>
        <location line="+20"/>
        <location line="+18"/>
        <location line="+18"/>
        <location line="+18"/>
        <location line="+18"/>
        <location line="+18"/>
        <source>not implemented</source>
        <translation type="unfinished">nicht implementiert</translation>
    </message>
</context>
</TS>
