<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>AerotechEnsemble</name>
    <message>
        <location filename="../aerotechEnsemble.cpp" line="+145"/>
        <source>asynchronous move (1), synchronous (0) [default]</source>
        <translation>Asynchrone Fahrt (1), synchrone Fahrt (0) [Voreinstellung]</translation>
    </message>
    <message>
        <location line="+6"/>
        <source>speed of every axis</source>
        <translation>Geschwindigkeit jeder Achse</translation>
    </message>
    <message>
        <location line="+448"/>
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
        <location line="-805"/>
        <source>Ensemble error %i: %s</source>
        <translation>Ensemble-Fehler %i: %s</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Unknown ensemble error since the error message was too long</source>
        <translation>Unbekannter Ensemble-Fehler. Die Fehlermeldung war zu lang</translation>
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
        <location line="+112"/>
        <source>Not all desired axes are connected to the controller (desired: %i, available: %i)</source>
        <translation>Nicht alle angeforderten Achsen sind mit dem Controller verbunden (angefordert: %i, verfügbar: %i)</translation>
    </message>
    <message>
        <location line="+179"/>
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
        <source>Aerotech Ensemble Handle is NULL</source>
        <translation>Aerotech Ensemble-Handle ist NULL</translation>
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
        <translation type="unfinished">Der Absolutwert der Differenz zwischen dem Positionsbefehl und der Positionsrückmeldung hat den durch den Parameter PositionErrorThreshold angegebenen Schwellenwert überschritten.</translation>
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
        <translation type="unfinished">Der Absolutwert der Differenz zwischen dem Geschwindigkeitsbefehl und der Geschwindigkeitsrückmeldung hat den durch den Parameter VelocityErrorThreshold angegebenen Schwellenwert überschritten.</translation>
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
        <translation type="unfinished">Der Controller hat die Verbindung zu einem oder mehreren Netzlaufwerken verloren.</translation>
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
    <name>AerotechEnsembleInterface</name>
    <message>
        <location line="-1207"/>
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
    <name>DockWidgetAerotechEnsemble</name>
    <message>
        <location filename="../dockWidgetAerotechEnsemble.ui" line="+20"/>
        <source>Form</source>
        <translation></translation>
    </message>
    <message>
        <location line="+24"/>
        <source>General Information</source>
        <translation>Allgemeine Informationen</translation>
    </message>
    <message>
        <location line="+24"/>
        <source>ID:</source>
        <translation></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>[ID]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Axis:</source>
        <translation>Achsen:</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>[Axis]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>Relative Positioning</source>
        <translation>Relative Positionierung</translation>
    </message>
    <message>
        <location line="+18"/>
        <source>Step Size</source>
        <translation>Schrittweite</translation>
    </message>
    <message>
        <location line="+102"/>
        <source>Actual</source>
        <translation>Aktuell</translation>
    </message>
    <message>
        <location line="-38"/>
        <source>Target</source>
        <translation>Ziel</translation>
    </message>
    <message>
        <location line="+104"/>
        <source>Start</source>
        <translation></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Stop</source>
        <translation>Abbruch</translation>
    </message>
    <message>
        <location line="-162"/>
        <location line="+70"/>
        <location line="+35"/>
        <location filename="../dockWidgetAerotechEnsemble.cpp" line="+124"/>
        <location line="+13"/>
        <source>mm</source>
        <translation></translation>
    </message>
    <message>
        <location line="-74"/>
        <source>Positioning</source>
        <translation>Positionierung</translation>
    </message>
    <message>
        <location line="+96"/>
        <source>Name 1</source>
        <translation></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>+</source>
        <translation></translation>
    </message>
    <message>
        <location line="-7"/>
        <source>-</source>
        <translation></translation>
    </message>
    <message>
        <location line="+45"/>
        <source>Refresh</source>
        <translation>Aktualisierung</translation>
    </message>
    <message>
        <location filename="../dockWidgetAerotechEnsemble.cpp" line="+172"/>
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
        <location filename="../aerotechEnsemble.cpp" line="-26"/>
        <source>Plugin for the Ensemble-controller of Aerotech</source>
        <translation>Plugin des Ensemble-Controllers von Aerotech</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>This plugin allows communicating with controllers of type Ensemble (4.xx Version) of company Aerotech.

If no parameters are given, the plugin connects to all available axes of the controller. Else you can provide a list of axis numbers (0..9) that should be connected. The first axis of this list then gets the axis ID 0, the second the axis ID 1 and so on.
For running this plugin you need an installed Ensemble driver and a connected device.

This plugin comes with version 4.06 of the Ensemble driver. You can change them by newer libraries (Version 4.XX). The manual of Ensemble allows redistributing the Ensemble libraries without having the end-user install the Ensemble software. For further information about license information of Aerotech see their documentation.

For loading the Ensemble library you need the Visual C++ 2008 SP1 Redistributable Package provided by Microsoft (see Ensemble Programming Help).</source>
        <translation type="unfinished">Dieses Plugin erlaubt die Kommunikation mit Controllern des Typs Ensemble (Version 4.xx) der Firma Aerotch.

Werden bei der Initialisierung keine Parameter angegeben, verbindet das Plugin alle verfügbaren Achsen des Controllers. Ansonsten kann eine Liste der Achsennummern (0..9), die verbunden werden sollen, angegeben werden. Die erste Achse der Liste erhält dann die Achsen-ID 0, die Zweite die ID 1 usw.
Um das Plugin zu starten wird ein installierter Ensemble-Treiber und ein verbundenes Gerät benötigt.

Das Plugin wird mit der Ensemble-Treiberversion 4.06 ausgeliefert. Dieser kann mit einer neuern Bibliothek (Version 4.XX) ausgetauscht werden. Es ist erlaub eine neue Bibliotheken ohne die Endbenutzer-Software von Ensemble zu installieren. Für weitere Informationen bezüglich der Lizent von Aerotech siehe deren Dokumentation.

Zum Starten der Ensemble-Bibliothek wird das Visual C++ 2008 SP1 Redistributable-Package von Microsoft benötigt (siehe  Ensemble Programmierhilfe).</translation>
    </message>
    <message>
        <source>N.A.</source>
        <translation type="vanished">K.A.</translation>
    </message>
</context>
<context>
    <name>dialogAerotechEnsemble</name>
    <message>
        <location filename="../dialogAerotechEnsemble.ui" line="+20"/>
        <source>Dialog</source>
        <translation></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Properties</source>
        <translation>Eigenschaften</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Speed X</source>
        <translation></translation>
    </message>
    <message>
        <location line="+7"/>
        <source> mm/s</source>
        <translation></translation>
    </message>
    <message>
        <location line="+21"/>
        <source>Axis</source>
        <translation>Achsen</translation>
    </message>
    <message>
        <location line="+11"/>
        <source>Enable A</source>
        <translation></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>Enable X</source>
        <translation></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>Calibrate</source>
        <translation>Kalibrieren</translation>
    </message>
    <message>
        <location line="+17"/>
        <source>OK</source>
        <translation></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>Cancel</source>
        <translation>Abbrechen</translation>
    </message>
    <message>
        <location filename="../dialogAerotechEnsemble.cpp" line="+57"/>
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
    <name>ito::AddInBase</name>
    <message>
        <source>function execution unused in this plugin</source>
        <translation type="obsolete">Diese Funktion wurde im Plugin nicht benutzt</translation>
    </message>
</context>
<context>
    <name>ito::AddInDataIO</name>
    <message>
        <source>the object could not been removed from the listener list</source>
        <translation type="obsolete">Das Objekt konnte nicht aus der Listener-Liste gelöscht werden</translation>
    </message>
    <message>
        <source>not implemented</source>
        <translation type="obsolete">nicht implementiert</translation>
    </message>
</context>
</TS>
