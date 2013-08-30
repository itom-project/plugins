<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.0" language="de">
<context>
    <name>DockWidgetSerialIO</name>
    <message>
        <location filename="../dockWidgetSerialIO.ui"/>
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
        <source>SerialIO Log</source>
        <translation>SerialIO-Protokoll</translation>
    </message>
    <message>
        <location/>
        <source>Decimal</source>
        <translation>Dezimal</translation>
    </message>
    <message>
        <location/>
        <source>Hexadecimal</source>
        <translation>Hexadezimal</translation>
    </message>
    <message>
        <location/>
        <source>Binary</source>
        <translation>Binär</translation>
    </message>
    <message>
        <location/>
        <source>Clear</source>
        <translation>Löschen</translation>
    </message>
    <message>
        <location/>
        <source>ASCII</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>ignor empty messages</source>
        <translation>Leere Nachrichten ignorieren</translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location filename="../SerialIO.cpp" line="+194"/>
        <location line="+174"/>
        <source>invalid baud rate</source>
        <translation>Ungültige Baudrate</translation>
    </message>
    <message>
        <location line="-151"/>
        <location line="+170"/>
        <source>invalid number of bits</source>
        <translation>Ungültige Anzahl Datenbits</translation>
    </message>
    <message>
        <location line="-157"/>
        <location line="+166"/>
        <source>invalid number of stopbits</source>
        <translation>Ungültige Anzahl Stopbits</translation>
    </message>
    <message>
        <location line="-146"/>
        <location line="+159"/>
        <source>invalid parity</source>
        <translation>Ungültige Parität</translation>
    </message>
    <message>
        <location line="+68"/>
        <source>error setting parameters</source>
        <translation>Fehler beim Setzten der Parameter</translation>
    </message>
    <message>
        <location line="+18"/>
        <source>error setting timeout</source>
        <translation>Fehler beim Setzten des Timeouts</translation>
    </message>
    <message>
        <location line="+31"/>
        <source>invalid endline character</source>
        <translation>Ungültiges Terminierungszeichen</translation>
    </message>
    <message>
        <location line="+69"/>
        <location line="+16"/>
        <source>could not open device</source>
        <translation>Kann Gerät nicht öffnen</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>unknown error opening com port</source>
        <translation>Unbekannter Fehler beim Öffnen des COM-Ports</translation>
    </message>
    <message>
        <location line="+63"/>
        <location line="+35"/>
        <location line="+57"/>
        <location line="+10"/>
        <location line="+47"/>
        <location line="+26"/>
        <location line="+48"/>
        <location line="+18"/>
        <source>com port not open</source>
        <translation>COM-Port nicht geöffnet</translation>
    </message>
    <message>
        <location line="-192"/>
        <location line="+12"/>
        <source>error reading from com port</source>
        <translation>Fehler beim Lesen vom COM-Port</translation>
    </message>
    <message>
        <location line="+36"/>
        <location line="+13"/>
        <location line="+4"/>
        <location line="+45"/>
        <location line="+10"/>
        <location line="+17"/>
        <location line="+4"/>
        <location line="+8"/>
        <location line="+4"/>
        <source>error writing to com port</source>
        <translation>Fehler beim Schreiben auf den COM-Port</translation>
    </message>
    <message>
        <location line="+29"/>
        <location line="+19"/>
        <source>invalid number of buffer type (0: input, 1: output)</source>
        <translation>Ungültige Nummer des Puffertyps (0: Input, 1: Output)</translation>
    </message>
    <message>
        <location line="-14"/>
        <location line="+20"/>
        <source>Unable to clear buffer</source>
        <translation>Puffer löschen war nicht erfolgreich</translation>
    </message>
    <message>
        <location line="+58"/>
        <source>licensed under LGPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>N.A.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Serialport-Interface</source>
        <translation type="obsolete">Serialport-Schnittstelle</translation>
    </message>
    <message>
        <source>The SerialIO is a itom-PlugIn to give a direct access to serial ports.
It is used by different plugins for communication, (e.g. &apos;MFUCtrl&apos;, &apos;UhlActuator&apos;).
The plugin is implemented for Windows or Linux. Parameters and initialization differs according to operation system.</source>
        <translation type="obsolete">SerialIO ist ein ITOM-Plugin um einen direkten Zugriff auf die seriellen Ports zu bekommen.
Es wird in unterschiedlichen Plugins zur Kommunikation benutzt (z. B. &apos;MFUCtrl&apos;, &apos;Uhltisch&apos;).
Das Plugin wurde für Windows und Linux implementiert. Parameter und Initialisierung sind betriebssystemabhängig.</translation>
    </message>
</context>
<context>
    <name>SerialIO</name>
    <message>
        <location line="+56"/>
        <source>Serial port number of this device</source>
        <translation>Serielle Portnummer dieses Geräts</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Number of bits to be written in line</source>
        <translation>Anzahl der pro Zeile geschriebenen Bits</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Stop bits after every n bits</source>
        <translation>Stopbits nach jedem n-ten Bit</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Toggle parity check</source>
        <translation>Schalter Paritätsprüfung</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Bitmask for flow control as integer</source>
        <translation>Bitmaske für Flusskontrolle als Ganzzahl</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Endline character, will be added automatically during setVal</source>
        <translation>Terminierung, wird bei setVal() automatisch hinzugefügt</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Toggle: write output buffer as block @ once or single characters</source>
        <translation>Schalter: Schreibt Puffer als Block oder als einzelne Zeichen</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Timeout for reading commands in [s]</source>
        <translation>Timeout für Lesebefehle in [s]</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>If true, all out and inputs are written to dockingWidget</source>
        <translation>Wenn aktiviert, wird die Kommunikation im dockingWidget protokolliert</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Clears the input buffer of serial port</source>
        <translation>Löscht den Input-Puffer der seriellen Schnittstelle</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Clears the output buffer of serial port</source>
        <translation>Löscht den Output-Puffer der seriellen Schnittstelle</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Clears input (0) or output (1) buffer</source>
        <translation>Löscht den Input- (0) oder Output- (1) Puffer</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Clears the input or output buffer of serial port</source>
        <translation>Löscht den Input- oder Output-Puffer der seriellen Schnittstelle</translation>
    </message>
    <message>
        <location line="+50"/>
        <source>name of requested parameter is empty.</source>
        <translation>Name des angeforderten Parameters ist leer.</translation>
    </message>
    <message>
        <location line="+11"/>
        <source>parameter not found in m_params.</source>
        <translation>Parameter nicht in m_params gefunden.</translation>
    </message>
    <message>
        <location line="+22"/>
        <source>name of given parameter is empty.</source>
        <translation>Name des Parameters ist leer.</translation>
    </message>
    <message>
        <location line="+18"/>
        <source>Parameter is read only, input ignored</source>
        <translation>Parameter ist schreibgeschützt, Eingabe wurde ignoriert</translation>
    </message>
    <message>
        <location line="+8"/>
        <source>New value is larger than parameter range, input ignored</source>
        <translation>Wert ist größer als der zugelassene Bereich, Eingabe wurde ignoriert</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>New value is smaller than parameter range, input ignored</source>
        <translation>Wert ist kleiner als der zugelassene Bereich, Eingabe wurde ignoriert</translation>
    </message>
    <message>
        <location line="+14"/>
        <source>Parameter type conflict</source>
        <translation>Konflikt mit Parametertyp</translation>
    </message>
    <message>
        <location line="+17"/>
        <source>Parameter not found</source>
        <translation>Parameter wurde nicht gefunden</translation>
    </message>
    <message>
        <location line="+125"/>
        <source>StartDevice not necessary</source>
        <translation>&quot;StartDevice&quot; nicht nötig</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>StopDevice not necessary</source>
        <translation>&quot;StopDevice&quot; nicht nötig</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>Acquire not necessary</source>
        <translation>&quot;Acquire&quot; nicht nötig</translation>
    </message>
    <message>
        <location line="-327"/>
        <source>Current baudrate in bits/s</source>
        <translation>Aktuelle Baudrate ist bits/s</translation>
    </message>
</context>
<context>
    <name>SerialIOInterface</name>
    <message>
        <source>Serialport-Interface</source>
        <translation type="obsolete">Serielle Schnittstelle</translation>
    </message>
    <message>
        <source>The SerialIO is a itom-PlugIn to give a direct access to serial ports.
It is used by different plugins for communication, (e.g. &apos;MFUCtrl&apos;, &apos;UhlActuator&apos;).
The plugin is implemented for Windows or Linux. Parameters and initialization differs according to operation system.</source>
        <translation type="obsolete">SerialIO ist ein itom-Plugin um einen direkten Zugriff auf die serielle Schnittstelle zu bekommen.
Es wird in unterschiedlichen Plugins zur Kommunikation benutzt (z. B. &apos;MFUCtrl&apos;, &apos;Uhltisch&apos;).
Das Plugin wurde für Windows und Linux implementiert. Parameter und Initialisierung sind betriebssystemabhängig.</translation>
    </message>
    <message>
        <location line="-71"/>
        <source>itom-plugin for a serial port communication</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>The number of the serial port, starting with 1</source>
        <translation>Die Nummer des Ports der seriellen Schnittstelle beginnt mit 1</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>The endline character, which is added automatically after every setVal()</source>
        <translation>Der Terminierungsstring, der automatisch nach jedem setVal() hinzugefügt wird</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Number of bits to be written in line</source>
        <translation>Anzahl der pro Zeile geschriebenen Bits</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Stop bits after every n bits</source>
        <translation>Stopbits nach jedem n-ten Bit</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Toggle parity check</source>
        <translation>Schalter Paritätsprüfung</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Bitmask for flow control</source>
        <translation>Bitmaske für Flusskontrolle</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Toggle: write output buffer as block or single characters</source>
        <translation>Schalter: Schreibt Puffer als Block oder als einzelne Zeichen</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Timeout for reading commands in [s]</source>
        <translation>Timeout für Lesebefehle in [s]</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Initialised &apos;debug&apos;-parameter with given value. If debug-param is true, all out and inputs are written to dockingWidget</source>
        <translation>Schaltet Protokollierung ein und aus. Wurde sie aktiviert, wird die Kommunikation im dockingWidget protokolliert</translation>
    </message>
    <message>
        <location line="-17"/>
        <source>The baudrate of the port</source>
        <translation>Die Baudrate des Ports</translation>
    </message>
</context>
<context>
    <name>dialogSerialIO</name>
    <message>
        <location filename="../dialogSerialIO.cpp" line="+211"/>
        <source>Configuration Dialog</source>
        <translation></translation>
    </message>
    <message>
        <location line="+142"/>
        <source>Error: malformed command string - not send</source>
        <translation>Fehler: Syntaxfehler- nicht gesendet</translation>
    </message>
    <message>
        <location line="+79"/>
        <source>Error: &apos;%1&apos; could not be interpreted - not send</source>
        <translation>Fehler: %1 konnte nicht interpretiert werden - nicht gesendet</translation>
    </message>
    <message>
        <location filename="../dialogSerialIO.ui"/>
        <source>Dialog</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Settings</source>
        <translation>Einstellungen</translation>
    </message>
    <message>
        <location/>
        <source>Basic</source>
        <translation>Grundeinstellungen</translation>
    </message>
    <message>
        <location/>
        <source>1</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>2</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Baud</source>
        <translation>Baudrate</translation>
    </message>
    <message>
        <location/>
        <source>110</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>300</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>600</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>1200</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>2400</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>4800</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>9600</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>19200</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>38400</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>57600</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>115200</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Bits</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>5</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>6</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>7</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>8</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Endline</source>
        <translation>Terminierung</translation>
    </message>
    <message>
        <location/>
        <source>\r</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>\n</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>\r\n</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>&lt;none&gt;</source>
        <translation>&lt;nichts&gt;</translation>
    </message>
    <message>
        <location/>
        <source>Stopbits</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Parity</source>
        <translation>Parität</translation>
    </message>
    <message>
        <location/>
        <source>disable</source>
        <translation>ausschalten</translation>
    </message>
    <message>
        <location/>
        <source>enable</source>
        <translation>einschalten</translation>
    </message>
    <message>
        <location/>
        <source>ms</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Time out</source>
        <translation>Timeout</translation>
    </message>
    <message>
        <location/>
        <source>Single char</source>
        <translation>Einzelzeichen</translation>
    </message>
    <message>
        <location/>
        <source>Debug Mode</source>
        <translation>Protokollierung</translation>
    </message>
    <message>
        <location/>
        <source>Read delay</source>
        <translation>Leseverzögerung</translation>
    </message>
    <message>
        <location/>
        <source>Flow control</source>
        <translation>Flusskontrolle</translation>
    </message>
    <message>
        <location/>
        <source>Xon/Xoff</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>rts</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>cts</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>dtr</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>handshake</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>dsr</source>
        <translation></translation>
    </message>
    <message>
        <source>Set Parameters</source>
        <translation type="obsolete">Parameter speichern</translation>
    </message>
    <message>
        <source>Create Command</source>
        <translation type="obsolete">Python-Code erstellen</translation>
    </message>
    <message>
        <location/>
        <source>Send message</source>
        <translation>Nachricht versenden</translation>
    </message>
    <message>
        <location/>
        <source>Transfer</source>
        <translation>Übertragung</translation>
    </message>
    <message>
        <location/>
        <source>ASCII</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Decimal</source>
        <translation>Dezimal</translation>
    </message>
    <message>
        <location/>
        <source>Hexadecimal</source>
        <translation>Hexadezimal</translation>
    </message>
    <message>
        <location/>
        <source>Binary</source>
        <translation>Binär</translation>
    </message>
    <message>
        <location/>
        <source>Send</source>
        <translation>Senden</translation>
    </message>
    <message>
        <location/>
        <source>Read</source>
        <translation>Lesen</translation>
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
        <source>Create Python Command</source>
        <translation type="obsolete">Python-Code erstellen</translation>
    </message>
    <message>
        <location/>
        <source>Use input below to send characters to the serial port. Characters will be send as their ASCII code from the character written. To directly write ASCII codes use the format $(code) or select Decimal, Hexadecimal or Binary separated by space.</source>
        <translation>Um Nachrichten an die serielle Schnittstelle zu senden, die Eingabezeile benutzen. Zeichen werden als ASCII-Zeichen übermittelt. Um direkt ASCII-Code zu schreiben, das Format $(Code) oder Dezimal, Hexadezimal oder Binär mit Leerzeichen getrennt benutzen.</translation>
    </message>
    <message>
        <location/>
        <source>50</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>75</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>134</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>150</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>200</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>1800</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>230400</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>460800</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>500000</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>576000</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>921600</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>1000000</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>1152000</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>1500000</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>2000000</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>2500000</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>3000000</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>3500000</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>4000000</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Apply</source>
        <translation>Übernehmen</translation>
    </message>
    <message>
        <location/>
        <source>Python Command</source>
        <translation>Python-Befehl</translation>
    </message>
    <message>
        <location/>
        <source>Create</source>
        <translation>Erzeugen</translation>
    </message>
    <message>
        <location/>
        <source>Clear</source>
        <translation>Löschen</translation>
    </message>
</context>
<context>
    <name>ito::AddInActuator</name>
    <message>
        <location filename="../../../../../Build/itom/SDK/include/common/addInInterface.cpp" line="+683"/>
        <source>Constructor must be overwritten</source>
        <translation>Konstruktor muss überschrieben werden</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Destructor must be overwritten</source>
        <translation>Destruktor muss überschrieben werden</translation>
    </message>
</context>
<context>
    <name>ito::AddInAlgo</name>
    <message>
        <location line="+92"/>
        <source>Constructor must be overwritten</source>
        <translation>Konstruktor muss überschrieben werden</translation>
    </message>
    <message>
        <location filename="../../../../../Build/itom/SDK/include/common/addInInterface.h" line="+985"/>
        <source>uninitialized vector for mandatory parameters!</source>
        <translation>Uninitialisierte Vektoren für Pflichtparameter!</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>uninitialized vector for optional parameters!</source>
        <translation>Uninitialisierte Vektoren für optionale Parameter!</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>uninitialized vector for output parameters!</source>
        <translation>Uninitialisierte Vektoren für Ausgabeparameter!</translation>
    </message>
</context>
<context>
    <name>ito::AddInBase</name>
    <message>
        <location filename="../../../../../Build/itom/SDK/include/common/addInInterface.cpp" line="-577"/>
        <source>function execution unused in this plugin</source>
        <translation>Funktion &quot;Execution&quot; in diesem Plugin nicht verwendet</translation>
    </message>
    <message>
        <location line="+29"/>
        <source>Toolbox</source>
        <translation></translation>
    </message>
    <message>
        <location line="+147"/>
        <source>Your plugin is supposed to have a configuration dialog, but you did not implement the showConfDialog-method</source>
        <translation>Ihr Plugin hat vermutlich einen Konfigurationsdialog, doch die showConfDialog-Methode wurde nicht implementiert</translation>
    </message>
</context>
<context>
    <name>ito::AddInDataIO</name>
    <message>
        <location line="+13"/>
        <source>Constructor must be overwritten</source>
        <translation>Konstruktor muss überschrieben werden</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Destructor must be overwritten</source>
        <translation>Destruktor muss überschrieben werden</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>listener does not have a slot </source>
        <translation>Listener hat keinen Slot </translation>
    </message>
    <message>
        <location line="+4"/>
        <source>this object already has been registered as listener</source>
        <translation>Dieses Objekt wurde bereits als Listener registriert</translation>
    </message>
    <message>
        <location line="+12"/>
        <source>timer could not be set</source>
        <translation>Timer konnte nicht gesetzt werden</translation>
    </message>
    <message>
        <location line="+25"/>
        <source>the object could not been removed from the listener list</source>
        <translation>Das Objekt konnte nicht von der Listener-Liste gelöscht werden</translation>
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
        <translation>Nicht implementiert</translation>
    </message>
</context>
</TS>
