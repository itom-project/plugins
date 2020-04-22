<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DockWidgetSerialIO</name>
    <message>
        <location filename="../dockWidgetSerialIO.ui" line="+44"/>
        <source>General Information</source>
        <translation>Allgemeine Informationen</translation>
    </message>
    <message>
        <location line="+41"/>
        <source>SerialIO Log</source>
        <translation>SerialIO-Protokoll</translation>
    </message>
    <message>
        <location line="+43"/>
        <source>Decimal</source>
        <translation>Dezimal</translation>
    </message>
    <message>
        <location line="+16"/>
        <source>Hexadecimal</source>
        <translation>Hexadezimal</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Binary</source>
        <translation>Binär</translation>
    </message>
    <message>
        <location line="+18"/>
        <source>Clear</source>
        <translation>Löschen</translation>
    </message>
    <message>
        <location line="-155"/>
        <source>Form</source>
        <translation></translation>
    </message>
    <message>
        <location line="+92"/>
        <source>ASCII</source>
        <translation></translation>
    </message>
    <message>
        <location line="+56"/>
        <source>ignore empty messages</source>
        <translation>Leere Nachrichten ignorieren</translation>
    </message>
    <message>
        <location line="-118"/>
        <source>Port:</source>
        <translation></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>[Identifier]</source>
        <translation></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location filename="../SerialIO.cpp" line="+270"/>
        <location line="+157"/>
        <source>invalid baud rate</source>
        <translation>Ungültige Baudrate</translation>
    </message>
    <message>
        <location line="-134"/>
        <location line="+144"/>
        <source>invalid number of bits</source>
        <translation>Ungültige Anzahl Datenbits</translation>
    </message>
    <message>
        <location line="-131"/>
        <location line="+140"/>
        <source>invalid number of stopbits</source>
        <translation>Ungültige Anzahl Stopbits</translation>
    </message>
    <message>
        <location line="-111"/>
        <location line="+126"/>
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
        <source>invalid endline character</source>
        <translation type="vanished">Ungültiges Terminierungszeichen</translation>
    </message>
    <message>
        <location line="+86"/>
        <location line="+25"/>
        <location line="+15"/>
        <location line="+20"/>
        <source>could not open device</source>
        <translation>Kann Gerät nicht öffnen</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>unknown error opening com port</source>
        <translation>Unbekannter Fehler beim Öffnen des COM-Ports</translation>
    </message>
    <message>
        <location line="+75"/>
        <location line="+62"/>
        <location line="+10"/>
        <location line="+48"/>
        <location line="+30"/>
        <location line="+54"/>
        <location line="+18"/>
        <source>com port not open</source>
        <translation>COM-Port nicht geöffnet</translation>
    </message>
    <message>
        <location line="-202"/>
        <location line="+18"/>
        <source>error reading from com port</source>
        <translation>Fehler beim Lesen vom COM-Port</translation>
    </message>
    <message>
        <location line="+29"/>
        <location line="+13"/>
        <location line="+4"/>
        <location line="+47"/>
        <location line="+12"/>
        <location line="+19"/>
        <location line="+5"/>
        <location line="+10"/>
        <location line="+5"/>
        <source>error writing to com port</source>
        <translation>Fehler beim Schreiben auf den COM-Port</translation>
    </message>
    <message>
        <location line="+30"/>
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
        <location line="+106"/>
        <source>licensed under LGPL</source>
        <translation></translation>
    </message>
    <message>
        <source>N.A.</source>
        <translation type="vanished">K.A.</translation>
    </message>
    <message>
        <location filename="../dialogSerialIO.cpp" line="+522"/>
        <source>Char token not closed correctly or number to big.</source>
        <translation>Das Char-Token wurde nicht korrekt geschlossen oder die Anzahl der Zeichen ist zu groß.</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Undefined error.</source>
        <translation>Unbekannter Fehler.</translation>
    </message>
</context>
<context>
    <name>SerialIO</name>
    <message>
        <location filename="../SerialIO.cpp" line="+69"/>
        <source>The number of the serial port, [0 999] = ttyS, [1000 1999] = ttyUSB, [2000 2999] = ttyACM</source>
        <translation>Portnummer der seriellen Schnittstelle, [0 999] = ttyS, [1000 1999] = ttyUSB, [2000 2999] = ttyACM</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Serial port number of this device</source>
        <translation>Portnummer der seriellen Schnittstelle dieses Geräts</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Number of bits to be written in line</source>
        <translation>Anzahl der pro Zeile geschriebenen Bits</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Stop bits after every n bits</source>
        <translation>Stopbits nach jedem n-ten Bit</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Bitmask for flow control as integer</source>
        <translation>Bitmaske für Flusskontrolle als Ganzzahl</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Endline character, will be added automatically during setVal</source>
        <translation>Terminierung, wird bei setVal() automatisch hinzugefügt</translation>
    </message>
    <message>
        <location line="+11"/>
        <source>Timeout for reading commands in [s]</source>
        <translation>Timeout für Lesebefehle in [s]</translation>
    </message>
    <message>
        <location line="-15"/>
        <source>Parity: 0 -&gt; none, 1 -&gt; odd parity, 2 -&gt; even parity, 3 -&gt; mark, 4 -&gt; space</source>
        <translation>Paritätsbit: 0 -&gt; keines, 1 -&gt; Paritätssumme ungerade (odd), 2 -&gt; Paritätssumme gerade (even), 3 -&gt; Mark-Parität, 4 -&gt; Space-Parität</translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Endline character, will be looking for during getVal by using readline</source>
        <translation>Terminierung für Leseoperationen, nach der bei aktivierter &apos;readline&apos;-Funktion in getVal() gesucht wird</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>If true, reading next line terminated by endlineRead.</source>
        <translation>Wenn aktiviert, wird beim Lesen nach der unter &apos;endlineRead&apos; angegebene Terminierung gesucht.</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>0: write output buffer as block, else: single characters with delay (1..65000 ms)</source>
        <translation>0 -&gt; Schreibt den Ausgabepuffer als ganzen Block oder die Zeichen einzeln mit einer Verzögerung (1..65000)</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>If true, all outputs and inputs are written to the toolbox</source>
        <translation>Wenn aktiviert, wird die Kommunikation in der Toolbox protokolliert</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>If debug-param is true, all outputs and inputs are written to the toolbox. If debugIgnoreEmpty is true, empty messages will be ignored</source>
        <translation>Wenn das Protokoll aktiviert wurde, können mit diesem Parameter leer Nachrichten ignoriert werden</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Clears the input buffer of serial port</source>
        <translation>Löscht den Eingabepuffer der seriellen Schnittstelle</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Clears the output buffer of serial port</source>
        <translation>Löscht den Ausgabepuffer der seriellen Schnittstelle</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Clears input (0) or output (1) buffer</source>
        <translation>Löscht den Eingabe- (0) oder Ausgabe- (1) Puffer</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Clears the input or output buffer of serial port</source>
        <translation>Löscht den Eingabe- oder Ausgabepuffer der seriellen Schnittstelle</translation>
    </message>
    <message>
        <location line="+177"/>
        <source>Mandatory paramers are NULL</source>
        <translation>Pflichtparameter ist NULL</translation>
    </message>
    <message>
        <location line="+20"/>
        <source>Optinal paramers are NULL</source>
        <translation>Optionaler Parameter ist NULL</translation>
    </message>
    <message>
        <location line="+225"/>
        <source>timeout while reading from serial port.</source>
        <translation type="unfinished">Zeitüberschreitung beim Lesen vom Serial-Port.</translation>
    </message>
    <message>
        <source>timeout</source>
        <translation type="vanished">Zeitüberschreitung</translation>
    </message>
    <message>
        <location line="-138"/>
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
        <location line="-350"/>
        <source>Current baudrate in bits/s</source>
        <translation>Aktuelle Baudrate ist bits/s</translation>
    </message>
</context>
<context>
    <name>SerialIOInterface</name>
    <message>
        <location line="-147"/>
        <source>itom-plugin for a serial port communication</source>
        <translation>Itom-Plugin zur Kommunikation über die Serielle Schnittstelle</translation>
    </message>
    <message>
        <location line="+36"/>
        <source>SerialIO is a itom-Plugin which gives direct access to serial ports.
It is used by different plugins for communication, (e.g. &apos;PIPiezoCtrl&apos;, &apos;UhlActuator&apos;, &apos;LeicaMotorFocus&apos;).
The plugin is implemented for Windows or Linux; the possible baudrates depend on the possibilites of the operating system. 

flow bitmask 
-------------- 

The flow bitmask is an OR combination of the following possible values:
Xon/Xoff - default: Xoff, Xon=1 (1. bit)
rts control - default: disabled, enabled=2, handshake=4 or (4+2) (2. and 3. bit)
cts control - default: disabled, enabled=8 (4. bit)
dtr control - default: disabled, enabled = 16, handshake = 32 or (32+16) (5. and 6. bit) 
dsr control - default: disabled, enabled = 64 

If an endline character is given, this is automatically appended to each sequence that is send using the setVal-command.
On the other side, any obtained value from the serial port is scanned for &apos;endlineRead&apos; character and automatically split.
Use an empty endline character if you want to organize all this by yourself.

Example
--------

..
    
    s = dataIO(&quot;SerialIO&quot;,port=1,baud=9600,endline=&quot;&quot;,bits=8,stopbits=1,parity=0,flow=16)
    
    #send command
    sendString = bytearray(b&quot;POS?&quot;) #or bytearray([80,79,83,63]);
    s.setVal(sendString)
    
    #get result
    answer = bytearray(9) #supposed length is 9 characters
    num = s.getVal(answer) #if ok, num contains the number of received characters(max: length of answer), immediately returns</source>
        <translation>SerialIO ist ein itom-Plugin, welches direkten Zugriff auf den Seriellen Port ermöglicht.
Es wird zur Kommunikation mit unterschiedlichen Plugins (z. B. &apos;PIPiezoCtrl&apos;, &apos;UhlActuator&apos;, &apos;LeicaMotorFocus&apos;) verwendet.
Das Plugin wurde für Linux und Windows implementiert, die möglichen Baudraten sind Betriebssystemabhängig.

Flusskontrolle
--------------

Die Flusskontrolle ist eine &apos;OR&apos;-Verknüpfung mit folgenden möglichen Werten:
Xon/Xoff - Standard: Xoff, Xon=1 (1. bit) 
rts control - Standard: disabled, enabled=2, handshake=(4 or (4+2)) (2. und 3. bit) 
cts control - Standard: disabled, enabled=8 (4. bit) 
dtr control - Standard: disabled, enabled = 16, handshake =( 32 or (32+16)) (5. und 6. bit) 
dsr control - Standard: disabled, enabled = 64 

Wurde eine Terminierung (Endline) mit angegeben, wird diese automatisch bei jeder Nachricht an das Gerät angehängt.
Beim Lesen hingegen wird die Antwort nach der &apos;endlineRead&apos;-Terminierung gescannt und automatisch abgeschnitten.
Um die Terminierung selbst zu organisieren, dürfen diese Terminierungen keine Zeichen enthalten.

Beispiel
--------

..
    
    s = dataIO(&quot;SerialIO&quot;,port=1,baud=9600,endline=&quot;&quot;,bits=8,stopbits=1,parity=0,flow=16)
    
    #Kommando senden
    sendString = bytearray(b&quot;POS?&quot;) #Oder bytearray([80,79,83,63]);
    s.setVal(sendString)
    
    #Antwort
    answer = bytearray(9) #angenommene Antwortlänge ist 9 Zeichen
    num = s.getVal(answer) #Wenn ok, dann enthält num die Anzahl der empfangenen Zeichen (max: Länge der Antwort)</translation>
    </message>
    <message>
        <location line="+39"/>
        <source>The number of the serial port, [0 999] = ttyS, [1000 1999] = ttyUSB, [2000 2999] = ttyACM</source>
        <translation>Portnummer der seriellen Schnittstelle, [0 999] = ttyS, [1000 1999] = ttyUSB, [2000 2999] = ttyACM</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>The number of the serial port, starting with 1 (linux 0)</source>
        <translation>Nummer der seriellen Portnummer, beginnend mit 1 (bei Linux mit 0)</translation>
    </message>
    <message>
        <location line="+5"/>
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
        <source>Parity: 0 -&gt; none, 1 -&gt; odd parity, 2 -&gt; even parity, 3 -&gt; mark, 4 -&gt; space</source>
        <translation>Paritätsbit: 0 -&gt; keines, 1 -&gt; Paritätssumme ungerade (odd), 2 -&gt; Paritätssumme gerade (even), 3 -&gt; Mark-Parität, 4 -&gt; Space-Parität</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>0: write output buffer as block, else: single characters with delay (1..65000 ms)</source>
        <translation>0 -&gt; Schreibt den Ausgabepuffer als ganzen Block oder die Zeichen einzeln mit einer Verzögerung (1..65000)</translation>
    </message>
    <message>
        <location line="-2"/>
        <source>Bitmask for flow control (see docstring for more information)</source>
        <translation>Bitmaske für Flusskontrolle (für nähere Informationen siehe Docstring)</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Timeout for reading commands in [s]</source>
        <translation>Timeout für Lesebefehle in [s]</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Initialised &apos;debug&apos;-parameter with given value. If debug-param is true, all out and inputs are written to dockingWidget</source>
        <translation>Schaltet Protokollierung ein und aus. Wurde diese aktiviert, wird die Kommunikation in der Toolbox protokolliert</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>If debug-param is true, all out and inputs are written to dockingWidget. If debugIgnoreEmpty is true, empty messages will be ignored</source>
        <translation>Wenn das Protokoll aktiviert wurde, können mit diesem Parameter leer Nachrichten ignoriert werden</translation>
    </message>
    <message>
        <location line="-19"/>
        <source>The baudrate of the port</source>
        <translation>Die Baudrate des Ports</translation>
    </message>
</context>
<context>
    <name>dialogSerialIO</name>
    <message>
        <location filename="../dialogSerialIO.cpp" line="-73"/>
        <source>Configuration Dialog</source>
        <translation>Konfigurationsdialog</translation>
    </message>
    <message>
        <location line="+145"/>
        <source>Error: malformed command string - not send</source>
        <translation>Fehler: Syntaxfehler- nicht gesendet</translation>
    </message>
    <message>
        <location line="+109"/>
        <source>Error: &apos;%1&apos; could not be interpreted - not send</source>
        <translation>Fehler: %1 konnte nicht interpretiert werden - nicht gesendet</translation>
    </message>
    <message>
        <location filename="../dialogSerialIO.ui" line="+33"/>
        <source>Settings</source>
        <translation>Einstellungen</translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Basic</source>
        <translation>Grundeinstellungen</translation>
    </message>
    <message>
        <location line="+42"/>
        <source>Baud</source>
        <translation>Baudrate</translation>
    </message>
    <message>
        <location line="-15"/>
        <source>Endline</source>
        <translation>Terminierung</translation>
    </message>
    <message>
        <location line="+176"/>
        <location line="+99"/>
        <source>&lt;none&gt;</source>
        <translation>&lt;nichts&gt;</translation>
    </message>
    <message>
        <location line="-238"/>
        <source>Parity</source>
        <translation>Paritätsbit</translation>
    </message>
    <message>
        <location line="+330"/>
        <location line="+66"/>
        <location line="+32"/>
        <location line="+50"/>
        <location line="+17"/>
        <source>disable</source>
        <translation>ausschalten</translation>
    </message>
    <message>
        <location line="-160"/>
        <location line="+66"/>
        <location line="+32"/>
        <location line="+50"/>
        <location line="+17"/>
        <source>enable</source>
        <translation>einschalten</translation>
    </message>
    <message>
        <location line="-397"/>
        <source>Time out</source>
        <translation>Timeout</translation>
    </message>
    <message>
        <location line="+148"/>
        <source>Debug Mode</source>
        <translation>Protokollierung</translation>
    </message>
    <message>
        <location line="+364"/>
        <source>Read delay</source>
        <translation>Leseverzögerung</translation>
    </message>
    <message>
        <location line="-322"/>
        <source>Flow control</source>
        <translation>Flusskontrolle</translation>
    </message>
    <message>
        <location line="+269"/>
        <source>Send message</source>
        <translation>Nachricht versenden</translation>
    </message>
    <message>
        <location line="+26"/>
        <source>Transfer</source>
        <translation>Übertragung</translation>
    </message>
    <message>
        <location line="+103"/>
        <source>Decimal</source>
        <translation>Dezimal</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Hexadecimal</source>
        <translation>Hexadezimal</translation>
    </message>
    <message>
        <location line="+10"/>
        <source>Binary</source>
        <translation>Binär</translation>
    </message>
    <message>
        <location line="-63"/>
        <source>Send</source>
        <translation>Senden</translation>
    </message>
    <message>
        <location line="+72"/>
        <source>Read</source>
        <translation>Lesen</translation>
    </message>
    <message>
        <location line="+61"/>
        <source>Cancel</source>
        <translation>Abbrechen</translation>
    </message>
    <message>
        <location line="-211"/>
        <source>Use input below to send characters to the serial port. Characters will be send as their ASCII code from the character written. To directly write ASCII codes use the format $(code) or select Decimal, Hexadecimal or Binary separated by space.</source>
        <translation>Um Nachrichten an die serielle Schnittstelle zu senden, die Eingabezeile benutzen. Zeichen werden als ASCII-Zeichen übermittelt. Um direkt ASCII-Code zu schreiben, das Format $(Code) oder Dezimal, Hexadezimal oder Binär mit Leerzeichen getrennt benutzen.</translation>
    </message>
    <message>
        <location line="+221"/>
        <source>Apply</source>
        <translation>Übernehmen</translation>
    </message>
    <message>
        <location line="-278"/>
        <source>Python Command</source>
        <translation>Python-Befehl</translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Create</source>
        <translation>Erzeugen</translation>
    </message>
    <message>
        <location line="+221"/>
        <source>Clear</source>
        <translation>Löschen</translation>
    </message>
    <message>
        <location line="-591"/>
        <source>Send Delay</source>
        <translation>Verzögerung</translation>
    </message>
    <message>
        <location line="-241"/>
        <source>Dialog</source>
        <translation></translation>
    </message>
    <message>
        <location line="+103"/>
        <source>none</source>
        <translation>keine</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>odd</source>
        <translation>ungerade</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>even</source>
        <translation>gerade</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>mark</source>
        <translation></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>space</source>
        <translation></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Bits</source>
        <translation></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>5</source>
        <translation></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>6</source>
        <translation></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>7</source>
        <translation></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>8</source>
        <translation></translation>
    </message>
    <message>
        <location line="+34"/>
        <location line="+49"/>
        <location line="+463"/>
        <source>ms</source>
        <translation></translation>
    </message>
    <message>
        <location line="-498"/>
        <location line="+99"/>
        <source>\r</source>
        <translation></translation>
    </message>
    <message>
        <location line="-94"/>
        <location line="+99"/>
        <source>\n</source>
        <translation></translation>
    </message>
    <message>
        <location line="-94"/>
        <location line="+99"/>
        <source>\r\n</source>
        <translation></translation>
    </message>
    <message>
        <location line="-59"/>
        <source>Stopbits</source>
        <translation></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>1</source>
        <translation></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>2</source>
        <translation></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Endline Read</source>
        <translation>Leseterminierung</translation>
    </message>
    <message>
        <location line="+67"/>
        <source>Readline</source>
        <translation>Zeilenlesen</translation>
    </message>
    <message>
        <location line="+40"/>
        <source>Xon/Xoff</source>
        <translation></translation>
    </message>
    <message>
        <location line="+54"/>
        <source>rts</source>
        <translation></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>cts</source>
        <translation></translation>
    </message>
    <message>
        <location line="+32"/>
        <source>dtr</source>
        <translation></translation>
    </message>
    <message>
        <location line="+24"/>
        <location line="+67"/>
        <source>handshake</source>
        <translation></translation>
    </message>
    <message>
        <location line="-41"/>
        <source>dsr</source>
        <translation></translation>
    </message>
    <message>
        <location line="+205"/>
        <source>E.g. Test$(10)</source>
        <translation type="unfinished">z. B. Test$(10)</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>ASCII</source>
        <translation></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>Bytes separated by space, e.g. 255 10 13</source>
        <translation type="unfinished">Bytes getrennt durch ein Leerzeichen, z. B. 255 10 13</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Bytes separated by space, e.g. 4f a 1b</source>
        <translation type="unfinished">Bytes getrennt durch ein Leerzeichen, z. B. 4f a 1b</translation>
    </message>
    <message>
        <location line="+10"/>
        <source>Bytes separated by space, e.g. 1010 11111111 100</source>
        <translation type="unfinished">Bytes getrennt durch ein Leerzeichen, z. B. 1010 11111111 100</translation>
    </message>
    <message>
        <location line="+22"/>
        <source>automatically read answer after send</source>
        <translation type="unfinished">Automatisches Lesen nach dem Senden</translation>
    </message>
    <message>
        <location line="+41"/>
        <source>OK</source>
        <translation></translation>
    </message>
</context>
</TS>
