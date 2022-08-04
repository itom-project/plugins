<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DockWidgetSerialIO</name>
    <message>
        <source>General Information</source>
        <translation type="vanished">Allgemeine Informationen</translation>
    </message>
    <message>
        <source>SerialIO Log</source>
        <translation type="vanished">SerialIO-Protokoll</translation>
    </message>
    <message>
        <source>Decimal</source>
        <translation type="vanished">Dezimal</translation>
    </message>
    <message>
        <source>Hexadecimal</source>
        <translation type="vanished">Hexadezimal</translation>
    </message>
    <message>
        <source>Binary</source>
        <translation type="vanished">Binär</translation>
    </message>
    <message>
        <source>Clear</source>
        <translation type="vanished">Löschen</translation>
    </message>
    <message>
        <source>ignor empty messages</source>
        <translation type="vanished">Leere Nachrichten ignorieren</translation>
    </message>
</context>
<context>
    <name>LibModBus</name>
    <message>
        <location filename="../LibModBus.cpp" line="+138"/>
        <source>IP Adress or COM-Port of the target device</source>
        <translation type="unfinished">IP-Adresse oder COM-Port des Zielgeräts</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>TCP Port for ModbusTCP or slave ID for ModbusRTU</source>
        <translation type="unfinished">TCP-Port für ModbusTCP oder Slave-ID für ModbusRTU</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>The baudrate of the port for RTU communication</source>
        <translation type="unfinished">Baudrate für den Port der RTU-Kommunikation</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Parity for RTU communication (N,E,O)</source>
        <translation type="unfinished">Parität für die RTU-Kommunikation (N,E,O)</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Number of bits to be written in line for RTU communication</source>
        <translation type="unfinished">Anzahl Bits die pro Zeile über die RTU-Kommunikation geschrieben werden</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Stop bits after every n bits for RTU communication</source>
        <translation type="unfinished">Stop-Bits nach jedem n-ten Bit für die RTU-Kommunikation</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Enables command-line output of different readouts (e.g. register values of getVal)</source>
        <translation type="unfinished">Aktiviert Kommandozeilenausgabe von unterschliedlichen Anzeigen (z.B. Registerwerte von getVal)</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Default string for register addressing. Coding is &apos;Reg1Address,Reg1Size;Reg2Address,Reg2Size...&apos;</source>
        <translation type="unfinished">Standard-String für Registeradressierung. Die Kodierung ist &apos;Reg1Address, Reg1Size; Reg2Address, Reg2Size ...&apos;</translation>
    </message>
    <message>
        <location line="+176"/>
        <source>invalid target device</source>
        <translation type="unfinished">Ungültiges Zeilgerät</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Unable to allocate libmodbus context</source>
        <translation type="unfinished">Aktivieren um libmodbus-Kontext zuzuordnen</translation>
    </message>
    <message>
        <location line="+63"/>
        <source>StartDevice not necessary</source>
        <translation type="unfinished">&quot;StartDevice&quot; nicht nötig</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>StopDevice not necessary</source>
        <translation type="unfinished">&quot;StopDevice&quot; nicht nötig</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>Acquire not necessary</source>
        <translation type="unfinished">&quot;Acquire&quot; nicht nötig</translation>
    </message>
</context>
<context>
    <name>LibModBusInterface</name>
    <message>
        <location line="-362"/>
        <source>itom-plugin for a modbus communication</source>
        <translation type="unfinished">itom-Plugin für eine modbus-Komminikation</translation>
    </message>
    <message>
        <location line="+11"/>
        <source>LibModBus is a itom-Plugin which provides modbusTCP and modbusRTU communication.
The plugin is based on libmodbus v3.1.1 library and tested under Windows only atm.
Registers are addressed using the modbus_read_registers (0x03) and modbus_write_registers (0x10) functions of libmodbus, coils are addressed using the modbus_read_bits (0x01) and modbus_write_bits (0x0F) functions. 
The plugin-functions used are getVal(dObj) and setVal(dObj) with a data object of the size 1xN with N the number of registers to be read/written. 
The content of the registers is expected as data in the uint16 data object for registers or uint8 data object for coils, the addressing of the registers is performed by a dObj-MetaTag &apos;registers&apos; containing a string with address and number of consecutive registers seperated by &apos;,&apos; and different registers seperated by &apos;;&apos; i.e.: &apos;10,2;34,1;77,4&apos; to address registers 10,11;34;77..80. Number 1 of consecutive registers can be left out i.e.:&apos;10,2;34;77,4&apos; 
If no MetaTag is set, values of m_params[&apos;registers&apos;] is tried to be used for addressing.</source>
        <translation type="unfinished">LibModBus ist ein itom-Plugin, das eine Kommunikationsschnittstelle mit modbusTCP und modbusRTU bereitstellt.
Das Plugin basiert auf der libmodbus v3.1.1 Bibliothek und wurde nur unter Windows getestet.
Register werden durch modbus_read_registers (0x03) und modbus_write_registers (0x10) Funktionen von libmodbus adressiert, Spulen durch modbus_read_bits (0x01) und modbus_write_bits (0x0F).
Die genutzten Plugin-Funktionen sind getVal(dObj) und setVal(dObj) mit einem dataObject der Größe 1xN, während N die Nummer des Registers ist, welches gelesen/geschrieben werden soll.
Erwartet wird ein dataObject entweder vom Typ uint16 für Register oder uint8 für Spulen. Die Adressierung des Registers wird über das MetaTag &apos;registers&apos; des dataObjects durchgeführt, welches einen String mit der Adresse und Anzahl der aufeinanderfolgenden Register getrennt durch Komme und verschiedene Register getrennt durch Semikolon, z.B. &apos;10,2;34,1;77,4&apos; zur Adressierung der Register 10,11;34;77..80. Anzahl 1 von fortlaufenden Registern kann weggelassen werden; im Beispiel: &apos;10,2;34;77,4&apos;
Falls keine MetaTag gesetzt wurde, wird der Wert von m_params[&apos;registers&apos;] zur Adressierung genutzt.</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>Adress of the target device. IP-Adress for ModbusTCP (i.e. 127.0.0.1) or COM-Port for ModbusRTU (i.e. COM1)</source>
        <translation type="unfinished">Adresse des Zielgeräts: IP-Adresse für ModbusTCP (z.B. 127.0.0.1) oder COM-Port für ModbusRTU (z.B. COM1)</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>The number of the TCP port for ModBusTCP (default 502) or slave ID for ModbusRTU</source>
        <translation type="unfinished">TCP-Port für ModbusTCP (standardmäßig 502) oder Slave-ID für ModbusRTU</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>The baudrate of the port for RTU communication</source>
        <translation type="unfinished">Baudrate für den Port der RTU-Kommunikation</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Parity for RTU communication (N,E,O)</source>
        <translation type="unfinished">Parität für die RTU-Kommunikation (N,E,O)</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Number of bits to be written in line for RTU communication</source>
        <translation type="unfinished">Anzahl Bits die pro Zeile über die RTU-Kommunikation geschrieben werden</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Stop bits after every n bits for RTU communication</source>
        <translation type="unfinished">Stop-Bits nach jedem n-ten Bit für die RTU-Kommunikation</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Enables command-line output of different readouts (e.g. register values of getVal)</source>
        <translation type="unfinished">Aktiviert Commandozeilenausgabe für unterschiedliche Meldungen (z.B. Registerwert von getVal)</translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <source>invalid baud rate</source>
        <translation type="vanished">Ungültige Baudrate</translation>
    </message>
    <message>
        <source>invalid number of bits</source>
        <translation type="vanished">Ungültige Anzahl Datenbits</translation>
    </message>
    <message>
        <source>invalid number of stopbits</source>
        <translation type="vanished">Ungültige Anzahl Stopbits</translation>
    </message>
    <message>
        <source>invalid parity</source>
        <translation type="vanished">Ungültige Parität</translation>
    </message>
    <message>
        <source>error setting parameters</source>
        <translation type="vanished">Fehler beim Setzten der Parameter</translation>
    </message>
    <message>
        <source>error setting timeout</source>
        <translation type="vanished">Fehler beim Setzten des Timeouts</translation>
    </message>
    <message>
        <source>invalid endline character</source>
        <translation type="vanished">Ungültiges Terminierungszeichen</translation>
    </message>
    <message>
        <source>could not open device</source>
        <translation type="vanished">Kann Gerät nicht öffnen</translation>
    </message>
    <message>
        <source>unknown error opening com port</source>
        <translation type="vanished">Unbekannter Fehler beim Öffnen des COM-Ports</translation>
    </message>
    <message>
        <source>com port not open</source>
        <translation type="vanished">COM-Port nicht geöffnet</translation>
    </message>
    <message>
        <source>error reading from com port</source>
        <translation type="vanished">Fehler beim Lesen vom COM-Port</translation>
    </message>
    <message>
        <source>error writing to com port</source>
        <translation type="vanished">Fehler beim Schreiben auf den COM-Port</translation>
    </message>
    <message>
        <source>invalid number of buffer type (0: input, 1: output)</source>
        <translation type="vanished">Ungültige Nummer des Puffertyps (0: Input, 1: Output)</translation>
    </message>
    <message>
        <source>Unable to clear buffer</source>
        <translation type="vanished">Puffer löschen war nicht erfolgreich</translation>
    </message>
    <message>
        <source>N.A.</source>
        <translation type="vanished">K.A.</translation>
    </message>
    <message>
        <location line="-18"/>
        <source>licensed under GPL, since the libmodbus is also licensed under GPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+250"/>
        <source>Modbus-connect failed!</source>
        <translation type="unfinished">Die Modbus-Verbindung schlug fehl!</translation>
    </message>
    <message>
        <location line="+123"/>
        <location line="+131"/>
        <source>Data type of input object must be uint16 for registers or uint8 for coils</source>
        <translation type="unfinished">Datentyp des Eingangsobjekts muss für Register uint16 und für Spulen uint8 sein</translation>
    </message>
    <message>
        <location line="-55"/>
        <source>Size of given data object does not match number of requested registers</source>
        <translation type="unfinished">Die Größe des dataObjects entspricht nicht der Anzahl der geforderten Register</translation>
    </message>
    <message>
        <location line="+140"/>
        <source>Size of given data object does not match number of transmitted registers</source>
        <translation type="unfinished">Die Größe des dataObjects entspricht nicht der Anzahl der gesendeten Register</translation>
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
        <source>Serial port number of this device</source>
        <translation type="vanished">Serielle Portnummer dieses Geräts</translation>
    </message>
    <message>
        <source>Number of bits to be written in line</source>
        <translation type="vanished">Anzahl der pro Zeile geschriebenen Bits</translation>
    </message>
    <message>
        <source>Stop bits after every n bits</source>
        <translation type="vanished">Stopbits nach jedem n-ten Bit</translation>
    </message>
    <message>
        <source>Toggle parity check</source>
        <translation type="vanished">Schalter Paritätsprüfung</translation>
    </message>
    <message>
        <source>Bitmask for flow control as integer</source>
        <translation type="vanished">Bitmaske für Flusskontrolle als Ganzzahl</translation>
    </message>
    <message>
        <source>Endline character, will be added automatically during setVal</source>
        <translation type="vanished">Terminierung, wird bei setVal() automatisch hinzugefügt</translation>
    </message>
    <message>
        <source>Toggle: write output buffer as block @ once or single characters</source>
        <translation type="obsolete">Schalter: Schreibt Puffer als Block oder als einzelne Zeichen</translation>
    </message>
    <message>
        <source>Timeout for reading commands in [s]</source>
        <translation type="vanished">Timeout für Lesebefehle in [s]</translation>
    </message>
    <message>
        <source>If true, all out and inputs are written to dockingWidget</source>
        <translation type="vanished">Wenn aktiviert, wird die Kommunikation im dockingWidget protokolliert</translation>
    </message>
    <message>
        <source>Clears the input buffer of serial port</source>
        <translation type="vanished">Löscht den Input-Puffer der seriellen Schnittstelle</translation>
    </message>
    <message>
        <source>Clears the output buffer of serial port</source>
        <translation type="vanished">Löscht den Output-Puffer der seriellen Schnittstelle</translation>
    </message>
    <message>
        <source>Clears input (0) or output (1) buffer</source>
        <translation type="vanished">Löscht den Input- (0) oder Output- (1) Puffer</translation>
    </message>
    <message>
        <source>Clears the input or output buffer of serial port</source>
        <translation type="vanished">Löscht den Input- oder Output-Puffer der seriellen Schnittstelle</translation>
    </message>
    <message>
        <source>name of requested parameter is empty.</source>
        <translation type="vanished">Name des angeforderten Parameters ist leer.</translation>
    </message>
    <message>
        <source>parameter not found in m_params.</source>
        <translation type="vanished">Parameter nicht in m_params gefunden.</translation>
    </message>
    <message>
        <source>name of given parameter is empty.</source>
        <translation type="vanished">Name des Parameters ist leer.</translation>
    </message>
    <message>
        <source>Parameter is read only, input ignored</source>
        <translation type="vanished">Parameter ist schreibgeschützt, Eingabe wurde ignoriert</translation>
    </message>
    <message>
        <source>New value is larger than parameter range, input ignored</source>
        <translation type="vanished">Wert ist größer als der zugelassene Bereich, Eingabe wurde ignoriert</translation>
    </message>
    <message>
        <source>New value is smaller than parameter range, input ignored</source>
        <translation type="vanished">Wert ist kleiner als der zugelassene Bereich, Eingabe wurde ignoriert</translation>
    </message>
    <message>
        <source>Parameter type conflict</source>
        <translation type="vanished">Konflikt mit Parametertyp</translation>
    </message>
    <message>
        <source>Parameter not found</source>
        <translation type="vanished">Parameter wurde nicht gefunden</translation>
    </message>
    <message>
        <source>StartDevice not necessary</source>
        <translation type="vanished">&quot;StartDevice&quot; nicht nötig</translation>
    </message>
    <message>
        <source>StopDevice not necessary</source>
        <translation type="vanished">&quot;StopDevice&quot; nicht nötig</translation>
    </message>
    <message>
        <source>Acquire not necessary</source>
        <translation type="vanished">&quot;Acquire&quot; nicht nötig</translation>
    </message>
    <message>
        <source>Current baudrate in bits/s</source>
        <translation type="vanished">Aktuelle Baudrate ist bits/s</translation>
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
        <source>itom-plugin for a serial port communication</source>
        <translation type="vanished">Itom-Plugin zur Kommunikation über die Serielle Schnittstelle</translation>
    </message>
    <message>
        <source>The number of the serial port, starting with 1</source>
        <translation type="obsolete">Die Nummer des Ports der seriellen Schnittstelle beginnt mit 1</translation>
    </message>
    <message>
        <source>The endline character, which is added automatically after every setVal()</source>
        <translation type="vanished">Der Terminierungsstring, der automatisch nach jedem setVal() hinzugefügt wird</translation>
    </message>
    <message>
        <source>Number of bits to be written in line</source>
        <translation type="vanished">Anzahl der pro Zeile geschriebenen Bits</translation>
    </message>
    <message>
        <source>Stop bits after every n bits</source>
        <translation type="vanished">Stopbits nach jedem n-ten Bit</translation>
    </message>
    <message>
        <source>Parity: 0 -&gt; no parity, 1 -&gt; odd parity, 2 -&gt; even parity</source>
        <translation type="vanished">Paritätsprüfung: 0 -&gt; keine Parität, 1 -&gt; ODD Parität, 2 -&gt; Parität</translation>
    </message>
    <message>
        <source>Bitmask for flow control (see docstring for more information)</source>
        <translation type="vanished">Bitmaske für Flusskontrolle (für nähere Informationen siehe Docstring)</translation>
    </message>
    <message>
        <source>Toggle parity check</source>
        <translation type="obsolete">Schalter Paritätsprüfung</translation>
    </message>
    <message>
        <source>Bitmask for flow control</source>
        <translation type="obsolete">Bitmaske für Flusskontrolle</translation>
    </message>
    <message>
        <source>Toggle: write output buffer as block or single characters</source>
        <translation type="obsolete">Schalter: Schreibt Puffer als Block oder als einzelne Zeichen</translation>
    </message>
    <message>
        <source>Timeout for reading commands in [s]</source>
        <translation type="vanished">Timeout für Lesebefehle in [s]</translation>
    </message>
    <message>
        <source>Initialised &apos;debug&apos;-parameter with given value. If debug-param is true, all out and inputs are written to dockingWidget</source>
        <translation type="vanished">Schaltet Protokollierung ein und aus. Wurde sie aktiviert, wird die Kommunikation im dockingWidget protokolliert</translation>
    </message>
    <message>
        <source>The baudrate of the port</source>
        <translation type="vanished">Die Baudrate des Ports</translation>
    </message>
</context>
<context>
    <name>dialogSerialIO</name>
    <message>
        <source>Configuration Dialog</source>
        <translation type="vanished">Konfigurationsdialog</translation>
    </message>
    <message>
        <source>Error: malformed command string - not send</source>
        <translation type="vanished">Fehler: Syntaxfehler- nicht gesendet</translation>
    </message>
    <message>
        <source>Error: &apos;%1&apos; could not be interpreted - not send</source>
        <translation type="vanished">Fehler: %1 konnte nicht interpretiert werden - nicht gesendet</translation>
    </message>
    <message>
        <source>Settings</source>
        <translation type="vanished">Einstellungen</translation>
    </message>
    <message>
        <source>Basic</source>
        <translation type="vanished">Grundeinstellungen</translation>
    </message>
    <message>
        <source>Baud</source>
        <translation type="vanished">Baudrate</translation>
    </message>
    <message>
        <source>Endline</source>
        <translation type="vanished">Terminierung</translation>
    </message>
    <message>
        <source>&lt;none&gt;</source>
        <translation type="vanished">&lt;nichts&gt;</translation>
    </message>
    <message>
        <source>Parity</source>
        <translation type="vanished">Parität</translation>
    </message>
    <message>
        <source>disable</source>
        <translation type="vanished">ausschalten</translation>
    </message>
    <message>
        <source>enable</source>
        <translation type="vanished">einschalten</translation>
    </message>
    <message>
        <source>Time out</source>
        <translation type="vanished">Timeout</translation>
    </message>
    <message>
        <source>Single char</source>
        <translation type="obsolete">Einzelzeichen</translation>
    </message>
    <message>
        <source>Debug Mode</source>
        <translation type="vanished">Protokollierung</translation>
    </message>
    <message>
        <source>Read delay</source>
        <translation type="vanished">Leseverzögerung</translation>
    </message>
    <message>
        <source>Flow control</source>
        <translation type="vanished">Flusskontrolle</translation>
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
        <source>Send message</source>
        <translation type="vanished">Nachricht versenden</translation>
    </message>
    <message>
        <source>Transfer</source>
        <translation type="vanished">Übertragung</translation>
    </message>
    <message>
        <source>Decimal</source>
        <translation type="vanished">Dezimal</translation>
    </message>
    <message>
        <source>Hexadecimal</source>
        <translation type="vanished">Hexadezimal</translation>
    </message>
    <message>
        <source>Binary</source>
        <translation type="vanished">Binär</translation>
    </message>
    <message>
        <source>Send</source>
        <translation type="vanished">Senden</translation>
    </message>
    <message>
        <source>Read</source>
        <translation type="vanished">Lesen</translation>
    </message>
    <message>
        <source>Cancel</source>
        <translation type="vanished">Abbrechen</translation>
    </message>
    <message>
        <source>Create Python Command</source>
        <translation type="obsolete">Python-Code erstellen</translation>
    </message>
    <message>
        <source>Use input below to send characters to the serial port. Characters will be send as their ASCII code from the character written. To directly write ASCII codes use the format $(code) or select Decimal, Hexadecimal or Binary separated by space.</source>
        <translation type="vanished">Um Nachrichten an die serielle Schnittstelle zu senden, die Eingabezeile benutzen. Zeichen werden als ASCII-Zeichen übermittelt. Um direkt ASCII-Code zu schreiben, das Format $(Code) oder Dezimal, Hexadezimal oder Binär mit Leerzeichen getrennt benutzen.</translation>
    </message>
    <message>
        <source>Apply</source>
        <translation type="vanished">Übernehmen</translation>
    </message>
    <message>
        <source>Python Command</source>
        <translation type="vanished">Python-Befehl</translation>
    </message>
    <message>
        <source>Create</source>
        <translation type="vanished">Erzeugen</translation>
    </message>
    <message>
        <source>Clear</source>
        <translation type="vanished">Löschen</translation>
    </message>
</context>
<context>
    <name>ito::AddInActuator</name>
    <message>
        <source>Constructor must be overwritten</source>
        <translation type="obsolete">Konstruktor muss überschrieben werden</translation>
    </message>
    <message>
        <source>Destructor must be overwritten</source>
        <translation type="obsolete">Destruktor muss überschrieben werden</translation>
    </message>
</context>
<context>
    <name>ito::AddInAlgo</name>
    <message>
        <source>Constructor must be overwritten</source>
        <translation type="obsolete">Konstruktor muss überschrieben werden</translation>
    </message>
    <message>
        <source>uninitialized vector for mandatory parameters!</source>
        <translation type="vanished">Uninitialisierte Vektoren für Pflichtparameter!</translation>
    </message>
    <message>
        <source>uninitialized vector for optional parameters!</source>
        <translation type="vanished">Uninitialisierte Vektoren für optionale Parameter!</translation>
    </message>
    <message>
        <source>uninitialized vector for output parameters!</source>
        <translation type="vanished">Uninitialisierte Vektoren für Ausgabeparameter!</translation>
    </message>
</context>
<context>
    <name>ito::AddInBase</name>
    <message>
        <source>function execution unused in this plugin</source>
        <translation type="obsolete">Funktion &quot;Execution&quot; in diesem Plugin nicht verwendet</translation>
    </message>
    <message>
        <source>Your plugin is supposed to have a configuration dialog, but you did not implement the showConfDialog-method</source>
        <translation type="obsolete">Ihr Plugin hat vermutlich einen Konfigurationsdialog, doch die showConfDialog-Methode wurde nicht implementiert</translation>
    </message>
</context>
<context>
    <name>ito::AddInDataIO</name>
    <message>
        <source>Constructor must be overwritten</source>
        <translation type="obsolete">Konstruktor muss überschrieben werden</translation>
    </message>
    <message>
        <source>Destructor must be overwritten</source>
        <translation type="obsolete">Destruktor muss überschrieben werden</translation>
    </message>
    <message>
        <source>listener does not have a slot </source>
        <translation type="obsolete">Listener hat keinen Slot </translation>
    </message>
    <message>
        <source>this object already has been registered as listener</source>
        <translation type="obsolete">Dieses Objekt wurde bereits als Listener registriert</translation>
    </message>
    <message>
        <source>timer could not be set</source>
        <translation type="obsolete">Timer konnte nicht gesetzt werden</translation>
    </message>
    <message>
        <source>the object could not been removed from the listener list</source>
        <translation type="obsolete">Das Objekt konnte nicht von der Listener-Liste gelöscht werden</translation>
    </message>
    <message>
        <source>not implemented</source>
        <translation type="obsolete">Nicht implementiert</translation>
    </message>
</context>
</TS>
