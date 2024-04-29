<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DockWidgetHidApi</name>
    <message>
        <location filename="../dockWidgetHidApi.ui" line="+20"/>
        <source>Form</source>
        <translation></translation>
    </message>
    <message>
        <location line="+30"/>
        <source>General Information</source>
        <translation type="unfinished">Allgemeine Informationen</translation>
    </message>
    <message>
        <location line="+18"/>
        <source>ID:</source>
        <translation type="unfinished">ID:</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>[ID]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>USB Log</source>
        <translation type="unfinished">USB-Protokoll</translation>
    </message>
    <message>
        <location line="+33"/>
        <source>ASCII</source>
        <translation></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>Decimal</source>
        <translation type="unfinished">Dezimal</translation>
    </message>
    <message>
        <location line="+22"/>
        <source>Hexadecimal</source>
        <translation type="unfinished">Hexadezimal</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Binary</source>
        <translation type="unfinished">Binär</translation>
    </message>
    <message>
        <location line="+11"/>
        <source>ignore empty messages</source>
        <translation type="unfinished">Leere Nachrichten ignorieren</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Clear</source>
        <translation type="unfinished">Löschen</translation>
    </message>
</context>
<context>
    <name>DockWidgetLibUSB</name>
    <message>
        <source>General Information</source>
        <translation type="vanished">Allgemeine Informationen</translation>
    </message>
    <message>
        <source>ID:</source>
        <translation type="vanished">ID:</translation>
    </message>
    <message>
        <source>USB Log</source>
        <translation type="vanished">USB-Protokoll</translation>
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
        <source>ignor empty messages</source>
        <translation type="vanished">Leere Nachrichten ignorieren</translation>
    </message>
    <message>
        <source>Clear</source>
        <translation type="vanished">Löschen</translation>
    </message>
</context>
<context>
    <name>ItomHidApi</name>
    <message>
        <location filename="../itomHidApi.cpp" line="+119"/>
        <source>If true, all out and inputs are written to dockingWidget</source>
        <translation type="unfinished">Wenn aktiviert, wird die Kommunikation im dockingWidget protokolliert</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>manufacturer string</source>
        <translation type="unfinished">Herstellername</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>product string</source>
        <translation type="unfinished">Produkttext</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>serial number string</source>
        <translation type="unfinished">Seriennummer</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>if true, getVal and setVal will operate on feature reports, else on the output buffer (default)</source>
        <translation type="unfinished">Wenn aktiviert wird getVal und setVal über die Konsole ausgegeben, sonsten über den Output-Buffer (standard)</translation>
    </message>
    <message>
        <location line="+249"/>
        <source>StartDevice not necessary</source>
        <translation type="unfinished">&quot;StartDevice&quot; nicht nötig</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>StopDevice not necessary</source>
        <translation type="unfinished">&quot;StopDevice&quot; nicht nötig</translation>
    </message>
    <message>
        <location line="+14"/>
        <source>Acquire not necessary</source>
        <translation type="unfinished">&quot;Acquire&quot; nicht nötig</translation>
    </message>
</context>
<context>
    <name>ItomHidApiInterface</name>
    <message>
        <location line="-348"/>
        <source>itom-plugin for a usb HID API communication</source>
        <translation type="unfinished">itom-Plugin zur Nutzung einer USB-HID-API-Kommunikation</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>HidApi is a plugin which gives direct/raw access to HID compliant devices (e.g. via USB).
It can be used by plugins for communication analog to the serial port.
The plugin is implemented for Windows, Linux and Mac.

To connect to a device you need the vendor id and the product id.

The setVal and getVal functions will write and read on the output or on the feature.</source>
        <translation type="unfinished">HidApi ist ein Plugin mit direct/row-Zugriff zu HID-konformen Geräten (z.B. über USB).
Es kann für die analoge Kommunikation über den seriellen Port genutzt werden.
Das Plugin wurde für Windows, Linux und Mac entwickelt.

Für die Verbindung mit einem Gerät wird die Hersteller-ID und die Produkt-ID benötigt.

Die setVal- und getVal-Funktionen lesen und schreiben über die Konsole oder die Funktionen.</translation>
    </message>
    <message>
        <location line="+16"/>
        <source>The vendor id of the device to connect to (0 will return a list of all devices if &apos;print_info_about_all_devices&apos; is 1).</source>
        <translation type="unfinished">Hersteller-ID des Geräts, mit dem verbunden werden soll (0 liefert eine Liste aller Geräte falls &apos;print_info_about_all_devices&apos; 1 ist).</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>The product id of the device to connect to (0 will return a list of all devices if &apos;print_info_about_all_devices&apos; is 1).</source>
        <translation type="unfinished">Die Produkt-ID des Geräts, mit dem verbunden werden soll (0 liefert eine Liste aller Geräte falls &apos;print_info_about_all_devices&apos; 1 ist).</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Optional serial number of device that should be opened.</source>
        <translation type="unfinished">Optional: Seriennummer des Geräts, das geöffnet werden soll.</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>If true, all information about connected devices is print to the console.</source>
        <translation type="unfinished">Wenn aktiviert, werden alle Informationen über das verbundene Gerät über die Konsole ausgegeben.</translation>
    </message>
</context>
<context>
    <name>ItomUSBDevice</name>
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
        <translation type="vanished">Löscht den Eingabe- (0) oder Ausgabe- (1) Puffer</translation>
    </message>
    <message>
        <source>Clears the output buffer of serial port</source>
        <translation type="vanished">Löscht den Ausgabepuffer der seriellen Schnittstelle</translation>
    </message>
    <message>
        <source>Clears input (0) or output (1) buffer</source>
        <translation type="vanished">Löscht den Eingabe- (0) oder Ausgabe- (1) Puffer</translation>
    </message>
    <message>
        <source>Clears the input or output buffer of serial port</source>
        <translation type="vanished">Löscht den Eingabe- oder Ausgabepuffer der seriellen Schnittstelle</translation>
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
        <source>Try to open device directly failed!</source>
        <translation type="vanished">Der Versuch, das Gerät direkt zu öffnen, schlug fehl!</translation>
    </message>
    <message>
        <source>Endpoint index for reading operations. The used index is LIBUSB_ENDPOINT_IN + endpoint_read, with LIBUSB_ENDPOINT_IN = %1 (default: initialization parameter &apos;endpoint&apos;)</source>
        <translation type="obsolete">Endpoint-Index für Leseoperationen. Der benutzte Index entspricht LIBUSB_ENDPOINT_IN + endpoint_read, mit LIBUSB_ENDPOINT_IN = %1(Standard: Inizialisierungsparameter &apos;endpoint&apos;)</translation>
    </message>
    <message>
        <source>Endpoint index for writing operations. The used index is LIBUSB_ENDPOINT_OUT + endpoint_write, with LIBUSB_ENDPOINT_OUT = %1  (default: initialization parameter &apos;endpoint&apos;)</source>
        <translation type="obsolete">Endpoint-Index für Schreiboperationen. Der benutzte Index entspricht LIBUSB_ENDPOINT_OUT + endpoint_write, mit LIBUSB_ENDPOINT_OUT = %1(Standard: Inizialisierungsparameter &apos;endpoint&apos;)</translation>
    </message>
    <message>
        <source>could not find a known device - please specify type and/or vid:pid and/or bus,dev</source>
        <translation type="vanished">Das Gerät wurde nicht gefunden. Bitte den Typ, &apos;vid:pid&apos; und/oder den Bus spezifizieren</translation>
    </message>
    <message>
        <source>no device found to open</source>
        <translation type="obsolete">Kein Gerät zum Öffnen vorhanden</translation>
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
        <source>Number of written characters differ from designated size</source>
        <translation type="obsolete">Die Anzahl der übergebenen Zeichen entspricht nicht der Vorgabe</translation>
    </message>
</context>
<context>
    <name>ItomUSBDeviceInterface</name>
    <message>
        <source>itom-plugin for a usb port communication</source>
        <translation type="vanished">itom-Plugin für eine USB-Port Kommunikation</translation>
    </message>
    <message>
        <source>The vendor id of the device to connect to</source>
        <translation type="obsolete">Die Vendor-ID des Geräts für die Verbindung</translation>
    </message>
    <message>
        <source>The product id of the device to connect to</source>
        <translation type="obsolete">Die Produkt-ID des Geräts für die Verbindung</translation>
    </message>
    <message>
        <source>The endpoint to communicate with.</source>
        <translation type="obsolete">Der &apos;Endpoint&apos; für die Kommunikation.</translation>
    </message>
    <message>
        <source>Timeout for reading commands in [s]</source>
        <translation type="vanished">Timeout für Lesebefehle in [s]</translation>
    </message>
    <message>
        <source>Debug level: 0 (LIBUSB_LOG_LEVEL_NONE): no messages ever printed by the library. 1 (ERROR): error messages are printed to stderr, 2 (WARNING): warning and error messages are printed to stderr, 3 (INFO): informational messages are printed to stdout, warning and error messages are printed to stderr, 4 (DEBUG): like 3 but debug messages are also printed to stdout.</source>
        <translation type="obsolete">Debug-Level: 0 (LIBUSB_LOG_LEVEL_NONE): Keine Meldungen der Library. 1 (ERROR): Fehlermeldung wird an &apos;stderr&apos; übergeben, 2 (WARNING): Warnungen und Fehlermeldung werden an &apos;stderr&apos; übergeben, 3 (INFO): Informationen werden an &apos;stdout&apos;, Warnungen und Fehlermeldung an &apos;stderr&apos; übergeben, 4 (DEBUG): Wie 3, nur dass Debug-Meldungen ebenfalls an &apos;stdout&apos; übergeben werden.</translation>
    </message>
    <message>
        <source>If true, all information about connected devices is print to the console.</source>
        <translation type="obsolete">Wenn aktiviert, werden alle Informationen über das verbundene Gerät über die Konsole ausgegeben.</translation>
    </message>
    <message>
        <source>Initialised &apos;debug&apos;-parameter with given value. If debug-param is true, all out and inputs are written to dockingWidget</source>
        <translation type="obsolete">Initialisiert den &apos;Debug&apos;-Parameter mit dem übergebenen Wert. Ist dieser &apos;True&apos;, wird die gesammte Ein- und Ausgabekommunikation im DockWidget protokolliert</translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location line="-10"/>
        <source>licensed under LGPL</source>
        <translation></translation>
    </message>
    <message>
        <source>This plugin can be used for raw / lowlevel comminication with USB-devices</source>
        <translation type="obsolete">Dieses Plugin kann nicht für Raw-/Lowlevel-Kommunikation mit USB-Geräten genutzt werden</translation>
    </message>
    <message>
        <source>Mandatory paramers are NULL</source>
        <translation type="obsolete">Pflichtparameter ist NULL</translation>
    </message>
    <message>
        <source>Optional paramers are NULL</source>
        <translation type="obsolete">Optionaler Parameter ist NULL</translation>
    </message>
</context>
<context>
    <name>ito::AddInAlgo</name>
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
</TS>
