<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DockWidgetLibUSB</name>
    <message>
        <location filename="../dockWidgetLibUSB.ui" line="+20"/>
        <source>Form</source>
        <translation></translation>
    </message>
    <message>
        <location line="+24"/>
        <source>General Information</source>
        <translation>Allgemeine Informationen</translation>
    </message>
    <message>
        <location line="+6"/>
        <source>ID:</source>
        <translation>ID:</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>[ID]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>USB Log</source>
        <translation>USB-Protokoll</translation>
    </message>
    <message>
        <location line="+27"/>
        <source>ASCII</source>
        <translation></translation>
    </message>
    <message>
        <location line="+16"/>
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
        <location line="+11"/>
        <source>ignore empty messages</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Clear</source>
        <translation>Löschen</translation>
    </message>
</context>
<context>
    <name>ItomUSBDevice</name>
    <message>
        <location filename="../ItomLibUSB.cpp" line="+139"/>
        <source>Timeout for reading commands in [s]</source>
        <translation>Timeout für Lesebefehle in [s]</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>If true, all out and inputs are written to dockingWidget</source>
        <translation>Wenn aktiviert, wird die Kommunikation im dockingWidget protokolliert</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Endpoint index for reading operations. The used index is LIBUSB_ENDPOINT_IN + endpoint_read, with LIBUSB_ENDPOINT_IN = %1 (default: initialization parameter &apos;endpoint&apos;)</source>
        <translation type="unfinished">Endpoint-Index für Leseoperationen. Der benutzte Index entspricht LIBUSB_ENDPOINT_IN + endpoint_read, mit LIBUSB_ENDPOINT_IN = %1(Standard: Inizialisierungsparameter &apos;endpoint&apos;)</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Endpoint index for writing operations. The used index is LIBUSB_ENDPOINT_OUT + endpoint_write, with LIBUSB_ENDPOINT_OUT = %1  (default: initialization parameter &apos;endpoint&apos;)</source>
        <translation type="unfinished">Endpoint-Index für Schreiboperationen. Der benutzte Index entspricht LIBUSB_ENDPOINT_OUT + endpoint_write, mit LIBUSB_ENDPOINT_OUT = %1(Standard: Inizialisierungsparameter &apos;endpoint&apos;)</translation>
    </message>
    <message>
        <location line="+295"/>
        <source>could not find a known device - please specify type and/or vid:pid and/or bus,dev</source>
        <translation>Das Gerät wurde nicht gefunden. Bitte den Typ, &apos;vid:pid&apos; und/oder den Bus spezifizieren</translation>
    </message>
    <message>
        <location line="+25"/>
        <source>no of the %1 devices that fit to the vendor and product ID can be opened since they are already in use.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>no device found to open</source>
        <translation type="unfinished">Kein Gerät zum Öffnen vorhanden</translation>
    </message>
    <message>
        <location line="+179"/>
        <source>StartDevice not necessary</source>
        <translation>&quot;StartDevice&quot; nicht nötig</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>StopDevice not necessary</source>
        <translation>&quot;StopDevice&quot; nicht nötig</translation>
    </message>
    <message>
        <location line="+14"/>
        <source>Acquire not necessary</source>
        <translation>&quot;Acquire&quot; nicht nötig</translation>
    </message>
    <message>
        <location line="+61"/>
        <source>Number of written characters differ from designated size</source>
        <translation type="unfinished">Die Anzahl der übergebenen Zeichen entspricht nicht der Vorgabe</translation>
    </message>
</context>
<context>
    <name>ItomUSBDeviceInterface</name>
    <message>
        <location line="-672"/>
        <source>itom-plugin for a usb port communication</source>
        <translation>itom-Plugin für eine USB-Port Kommunikation</translation>
    </message>
    <message>
        <location line="+11"/>
        <source>LibUSB is a itom-Plugin which gives direct/raw access to a device connected to the serial port.
It can be used by plugins for communication analog to the serial port.
The plugin is implemented for Windows, but Linux should be possible due to libUSB is also available on Linux.

To connect to a device you need the vendor id and the product id.

The setVal and getVal functions will write and read on the specified endpoint.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>The vendor id of the device to connect to</source>
        <translation type="unfinished">Die Vendor-ID des Geräts für die Verbindung</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>The product id of the device to connect to</source>
        <translation type="unfinished">Die Produkt-ID des Geräts für die Verbindung</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>The endpoint to communicate with.</source>
        <translation type="unfinished">Der &apos;Endpoint&apos; für die Kommunikation.</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Timeout for reading commands in [s]</source>
        <translation>Timeout für Lesebefehle in [s]</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Debug level: 0 (LIBUSB_LOG_LEVEL_NONE): no messages ever printed by the library. 1 (ERROR): error messages are printed to stderr, 2 (WARNING): warning and error messages are printed to stderr, 3 (INFO): informational messages are printed to stdout, warning and error messages are printed to stderr, 4 (DEBUG): like 3 but debug messages are also printed to stdout.</source>
        <translation type="unfinished">Debug-Level: 0 (LIBUSB_LOG_LEVEL_NONE): Keine Meldungen der Library. 1 (ERROR): Fehlermeldung wird an &apos;stderr&apos; übergeben, 2 (WARNING): Warnungen und Fehlermeldung werden an &apos;stderr&apos; übergeben, 3 (INFO): Informationen werden an &apos;stdout&apos;, Warnungen und Fehlermeldung an &apos;stderr&apos; übergeben, 4 (DEBUG): Wie 3, nur dass Debug-Meldungen ebenfalls an &apos;stdout&apos; übergeben werden.</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>If true, all information about connected devices is print to the console.</source>
        <translation type="unfinished">Wenn aktiviert, werden alle Informationen über das verbundene Gerät über die Konsole ausgegeben.</translation>
    </message>
</context>
</TS>
