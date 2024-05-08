<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DialogVistek</name>
    <message>
        <location filename="../dialogVistek.cpp" line="+55"/>
        <source>Configuration Dialog</source>
        <translation>Konfigurationsdialog</translation>
    </message>
</context>
<context>
    <name>DockWidgetVistek</name>
    <message>
        <location filename="../dockWidgetVistek.ui" line="+14"/>
        <source>Form</source>
        <translation></translation>
    </message>
    <message>
        <source>General Information</source>
        <translation type="obsolete">Allgemeine Informationen</translation>
    </message>
    <message>
        <location line="+24"/>
        <source>Camera Data</source>
        <translation>Kameradaten</translation>
    </message>
    <message>
        <location line="+27"/>
        <source>Manufacturer:</source>
        <translation>Hersteller:</translation>
    </message>
    <message>
        <location line="+20"/>
        <source>Camera Model:</source>
        <translation>Kameramodell:</translation>
    </message>
    <message>
        <location line="-13"/>
        <source>[Manufacturer]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+20"/>
        <source>[Model]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Camera Serial Nr.:</source>
        <translation>Seriennummer:</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>[Serial Number]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Camera IP:</source>
        <translation>IP-Adresse:</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>[IP]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>Image Properties</source>
        <translation>Bildeigenschaften</translation>
    </message>
    <message>
        <location line="+109"/>
        <source> ms</source>
        <translation></translation>
    </message>
    <message>
        <source>Width [px]:</source>
        <translation type="obsolete">Breite [px]:</translation>
    </message>
    <message>
        <source>Height [px]:</source>
        <translation type="obsolete">Höhe [px]:</translation>
    </message>
    <message>
        <location line="-22"/>
        <source>Camera Properties</source>
        <translation>Kameraeigenschaften</translation>
    </message>
    <message>
        <source>Exposure (s):</source>
        <translation type="obsolete">Belichtung (s):</translation>
    </message>
    <message>
        <location line="-57"/>
        <source>Width:</source>
        <translation>Breite:</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Height:</source>
        <translation>Höhe:</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Bit depth:</source>
        <translation>Bit-Tiefe:</translation>
    </message>
    <message>
        <location line="+7"/>
        <location line="+7"/>
        <location line="+7"/>
        <source>-</source>
        <translation></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>Exposure:</source>
        <translation>Belichtung:</translation>
    </message>
    <message>
        <location line="+35"/>
        <source>Gain:</source>
        <translation></translation>
    </message>
    <message>
        <location line="+16"/>
        <source> dB</source>
        <translation></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Offset:</source>
        <translation></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location filename="../VistekInterface.cpp" line="+90"/>
        <source>SVS Vistek GigE grabber.</source>
        <translation>SVS Vistek GigE Grabber.</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>itom plugin for GigE cameras from SVS Vistek. Every camera is simply initialized by the serial number of the connected SVS Vistek camera. (see camera housing).

Some files of the SVGigE SDK are shipped within this plugin (currently 1.5.2). Please check the SVS Vistek website for newer versions of the SDK and replace the files if desired. Additionally, it is stated that SVS Vistek does not provide any support for this specific plugin wrapping the official SDK of SVS Vistek.

This plugin requires the necessary libraries from the SVS Vistek SDK (SVGigE.dll, SVGigETLFilter.dll, SVGigETLWinsock.dll or 64bit versions). Please check the right version and make these libraries available for itom (PATH environment variable, system directory...).

For a robust data communication please install the SVGigE FilterDriver and enable Jumbo frames at your network adapter.

Please notice: Currently, this plugin only works for Vistek drivers up to version 1.5.2. If you want to use a 2.x series of the Vistek drivers, use the GenICam plugin of itom that is able to communicate with Vistek USB3 and GigE cameras.</source>
        <translation type="unfinished">itom-Plugin für GigE-Kameras von SVS Vistek. Jede Kamera wird mit Hilfe deren Seriennummer (siehe Gehäuse der Kamera) initialisiert.

Einige Dateien des SVGigE-SDKs werden mit diesem Plugin mitgeliefert (aktuell 1.5.2). Bitte die SVS Vistek-Webseite auf neueren SDK-Versionen prüfen und ggf. die vorhandenen Dateien ersetzen. Für dieses spezielle Plugin bietet SVS Vistek jedoch keinen Support.

Dieses Plugin benötigt von SVS Vistek-SDK-Bibliothek folgende Dateien: SVGigE.dll, SVGigETLFilter.dll, SVGigETLWinsock.dll oder 64bit Versionen). Bitte auf die richtige Version überprüfen und diese Bibliotheken itom zur Verfügung stellen (PATH-Umgebungsvariable, Systemverzeichnis...).

Für eine stabile Datenkommunikation bitte den SVGigE-Filtertreiber installieren und im Netzwerkadapter die Option &apos;Jumbo frames&apos; aktivieren.

Achtung: Die aktuelle Version läuft nur mit Vistek-Treibern bis einschließlich Version 1.5.2 oder älter. Für die Nutzung der 2.x-Versionen der Vistek-Treiber bitte das GenICam-Plugin von itom nutzen. Dieses kann mit Vistek USB3 und GigE-Kameras kommunizieren.</translation>
    </message>
    <message>
        <location line="+19"/>
        <source>licensed under LGPL, the necessary Vistek drivers, header files and libraries have their own license.</source>
        <translation></translation>
    </message>
    <message>
        <source>SVS Vistek GigE camera grabber.</source>
        <translation type="obsolete">SVS Vistek GigE Kamera-Grabber.</translation>
    </message>
    <message>
        <source>Serial Number of the SVS Vistek camera (see camera housing)</source>
        <translation type="obsolete">Seriennummer der SVS Vistek-Kamera (siehe Kameragehäuse)</translation>
    </message>
</context>
<context>
    <name>Vistek</name>
    <message>
        <location filename="../Vistek.cpp" line="+106"/>
        <source>Camera Model ID</source>
        <translation>Kameramodell ID</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Camera manufacturer</source>
        <translation>Kamerahersteller</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Camera firmware version</source>
        <translation>Kamera Firmware-Version</translation>
    </message>
    <message>
        <location line="+8"/>
        <source>IP address of the camera</source>
        <translation>IP-Adresse der Kamera</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Camera Number</source>
        <translation>Kameranummer</translation>
    </message>
    <message>
        <source>Exposure time in [s]</source>
        <translation type="vanished">Belichtungszeit in [s]</translation>
    </message>
    <message>
        <location line="+10"/>
        <source>Gain [0..18 dB]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Binning mode (OFF = 0 [default], HORIZONTAL = 1 (or 102), VERTICAL = 2 (or 201),  2x2 = 3 (or 202), 3x3 = 4 (or 303), 4x4 = 5 (or 404)</source>
        <translation type="unfinished">Binning-Modus (Aus = 0 [standard], Horizontal = 1 (oder 102), Vertikal = 2 (oder 201),  2x2 = 3 (oder 202), 3x3 = 4 (oder 303), 4x4 = 5 (oder 404))</translation>
    </message>
    <message>
        <location line="+23"/>
        <source>Used streaming packet size (in bytes, more than 1500 usually only possible if you enable jumbo-frames at your network adapter)</source>
        <translation type="unfinished">Genutzte Streaming-Paketgröße (in Bytes; mehr als 1500 normalerweise nur möglich, wenn bei der Netzwerkkarte die Einstellung &apos;Jumbo-Frames&apos; aktiviert wurde)</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Log level. The logfile is Vistek_SVGigE.log in the current directory. 0 - logging off (default),  1 - CRITICAL errors that prevent from further operation, 2 - ERRORs that prevent from proper functioning, 3 - WARNINGs which usually do not affect proper work, 4 - INFO for listing camera communication (default), 5 - DIAGNOSTICS for investigating image callbacks, 6 - DEBUG for receiving multiple parameters for image callbacks, 7 - DETAIL for receiving multiple signals for each image callback</source>
        <translation type="unfinished">Protokollstufe: Die Protokolldatei heißt Vistek_SVGigE.log und befindet sich im aktuellen Verzeichnis; Stufen: 0 - deaktiviert (standard); 1 - KRITISCHe Fehler, die ein Weiterbetreiben verhindern; 2 - FEHLER, die ein ordnungsgemäßes Funktionieren verhindern; 3 - WARNUNGen, die in der Regel keine Auswirkungen auf die Funktion haben; 4 - INFO zur Auflistung der Kamerakommunikation; 5 - DIAGNOSE zur Untersuchung von Bild-Callbacks; 6 - DEBUG zum Empfangen mehrerer Parameter für Bild-Callbacks; 7 - DETAILs zum Empfangen mehrerer Signale für jeden Bild-Callback</translation>
    </message>
    <message>
        <location line="+629"/>
        <source>acquisition not possible, since Vistek camera has not been started.</source>
        <translation type="unfinished">Zugriff ist nicht möglich, da die Vistek-Kamere nicht gestartet wurde.</translation>
    </message>
    <message>
        <location line="+92"/>
        <source>getVal of Vistek can not be executed, since no image has been acquired.</source>
        <translation type="unfinished">getVal von Vistek kann nicht ausgeführt werden, da kein Bild aufgenommen wurde.</translation>
    </message>
    <message>
        <source>Binning of different pixel</source>
        <translation type="obsolete">Binning der unterschiedlichen Pixel</translation>
    </message>
    <message>
        <location line="-745"/>
        <source>Width of current camera frame</source>
        <translation>Breite des aktuellen Kamerabilds</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Height of current camera frame</source>
        <translation>Höhe des aktuellen Kamerabilds</translation>
    </message>
    <message>
        <source>Grabdepth for camera buffer</source>
        <translation type="obsolete">Farbtiefe des Kamerapuffers</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Time in ms since last image (end of exposure)</source>
        <translation>Zeit in ms seit dem letzten Bild (Ende der Belichtung)</translation>
    </message>
    <message>
        <location line="-44"/>
        <source>Serial number of the camera (see camera housing)</source>
        <translation>Seriennummer der Kamera (siehe Kameragehäuse)</translation>
    </message>
    <message>
        <location line="+12"/>
        <source>Exposure time in [s] (deprecated: use integration_time instead; this is an alias for integration_time only)</source>
        <translation type="unfinished">Belichtungszeit in [s] (veraltet! Stattdessen integration_time verwenden; dies ist ein Alias nur für integration_time)</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Exposure time in [s].</source>
        <translation type="unfinished">Belichtungszeit in [s].</translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Offset [0.0..1.0]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>bit-depth for camera buffer</source>
        <translation type="unfinished">Bit-Tiefe für Kamera-Buffer</translation>
    </message>
    <message>
        <location line="+128"/>
        <source>set log level</source>
        <translation type="unfinished">Protokollstufe setzen</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>set exposure time</source>
        <translation type="unfinished">Belichtungszeit setzeh</translation>
    </message>
    <message>
        <location line="+11"/>
        <source>set gain</source>
        <translation type="unfinished">Gain setzen</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>set offset</source>
        <translation type="unfinished">Offset setzen</translation>
    </message>
    <message>
        <location line="+109"/>
        <source>binning invalid: Accepted values are OFF = 0 [default], HORIZONTAL = 1 (or 102), VERTICAL = 2 (or 201),  2x2 = 3 (or 202), 3x3 = 4 (or 303), 4x4 = 5 (or 404)</source>
        <translation type="unfinished">Binning-Wert ungültig! Mögliche Werte sind: AUS = 0 [standard], HORIZONTAL = 1 (oder 102), VERTIKAL = 2 (oder 201),  2x2 = 3 (oder 202), 3x3 = 4 (oder 303), 4x4 = 5 (oder 404)</translation>
    </message>
    <message>
        <location line="-77"/>
        <location line="+25"/>
        <location line="+62"/>
        <location line="+59"/>
        <location line="+279"/>
        <source>stop camera</source>
        <translation type="unfinished">Kamera stoppen</translation>
    </message>
    <message>
        <location line="-334"/>
        <source>set binning</source>
        <translation type="unfinished">Binning setzen</translation>
    </message>
    <message>
        <location line="-82"/>
        <location line="+25"/>
        <location line="+62"/>
        <location line="+59"/>
        <source>restart camera 1</source>
        <translation type="unfinished">Neustart Kamera 1</translation>
    </message>
    <message>
        <location line="-331"/>
        <source>ROI (x0, y0, width, height)</source>
        <translation type="unfinished">ROI (x0, y0, Breite, Höhe)</translation>
    </message>
    <message>
        <location line="+180"/>
        <location line="+25"/>
        <source>set region of interest</source>
        <translation type="unfinished">Region of Interest (ROI) setzen</translation>
    </message>
    <message>
        <location line="-19"/>
        <location line="+25"/>
        <location line="+62"/>
        <location line="+59"/>
        <source>restart camera 2</source>
        <translation type="unfinished">Neustart Kamera 2</translation>
    </message>
    <message>
        <location line="-45"/>
        <source>8bit not supported by this camera</source>
        <translation type="unfinished">8bit wird von dieser Kamera nicht untersützt</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>10bit not supported by this camera</source>
        <translation type="unfinished">10bit wird von dieser Kamera nicht untersützt</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>10bit not supported by this driver version</source>
        <translation type="unfinished">10bit wird von dieser Treiberversion nicht unterstützt</translation>
    </message>
    <message>
        <location line="+6"/>
        <source>12bit not supported by this camera</source>
        <translation type="unfinished">12bit wird von dieser Kamera nicht untersützt</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>16bit not supported by this camera</source>
        <translation type="unfinished">16bit wird von dieser Kamera nicht untersützt</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>unknown bpp value (use 8bit, 10bit, 12bit or 16bit)</source>
        <translation type="unfinished">Unbekanntes bpp-Format (bitte 8bit, 10bit, 12bit oder 16bit nutzen)</translation>
    </message>
    <message>
        <location line="+12"/>
        <source>set pixel depth</source>
        <translation type="unfinished">Pixeltiefe setzen</translation>
    </message>
    <message>
        <location line="+90"/>
        <source>No free camera found</source>
        <translation type="unfinished">Keine freie Kamera gefunden</translation>
    </message>
    <message>
        <location line="+99"/>
        <source>%s: Vistek DLL error %i &apos;%s&apos; occurred</source>
        <translation type="unfinished">%s: Vistek-DLL-Fehler %i</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>%s: unknown Vistek DLL error %i occurred</source>
        <translation type="unfinished">%s: Unbekannter Vistek-DLL-Fehler %i</translation>
    </message>
    <message>
        <location line="+25"/>
        <source>streaming server not started</source>
        <translation type="unfinished">Der Streaming-Server ist nicht gestartet</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>set software trigger and start 1</source>
        <translation type="unfinished">Software-Trigger setzen und 1 starten</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>set software trigger and start 2</source>
        <translation type="unfinished">Software-Trigger setzen und 2 starten</translation>
    </message>
    <message>
        <location line="+49"/>
        <source>StopDevice of Vistek can not be executed, since camera has not been started.</source>
        <translation type="unfinished">StopDevice der Vistek kann nicht ausgeführt werden, da die Kamera nicht gestartet wurde.</translation>
    </message>
    <message>
        <location line="+54"/>
        <source>Camera trigger failed.</source>
        <translation type="unfinished">Kamera-Trigger fehlgeschlagen.</translation>
    </message>
    <message>
        <location line="+25"/>
        <source>Frame not completed within given timeout.</source>
        <translation type="unfinished">In der Angegebenen Zeit wurde der Frame nicht vervollständigt.</translation>
    </message>
    <message>
        <location line="+44"/>
        <source>getVal of Vistek can not be executed, since camera has not been started.</source>
        <translation type="unfinished">getVal von Vistek kann nicht ausgeführt werden, da die Kamera nicht gestartet wurde.</translation>
    </message>
    <message>
        <location line="+46"/>
        <source>copy buffer during getVal of Vistek can not be executed, since no DataType unknown.</source>
        <translation type="unfinished">Während der Ausführung von getVal kann der Buffer nicht kopiert werden, da zu diesem Zeitpunkt kein Datentyp definiert ist.</translation>
    </message>
    <message>
        <location line="+8"/>
        <source>invalid image data</source>
        <translation type="unfinished">ungültige Bilddaten</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>timeout while retrieving image</source>
        <translation type="unfinished">Zeitüberschreitung beim Abruf eines Bilds</translation>
    </message>
    <message>
        <location line="+7"/>
        <source>new image available, transfer was successful</source>
        <translation type="unfinished">Neues Bild verfügbar. Die Übermittlung war erfolgreich</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>an image could not be completed in time and was therefore abandoned</source>
        <translation type="unfinished">Ein Bild wurde nicht rechtzeitig fertig und wurde deshalb verworfen</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>end of exposure is currently mapped to transfer started</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>available network bandwidth has been exceeded</source>
        <translation type="unfinished">Die verfügbare Bandbreite wurde überschritten</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>driver problem due to old-style driver behavior (prior to 2003, not WDM driver)</source>
        <translation type="unfinished">Treiberproblem aufgrund eines zu alten Treibers (vor 2003, kein WDM-Treiber)</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>a test packet arrived</source>
        <translation type="unfinished">Ein Testpaket wurde empfangen</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>the camera has finished an image transfer</source>
        <translation type="unfinished">Die Kamera hat eine Bildübertragung beendet</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>connection to camera got lost</source>
        <translation type="unfinished">Die Verbindung zur Kamera wurde unterbrochen</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>an exceptional situation occurred during a multicast transmission</source>
        <translation type="unfinished">Während der Multicast-Übertragung hat sich ein Exception ereignet</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>a frame could not be properly completed</source>
        <translation type="unfinished">Ein Frame ist nicht vollständig</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>a next entry was put into the message FIFO before the old one was released</source>
        <translation type="unfinished">Bevor das alte Bild abgeholt wurde, wurde bereits ein neues Bild aufgenommen</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>the camera has finished a shutter sequence</source>
        <translation type="unfinished">Die Kamera hat eine Shutter-Sequenz beendet</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>the camera detected a trigger violation</source>
        <translation type="unfinished">Die Kamera hat ein Trigger-Problem erkannt</translation>
    </message>
    <message>
        <location line="+1"/>
        <source>any error occurred</source>
        <translation type="unfinished">Ein Fehler hat sich ereignet</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>connection to camera lost</source>
        <translation type="unfinished">Die Verbindung zur Kamera wurde unterbrochen</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>error while retrieving image: %i</source>
        <translation type="unfinished">Fehler beim Abruf von Bild %i</translation>
    </message>
    <message>
        <location line="+36"/>
        <source>data object of getVal is NULL or cast failed</source>
        <translation type="unfinished">DataObject von getVal ist NULL oder der Cast schlug fehl</translation>
    </message>
    <message>
        <location line="+40"/>
        <source>Empty object handle retrieved from caller</source>
        <translation type="unfinished">Es wurde ein leeres Objekt-Handle zurückgegeben</translation>
    </message>
    <message>
        <location line="+56"/>
        <source>Requested camera could not be selected.</source>
        <translation type="unfinished">Kamera kann nicht ausgewählt werden.</translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Requested camera is occupied by another application</source>
        <translation type="unfinished">Kamera wird von einem anderen Programm verwendet</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Enforcing valid network settings failed</source>
        <translation type="unfinished">Ungültige Netzwerkeinstellungen</translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Selected camera could not be opened.</source>
        <translation type="unfinished">Kamera kann nicht geöffnet werden.</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>Camera does not support software triggers. This camera cannot be used by this plugin.</source>
        <translation type="unfinished">Die Kamera unterstützt keinen Software-Trigger und kann deshalb nicht mit diesem Plugin betrieben werden.</translation>
    </message>
    <message>
        <location line="+147"/>
        <location line="+2"/>
        <source>Error getting image size</source>
        <translation type="unfinished">Bei der Ermittlung der Bildmaße ist ein -Fehler aufgetreten</translation>
    </message>
    <message>
        <location line="+55"/>
        <source>given pixeltype not supported. Supported is only MONO8, MONO12, MONO12_PACKED and MONO16</source>
        <translation type="unfinished">Der übergebene Pixeltyp wird nicht unterstützt. Mögliche Typen sind MONO8, MONO12, MONO12_PACKED und MONO16</translation>
    </message>
    <message>
        <location line="+16"/>
        <source>maximal packet size determination</source>
        <translation type="unfinished">Überschreitung der maximalen Paketgröße</translation>
    </message>
    <message>
        <location line="+16"/>
        <source>set streaming packet size</source>
        <translation type="unfinished">Streaming-Paketgröße setzen</translation>
    </message>
    <message>
        <location line="+21"/>
        <source>memory allocation error when creating stream with %i buffers. Retry and use less buffers</source>
        <translation type="unfinished">Speicherbelegungsfehler beim erstellen des Streams mit %i Buffer. Bitte mit kleinerem Buffer erneut versuchen</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Streaming channel creation failed</source>
        <translation type="unfinished">Die Erstellung des Streamig-Kanals schlug fehl</translation>
    </message>
    <message>
        <location line="+15"/>
        <source>Event creation failed</source>
        <translation type="unfinished">Fehler beim Auslesen von Kamera-Meldungen</translation>
    </message>
    <message>
        <location line="+26"/>
        <source>Message callback registration failed</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>VistekContainer</name>
    <message>
        <source>SVS Vistek: dll version mismatch.</source>
        <translation type="obsolete">SVS Vistek: DLL Versionsunterschied.</translation>
    </message>
    <message>
        <location filename="../VistekContainer.cpp" line="+115"/>
        <source>SVS Vistek: dll version mismatch, got: %1, expected: %2 (64bit).</source>
        <translation type="unfinished">SVS Vistek: Diskrepanz der DLL-Version, verwendet: %1, erwartet: %2 (64bit).</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>SVS Vistek: dll version mismatch, got: %1, expected: %2.</source>
        <translation type="unfinished">SVS Vistek: Diskrepanz der DLL-Version, verwendet: %1, erwartet: %2.</translation>
    </message>
    <message>
        <location line="+20"/>
        <source>Connecting via winsock failed.</source>
        <translation>Verbindung per Winsock fehlgeschlagen.</translation>
    </message>
    <message>
        <source>Camera discovery failed.</source>
        <translation type="obsolete">Kameraerkennung fehlgeschlagen.</translation>
    </message>
    <message>
        <location line="+23"/>
        <source>No cameras detected.</source>
        <translation>Keine Kamera gefunden.</translation>
    </message>
</context>
<context>
    <name>VistekInterface</name>
    <message>
        <location filename="../VistekInterface.cpp" line="+6"/>
        <source>Serial Number of the SVS Vistek camera (see camera housing)</source>
        <translation type="unfinished">Seriennummer der SVS Vistek-Kamera (siehe Kameragehäuse)</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>used streaming packet size (-1: use maximal available packet size, else value in bytes). Try to enable jumbo-frames at your network adapter in order to realize higher packet sizes</source>
        <translation type="unfinished">verwendete Streaming-Paketgröße (-1: maximal verfügbare Paketgröße, sonst Angabe in Bytes). Um größere Paketgrößen zu nutzen sollte bei der Netzwerkkarte die Option Jumbo-Frames aktiviert werden</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>number of streaming buffers. Increase if you get data losses, decrease if you want to consume less memory</source>
        <translation type="unfinished">Anzahl der Streaming Buffer. Bei Datenverlusten erhöhen, um Verbrauch von Arbeitsspeicher zu senken den Wert verringern</translation>
    </message>
</context>
<context>
    <name>dialogVistek</name>
    <message>
        <location filename="../dialogVistek.ui" line="+14"/>
        <source>Dialog</source>
        <translation></translation>
    </message>
    <message>
        <source>Cancel</source>
        <translation type="vanished">Abbrechen</translation>
    </message>
    <message>
        <location line="+124"/>
        <source>8</source>
        <translation></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>12</source>
        <translation></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>16</source>
        <translation></translation>
    </message>
    <message>
        <location line="+70"/>
        <source>Gain</source>
        <translation></translation>
    </message>
    <message>
        <location line="+39"/>
        <source>Offset</source>
        <translation></translation>
    </message>
    <message>
        <source># Frames</source>
        <translation type="obsolete">Anzahl Bilder</translation>
    </message>
    <message>
        <source>Bit per pixel</source>
        <translation type="obsolete">Bit pro Pixel</translation>
    </message>
    <message>
        <source>Size X</source>
        <translation type="obsolete">Größe X</translation>
    </message>
    <message>
        <source>Size Y</source>
        <translation type="obsolete">Größe Y</translation>
    </message>
    <message>
        <source>Mode</source>
        <translation type="obsolete">Modus</translation>
    </message>
    <message>
        <source>Signal width</source>
        <translation type="obsolete">Signalbreite</translation>
    </message>
    <message>
        <source>plain</source>
        <translation type="obsolete">Ebene</translation>
    </message>
    <message>
        <source>step</source>
        <translation type="obsolete">Stufen</translation>
    </message>
    <message>
        <source>sinus</source>
        <translation type="obsolete">Sinus</translation>
    </message>
    <message>
        <source>Featwidth</source>
        <translation type="obsolete">Breite</translation>
    </message>
    <message>
        <source>Noise</source>
        <translation type="obsolete">Rauschen</translation>
    </message>
    <message>
        <source>Featheight</source>
        <translation type="obsolete">Höhe</translation>
    </message>
    <message>
        <source>Step / Lambda</source>
        <translation type="obsolete">Schritt / Lambda</translation>
    </message>
    <message>
        <source>Initial phase (phi0)</source>
        <translation type="obsolete">Startphase (phi0)</translation>
    </message>
    <message>
        <source>µm / Step</source>
        <translation type="obsolete">µm / Schritt</translation>
    </message>
    <message>
        <source>Signal width (FWHM)</source>
        <translation type="obsolete">Signalbreite (FWHM)</translation>
    </message>
    <message>
        <location line="-235"/>
        <source>Information</source>
        <translation type="unfinished">Informationen</translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Model</source>
        <translation></translation>
    </message>
    <message>
        <location line="+7"/>
        <location line="+14"/>
        <location line="+14"/>
        <location line="+14"/>
        <location line="+14"/>
        <location line="+14"/>
        <source>-</source>
        <translation></translation>
    </message>
    <message>
        <location line="-63"/>
        <source>Serial No</source>
        <translation>Seriennummer</translation>
    </message>
    <message>
        <location line="+14"/>
        <source>Camera IP</source>
        <translation type="unfinished">Kamera-IP</translation>
    </message>
    <message>
        <location line="+14"/>
        <source>Manufacturer</source>
        <translation type="unfinished">Hersteller</translation>
    </message>
    <message>
        <location line="+14"/>
        <source>Width</source>
        <translation type="unfinished">Breite</translation>
    </message>
    <message>
        <location line="+14"/>
        <source>Height</source>
        <translation type="unfinished">Höhe</translation>
    </message>
    <message>
        <location line="+20"/>
        <source>Buffer and Binning</source>
        <translation type="unfinished">Buffer und Binning</translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Bit Depth</source>
        <translation type="unfinished">Bit-Tiefe</translation>
    </message>
    <message>
        <location line="+32"/>
        <source>Binning</source>
        <translation></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>Off (0)</source>
        <translation type="unfinished">Aus (0)</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Horizontal (1)</source>
        <translation></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Vertical (2)</source>
        <translation>Vertikal (2)</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>2x2 (3)</source>
        <translation></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>3x3 (4)</source>
        <translation></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>4x4 (5)</source>
        <translation></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>Integration</source>
        <translation type="unfinished">Integration</translation>
    </message>
    <message>
        <location line="+16"/>
        <source>%</source>
        <translation></translation>
    </message>
    <message>
        <location line="+36"/>
        <source>Exposure</source>
        <translation type="unfinished">Belichtung</translation>
    </message>
    <message>
        <location line="+13"/>
        <source> ms</source>
        <translation></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>Size</source>
        <translation type="unfinished">Größe</translation>
    </message>
    <message>
        <location line="+8"/>
        <source>x0</source>
        <translation></translation>
    </message>
    <message>
        <location line="+35"/>
        <source>x1</source>
        <translation></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>y0</source>
        <translation></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>y1</source>
        <translation></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>x-size</source>
        <translation type="unfinished">x-Größe</translation>
    </message>
    <message>
        <location line="+19"/>
        <location line="+29"/>
        <source> px</source>
        <translation></translation>
    </message>
    <message>
        <location line="-16"/>
        <source>y-size</source>
        <translation type="unfinished">y-Größe</translation>
    </message>
    <message>
        <location line="+42"/>
        <source>full ROI</source>
        <translation type="unfinished">Alles</translation>
    </message>
    <message>
        <location line="-186"/>
        <source> dB</source>
        <translation></translation>
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
        <translation type="obsolete">Nicht initialisierter Vektor für Pflichtparameter!</translation>
    </message>
    <message>
        <source>uninitialized vector for optional parameters!</source>
        <translation type="obsolete">Nicht initialisierter Vektor für optionale Parameter!</translation>
    </message>
    <message>
        <source>uninitialized vector for output parameters!</source>
        <translation type="obsolete">Nicht initialisierter Vektor fürAusgabeparameter!</translation>
    </message>
</context>
<context>
    <name>ito::AddInBase</name>
    <message>
        <source>function execution unused in this plugin</source>
        <translation type="obsolete">Die Funktion &apos;execution&apos; wurde in diesem Plugin nicht bentzt</translation>
    </message>
    <message>
        <source>Your plugin is supposed to have a configuration dialog, but you did not implement the showConfDialog-method</source>
        <translation type="obsolete">Das Plugin scheint einen Konfigurationsdialog zu besitzen, doch die Methode showConfDialog wurde nicht implementiert</translation>
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
        <source>timer could not be set</source>
        <translation type="obsolete">Timer konnte nicht gesendet werden</translation>
    </message>
    <message>
        <source>not implemented</source>
        <translation type="obsolete">Nicht implementiert</translation>
    </message>
</context>
</TS>
