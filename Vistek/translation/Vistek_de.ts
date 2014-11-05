<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.0" language="de">
<context>
    <name>DialogVistek</name>
    <message>
        <location filename="../dialogVistek.cpp" line="+50"/>
        <source>Configuration Dialog</source>
        <translation>Konfigurationsdialog</translation>
    </message>
    <message>
        <location line="+177"/>
        <source>error</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+0"/>
        <source>Error while setting parameters (%1)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>warning</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+0"/>
        <source>Warning while setting parameters (%1)</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>DockWidgetVistek</name>
    <message>
        <location filename="../dockWidgetVistek.ui"/>
        <source>Form</source>
        <translation></translation>
    </message>
    <message>
        <source>General Information</source>
        <translation type="obsolete">Allgemeine Informationen</translation>
    </message>
    <message>
        <location/>
        <source>Camera Data</source>
        <translation>Kameradaten</translation>
    </message>
    <message>
        <location/>
        <source>Manufacturer:</source>
        <translation>Hersteller:</translation>
    </message>
    <message>
        <location/>
        <source>Camera Model:</source>
        <translation>Kameramodell:</translation>
    </message>
    <message>
        <location/>
        <source>[Manufacturer]</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>[Model]</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Camera Serial Nr.:</source>
        <translation>Seriennummer:</translation>
    </message>
    <message>
        <location/>
        <source>[Serial Number]</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Camera IP:</source>
        <translation>IP-Adresse:</translation>
    </message>
    <message>
        <location/>
        <source>[IP]</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Image Properties</source>
        <translation>Bildeigenschaften</translation>
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
        <location/>
        <source>Camera Properties</source>
        <translation>Kameraeigenschaften</translation>
    </message>
    <message>
        <source>Exposure (s):</source>
        <translation type="obsolete">Belichtung (s):</translation>
    </message>
    <message>
        <location/>
        <source>Width:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Height:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Bit depth:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>-</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Exposure:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source> s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Gain:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source> dB</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Offset:</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location filename="../VistekInterface.cpp" line="+89"/>
        <source>SVS Vistek GigE grabber.</source>
        <translation>SVS Vistek GigE Grabber.</translation>
    </message>
    <message>
        <location line="+21"/>
        <source>licensed under LGPL, the necessary Vistek drivers, header files and libraries have their own license.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>N.A.</source>
        <translation type="unfinished"></translation>
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
        <location filename="../Vistek.cpp" line="+120"/>
        <source>Camera Model ID</source>
        <translation>Kameramodell ID</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Camera manufacturer</source>
        <translation>Kamerahersteller</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Camera firmware version</source>
        <translation>Kamera Firmware-Version</translation>
    </message>
    <message>
        <location line="+4"/>
        <source>IP adress of the camera</source>
        <translation>IP-Adresse der Kamera</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Camera Number</source>
        <translation>Kameranummer</translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Exposure time in [s]</source>
        <translation>Belichtungszeit in [s]</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Gain [0..18 dB]</source>
        <translation></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>Used streaming packet size (in bytes, more than 1500 usually only possible if you enable jumbo-frames at your network adapter)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Log level. The logfile is Vistek_SVGigE.log in the current directory. 0 - logging off (default),  1 - CRITICAL errors that prevent from further operation, 2 - ERRORs that prevent from proper functioning, 3 - WARNINGs which usually do not affect proper work, 4 - INFO for listing camera communication (default), 5 - DIAGNOSTICS for investigating image callbacks, 6 - DEBUG for receiving multiple parameters for image callbacks, 7 - DETAIL for receiving multiple signals for each image callback</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+539"/>
        <source>acquisition not possible, since Vistek camera has not been started.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+58"/>
        <source>getVal of Vistek can not be executed, since no image has been acquired.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Binning of different pixel</source>
        <translation type="obsolete">Binning der unterschiedlichen Pixel</translation>
    </message>
    <message>
        <location line="-610"/>
        <source>Width of current camera frame</source>
        <translation>Breite des aktuellen Kamerabilds</translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Height of current camera frame</source>
        <translation>Höhe des aktuellen Kamerabilds</translation>
    </message>
    <message>
        <source>Grabdepth for camera buffer</source>
        <translation type="obsolete">Farbtiefe des Kamerapuffers</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Time in ms since last image (end of exposure)</source>
        <translation>Zeit in ms seit dem letzten Bild (Ende der Belichtung)</translation>
    </message>
    <message>
        <location line="-23"/>
        <source>Serial number of the camera (see camera housing)</source>
        <translation>Seriennummer der Kamera (siehe Kameragehäuse)</translation>
    </message>
    <message>
        <location line="+11"/>
        <source>Offset [0.0..1.0]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Binning mode (OFF = 0 [default], HORIZONTAL = 1, VERTICAL = 2,  2x2 = 3, 3x3 = 4, 4x4 = 5</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>bit-depth for camera buffer</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+322"/>
        <source>No free camera found</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+190"/>
        <source>StopDevice of Vistek can not be executed, since camera has not been started.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+90"/>
        <source>getVal of Vistek can not be executed, since camera has not been started.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+41"/>
        <source>copy buffer during getVal of Vistek can not be executed, since no DataType unknown.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+74"/>
        <source>data object of getVal is NULL or cast failed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+41"/>
        <source>Empty object handle retrieved from caller</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+56"/>
        <source>Requested camera could not be selected.</source>
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
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>SVS Vistek: dll version mismatch, got: %1, expected: %2.</source>
        <translation type="unfinished"></translation>
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
        <location filename="../VistekInterface.cpp" line="+5"/>
        <source>Serial Number of the SVS Vistek camera (see camera housing)</source>
        <translation type="unfinished">Seriennummer der SVS Vistek-Kamera (siehe Kameragehäuse)</translation>
    </message>
    <message>
        <location line="+5"/>
        <source>used streaming packet size (-1: use maximal available packet size, else value in bytes). Try to enable jumbo-frames at your network adapter in order to realize higher packet sizes</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>number of streaming buffers. Increase if you get data losses, decrease if you want to consume less memory</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>dialogVistek</name>
    <message>
        <location filename="../dialogVistek.ui"/>
        <source>Dialog</source>
        <translation></translation>
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
        <location/>
        <source>8</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>12</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>16</source>
        <translation></translation>
    </message>
    <message>
        <location/>
        <source>Gain</source>
        <translation></translation>
    </message>
    <message>
        <location/>
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
    <message utf8="true">
        <source>µm / Step</source>
        <translation type="obsolete">µm / Schritt</translation>
    </message>
    <message>
        <source>Signal width (FWHM)</source>
        <translation type="obsolete">Signalbreite (FWHM)</translation>
    </message>
    <message>
        <location/>
        <source>Information</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Model</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>-</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Serial No</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Camera IP</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Manufacturer</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Width</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Height</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Buffer and Binning</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Bit Depth</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Binning</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Off (0)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Horizontal (1)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Vertical (2)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>2x2 (3)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>3x3 (4)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>4x4 (5)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Integration</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>%</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Exposure</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>ms</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source> dB</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location/>
        <source>Apply</source>
        <translation type="unfinished"></translation>
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
