<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>AvantesAvaSpec</name>
    <message>
        <location filename="../avantesAvaSpec.cpp" line="+194"/>
        <source>ROI (x,y,width,height)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>current width of ROI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>current height</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Number of averages for every frame</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Some detectors have dark pixels, that can be used for a dark detection. If enabled, the output
dataObject will always be float32. Static (1) subtracts the mean value of all dark pixels from all values.
Dynamic (2) is only available for some devices (see if dyn. dark correction is enabled in the software
AvaSpec) and subtracts different mean values for odd and even pixels. Off (0): default.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Coefficients for polynom that determines lambda_table (lambda_table[idx] = c[0] + c[1]*idx + c[2]*idx^2 + ... + c[4]*idx^4)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Vector with the wavelength of all active pixels</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-32"/>
        <source>Integration time of CCD programmed in s, some devices do not accept the full range of allowed values (see AvaSpec for real minimum value of your device).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>Bit depth. The output object is float32 for all cases but uint16 only if no averaging is enabled and the dark_correction is disabled or no dark correction pixels are available for this sensor.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+20"/>
        <source>Serial number of spectrometer. Same as identifier.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Name of the detector.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+576"/>
        <source>Tried to acquire without starting device</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+246"/>
        <source>Wrong picture type</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+54"/>
        <source>Error during check data, external dataObject invalid. Object has more or less than 1 plane. It must be of right size and type or an uninitilized image.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+44"/>
        <source>Empty object handle retrieved from caller</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>AvantesAvaSpecInterface</name>
    <message>
        <location line="-1006"/>
        <source>VendorID of spectrometer, 0x1992 for AvaSpec-3648, 0x471 for AvaSpec-ULS3648...</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>ProductID of spectrometer, 0x0667 for spectrometer</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Indicates if the device is a USB3 device</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>DockWidgetAvantesAvaSpec</name>
    <message>
        <location filename="../dockWidgetAvantesAvaSpec.ui" line="+14"/>
        <source>Form</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+19"/>
        <source>General Information</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+18"/>
        <source>ID:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>[ID]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>ROI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>Acquisition</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>Integrationtime</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>ms</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>Average</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>Dark correction</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location filename="../avantesAvaSpec.cpp" line="-25"/>
        <source>Avantes AvaSpec</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>This DLL integrates the Avantis AvantesAvaSpec spectrometer series into itom. It uses a low-level libusb connection to communicate with the devices and has been tested with the following spectrometers: * AvaSpec 3468 USB-Spectrometer * AvaSpec 2048 USB-Spectrometer * AvaSpec-ULS2048CL-EVO USB3-Spectrometer.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>dialogAS5216</name>
    <message>
        <location filename="../dialogAvantesAvaSpec.ui" line="+14"/>
        <source>Dialog</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>Camera Settings</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Format</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>Mode</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>Rate</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+225"/>
        <source>Cancel</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-205"/>
        <source>Integration</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Offset</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+26"/>
        <source>Gain</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+56"/>
        <source>Size</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>X0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+36"/>
        <source>Size X</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+39"/>
        <source>Set X Max</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+23"/>
        <source>OK</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
