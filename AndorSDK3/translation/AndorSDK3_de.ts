<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>AndorSDK3</name>
    <message>
        <location filename="../AndorSDK3.cpp" line="+64"/>
        <source>Exposure time of chip (in seconds).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Gain (normalized value 0..1)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Pixelsize in x (cols)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Pixelsize in y (rows)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>Bitdepth of each pixel</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-17"/>
        <source>Horizontal and vertical binning, depending on camera ability. 104 means a 1x binning in horizontal and 4x binning in vertical direction. (only symmetric binning is allowed; if read only binning is not supported)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <source>ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>acquisition timeout in secs</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>frame rate in Hz</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>camera interface type</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>firmware version</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>model name of camera</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>name of camera</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>serial number of camera</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Andor SDK3 version</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>camera trigger (Internal, Software, External, External Start, External Exposure)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>fan speed (Off, Low, On - not all values are available for every camera)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>pixel readout rate in MHz (&apos;100 MHz&apos;, &apos;200 MHz&apos;, &apos;280 MHz&apos;, &apos;550 MHz&apos;; not all options are available for all cameras)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>0: rolling shutter (for highest frame rate, best noise performance, default), 1: global shutter (for pulsed, fast moving images)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>indicates if full AOI control is available (usually yes, for some Neo cameras it isn&apos;t and you can only apply certain ROI sizes (see camera manual))</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>time to readout data from the sensor in the current configuration (0.0 if not implemented)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>current temperature of sensor in °C (inf if not implemented)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>state of the sensor cooling. Cooling is disabled (0) by default at power up and must be enabled (1) for the camera to achieve its target temperature.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+699"/>
        <source>StopDevice of AndorSDK3 can not be executed, since camera has not been started.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <source>Acquire of AndorSDK3 can not be executed, since camera has not been started.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>no valid camera memory has been allocated</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+94"/>
        <source>data object of getVal is NULL or cast failed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+40"/>
        <source>Empty object handle retrieved from caller</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+705"/>
        <source>Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or a uninitilized image.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>AndorSDK3Interface</name>
    <message>
        <location filename="../AndorSDK3Interface.cpp" line="+66"/>
        <source>This plugin supports Andor cameras that can be run using the SDK3 from Andor (e.g. Neo and Zyla series). It has been tested with the following models:

- Zyla 5.5 (Dual Camera Link)

The plugin has been compiled using the Andor SDK 3.8

In order to run your camera, please purchase and install the Andor SDK 3.8 or higher and make sure that the necessary libraries are accessible
by the Windows path environment variable (e.g. append &quot;C:/Program Files/Andor SDK3&quot; to the path variable).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>camera index that should be opened. The first camera is 0, the second 1...</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>DialogAndorSDK3</name>
    <message>
        <location filename="../DialogAndorSDK3.ui" line="+20"/>
        <source>Andor SDK3</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>Buffer</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Bits per Pixel</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>Binning (horizontal, vertical)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>Data Acquisition</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Rolling Shutter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Global Shutter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Timeout</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+26"/>
        <location line="+26"/>
        <source> s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-13"/>
        <source>Gain</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+20"/>
        <source>Integration Time</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Shutter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Trigger</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>Fan Speed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+19"/>
        <source>General Settings</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Sensor Cooling</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Pixel Readout Rate</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>Size</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>[full AOI control]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>x0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+35"/>
        <source>x1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>y0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>y1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>x-size</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+19"/>
        <location line="+29"/>
        <source> px</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-16"/>
        <source>y-size</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+42"/>
        <source>full ROI</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>DialogSDK3</name>
    <message>
        <location filename="../DialogAndorSDK3.cpp" line="+50"/>
        <source>Configuration Dialog</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>DockWidgetAndorSDK3</name>
    <message>
        <location filename="../DockWidgetAndorSDK3.ui" line="+14"/>
        <source>Form</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <source>Camera Data</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+21"/>
        <source>[Name]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Serial:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Camera Model:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>Camera Name:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>[Model]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>[Serial]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>Image Properties</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <source>Width:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <location line="+20"/>
        <location line="+20"/>
        <source>-</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-27"/>
        <source>Height:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+20"/>
        <source>Bit depth:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>Camera Properties</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Integration Time:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <source> s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>Gain:</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location filename="../AndorSDK3Interface.cpp" line="-30"/>
        <source>Andor cameras via its SDK3 (Neo, Zyla).</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
