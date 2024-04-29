<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>CVBInterface</name>
    <message>
        <location filename="../commonVisionBloxInterface.cpp" line="+67"/>
        <source>This plugin can connect to various cameras via the GenICam interface of the commercial tool Common Vision Blox from company Stemmer.

 Until now, the plugin is only implemented for monochrome pixel formats mono8, mono10, mono12, mono14 and mono16. Besides the ROI and
 exposure time, all parameters need to be read and set using the parameter raw:suffix where suffix is the real GenICam parameter, obtained
 via the Stemmer configuration tool. If a bitdepth &gt; 8 bit is chosen, an error might occur during acquisition. Then check the indicated ini file
 from Stemmer GenICam and don&apos;t set the pixelFormat property to auto but Mono16.

 In case of a slow connection, check the communication center of Stemmer for hints or bugs in the connection, e.g. use the filter driver for GigE connections.

 This plugin has been tested with DALSA Genie HM1400 and Xenics Bobcat 640 GigE.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>Licensed under LGPL, Stemmer Common Vision Blox under its own license.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>If 1 scan for new cameras, else take the last opened camera (default). If you scan for new cameras, the configuration file (ini) created in CommonVisionBlox for GenICam or other cameras will be reset to the default values.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>desired monochromatic bitdepth. Dependent on this parameter PixelFormat is set to mono8, mono10, mono12, mono14 or mono16. Make sure the bitdepth is supported by your camera</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>CommonVisionBlox</name>
    <message>
        <location filename="../commonVisionBlox.cpp" line="+53"/>
        <source>Exposure time of chip (in seconds).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Heartbeat timeout of GigE Vision Transport Layer.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>&apos;snap&apos; is a single image acquisition (only possible in trigger_mode &apos;off&apos;), &apos;grab&apos; is a continuous acquisition</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>&apos;off&apos;: camera is not explicitly triggered but operated in freerun mode. The next acquired image is provided upon acquire, &apos;software&apos; sets trigger mode to On and fires a software trigger at acquire (only possible in acquisition_mode &apos;grab&apos;).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Pixelsize in x (cols)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Pixelsize in y (rows)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>bit depth</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>left end of the ROI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>right end of the ROI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>upper end of the ROI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>downer end of the ROI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>use raw:paramname to set internal paramname of camera to value. paramname is the original GenICam parameter name.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>vendor name</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>model name</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+51"/>
        <source>Error loading %s driver! Probably no cameras were found during discovery</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Error loading %s driver! Probably no cameras are configured / reachable!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>camera does not support the Grab2 interface from Common Vision Blox</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>camera does not support the Grabber interface from Common Vision Blox</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+18"/>
        <source>chosen bitdepth is not supported by this camera.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+42"/>
        <source>unsupported pixel format %s (supported is Mono8 and Mono16</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+54"/>
        <source>no node &apos;ExposureTime&apos; or &apos;ExposureTimeAbs&apos; available</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+142"/>
        <source>%s is no node</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+95"/>
        <source>unsupported property type (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+78"/>
        <source>you need to indiciate a suffix for the node you want to set</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+31"/>
        <source>invalid index</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+47"/>
        <source>trigger_mode &apos;software&apos; can only be set in acquisition_mode &apos;grab&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+21"/>
        <source>acquisition_mode &apos;snap&apos; can only be set in trigger_mode &apos;off&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+92"/>
        <source>camera device is not initialised. start device failed.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+56"/>
        <source>StopDevice of CommonVisionBlox can not be executed, since camera has not been started.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <source>Acquire of CommonVisionBlox can not be executed, since camera has not been started.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+67"/>
        <source>Obtained image has %i bits per pixel instead of %i given by the camera. Both values must be equal. Change property PixelFormat in %s of Common Vision Blox or use the configuration tool to adjust the CVB Color Format</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>currently unsupported image format</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <source>currently unsupported bit depth of image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+83"/>
        <source>data object of getVal is NULL or cast failed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+40"/>
        <source>Empty object handle retrieved from caller</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+33"/>
        <source>image could not be obtained since no image has been acquired.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+175"/>
        <location line="+32"/>
        <location line="+33"/>
        <location line="+26"/>
        <location line="+27"/>
        <location line="+26"/>
        <location line="+27"/>
        <location line="+68"/>
        <location line="+41"/>
        <location line="+45"/>
        <source>node map not avaible</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-8"/>
        <source>node %s is no enumeration</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+49"/>
        <source>failure while scanning for cameras</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>%s%sAn unspecified error occurred for which no further information is available. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sA function was called with an invalid parameter value for one of the function&apos;s arguments. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn error occurred during a File I/O operation (e.g. while loading/saving data from/to a file). (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sA timeout occurred in an asynchronous function call. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe allocation of a block of memory failed, typically because the amount of memory available to the application was insufficient for the operation at hand. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sA plane index was specified that was either negative or bigger than or equal to the number of planes in the image. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe data type of the input image is not supported by the function that has been called. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn attempt to switch to an invalid/unavailable camera port was made. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn attempt to switch to an invalid/unavailable board was made. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn attempt to select and invalid/unsupported trigger mode was made. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn attempt to read a property was unsuccessful, either because the property does not exist or because the device or property is in a state that prevents the property from being read. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn attempt to write a property was unsuccessful, either because the property does not exist or because the device or property is in a state that prevents the property from being written. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn invalid/unavailable port was selected. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sA port read operation failed. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sA port write operation failed. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe handle to at least one of the input images does not point to a valid image. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe input image object does not support the interface required for this operation. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe operation failed because the hardware is not in a state where it can handle that operation. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe requested feature is not supported. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe current grab operation was aborted. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAt least one of the input handles does not point to a valid pixel list. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe amount of input data is insufficient for the requested operation. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe object handle that was passed to the function does not point to a valid non linear transformation object. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe function only works on images with a linear VPAT layout. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sA division by zero was attempted. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn invalid number of planes was specified. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn invalid/undefined color model mode has been passed to the function. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn invalid data type descriptor has been passed to the function. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAt least one of the input handles points to the wrong type of object. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe device or object is not ready to handle the requested operation. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe input handle does not point to a valid angle object. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe input handle does not point to a valid 2D vector object. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe input handle does not point to a valid 2D line object. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe operation failed because the vector passed to this function has length zero. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe function call failed because the input vectors to this operation are identical. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe operation failed because the line object passed to it is vertical. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe operation failed because the line object passed to it is horizontal. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe argument cannot be calculated for a vector of length zero. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe operation failed because the input line 2D object has not yet been defined  (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe operation failed because the the line objects provided to the function do not intersect  (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sNo clipping points available. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sNot enough lines available to calculate the intersection reliably. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn input value was too big or did lead to a too big result. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe input handle does not point to a valid 2D circle object. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sA feature access failed. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe requested operation failed because the selected feature is not present. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sThe requested feature is not supported (may happen if a specific interface implementation does not implement all functionality). (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn index value in an indexed access exceeded its limits. (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sAn image object that is expected to have overlay bits does not have overlay bits (see #DT_Overlay). (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>%s%sCommon Vision Blox Error %i</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location filename="../commonVisionBloxInterface.cpp" line="-37"/>
        <source>GenICam cameras via Common Vision Blox from Stemmer</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
