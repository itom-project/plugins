<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DslrRemote</name>
    <message>
        <source>not yet implemented</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Minimum time between the start of two consecutive acquisitions [s], default: 0.0.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Minimum integration time for an acquisition [s], default: 0.0 (as fast as possible).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Virtual gain</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Virtual offset</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Binning of different pixel, binning = x-factor * 100 + y-factor</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>size in x (cols) [px]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>size in y (rows) [px]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>bitdepth of images</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>No camera auto detected
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>stopDevice of DslrRemote can not be executed, since camera has not been started.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>libpghoto error acquiring image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>wrong bit depth</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Acquire of DslrRemote can not be executed, since camera has not been started.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>data object of getVal is NULL or cast failed</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Empty object handle retrieved from caller</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>image could not be obtained since no image has been acquired.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>time out while waiting for acquire picture to terminate</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>received unexpected event from camera</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>DslrRemoteInterface</name>
    <message>
        <source>Width of virtual sensor chip</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Height of virtual sensor chip, please set this value to 1 (line camera) or a value dividable by 4 for a 2D camera.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Bits per Pixel, usually 8-16bit grayvalues</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <source>A virtual white noise grabber</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The DummyGrabber is a virtual camera which emulates a camera with white noise.

The camera is initialized with a maximum width and height of the simulated camera chip (both need to be a multiple of 4). The noise is always scaled in the range between 0 and the current bitdepth (bpp - bit per pixel). The real size of the camera image is controlled using the parameter &apos;roi&apos; if the sizes stay within the limits given by the size of the camera chip.

You can initialize this camera either as a 2D sensor with a width and height &gt;= 4 or as line camera whose height is equal to 1.

This plugin can also be used as template for other grabber.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
