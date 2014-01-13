<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.0" language="de">
<context>
    <name>OpenCVFilters</name>
    <message>
        <location filename="../OpenCVFilters.cpp" line="+128"/>
        <source>Input image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Output image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <location line="+367"/>
        <location line="+171"/>
        <location line="+242"/>
        <location line="+466"/>
        <source>Error: source image ptr empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1241"/>
        <location line="+367"/>
        <location line="+172"/>
        <location line="+241"/>
        <location line="+466"/>
        <source>Error: dest image ptr empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1207"/>
        <source>Error: pointer of input and output objects are equal</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>Error: the check command is currently not implemented for more than 3 dims</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+45"/>
        <location line="+273"/>
        <location line="+413"/>
        <source>Error: source is not a matrix or image stack</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-367"/>
        <source>Error: anchor should be &apos;int8&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-194"/>
        <location line="+17"/>
        <location line="+204"/>
        <location line="+375"/>
        <location line="+479"/>
        <location line="+202"/>
        <source>%1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1037"/>
        <source>Unknown or unexpected CV-Datatype recived.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-149"/>
        <source>All types except complex64 and complex128 are accepted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-250"/>
        <source>input data object of type uint8, uint16, int16, float32, float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>output image with the same type and size than input (inplace allowed)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>structuring element used for the morpholocial operation (default: None, a 3x3 rectangular structuring element is used). Else: An uint8 data object where values &gt; 0 are considered for the operation.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>position of the anchor within the element. If not given or if (-1,-1), the anchor is at the element center [default].</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>number of times the morpholocial operation is applied [default: 1]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>This string defines how the filter should hande pixels at the border of the matrix. Allowed is CONSTANT [default], REPLICATE, REFLECT, WRAP, REFLECT_101. In case of a constant border, only pixels inside of the element mask are considered (morphologyDefaultBorderValue)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>source and destination object must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+161"/>
        <source>erosion with (y,x) kernel(%1, %2), anchor(%3, %4), %5 iterations, borderType %6</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>dilation with (y,x) kernel(%1, %2), anchor(%3, %4), %5 iterations, borderType %6</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+60"/>
        <source>Empty object handle. Image will be of src-type</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Kernelsize for x-axis</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Kernelsize for y-axis</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Position of the kernel anchor, see openCV-Help</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>border mode used to extrapolate pixels outside of the image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+102"/>
        <source>Error: anchor has wrong size or number of dims</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+32"/>
        <source>OpenCV blur-filter with (y,x) kernel(%1, %2), anchor(%3, %4), borderType %5</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+36"/>
        <source>Must be 8bit</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>dp: Inverse ratio of the accumulator resolution to the image resolution.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Minimum center distance of the circles.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>The higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>The accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Min Radius in x/y</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Max Radius in x/y</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <source>Error: source is not an image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+93"/>
        <source>Input Object handle, must be a single plane</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+116"/>
        <source>Image of type Integer or float32</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Empty dataObject-hanlde. Destination is of source type</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Kernelsize in x/y</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+39"/>
        <location line="+584"/>
        <source>Error: kernel must be odd</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-579"/>
        <source>Error: kernelsize &gt; 3 and object is not uint8</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <source>OpenCV medianblur-filter with kernel size = %1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Unknown or unexpected CV-Datatype received.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+519"/>
        <source>sourceObject must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>destinationObject must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>sourceObject is not a matrix or image stack</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+192"/>
        <source>Spike removal filter with kernel(%1, %1) and range ]%2, %3[</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-314"/>
        <source>Error: nDim-stacks not supported yet</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+67"/>
        <source>Flipped left/rigth with cvFlip-Filter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+0"/>
        <source>Flipped upside/down with cvFlip-Filter</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location line="-1396"/>
        <source>Wrapped algorithms from OpenCV</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>LGPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>N.A.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInActuator</name>
    <message>
        <location filename="../../../../build/itom/SDK/include/common/addInInterface.cpp" line="+687"/>
        <source>Constructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Destructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInAlgo</name>
    <message>
        <location line="+92"/>
        <source>Constructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../../../../build/itom/SDK/include/common/addInInterface.h" line="+986"/>
        <source>uninitialized vector for mandatory parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>uninitialized vector for optional parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>uninitialized vector for output parameters!</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInBase</name>
    <message>
        <location filename="../../../../build/itom/SDK/include/common/addInInterface.cpp" line="-577"/>
        <source>function execution unused in this plugin</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+29"/>
        <source>Toolbox</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+147"/>
        <source>Your plugin is supposed to have a configuration dialog, but you did not implement the showConfDialog-method</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>ito::AddInDataIO</name>
    <message>
        <location line="+13"/>
        <source>Constructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Destructor must be overwritten</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>listener does not have a slot </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>this object already has been registered as listener</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <source>timer could not be set</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+25"/>
        <source>the object could not been removed from the listener list</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+94"/>
        <location line="+20"/>
        <location line="+18"/>
        <location line="+18"/>
        <location line="+18"/>
        <location line="+18"/>
        <location line="+18"/>
        <source>not implemented</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
