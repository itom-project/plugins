<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DataObjectArithmetic</name>
    <message>
        <location filename="../dataobjectarithmetic.cpp" line="+112"/>
        <location line="+13"/>
        <location line="+3"/>
        <location line="+16"/>
        <location line="+166"/>
        <location line="+121"/>
        <source>source image data object for operation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-317"/>
        <location line="+17"/>
        <location line="+17"/>
        <source>result of calculation. This param can be int or double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-2"/>
        <location line="+166"/>
        <location line="+123"/>
        <source>Ignore invalid-Values for floating point</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-286"/>
        <location line="+168"/>
        <location line="+4"/>
        <source>Index of the plane, which contains the result.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-171"/>
        <location line="+168"/>
        <location line="+4"/>
        <source>Pixelindex in y-direction.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-171"/>
        <location line="+168"/>
        <location line="+4"/>
        <source>Pixelindex in x-direction.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-159"/>
        <source>1. source image data object for operation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>2. source image data object for operation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>0 if both data objects are not equal, else 1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+82"/>
        <location line="+86"/>
        <location line="+69"/>
        <source>Error: source image is NUL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+431"/>
        <location line="+186"/>
        <location line="+917"/>
        <source>2D source image data object (u)int8, (u)int16, int32, float32 or float64 only.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1101"/>
        <source>values &lt; lowThreshold are ignored. lowThreshold is subtracted from each valid value before COG determination.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <location line="+193"/>
        <source>values &gt; highThreshold are ignored.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-188"/>
        <source>y-Coordinate of COG (physical unit)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>x-Coordinate of COG (physical unit)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <location line="+513"/>
        <location line="+587"/>
        <source>Error: sourceImage is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1096"/>
        <location line="+513"/>
        <location line="+588"/>
        <source>Error: sourceImage is not initialized</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-939"/>
        <source>Mx3 or Mx4 2D data object of type uint16, each row corresponds to one spot. The line contains [px_x, px_y, circle_diameter] if the cog should be determined within a circle or [px_x, px_y, width, height] if the cog should be determined within a rectangle. circle_diameter, width or height have to be odd.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>resulting Mx3 data object of type float64 with the sub-pixel precise position of the spots (all is given in pixel coordinates, never physical coordinates). Each row is [subpix_x, subpix_y, nr_of_valid_elements_within_search_mask] or [px_x, px_y, 0 | 1] if the spot only contained one or no valid values.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>values &lt; lowThreshold are ignored. lowThreshold is subtracted from each valid value before COG determination. if lowThreshold is NaN (default), the lowest value within each spot search area is taken as local minimum value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+303"/>
        <source>source image data (2D or 3D) object for operation (u)int8, (u)int16, int32, float32 or float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>destination image data object for operation, will contain evaluated COG (in physical coordinates)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>destination image data object for operation, will contain maximal intensity</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>if (max-min) along the search direction is lower or equal this pvThreshold (peak-to-valley), no cog is determined and a NaN value is set into the resulting position array (default: this threshold is not considered).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>If != 0.0, values &lt;= (max+min)*dynamicThreshold will be ignored. To only consider values above the FWHM, set this value to 0.5 (default).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>values &lt;= lowerThreshold will not be considered for the cog calculation (default: this threshold is not considered).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>The search direction is along each column if 1, else along each row (default, 0)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+35"/>
        <source>Error: destCOG image is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Error: destIntensity image is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>Error: destCOG and destIntensity must not be the same data objects.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+144"/>
        <source>Center of gravity can only be calculated for (u)int8, (u)int16, (u)int32, float32 or float64 data objects.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+249"/>
        <source>valid non-complex data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>percentage value [0.0, 100.0]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>threshold value (NaN if data object was empty or only contained invalid values)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>input data is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+31"/>
        <source>not implemented for complex64 or complex128</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+108"/>
        <source>only values &gt;= lowThreshold are considered for the ROI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>if given, only values &lt;= highThreshold are considered for the ROI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>ROI of bounding box [x0,y0,width,height]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+27"/>
        <source>Error: source image must have one plane</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1750"/>
        <location line="+61"/>
        <location line="+90"/>
        <source>data type not supported</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+63"/>
        <source>mean result</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>deviation result</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-243"/>
        <location line="+60"/>
        <location line="+86"/>
        <location line="+69"/>
        <location line="+59"/>
        <source>Error, object dimensions must be unequal zero</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-156"/>
        <source>Switch complex handling, 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Minimal value, this parameter be int or double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Maximum value, this parameter. This param can be int or double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+113"/>
        <source>Toggles the calculation mode of standard deviation over N or N-1 elements</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-242"/>
        <location line="+268"/>
        <location line="+111"/>
        <location line="+7"/>
        <source>Error: source image is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+232"/>
        <source>type not supported</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+35"/>
        <source>y-Coordinate of COG (index)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>x-Coordinate of COG (index)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+37"/>
        <source>Error: source image must not have multiple planes</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+33"/>
        <location line="+1092"/>
        <source>Unknown type or type not implemented for phase shifting evaluation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../autofocus.cpp" line="+172"/>
        <source>2D or 3D source image data object (u)int8, (u)int16, int32 only.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>method used to determine the autofocus.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>auto focus measure values for every plane in the source image.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>DataObjectArithmeticInterface</name>
    <message>
        <location filename="../dataobjectarithmetic.cpp" line="-1923"/>
        <source>Arithmetic algorithms filters.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location line="-12"/>
        <source>Operations and arithmetic calculations of dataObject.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>This plugin provides several arithmetic calculations for dataObject. These are for instance: 
- min- or maximum value
- centroid along dimensions or inplane 

This plugin does not have any unusual dependencies.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>LGPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+96"/>
        <source>This filter calculated the minimal value and its first location within the dataObject. 

The result value will be Integer vor all integer types or Double for all floating point types

The filter is implemented for all data types besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+60"/>
        <source>This filter calculated the maximal value and its first location within the dataObject. 

The result value will be Integer vor all integer types or Double for all floating point types

The filter is implemented for all data types besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+61"/>
        <source>This filter calculated the minimal and maximal value and its first location within the dataObject. 

The result value will be Integer vor all integer types or Double for all floating point types

The filter do not work with RGBA32 but with all other data-types

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+94"/>
        <source>This filter calculated the mean value within the dataObject. 

The return value containing the mean value of the dataObject.

The filter is implemented for all data types besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+55"/>
        <source>The filter returns the arithmetic mean and the standard deviation of the given dataObject within its ROI.
The optinal flag to toggles if (flag==0) the deviation is calculated by 1/(n-1)*sqrt(sum(x-xm)^2)
or (flag ==1) by 1/(n)*sqrt(sum(x-xm)^2)

The filter is implemented for all data types besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>Check pixel-wise wether two dataObjects are equal. 
The filter returns 1 if both objects are pixel-wise equal, else returns 0.

The filter is implemented for all data types besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+334"/>
        <source>This filter calculates the center of gravity of a 2D real image. 

The return value contains the column and row position in pixel and physical coordinates.

For the determination, only values in the range [lowThreshold, highThreshold] are considered. The COG algorithm requires, that all values 
that do not belong to the required peak have values around zero. In order to achieve this, the &apos;lowThreshold&apos; value is subtracted from each 
valid intensity value before calculating the COG with the following equations: 

cXI = \frac{\sum{idx_x * (I - lowThreshold)}}{\sum{(I - lowThreshold)} 
cYI = \frac{\sum{idx_y * (I - lowThreshold)}}{\sum{(I - lowThreshold)} 

The filter does not work with RGBA32, Complex64 and Complex128, but with all other data-types.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+180"/>
        <source>This filter determines the sub-pixel 
spot position of multiple spots in an image. The pixel-precise spot position must be given including the size 
of the area around the coarse spot position over which the center of gravity algorithm is applied. 

The area can either be a rectangle (width and height, odd values) or a circle (odd diameter). 

The COG is calculated by the following algorithm: 

cXI = \frac{\sum{idx_x * (I - lowThreshold)}}{\sum{(I - lowThreshold)} 
cYI = \frac{\sum{idx_y * (I - lowThreshold)}}{\sum{(I - lowThreshold)} 

The lowThreshold can either be given or (if it is NaN), the minimum value of each area will be taken as local lower threshold. 
Only values &lt;= highThreshold are considered, set highThreshold to NaN or Inf in order to do not consider this constraint. 

Usually, the resulting &apos;centroids&apos; object contains the sub-pixel x and y position as well as the number of valid pixels in each row. 
If no or only one valid pixel has been encountered, the coarse pixel x and y position as well as 0 or 1 (for no or one valid pixel) is returned. 

If the coarse spot position lies outside of the image, the resulting row in &apos;centroids&apos; contains NaN coordinates. 
Please consider, that all input and output coordinates are assumed to be pixel values, the scaling and offset of the image are not considered.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+307"/>
        <source>Calculate center of gravity for each plane along the x- or y-direction. 

This methods creates the two given data objects &apos;destCOG&apos; and &apos;destIntensity&apos; in the following way: 

- destCOG, ito::float64, sizes: [nrOfPlanes x sizeOfElements], contains the sub-pixel wise one-dimensional coordinate of the center of gravity (in physical coordinates) or NaN if it could not be determined. 
- destIntensity, same type than input object, sizes: [nrOfPlanes x sizeOfElements], contains the absolute maximum along the search direction. 

If the center of gravity should be calculated along each row of each plane inside of the given &apos;sourceStack&apos; data object, the parameter &apos;columnWise&apos; must be 0, for a column-wise 
calculation is must be set to 1. Along each search direction, the corresponding minimum and maximum value is determined and the center of gravity is determined using: 

cog = \frac{\sum{idx * (I - lowerBoundary)}}{\sum{(I - lowerBoundary)} 

A value *I* is only valid and considered in the equation above if: 

- (max - min) &gt; pvThreshold (peak-to-valley threshold, if not given, destCOG contains NaN at this position) 
- I &gt; lowerThreshold (only checked if lowerThreshold &gt; minimum possible value of the given data type) 
- I &gt; (max + min) * dynamicThreshold (only checked if dynamicThreshold &gt; 0.0) 

The value &apos;lowerBoundary&apos; is set to the corresponding maximum of &apos;lowerThreshold&apos; and &apos;dynamicThreshold&apos; if one of those is checked; else the given data is considered that the values 
all drop to zero at the edge of each search range; for a valid cog determination, it is necessary to assume that all values that are far away from the cog position have values around zero; 
if this is not the case consider to set an appropriate value &apos;lowerThreshold&apos; and / or &apos;dynamicThreshold&apos;. 

The filter is not implemented for complex data types and the type rgba32.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+469"/>
        <source>analyzes all values in the given data object and returns the value, which is at a given percentage in the sorted value list.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+150"/>
        <source>This filter calculates the minimum ROI that contains all values within a lower and optional upper threshold. 

The return value contains the [x0,y0,width,height] of the minimum ROI.

Values of the data object belong to the ROI if they are &gt;= lowThreshold and &lt;= highThreshold. 
The highThreshold is only checked, if it is different than the default value (maximum value of double). 

The filter does not work with RGBA32, Complex64 and Complex128, but with all other data-types. This filter has got a fast 
implementation for fixed-point data types without an higher threshold (since version 0.0.3).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../autofocus.cpp" line="-32"/>
        <source>Determines an auto focus estimate for every plane in a given 2D or 3D dataObject. 

The estimate is returned in terms of a tuple of double values for each plane. The higher the estimate, the &apos;sharper&apos; the image. 
There are different methods implemented how the auto focus estimate is calculated. 

The methods are partially taken from H. Mir, P. Xu, P. van Beek, &apos;An extensive empirical evaluation of focus measures for digital photography&apos;, Proc. SPIE 9023 (2014). 
Many methods are based on linear filters. If so, their horizontal and vertical version is applied and the result is determined by: 

result = sum(sqrt(H*H + V*V)) / numPixelsPerPlane</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
