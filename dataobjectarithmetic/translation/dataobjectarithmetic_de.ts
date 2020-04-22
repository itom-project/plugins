<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DataObjectArithmetic</name>
    <message>
        <location filename="../dataobjectarithmetic.cpp" line="+108"/>
        <location line="+13"/>
        <location line="+3"/>
        <location line="+16"/>
        <location line="+164"/>
        <location line="+155"/>
        <source>source image data object for operation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-349"/>
        <location line="+17"/>
        <location line="+17"/>
        <source>result of calculation. This param can be int or double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-2"/>
        <location line="+164"/>
        <location line="+157"/>
        <source>Ignore invalid-Values for floating point</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-318"/>
        <location line="+166"/>
        <location line="+4"/>
        <source>Index of the plane, which contains the result.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-169"/>
        <location line="+166"/>
        <location line="+4"/>
        <source>Pixelindex in y-direction.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-169"/>
        <location line="+166"/>
        <location line="+4"/>
        <source>Pixelindex in x-direction.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-157"/>
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
        <location line="+708"/>
        <location line="+186"/>
        <location line="+915"/>
        <source>2D source image data object (u)int8, (u)int16, int32, float32 or float64 only.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1099"/>
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
        <location line="+585"/>
        <source>Error: sourceImage is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1094"/>
        <location line="+513"/>
        <location line="+586"/>
        <source>Error: sourceImage is not initialized</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-937"/>
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
        <location line="+6"/>
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
        <location line="+877"/>
        <source>input 2D or 3D uint8 or uint16 data object (in case of 3D, every plane is analyzed independently and the resulting spot object is 3D as well. Indicate parameter &apos;maxNrOfSpots&apos; in case of 3D.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>resulting data object with spot coordinates. Every line consists of the following entries: [sub-pixel wise row (physical coordinates), sub-pixel wise column (physical coordinates), coarse intensity of the peak, area of the peak (nr of pixels brighter than background)].</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>maximum difference between two adjacent background values (used for deciding if pixel belongs to background or peak, only necessary in mode 0)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>minimum height of a peak (its maximum and the neighbouring background, only necessary in mode 0).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>maximum diameter of a peak (this is used to distinguish between neighbouring peaks and the determination of the sub-pixel peak position).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>step size in pixel for the coarse search of peaks (for rows and columns)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>maximum background level for subpixel determination, in mode 2 this value is the single value used to determine if value is a peak.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>implemented modes are 0, 2 or 4. Depending on each mode, the search strategy of possible points in each line is kindly different and varies in speed and accuracy.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>if &gt; 0 the resulting spots object is limited to the maximum number of spots (unsorted), else it contains as many lines as detected spots. In case of a 3D image, every plane is analyzed. Then it becomes necessary to indicate this parameter. If &apos;spots&apos; is then allocated with a bigger number of lines than detected peaks, the additional lines are filled with 0.0.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+33"/>
        <source>image must be of type uint8 or uint16</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>image must have at least one plane.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>in case of a 3D input image please indicate the optional parameter &apos;maxNrOfSpots&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-938"/>
        <source>destination object for center of gravity values (in physical coordinates), float64, size: [numPlanes x sizeOfElements]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>destination object for the absolute maximum along the search direction, same type than source image, size: [numPlanes x sizeOfElements]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>0: COG search along each row (default), 1: along each column</source>
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
        <location line="+247"/>
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
        <location line="-1789"/>
        <location line="+60"/>
        <location line="+90"/>
        <source>data type not supported</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+97"/>
        <source>mean result</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>deviation result</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-275"/>
        <location line="+59"/>
        <location line="+85"/>
        <location line="+69"/>
        <location line="+34"/>
        <location line="+68"/>
        <source>Error, object dimensions must be unequal zero</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-199"/>
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
        <location line="+147"/>
        <source>Toggles the calculation mode of standard deviation over N or N-1 elements</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-215"/>
        <location line="+85"/>
        <location line="+69"/>
        <location line="+34"/>
        <source>Error: source image is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-247"/>
        <location line="+309"/>
        <location line="+110"/>
        <location line="+7"/>
        <source>Error: source image is NULLL</source>
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
        <location line="+1090"/>
        <source>Unknown type or type not implemented for phase shifting evaluation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../autofocus.cpp" line="+178"/>
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
    <name>QObject</name>
    <message>
        <location filename="../dataobjectarithmetic.cpp" line="-1970"/>
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
        <location line="+93"/>
        <source>This filter calculates the global minimum value and its first location within the dataObject. 

The returned value will be an integer for all fixed-point data types and float for all floating point types. 

The filter is implemented for all data types besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+59"/>
        <source>This filter calculates the global maximum value and its first location within the dataObject. 

The returned value will be an integer for all fixed-point data types and float for all floating point types. 
The global maximum of complex data types is defined to be the global maximum of all absolute values. 

The filter is implemented for all data types besides RGBA32</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+60"/>
        <source>This filter calculates the minimal and maximal value and its first location within the dataObject. 

The returned values will be integer for all fixed-point data types or float for all floating point types. 

The filter does not work with RGBA32 but with all other datatypes.

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+94"/>
        <source>This filter calculates the mean value within the dataObject. 

The return value containing the mean value of the dataObject.

The filter is implemented for all datatypes besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <source>This method calculates the median value over all values in the data object. 

The returned median values is given as double.

This method is implemented for all datatypes besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+55"/>
        <source>This method returns the arithmetic mean and the standard deviation of the given dataObject within its ROI.

Use the optional argument &apos;flag&apos; to choose between two formulas for the determination of the standard deviation. 
Either (flag = 0): 


    \sqrt(\sum{(x-xm)^2} / (n-1))

or (flag = 1):


    \sqrt(\sum{(x-xm)^2} / n)

This method is implemented for all datatypes besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+47"/>
        <source>Element-wise check if two dataObjects are equal. 
The filter returns 1 if all values of both objects are equal, else 0.

The filter is implemented for all datatypes besides RGBA32, Complex64 and Complex128.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+333"/>
        <source>This filter calculates the center of gravity of a 2D real image. 

The return value contains the column and row position in pixel and physical coordinates.

For the determination, only values in the range [lowThreshold, highThreshold] are considered. The COG algorithm requires, that all values 
that do not belong to the required peak have values around zero. In order to achieve this, the &apos;lowThreshold&apos; value is subtracted from each 
valid intensity value before calculating the COG with the following equations: 

cXI = \frac{\sum{idx_x * (I - lowThreshold)}}{\sum{(I - lowThreshold)} 
cYI = \frac{\sum{idx_y * (I - lowThreshold)}}{\sum{(I - lowThreshold)} 

The filter does not work with RGBA32, Complex64 and Complex128, but with all other datatypes.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1104"/>
        <source>This filter calculates the minimum ROI that contains all values within a lower and optional upper threshold. 

The return value contains the [x0,y0,width,height] of the minimum ROI.

Values of the data object belong to the ROI if they are &gt;= lowThreshold and &lt;= highThreshold. 
The highThreshold is only checked, if it is different than the default value (maximum value of double). 

The filter does not work with RGBA32, Complex64 and Complex128, but with all other datatypes. This filter has got a fast 
implementation for fixed-point data types without an higher threshold (since version 0.0.3).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+265"/>
        <source>This method determines the sub-pixel peak position of multiple spots in an image. 

This algorithm is implemented for 2D or 3D input images of type uint8 or uint16 only and has been developped with respect to a fast implementation. At first, the image is analyzed line-wise with a line distancen of &apos;searchStepSize&apos;. 
In every line the coarse peak position of every 1D peak is analyzed. This can be done in two different ways (depending on the 
parameter &apos;mode&apos; (0, 2 or 4): 

In mode 0 (slightly slower) pixels belong to the background if their distance to the previous pixel (the search step size is also considered in each line) is smaller than &apos;backgroundNoise&apos;. If this is not the case, a potential peak starts. However this peak is only a true peak, if the peak&apos;s height is bigger than &apos;minPeakHeight&apos;. 

In mode 2, a peak consists of a sequence of pixels whose gray-value are all &gt;= &apos;maxBackgroundLevel&apos; (fast, but requires homogeneous background and peak levels). 

In mode 4, a peak can only start if a current gray-value is &gt;= &apos;minPeakHeight&apos; and if the difference to its previous pixel 
is bigger than &apos;backgroundNoise&apos;. The peak is only finished and hence stopped if the difference between its highest gray-value 
and the start-value has been at least &apos;minPeakHeight&apos;, checked at the moment if the gradient is currently negative and its current 
gray value is either below &apos;minPeakHeight&apos; or its difference to the previous value is &lt;= &apos;backgroundNoise&apos;. 

After all peaks in all analyzed lines have been detected, peaks in adjacent lines(step size of &apos;searchStepSize&apos;) are clustered considering the parameter &apos;maxPeakDiameter&apos;.Finally the center of gravity is determined around each local maximum using &apos;maxPeakDiameter&apos; as rectangular size of the search rectangle around the coarse maximum position.The results are stored in the data object &apos;spots&apos;. The &apos;spots&apos; object is two dimensional for a 2D input image, else 3D where the first dimension corresponds to the number of planes in &apos;input&apos;. Each line corresponds to one peak and contains its sub - pixel precise row and column as well as the coarse intensity value and the area of the peak. This value may differ from the real peak value due to the search grid size of &apos;searchStepSize&apos;. 

The parameter &apos;searchStepSize&apos; is a list of two values, the first describes the vertical step size, the second the horizontal step size.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1189"/>
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

The filter is not implemented for complex data types and the type rgba32 since there is no maximum value defined for these types.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+467"/>
        <source>analyzes all values in the given data object and returns the value, which is at a given percentage in the sorted value list.</source>
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
