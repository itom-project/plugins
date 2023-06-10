<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DataObjectArithmetic</name>
    <message>
        <location filename="../dataobjectarithmetic.cpp" line="+114"/>
        <location line="+23"/>
        <location line="+9"/>
        <location line="+26"/>
        <location line="+233"/>
        <location line="+249"/>
        <source>source image data object for operation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-533"/>
        <location line="+33"/>
        <location line="+33"/>
        <source>result of calculation. This param can be int or double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-7"/>
        <location line="+233"/>
        <location line="+259"/>
        <source>Ignore invalid-Values for floating point</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-479"/>
        <location line="+244"/>
        <location line="+26"/>
        <source>Index of the plane, which contains the result.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-264"/>
        <location line="+244"/>
        <location line="+26"/>
        <source>Pixelindex in y-direction.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-264"/>
        <location line="+244"/>
        <location line="+26"/>
        <source>Pixelindex in x-direction.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-252"/>
        <source>1. source image data object for operation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>2. source image data object for operation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>0 if both data objects are not equal, else 1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+909"/>
        <location line="+270"/>
        <location line="+1498"/>
        <source>2D source image data object (u)int8, (u)int16, int32, float32 or float64 only.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1758"/>
        <source>values &lt; lowThreshold are ignored. lowThreshold is subtracted from each valid value before COG determination.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <location line="+296"/>
        <source>values &gt; highThreshold are ignored.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-276"/>
        <source>y-Coordinate of COG (physical unit)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>x-Coordinate of COG (physical unit)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <location line="+687"/>
        <location line="+1061"/>
        <source>Error: sourceImage is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1743"/>
        <location line="+687"/>
        <location line="+1062"/>
        <source>Error: sourceImage is not initialized</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1538"/>
        <source>Mx3 or Mx4 2D data object of type uint16, each row corresponds to one spot. The line contains [px_x, px_y, circle_diameter] if the cog should be determined within a circle or [px_x, px_y, width, height] if the cog should be determined within a rectangle. circle_diameter, width or height have to be odd.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <source>resulting Mx3 data object of type float64 with the sub-pixel precise position of the spots (all is given in pixel coordinates, never physical coordinates). Each row is [subpix_x, subpix_y, nr_of_valid_elements_within_search_mask] or [px_x, px_y, 0 | 1] if the spot only contained one or no valid values.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>values &lt; lowThreshold are ignored. lowThreshold is subtracted from each valid value before COG determination. if lowThreshold is NaN (default), the lowest value within each spot search area is taken as local minimum value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+362"/>
        <source>source image data (2D or 3D) object for operation (u)int8, (u)int16, int32, float32 or float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+29"/>
        <source>if (max-min) along the search direction is lower or equal this pvThreshold (peak-to-valley), no cog is determined and a NaN value is set into the resulting position array (default: this threshold is not considered).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <source>If != 0.0, values &lt;= (max+min)*dynamicThreshold will be ignored. To only consider values above the FWHM, set this value to 0.5 (default).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>values &lt;= lowerThreshold will not be considered for the cog calculation (default: this threshold is not considered).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+591"/>
        <source>1xN, float64 x coordinates</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>1xN, float64 y coordinates</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>Stop criteria for the iterative approach. If the norm of the difference of the internal
vector (a,b,c) (see referenced paper) is smaller than this tolerance, the iteration is stopped.
The maximum iteration count is 20 in any case.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>If 1, NaN values in x and / or y are ignored.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>The magnitude A</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>The center mu</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>The value sigma</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+19"/>
        <source>Error: x or y is empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+733"/>
        <source>input 2D or 3D uint8 or uint16 data object (in case of 3D, every plane is analyzed independently and the resulting spot object is 3D as well. Indicate parameter &apos;maxNrOfSpots&apos; in case of 3D.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>resulting data object with spot coordinates. Every line consists of the following entries: [sub-pixel wise row (physical coordinates), sub-pixel wise column (physical coordinates), coarse intensity of the peak, area of the peak (nr of pixels brighter than background)].</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <source>maximum difference between two adjacent background values (used for deciding if pixel belongs to background or peak, only necessary in mode 0)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>minimum height of a peak (its maximum and the neighbouring background, only necessary in mode 0).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>maximum diameter of a peak (this is used to distinguish between neighbouring peaks and the determination of the sub-pixel peak position).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>step size in pixel for the coarse search of peaks (for rows and columns)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>maximum background level for subpixel determination, in mode 2 this value is the single value used to determine if value is a peak.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>implemented modes are 0, 2 or 4. Depending on each mode, the search strategy of possible points in each line is kindly different and varies in speed and accuracy.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>if &gt; 0 the resulting spots object is limited to the maximum number of spots (unsorted), else it contains as many lines as detected spots. In case of a 3D image, every plane is analyzed. Then it becomes necessary to indicate this parameter. If &apos;spots&apos; is then allocated with a bigger number of lines than detected peaks, the additional lines are filled with 0.0.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+43"/>
        <source>image must be of type uint8 or uint16</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>image must have at least one plane.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>in case of a 3D input image please indicate the optional parameter &apos;maxNrOfSpots&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1577"/>
        <source>destination object for center of gravity values (in physical coordinates), float64, size: [numPlanes x sizeOfElements]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>destination object for the absolute maximum along the search direction, same type than source image, size: [numPlanes x sizeOfElements]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+45"/>
        <source>0: COG search along each row (default), 1: along each column</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+30"/>
        <source>Error: destCOG image is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Error: destIntensity image is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Error: destCOG and destIntensity must not be the same data objects.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+246"/>
        <source>Center of gravity can only be calculated for (u)int8, (u)int16, (u)int32, float32 or float64 data objects.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+559"/>
        <source>valid non-complex data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>percentage value [0.0, 100.0]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>threshold value (NaN if data object was empty or only contained invalid values)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+20"/>
        <source>input data is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <source>not implemented for complex64 or complex128</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+132"/>
        <source>only values &gt;= lowThreshold are considered for the ROI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>if given, only values &lt;= highThreshold are considered for the ROI</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>ROI of bounding box [x0,y0,width,height]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+42"/>
        <source>Error: source image must have one plane</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-2683"/>
        <location line="+72"/>
        <location line="+167"/>
        <source>data type not supported</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+142"/>
        <source>mean result</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>deviation result</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-416"/>
        <location line="+71"/>
        <location line="+159"/>
        <location line="+87"/>
        <location line="+42"/>
        <location line="+104"/>
        <source>Error, object dimensions must be unequal zero</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-312"/>
        <source>Switch complex handling, 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>Minimal value, this parameter be int or double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+24"/>
        <source>Maximum value, this parameter. This param can be int or double</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+207"/>
        <source>Toggles the calculation mode of standard deviation over N or N-1 elements</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-329"/>
        <location line="+159"/>
        <location line="+87"/>
        <location line="+42"/>
        <source>Error: source image is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-359"/>
        <location line="+457"/>
        <location line="+145"/>
        <location line="+7"/>
        <source>Error: source image is NULLL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+240"/>
        <source>type not supported</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+63"/>
        <source>y-Coordinate of COG (index)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>x-Coordinate of COG (index)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+64"/>
        <source>Error: source image must not have multiple planes</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+36"/>
        <location line="+1738"/>
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
        <location filename="../dataobjectarithmetic.cpp" line="-2947"/>
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
        <location line="+8"/>
        <source>LGPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+163"/>
        <source>This filter calculates the global minimum value and its first location within the dataObject.

The returned value will be an integer for all fixed-point data types and float for all floating point types.

The filter is implemented for all data types besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+71"/>
        <source>This filter calculates the global maximum value and its first location within the dataObject.

The returned value will be an integer for all fixed-point data types and float for all floating point types.
The global maximum of complex data types is defined to be the global maximum of all absolute values.

The filter is implemented for all data types besides RGBA32</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+72"/>
        <source>This filter calculates the minimal and maximal value and its first location within the dataObject.

The returned values will be integer for all fixed-point data types or float for all floating point types.

The filter does not work with RGBA32 but with all other datatypes.

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+175"/>
        <source>This filter calculates the mean value within the dataObject.

The return value containing the mean value of the dataObject.

The filter is implemented for all datatypes besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+42"/>
        <source>This method calculates the median value over all values in the data object.

The returned median values is given as double.

This method is implemented for all datatypes besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+90"/>
        <source>This method returns the arithmetic mean and the standard deviation of the given dataObject within its ROI.

Use the optional argument &apos;flag&apos; to choose between two formulas for the determination of the standard deviation.
Either (flag = 0):

.. math::  \sqrt\frac{\sum{\left(x - xm\right)^2}}{n - 1}

or (flag = 1):

.. math::  \sqrt\frac{\sum{\left(x - xm\right)^2}}{n}

This method is implemented for all datatypes besides RGBA32, Complex64 and Complex128

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+428"/>
        <source>This filter calculates the center of gravity of a 2D real image.

The return value contains the column and row position in pixel and physical coordinates.

For the determination, only values in the range [lowThreshold, highThreshold] are considered. The COG algorithm requires, that all values
that do not belong to the required peak have values around zero. In order to achieve this, the &apos;lowThreshold&apos; value is subtracted from each
valid intensity value before calculating the COG with the following equations:

.. math:: cXI = \frac{\sum idx_x \cdot \left(I - lowThreshold\right)}{\sum \left(I - lowThreshold\right)}
.. math:: cYI = \frac{\sum idx_y \cdot \left(I - lowThreshold\right)}{\sum \left(I - lowThreshold\right)}

The filter does not work with RGBA32, Complex64 and Complex128, but with all other datatypes.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+264"/>
        <source>This filter determines the sub-pixel
spot position of multiple spots in an image. The pixel-precise spot position must be given including the size
of the area around the coarse spot position over which the center of gravity algorithm is applied.

The area can either be a rectangle (width and height, odd values) or a circle (odd diameter).

The COG is calculated by the following algorithm:

.. math:: cXI = \frac{ \sum idx_x \cdot \left(I - lowThreshold\right)}{\sum \left(I - lowThreshold\right)}
.. math:: cYI = \frac{ \sum idx_y \cdot \left(I - lowThreshold\right)}{\sum \left(I - lowThreshold\right)}

The lowThreshold can either be given or (if it is NaN), the minimum value of each area will be taken as local lower threshold.
Only values &lt;= highThreshold are considered, set highThreshold to NaN or Inf in order to do not consider this constraint.

Usually, the resulting &apos;centroids&apos; object contains the sub-pixel x and y position as well as the number of valid pixels in each row.
If no or only one valid pixel has been encountered, the coarse pixel x and y position as well as 0 or 1 (for no or one valid pixel) is returned.

If the coarse spot position lies outside of the image, the resulting row in &apos;centroids&apos; contains NaN coordinates.
Please consider, that all input and output coordinates are assumed to be pixel values, the scaling and offset of the image are not considered.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+392"/>
        <source>Calculate center of gravity for each plane along the x- or y-direction.

This methods creates the two given data objects &apos;destCOG&apos; and &apos;destIntensity&apos; in the following way:

- destCOG, ito::float64, sizes: [nrOfPlanes x sizeOfElements], contains the sub-pixel wise one-dimensional coordinate of the center of gravity (in physical coordinates) or NaN if it could not be determined.
- destIntensity, same type than input object, sizes: [nrOfPlanes x sizeOfElements], contains the absolute maximum along the search direction.

If the center of gravity should be calculated along each row of each plane inside of the given &apos;sourceStack&apos; data object, the parameter &apos;columnWise&apos; must be 0, for a column-wise
calculation is must be set to 1. Along each search direction, the corresponding minimum and maximum value is determined and the center of gravity is determined using:

.. math:: cog = \frac {\sum \left[idx \cdot \left(I - lowerBoundary\right)\right]}{\sum \left(I - lowerBoundary\right)}

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
        <location line="+646"/>
        <source>Fits Gaussian for given x and y values.

This method fits a Gaussian curve of the form:

y = A * exp(-(x - mu)^2 / (2 * sigma^2))

It implements the iterative method, described in the paper

Hongwei Guo et. al, A Simple Algorithm for Fitting a Gaussian Function,
IEEE Signal Processing Magazine, 28(5), 2011

There is no additional bias or offset considered, such that a fit will provide
valid results only if the y-values trend towards zero at the edges. If less
than three values &gt; 0 are given, the fit will fail.

The returned values are the coefficients A, mu and sigma.

For the calculation, an internal accumulator has to be created. To avoid a buffer
overflow of this accumulator, do not use to big values. Possibly downscale the
values of the source objects.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1673"/>
        <source>Element-wise check if two dataObjects are equal.
The filter returns 1 if all values of both objects are equal, else 0.

The filter is implemented for all datatypes besides RGBA32, Complex64 and Complex128.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2141"/>
        <source>This filter calculates the minimum ROI that contains all values within a lower and optional upper threshold.

The return value contains the [x0,y0,width,height] of the minimum ROI.

Values of the data object belong to the ROI if they are &gt;= lowThreshold and &lt;= highThreshold.
The highThreshold is only checked, if it is different than the default value (maximum value of double).

The filter does not work with RGBA32, Complex64 and Complex128, but with all other datatypes. This filter has got a fast
implementation for fixed-point data types without an higher threshold (since version 0.0.3).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+327"/>
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
        <location line="-512"/>
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
