<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>FittingFilters</name>
    <message>
        <source>fits plane in 2D-dataObject and returns plane-parameters A,B,C (z=A+Bx+Cy)</source>
        <translation type="obsolete">Fittet eine Ebene in 2D-Datenobjekten und gibt Ebenenparameter A, B, C (z=A+Bx+Cy) zurück</translation>
    </message>
    <message>
        <source>subtracts plane from 2D-dataObject given by plane-parameters A,B,C (z=A+Bx+Cy)</source>
        <translation type="obsolete">Subtrahiert eine Ebene vom 2D-Datenobjekt, gegeben durch die Ebenenparameter A, B, C (z=A+Bx+Cy)</translation>
    </message>
    <message>
        <source>fits plane in 2D-dataObject and subtracts this plane from the dataObject -&gt; this is a combination of fitPlane and subtractPlane</source>
        <translation type="obsolete">Fittet eine Ebene im 2D-Datenobjekt und subtrahiert diese Ebene vom Datenobjekt -&gt; dies ist eine Kombination von &apos;fitPlane&apos; und &apos;subtractPlane&apos;</translation>
    </message>
    <message>
        <source>fits 2D-polynomial in 2D-dataObject and returns a double-DataObject with the fitted surface as well as an error value sigma</source>
        <translation type="vanished">Fittet ein 2D-Polynom n-ter Ordnung in ein 2D-Datenobjekt und gibt ein &apos;Double&apos;-Datenobjekt mit den gefitteten Daten und der mittleren Abweichung (sigma-Wert) zurück</translation>
    </message>
    <message>
        <location filename="../polyfit2d.cpp" line="+51"/>
        <source>uninitialized vector for mandatory, optional or output parameters!</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <location filename="../fittingfilters.cpp" line="+131"/>
        <location line="+136"/>
        <location line="+111"/>
        <source>source image data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../fittingfilters.cpp" line="-245"/>
        <location line="+248"/>
        <source>fitting method (leastSquareFit [default], leastSquareFitSVD, leastMedianFit)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-242"/>
        <location line="+248"/>
        <location line="+825"/>
        <source>probability that 3 randomly selected point of all points only contain trustful (valid) points. (only important for leastMedianFit)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1072"/>
        <location line="+248"/>
        <location line="+825"/>
        <source>allowed probability that the fit is based on a possible outlier (non correct fit). (only important for leastMedianFit)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-944"/>
        <source>Parameter A of regression plane z = A + Bx + Cy, which is subtracted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Parameter B of regression plane z = A + Bx + Cy, which is subtracted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Parameter C of regression plane z = A + Bx + Cy, which is subtracted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+20"/>
        <source>Error: source image must be two-dimensional.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1243"/>
        <source>LeastSquaresPlane (default), LMedSPlane (Least median of squares), Median, Mean</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>BoundingBox (default): a bounding box is set around each invalid area and the interpolation is done over all valid values within the bounding box. For other region types see the description of this filter.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>The invalid region is extend by the &apos;regionType&apos;. The size of the extend in x- and y- direction is given by this parameter as tuple (dx,dy).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Only invalid area whose number of pixels is below this value are filled by interpolated values.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>probability that 3 randomly selected point of all points only contain trustful (valid) points. (only important for leastMedianFitPlane)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>allowed probability that the fit is based on a possible outlier (non correct fit). (only important for leastMedianFitPlane)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>optional dataObject (only possible if &apos;inputObject&apos; only contains one plane). If given, the dataObject will be a float32 Mx8 dataObject where each row corresponds to one detected area. Its content is [inclusive leftmost pixel-coordinate of the area, inclusive topmost pixel-coordinate of the area, width of bounding box of area in pixel, height of bound box of area in pixel, total number of pixels in area, x-coordinate of centroid of area in physical units, y-coordinate of centroid of area in physical units, 1: area has been filled, else 0]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>total number of detected invalid areas</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>number of areas that where filled (their area size was &lt;= maxAreaSize and the interpolation values based on surrounding pixels are valid).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+336"/>
        <source>%1 of %2 found invalid areas filled with method = %3, maxAreaSize = %4, regionType = %5, regionExtend = %6, validPointProbability = %7, allowedErrorProbability = %8</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>%1 of %2 found invalid areas filled with method = %3, maxAreaSize = %4, regionType = %5, regionExtend = %6</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1281"/>
        <source>Generated object via polyval2D with order X = %1, Y = %2</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+66"/>
        <source>Generated object via polyval2DSinglePoints with order X = %1, Y = %2</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+220"/>
        <source>Calculated polynomical coeffs along z-direction with order Z = %1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+268"/>
        <source>Subtracted polynomial of %1th order along axis %2</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+34"/>
        <source>LeastSquares (default), LMedS (Least median of squares)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1065"/>
        <source>Parameter A of regression plane z = A + Bx + Cy</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Parameter B of regression plane z = A + Bx + Cy</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Parameter C of regression plane z = A + Bx + Cy</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+152"/>
        <source>source matrix must be of type (u)int8, (u)int16, (u)int32, float32 or float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-28"/>
        <location line="+111"/>
        <source>destination image data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../polyfit2d.cpp" line="+55"/>
        <location filename="../fittingfilters.cpp" line="-78"/>
        <source>destination matrix is NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../fittingfilters.cpp" line="+60"/>
        <source>Substracted plane with A = %1, B = %2, C = %3</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../polyfit2d.cpp" line="+49"/>
        <source>2D polynomical fit with order x = %1 and y = %2</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-101"/>
        <source>destination data object with fitted values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>order of the fitting polynomial in x-direction</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>order of the fitting polynomial y-direction</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>if 0 infinite values in input image will be copied to output</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Variance value *sigma* of polynomial fit.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+166"/>
        <source>2:  polynomial orders must not be below 1, aborted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>3:  polynomial order in x and/or y too big, aborted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+31"/>
        <source>5:  error allocating memory for recursion coefficients</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+26"/>
        <source>6: error allocating memory for NormX, NormY or Sum</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+32"/>
        <source>7:  error allocating memory for line sum</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+184"/>
        <source>error while allocating memory</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <source>Filter-Plugin for fitting-methods.</source>
        <translation type="obsolete">Filter-Plugin für Fitting-Methoden.</translation>
    </message>
    <message>
        <location filename="../fittingfilters.cpp" line="-290"/>
        <source>Plugin with fitting algorithms.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>This plugin contains algorithms for fitting planes and other two dimensional polynomials to dataObjects mainly using the method of least-squares. Some of the included algorithms can also be called with weighted values, such that more precise fitting results are achievable. 

Furthermore this plugin also contains methods to finally subtract or reconstruct the fitted surfaces.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>licensed under LPGL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+27"/>
        <source>fits plane in 2D-dataObject and returns plane-parameters A,B,C (z=A+Bx+Cy) 

This fit can be executed by different fit strategies: 
- leastSquareFit minimizes the sum of  the squared distances of all valid points to the plane (direct solution)
- leastSquareFitSVD does the same using a svd algorithm 
- leastMedianFit minimizes the median of the absolute distances of all valid points to the plane 

The probability values are only important for the least median fit and determine the number of iterations for the 
a random search using the equation 

iterations &gt;= ceil(log(allowedErrorProbability)/log(1-validPointProbability)))</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+142"/>
        <source>subtracts plane from 2D-dataObject given by plane-parameters A,B,C (z=A+Bx+Cy) 

If the destinationImage is not the same than the sourceImage, the destinationImage finally is a new data object with the same size and type than the sourceImage and contains the data of the sourceImage subtracted by the given plane. If both are the same, the subtraction is executed in-place. 

If the input dataObject contains more than one plane, the subtraction is executed separately for each plane.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+115"/>
        <source>subtracts a fitted regression plane from the given 2D input dataObject . 

This method firstly executes the filter *fitPlane* followed by *subtractPlane*.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+58"/>
        <source>This method fits a two-dimensional polynomial of given order in x- and y-direction to the data &apos;inputData&apos;. 

For the fit, the optional scale and offset values of the input data object are considered. The fit is executed in double precision, such that the input is converted to float64 (if not yet done). NaN values in the input data object are ignored. Optionally, you can give a weighting data object (needs to have the same dimension and size than inputData) such that the values are weighted with the values of the data object &apos;weights&apos;. Values with corresponding weights &lt;= 0 are ignored as well. 

Depending on the orders, the fitted polynomial, whose coefficients are returned by this filter, has the following form: 

    if (orderX &lt;= orderY): 
    
        f(x,y) = \sum_{i=0}^orderX \sum_{j=0}^{orderY-i} p_{ij} x^i y^j 
    else: 
    
        f(x,y) = \sum_{j=0}^orderY \sum_{i=0}^{orderX-i} p_{ij} x^i y^j 

The coefficients p_ij are stored in the coefficients vector in the order they appear in the equation above. 

The solver uses a Vandermonde matrix V as solving strategy and tries to solve V*p=Z, where Z are the valid values of the input data object. The overdetermined system of linear equations is finally solved using a QR factorization of V. If this module is compiled with LAPACK, its solvers are used, else the solve-command of OpenCV (slower) is called. In order to speed up the calculation you can use the parameter &apos;reduceFactor&apos;. If it is set to any value &gt;= 1, The input plane is divided into a grid of (orderY+1)*reduceFactor x (orderX+1)*reduceFactor rectangles. In every rectangle an arbitrary valid value is selected and used for the determination only. If no valid value could be found after a certain number of new random values, no value is taken from this rectangle. The algorithm returns an error if less values could have been selected than are needed for the fit of given orders. 

The definition of the polynomial function is slightly different than the one used in the similar fitting function &apos;fitPolynom2D&apos;.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+69"/>
        <source>This method fits a two-dimensional polynomial of given order in x- and y-direction to the points whose x, y and z coordinates are given in &apos;xData&apos;, &apos;yData&apos; and &apos;zData&apos;. 

The fit is executed in double precision, such that the input is converted to float64 (if not yet done). NaN values in the x, y or z data objects are ignored. Optionally, you can give a weighting data object (needs to have the same dimension and size than inputData) such that the values are weighted with the values of the data object &apos;weights&apos;. Values with corresponding weights &lt;= 0 are ignored as well. 

All input data objects must have the same size. 

Depending on the orders, the fitted polynomial, whose coefficients are returned by this filter, has the following form: 

    if (orderX &lt;= orderY): 
    
        z = f(x,y) = \sum_{i=0}^orderX \sum_{j=0}^{orderY-i} p_{ij} x^i y^j 
    else: 
    
        z = f(x,y) = \sum_{j=0}^orderY \sum_{i=0}^{orderX-i} p_{ij} x^i y^j 

The coefficients p_ij are stored in the coefficients vector in the order they appear in the equation above. 

The solver uses a Vandermonde matrix V as solving strategy and tries to solve V*p=Z, where Z are the valid values of the input data object. The overdetermined system of linear equations is finally solved using a QR factorization of V. If this module is compiled with LAPACK, its solvers are used, else the solve-command of OpenCV (slower) is called. 

The definition of the polynomial function is slightly different than the one used in the similar fitting function &apos;fitPolynom2D&apos;.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+70"/>
        <source>This method evaluates a two-dimensional polynom for every point in a given data object

For every single pixel in the input data object &apos;dataZ&apos;, its physical coordinate (using scale and offset of the data object) is taken and the polynomial (given by its coefficients) is evaluated and stored in the pixel. The data object is hereby converted to float64. 

The polynomial coefficients (p0, p1, ...) are those returned by the filter &apos;fitPolynom2D&apos; and depend on the polynomial order in X and Y direction: 

    if (orderX &lt;= orderY): 
    
        f(x,y) = \sum_{i=0}^orderX \sum_{j=0}^{orderY-i} p_{ij} x^i y^j 
    else: 
    
        f(x,y) = \sum_{j=0}^orderY \sum_{i=0}^{orderX-i} p_{ij} x^i y^j 

The coefficients p_ij are stored in the coefficients vector in the order they appear in the equation above.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+61"/>
        <source>This method evaluates a two-dimensional polynom for every x- and y- coordinate given in xData in yData

For every single pixel whose x- and y-coordinate is given by corresponding values in xData and yData the polynomial (given by its coefficients) is evaluated and stored in zData (float64, same size than xData and yData). 
The polynomial coefficients (p0, p1, ...) are those returned by the filter &apos;fitPolynom2D&apos; and depend on the polynomial order in X and Y direction: 

    if (orderX &lt;= orderY): 
    
        f(x,y) = \sum_{i=0}^orderX \sum_{j=0}^{orderY-i} p_{ij} x^i y^j 
    else: 
    
        f(x,y) = \sum_{j=0}^orderY \sum_{i=0}^{orderX-i} p_{ij} x^i y^j 

The coefficients p_ij are stored in the coefficients vector in the order they appear in the equation above.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+66"/>
        <source>One-dimensional polynomial fit in z-direction for a 3D - data object. 

The input data object must be three-dimensional and is internally casted to float64 (if not yet done). The resulting polynomial parameters per pixel are stored in the output data object &apos;polynoms&apos; whose z-dimension is equal to (order+2). The first (order+1) planes contain the coefficients p0...pn and the last plane contain the pixel wise residual error. 

The polynomial is y(x) = p0 + x*p1 ... + x^n*pn 
The residual is the sum of the quadratical errors from each valid pixel to the fitted polynomial. 

If no &apos;xVals&apos; are assigned, the x-values for each plane are calculated using the offset and scale of the data-object in z-direction, such that an equally spaced vector of (0,1,2,3...) is the default. 

You can additionally give a weight data object (same dimension than &apos;data&apos;) for weighting the values. NaN values in &apos;data&apos; and weights &lt;= 0 are ignored. If a fit cannot be done due to too less or degenerated values, NaN is returned in &apos;polynoms&apos; at this pixel. 

For a first order fit, a direct least squares solution is used which is very fast, for the other orders a system of linear equations is solved (using a SVD decomposition) which can be slower. On a multi-core processor you can assign a number of threads that are used to parallely compute the approximations for each pixel.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+221"/>
        <source>subtracts a one-dimensional fitted polynom (along a given axis) from the values in the given 1D or 2D data object. 

This filter operates inplace and subtracts from fitted one-dimensional polynoms from the values in the given 2D data object &apos;data&apos;. The polynoms are either fitted along the 
vertical or horizontal axis. You can choose a polynomial order between 1 (line) and 7. The values are uniformly weighted for the fit. The algorithm uses a fast, direct solution 
for the line regression fits and a singular value decomposition for all other cases (see fitPolynom1D_Z). The fit is done in double precision while the type of &apos;data&apos; is not changed.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+268"/>
        <source>returns the linearly interpolated values of a given input dataObject at specific 2D point coordinates. 

The given input data object must be a real valued object with two dimensions or a region of interest that only contains one plane (e.g. 1xMxN). 
The point coordinates (coordsSubPix) is a Nx2 floating point data object where each row is the row and column coordinate (sub-pixel) of the desired value. The values must be given 
in the coordinates of the data object (scale values).
The resulting interpolated values are returned as &apos;values&apos; list. The input data object is allowed to contain non-finite values. 

For the interpolation a search rectangle whose height and width is given by &apos;searchRect&apos; is centered at the rounded coordinate and a plane is robustly fitted into the valid 
values that lie within the rectangle. The value is then determined using the coefficients of the fitted plane. Infinite values are ignored for the determination of the plane. 
The plane is calculated by least-squares fit. If the rectangle exceeds the boundaries of the given matrix, it moved inside of the matrix such that the searched coordinate still lies within 
the rectangle. If this is not possible, NaN is returned as value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+314"/>
        <source>This filter can be used to fill NaN areas in a float32 or float64 input object with interpolated values. 

This filter processes independently every plane of the input object. For every plane the following steps are done: 

1. Locate connected areas with NaN values (using OpenCVs method &apos;connectedComponentsWithStats&apos;). 
2. For every area whose size does not exceed &apos;maxAreaSize&apos;: 
   
   - depending on &apos;regionType&apos; and &apos;regionExtend&apos; an enlarged area around the NaN area is taken 
   - depending on &apos;method&apos; an interpolation plane or single value is calculated using all valid values within the enlarged area 
   - The NaN values within the area are filled with the interpolated plane value of the interpolated scalar 

**RegionType** 

* Per default, the region type is &apos;BoundingBox&apos;. Then, a rectangular bounding box is set around each area of invalid values and the size of the bounding box. 
  is increased in all directions by the parameter &apos;regionExtend&apos; (different value possible for horizontal and vertical extend). &apos;BoundingBox&apos; is the fastest
  method, however may produce very varying results, if the shape of the areas is different from simple rectangles. 
* ErodeRect: Around the invalid area, an erosion is calculated with a rectangle element. The interpolation is only done with values within the eroded ribbon. 
  The extend is the maximum horizontal and vertical distance from a edge element of the invalid area, e.g. (1,1) is a 1-pixel ribbon whose valid values are used for interpolation. 
* ErodeEllipse: This is the same than &apos;ErodeRect&apos;, however the element has an ellipse form and not a rectangle making the ribbon more smooth. 

**Method** 

* LeastSquaresPlane: A plane is fitted into the valid values of the region using the least squares fit approach. 
* LMedSPlane: A plane is fitted into the valid values of the region using the approach to minimize the median of the squared distances. 
* Mean: A mean value of a valid values of the region replaces the invalid values within the region. 
* Median: A median value of a valid values of the region replaces the invalid values within the region. 

Currently, the filter does not work inplace such that the output object is always a newly allocated dataObject of the same type and size than the input object.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../polyfit2d.cpp" line="-484"/>
        <source>Fit a polynomial p(x,y) of order (orderX, orderY) in x- and y-direction. 

The fit function always looks like this: 

f(x,y) = sum_{m=0}^{M} sum_{n=0}^{N} ( p_nm * x^n * y^m ) 

This definition is slightly different from the polynomial fitted by the similar function &apos;polyfitWeighted2d&apos;. 

Puts the fitted points into the data object &apos;fittedImage&apos;. This method does not weight the input values and does not 
return the coefficients for the polynomial. Use polyfitWeighted2d if you want to have an enhanced fit.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
