<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>OpenCVFilters</name>
    <message>
        <location filename="../OpenCVFilters.cpp" line="+164"/>
        <source>Input image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Output image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <location line="+807"/>
        <location line="+371"/>
        <location line="+304"/>
        <location line="+209"/>
        <location line="+194"/>
        <location line="+973"/>
        <location line="+221"/>
        <source>Error: source image ptr empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../calib3d.cpp" line="+95"/>
        <location filename="../OpenCVFilters.cpp" line="-3074"/>
        <location line="+807"/>
        <location line="+371"/>
        <location line="+304"/>
        <location line="+209"/>
        <location line="+194"/>
        <location line="+973"/>
        <location line="+221"/>
        <source>Error: dest image ptr empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../OpenCVFilters.cpp" line="-3030"/>
        <source>Error: pointer of input and output objects are equal</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+20"/>
        <source>Error: the check command is currently not implemented for more than 3 dims</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+102"/>
        <location line="+362"/>
        <location line="+280"/>
        <location line="+371"/>
        <location line="+2450"/>
        <source>Error: source is not a matrix or image stack</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-2760"/>
        <source>Error: anchor should be &apos;int8&apos;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../calib3d.cpp" line="+443"/>
        <location filename="../OpenCVFilters.cpp" line="-555"/>
        <location line="+18"/>
        <location line="+342"/>
        <location line="+228"/>
        <location line="+330"/>
        <location line="+365"/>
        <location line="+205"/>
        <location line="+188"/>
        <location line="+257"/>
        <location line="+709"/>
        <location line="+224"/>
        <location line="+283"/>
        <location line="+275"/>
        <source>%1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../OpenCVFilters.cpp" line="-3017"/>
        <source>All types except complex64 and complex128 are accepted</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-638"/>
        <location line="+350"/>
        <source>input data object of type uint8, uint16, int16, float32, float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-342"/>
        <location line="+350"/>
        <source>output image with the same type and size than input (inplace allowed)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-341"/>
        <location line="+361"/>
        <source>structuring element used for the morpholocial operation (default: None, a 3x3 rectangular structuring element is used). Else: An uint8 data object where values &gt; 0 are considered for the operation.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-351"/>
        <location line="+361"/>
        <source>position of the anchor within the element. If not given or if (-1,-1), the anchor is at the element center [default].</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-350"/>
        <location line="+361"/>
        <source>number of times the morpholocial operation is applied [default: 1]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>This string defines how the filter should hande pixels at the border of the matrix. Allowed is CONSTANT [default], REPLICATE, REFLECT, WRAP, REFLECT_101. In case of a constant border, only pixels inside of the element mask are considered (morphologyDefaultBorderValue)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-332"/>
        <location line="+362"/>
        <location line="+3103"/>
        <source>source and destination object must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-3494"/>
        <source>This string defines how the filter should handle pixels at the border of the matrix. Allowed is CONSTANT [default], REPLICATE, REFLECT, WRAP, REFLECT_101. In case of a constant border, only pixels inside of the element mask are considered (morphologyDefaultBorderValue)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+99"/>
        <location line="+362"/>
        <location line="+3094"/>
        <location line="+36"/>
        <source>border type %1 is unknown</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-3472"/>
        <location line="+362"/>
        <source>anchor must be in range [0,%1];[0,%2]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-347"/>
        <location line="+362"/>
        <source>anchor must have either 2 values or none</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-282"/>
        <source>erosion with (y,x) kernel(%1, %2), anchor(%3, %4), %5 iterations, borderType %6</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>dilation with (y,x) kernel(%1, %2), anchor(%3, %4), %5 iterations, borderType %6</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+97"/>
        <source>This parameters defines the operation type, 0: Erode, 1: Dilate, 2: Open, 3: Close, 4: Gradient, 5: Tophat, 6: Blackhat, 7: Hit or miss</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+232"/>
        <source>morphologyEx with (y,x) kernel(%1, %2), anchor(%3, %4), %5 iterations, borderType %6</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+44"/>
        <source>Empty object handle. Image will be of src-type</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Kernelsize for x-axis</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Kernelsize for y-axis</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Position of the kernel anchor, see openCV-Help</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>border mode used to extrapolate pixels outside of the image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+127"/>
        <source>Error: anchor has wrong size or number of dims</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+36"/>
        <source>OpenCV blur-filter with (y,x) kernel(%1, %2), anchor(%3, %4), borderType %5</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+13"/>
        <location line="+326"/>
        <location line="+2229"/>
        <location line="+270"/>
        <source>No compatible dataObject type found for given OpenCV matrix type.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-2265"/>
        <location line="+209"/>
        <location line="+194"/>
        <source>Error: input object must not be empty.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-52"/>
        <source>Rotated object by 90� clockwise with cvRotateM90-Filter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Rotated object by 90� counter clockwise with cvRotateP90-Filter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+166"/>
        <source>Rotated object by 180� using cvRotate180-Filter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+784"/>
        <location line="+229"/>
        <source>Output Object handle. Will be come complex-type</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-220"/>
        <source>Low Threshold</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Ratio between High Threshold and Low Threshold, Canny&apos;s recommendation is three</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>Kernel size for Sobel filter, default is 3</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+155"/>
        <source>Canny edge filter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+56"/>
        <source>Transformation code, see (OpenCV) documentation</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>number of color channels of destination image, for 0 the number of channels is derived from the transformation (default)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+206"/>
        <source>CvtColor conversion with code: %1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+51"/>
        <source>threshold value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>maximum value to use with the THRESH_BINARY and THRESH_BINARY_INV thresholding types.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>threshold type
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+48"/>
        <source>Error: input dataObject must be two-dimensional.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+24"/>
        <source>Image has been threshold filter by a value of %1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+37"/>
        <source>center coordinates of the rotation of shape (x, y) in physical values of the source image.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Rotation angle in degrees. Positive values mean counter-clockwise rotation (the coordinate origin is assumed to be the top-left corner).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Isotropic scale factor.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>rotation matrix</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>Rotation Matrix for %1 deg angle with scale factor of %2</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+51"/>
        <source>input data object of type uint8, uint16, int16, float32, float64.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>output image with the same type and size than input (inplace allowed).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+9"/>
        <source>transformation matrix dataObject of shape 2x3.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>List of (width, height) of the destination dataObject.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+46"/>
        <source>value used in case of a constant border; by default, it is 0.0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>The rotation matrix with the shape of %1x%2 does not correspond to the shape 2x3.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../calib3d.cpp" line="-483"/>
        <source>input image of type uint8</source>
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
        <source>Max Radius in x/y (if 0: the maximum of the image width or height is taken)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+120"/>
        <source>OR Combination of various flags: 

</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+65"/>
        <source>rgba32 input and destination image (must be of type ito::rgba32).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Number of inner corners per chessboard row and column (points_per_row, points_per_column)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>array of detected corners (n x 2), the output of cvFindChessboardCorners or cvCornerSubPix</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Parameter indicating whether the complete board was found or not.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+66"/>
        <source>8bit grayscale input image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>initial coordinates of the input corners and refined coordinates provided for output</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Half of the side length of the search window. Example: (5,5) leads to a (11x11) search window</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>Half of the size of the dead region in the middle of the search zone over which the summation is not done. (-1,-1) indicates that there is no such a size</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>position refinement stops after this maximum number of iterations</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>position refinement stops when the corner position moves by less than this value</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+15"/>
        <source>Parameters &apos;image&apos; and &apos;corners&apos; may not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+19"/>
        <source>corners must be a continuous data object. If you passed a subset of columns of a data object, get a region-only copy before calling this function</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+48"/>
        <source>[NrOfViews x NrOfPoints x 3] float32 matrix with the coordinates of all points in object space (coordinate system of calibration pattern).. Non-finite rows at the end of each matrix-plane will be truncated.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>[NrOfViews x NrOfPoints x 2] float32 matrix with the pixel coordinates (u,v) of the corresponding plane in each view. Non-finite rows at the end of each matrix-plane will be truncated.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>[height,width] of the camera image (in pixels)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Output 3x3 float64 camera patrix. If flags CV_CALIB_USE_INTRINSIC_GUESS and/or CV_CALIB_FIX_ASPECT_RATIO are specified, this matrix must be initialized with right values and is unchanged</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Output 1x4, 1x5 or 1x8 distortion values (float64). (k1, k2, p1, p2 [,k3 [,k4 ,k5 ,k6]])</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>3 x NrOfViews float64 output vector, where each column is the rotation vector estimated for each pattern view (Rodrigues coordinates)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>3 x NrOfViews float64 output vector, where each column is the translation vector estimated for each pattern view</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Different flags that may be a combination of the following values: </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>if &gt; 0, maximum number of counts, 0: unlimited number of counts allowed [default: 30]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>if &gt; 0.0, desired accuracy at which the iterative algorithm stops, 0.0: no epsilon criteria [default: DBL_EPSILON]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>resulting re-projection error</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+161"/>
        <source>[n x 3] array of source points (will be converted to float64).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>[n x 3] array of destination points (must have the same size than sources, will be converted to float64).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Output 3D affine transformation matrix 3x4 (float64)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Output vector indicating which points are inliers (uint8)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Maximum reprojection error in the RANSAC algorithm to consider a point as an inlier (default: 3.0)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Confidence level, between 0 and 1, for the estimated transformation. Anything between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>return value</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+70"/>
        <source>Input (distorted) image (all datatypes)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location line="+69"/>
        <source>Output (corrected) image that has the same size and type as source</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-68"/>
        <location line="+69"/>
        <location line="+76"/>
        <source>Input camera matrix A = [[fx 0 cx];[0 fy cy];[0 0 1]]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-144"/>
        <location line="+69"/>
        <location line="+76"/>
        <source>Input vector of distortion coefficients [1 x 4,5,8] (k1, k2, p1, p2 [, k3[, k4, k5, k6]]) of 4, 5 or 8 elements.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-144"/>
        <source>Camera matrix of the distorted image. By default (if not given), it is the same as cameraMatrix but you may additionally scale and shift the result by using a different matrix.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>&apos;destination&apos; must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>source data object must be two dimensional</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+40"/>
        <source>Observed point coordinates (Nx2) float32</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <location line="+81"/>
        <source>Rectification transformation in the object space (3x3 matrix). If not given, the identity transformation is used.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-80"/>
        <source>New camera matrix (3x3) or new projection matrix (3x4). If not given, the identity new camera matrix is used.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+12"/>
        <location line="+180"/>
        <location line="+295"/>
        <source>destination is empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-411"/>
        <source>undistorted image size</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>The first output map, type is float32.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>The second output map, type is float32.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>New camera matrix A&apos;. If not given, the camera matrix is used.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+16"/>
        <source>map1 or map2 are NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+59"/>
        <location filename="../OpenCVFilters.cpp" line="-354"/>
        <source>source image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <location filename="../OpenCVFilters.cpp" line="+5"/>
        <source>destination image. It hast the same size as map1 and the same type as src.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>The first map of x values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>The second map of y values</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Interpolation method. The following values are possible:
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+7"/>
        <source>Pixel extrapolation method. When boderMode == BORDER_TRANSPARENT (%1), it means that the pixels in the destination image that corresponds to the outliers in the source image are not modified by the function. 
The following values are possible:
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+303"/>
        <source>Interpolation method. The following values are possible: </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-292"/>
        <source>value used in case of a constant border. By default, it is 0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+69"/>
        <source>coordinates of the points in the original plane, a matrix of type [Nx2], float32</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>coordinates of the points in the target plane, a matrix of type [Nx2], float32</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>3x3 homography matrix (output)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Method. The following values are possible: </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>maximum allowed reprojection error to treat a point pair as an inlier (used for RANSAC only)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+14"/>
        <source>homography matrix is empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+48"/>
        <source>coordinates of the points in the first image, a matrix of type [Nx2], float32 or float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>coordinates of the points in the second image, a matrix of type [Nx2], float32 or float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>output, fundamental matrix [3x3], float64</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Method for computing a fundamental matrix. The following values are possible: </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Parameter used for RANSAC. It is the maximum distance from a point to an epipolar line in pixels, beyond which the point is considered an outlier and is not used for computing the final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the point localization, image resolution, and the image noise.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of confidence (probability) that the estimated matrix is correct.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Output array of N elements, every element of which is set to 0 for outliers and to 1 for the other points. The array is computed only in the RANSAC and LMedS methods. For other methods, it is set to all 1�s. If not given, no status information is returned.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+18"/>
        <source>method must be either CV_FM_7POINT (%i), CV_FM_8POINT (%i), CV_FM_RANSAC (%i) or CV_FM_LMEDS (%i)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+63"/>
        <source>coordinates of the image points in the one image, a matrix of type [Nx2], float32</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Index of the image (1 or 2) that contains the points.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Fundamental matrix that can be estimated using cvFindFundamentalMat() or cvStereoRectify()</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Output vector of the epipolar lines corresponding to the points in the other image. Each line ax + by + c=0 is encoded by 3 numbers (a, b, c)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+51"/>
        <location line="+62"/>
        <source>input image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-61"/>
        <location line="+62"/>
        <source>output image that has the size dsize and the same type as input image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-61"/>
        <source>3x3 transformation matrix</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+62"/>
        <source>3x3 camera fundamental matrix</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>matrix with distortion coefficients</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>rotation vector</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>translation vector</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>destination object must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../OpenCVFilters.cpp" line="-2287"/>
        <location line="+1788"/>
        <location line="+229"/>
        <source>Input Object handle, must be a single plane</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-1883"/>
        <source>Image of type Integer or float32</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>Empty dataObject-hanlde. Destination is of source type</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>Kernelsize in x/y</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+54"/>
        <location line="+912"/>
        <source>Error: kernel must be odd</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-902"/>
        <source>Error: kernelsize &gt; 3 and object is not uint8</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+38"/>
        <source>OpenCV medianblur-filter with kernel size = %1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+236"/>
        <location line="+209"/>
        <location line="+194"/>
        <location line="+973"/>
        <location line="+221"/>
        <source>Error: nDim-stacks not supported yet, only 2D and 3D.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-991"/>
        <source>sourceObject must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+6"/>
        <source>destinationObject must not be NULL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+8"/>
        <source>sourceObject is not a matrix or image stack</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+205"/>
        <source>Spike removal filter with kernel(%1, %1) and range ]%2, %3[</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-694"/>
        <source>Flipped left/rigth with cvFlip-Filter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Flipped upside/down with cvFlip-Filter</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../features2d.cpp" line="+54"/>
        <location line="+141"/>
        <source>error while executing some methods</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="-137"/>
        <source>Input parameter - (n x 128) float32 data object of descriptors from first image (queryDescriptors). These descriptors can be computed from sift/surf algorithms.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Input parameter - (n x 128) float32 data object of descriptors from second image (trainDescriptors). These descriptors can be computed from sift/surf algorithms.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Output parameter - (n x 4) float32 data object of Matching descriptor vectors using FLANN matcher. Every row contains the values (queryIdx,trainIdx,imgIdx,distance)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>Maximum distance between two pair of points to calculate the best matching.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Optional input parameter - corresponding key points of the first image (n x 7) float32 data object, must have the same number of rows than first_descriptors.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Optional input parameter - corresponding key points of the second image (n x 7) float32 data object, must have the same number of rows than second_descriptors.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Optional output parameter - (m x 2) float32 data object of best matching points from first image. each row includes (x and y coordinates), and m is the number of best matching points </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Optional output parameter - (m x 2) float32 data object of best matching points from second image. each row includes (x and y coordinates), and m is the number of best matching points</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Optional output parameter - (m x 4) float32 data object of good matching descriptor vectors using FLANN matcher. Every row contains the values (queryIdx,trainIdx,imgIdx,distance)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+17"/>
        <source>The descriptors of the first image is empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+5"/>
        <source>The descriptors of the second image is empty</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+106"/>
        <source>Source image (uint8 or rgba32).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>keypoints of the source image (n x 7) float32 data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>Output image. Its content depends on the flags value defining what is drawn in the output image. See possible flags bit values below.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>color of keypoints (pass a rgba32 value). If 0 or omitted, random colors will be used.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>flags for drawing features (bit-combination): 
- 0: DEFAULT (Output image matrix will be created (Mat::create), i.e. existing memory of output image may be reused.      Two source images, matches, and single keypoints will be drawn. For each keypoint, only the center point will be      drawn (without a circle around the keypoint with the keypoint size and orientation). 
- 1: DRAW_OVER_OUTIMG: Output image matrix will not be created (using Mat::create). Matches will be drawn      on existing content of output image. 
- 4: DRAW_RICH_KEYPOINTS: For each keypoint, the circle around keypoint with keypoint size and orientation will be drawn.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+100"/>
        <source>Input parameter - first image to draw the matching points</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Input parameter - second image to draw the matchibg points</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>keypoints of the first image (n x 7) float32 data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>keypoints of the second image (n x 7) float32 data object</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Input parameter -  Matches from the first image to the second one, which means that keypoints1[i] has a corresponding point in keypoints2[matches[i]]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+1"/>
        <source>Output parameter - Output image</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>color of matches (pass a rgba32 value). If 0 or omitted, random colors will be used.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+3"/>
        <source>color of single keypoints (pass a rgba32 value). If 0 or omitted, random colors will be used.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+4"/>
        <source>flags for drawing features (bit-combination): 
- 0: DEFAULT: Output image matrix will be created (Mat::create), i.e. existing memory of output image may be reused.      Two source images, matches, and single keypoints will be drawn. For each keypoint, only the center point will be      drawn (without a circle around the keypoint with the keypoint size and orientation). 
- 1: DRAW_OVER_OUTIMG: Output image matrix will not be created (using Mat::create). Matches will be drawn      on existing content of output image. 
- 2: NOT_DRAW_SINGLE_POINTS: Single keypoints will not be drawn. 
- 4: DRAW_RICH_KEYPOINTS: For each keypoint, the circle around keypoint with keypoint size and orientation will be drawn.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>max match distance that should be drawn. If 0, every match is drawn [default]</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <location filename="../OpenCVFilters.cpp" line="-1727"/>
        <source>Wrapped algorithms from OpenCV</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+2"/>
        <source>This plugin provides wrappers for various OpenCV algorithms. These are for instance: 

* morphological filters (dilation, erosion) 
* image filtering (blur, median blur...) 
* 1d and 2d fft and ifft 
* histogram determination 
* feature detections (circles, chessboard corners...) 

This plugin not only requires access to the core library of OpenCV but also to further libraries like imgproc and calib3d. 

This plugin has been created at a time when OpenCV did not yet provide bindings for Python 3. 
From OpenCV 3 on, these bindings exist. Therefore, it is possible to access almost all OpenCV 
methods via the cv2 python package. The wrapped methods within this plugin can still be used; 
In addition to the cv2 methods, they can sometimes operate on multi-plane dataObjects, preserve 
the tags and meta information and save protocol data.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+22"/>
        <source>LGPL</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+457"/>
        <source>Dilates every plane of a data object by using a specific structuring element. 

This filter applies the dialation method cvDilate of OpenCV to every plane in the source data object. The result is contained in the destination object. It can handle data objects of type uint8, uint16, int16, float32 and float64 only. 

It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified if it fits to the size and type of the source data object and if not a new one is allocated. 

The dilation is executed using a structuring element which is (if not otherwise stated) a 3x3 kernel filled with ones. Else you can give an two-dimensional uint8 data object. Then, the function dilates the source image using the specified structuring element that determines the shape of a pixel neighborhood over which the maximum is taken: 

dst(x,y) = max_{(x&apos;,y&apos;):element(x&apos;,y&apos;)!=0} src(x+x&apos;,y+y&apos;) 

Dilation can be applied several times (parameter &apos;iterations&apos;).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+26"/>
        <source>Erodes every plane of a data object by using a specific structuring element. 

This filter applies the erosion method cvErode of OpenCV to every plane in the source data object. The result is contained in the destination object. It can handle data objects of type uint8, uint16, int16, float32 and float64 only. 

It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified if it fits to the size and type of the source data object and if not a new one is allocated. 

The erosion is executed using a structuring element which is (if not otherwise stated) a 3x3 kernel filled with ones. Else you can give an two-dimensional uint8 data object. Then, the function dilates the source image using the specified structuring element that determines the shape of a pixel neighborhood over which the maximum is taken: 

dst(x,y) = min_{(x&apos;,y&apos;):element(x&apos;,y&apos;)!=0} src(x+x&apos;,y+y&apos;) 

Erosion can be applied several times (parameter &apos;iterations&apos;).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+303"/>
        <source>Erodes every plane of a data object by using a specific structuring element. 

Performs advanced morphological transformations.The function cv::morphologyEx can perform advanced morphological transformations using an erosion and dilation as basic operations.MORPH_ERODE Any of the operations can be done in - place.In case of multi - channel images, each channel is processed independently.).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+63"/>
        <source>Planewise median blur filter.

This filter applies the method cv::blur to every plane in the source data object. The function smoothes the images by a simple mean-filter. Theresult is contained in the destination object. It can handle data objects of type uint8, uint16, int16, ito::tInt32, float32 and float64 only. 

The cv::blur interally calls the cv::boxfilter()-method.

The itom-wrapping does not work inplace currently. A new dataObject is allocated.

borderType: This string defines how the filter should hande pixels at the border of the matrix.Allowed is CONSTANT [default], REPLICATE, REFLECT, WRAP, REFLECT_101. In case of a constant border, only pixels inside of the element mask are considered (morphologyDefaultBorderValue)
Warning: NaN-handling for floats not verified.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+187"/>
        <source>2D-dimentional fourier-transformation using cv::DFT.

This filter tries to perform an inplace FFT for a given 2D-dataObject. The FFT is calculated planewise.The result is a complex-dataObject. The axis-scales and units are invertes and modified.

This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, false, false, false)-function.
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>2D-dimentional inverse fourier-transformation using cv::DFT.

This filter tries to perform an inplace FFT for a given 2D-dataObject. The FFT is calculated planewise.The result is a real-dataObject. The axis-scales and units are invertes and modified.

This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, true, true, false)-function.
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>1D-dimentional fourier-transformation using cv::DFT.

This filter tries to perform an inplace FFT for a given line or 2D-dataObject. The FFT is calculated linewise.The result is a complex-dataObject. The axis-scales and units are invertes and modified.

This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, false, false, true)-function.
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>1D-dimentional inverse fourier-transformation using cv::DFT.

This filter tries to perform an inplace FFT for a given line or 2D-dataObject. The FFT is calculated linewise.The result is a real-dataObject. The axis-scales and units are invertes and modified.

This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, true, true, true)-function.
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+133"/>
        <source>Planewise median blur filter.

The function smoothes an image using the median filter with the kernel-size x kernel-size aperture. Each channel of a multi-channel image is processed independently. It can handle data objects of type uint8, uint16, int16, ito::tInt32, float32 and float64 only. 

The itom-wrapping does not work inplace currently. A new dataObject is allocated.

Warning: NaN-handling for floats not verified.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+303"/>
        <source>This filter flips the image left to right. 

This filter applies the flip method cvFlip of OpenCV with the flipCode &gt; 0 to a 2D source data object. The result is contained in the destination object

It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified if it fits to the size and type of the source data object and if not a new one is allocated
.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+11"/>
        <source>This filter flips the image upside down. 

This filter applies the flip method cvFlip of OpenCV with the flipCode = 0 to a 2D source data object. The result is contained in the destination object.

It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified if it fits to the size and type of the source data object and if not a new one is allocated
.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+199"/>
        <source>This filter rotates the image by 90� count clock wise. 

This filter applies the flip method cvFlip and the transpose method cvTranspose of OpenCV to rotate the object. The result is contained in the destination object

It is allowed to let the filter work pseudo inplace if you give the same input than destination data object, else the output data object is verified if it fits to the size and type of the source data object and if not a new one is allocated.
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+10"/>
        <source>This filter rotates the image by 90� clock wise. 

This filter applies the flip method cvFlip and the transpose method cvTranspose of OpenCV to rotate the object. The result is contained in the destination object

It is allowed to let the filter work pseudo inplace if you give the same input than destination data object, else the output data object is verified if it fits to the size and type of the source data object and if not a new one is allocated.
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+215"/>
        <source>This filter rotates the image by 180�. 

This filter applies the flip method cvFlip from OpenCV horizontally and vertically to rotate the object. The result is contained in the destination object

It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified if it fits to the size and type of the source data object and if not a new one is allocated.
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+167"/>
        <source>Set single spikes at measurement edges to a new value. 

This filter creates a binary mask for the input object. The value of mask(y,x) will be 1 if value of input(y,x) is within the specified range and is finite.The mask is eroded and than dilated by kernel size using openCV cv::erode and cv::dilate with a single iteration. In the last step the value of output(y,x) is set to newValue if mask(y,x) is 0.

It is allowed to let the filter work inplace if you give the same source and destination data object, else the destination data object is verified if it fits to the size and type of the source data object and if not a new one is allocated and the input data is copied to the new object. 
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+304"/>
        <source>Converts a rgba32 data object (with four channels blue, green, red, alpha) into 
an output data object of type &apos;uint8&apos; and a shape that has one dimension more than the input object and the first dimension is equal to 4. 
The four color components are then distributed into the 4 planes of the first dimension. 

For instance a 4x5x3, rgba32 data objects leads to a 4x4x5x3 uint8 data object.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+88"/>
        <source>Reduces a [4x...xMxN] or [3x...xMxN] uint8 data object to a [...xMxN] rgba32 data object where the 
first dimension is merged into the color type. If the first dimension is equal to 4, the planes are used for the blue, green, red and alpha 
component, in case of three, the alpha component is set to the optional alpha value.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+110"/>
        <location line="+118"/>
        <source>Resizes an image 

The function resize resizes the image &apos;inputObject&apos; down to or up by the specific factors. 

To shrink an image, it will generally look best with CV_INTER_AREA interpolation, whereas to enlarge an image, 
it will generally look best with CV_INTER_CUBIC (slow) or CV_INTER_LINEAR (faster but still looks OK). 
The axisScale properties of the x- and y-axes of the outputObject are divided by fx and fy respectively, while the offset values are multiplied with fx and fy.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+134"/>
        <source>Canny Edge detector using cv::DFT.

It&apos;s just Canny&apos;s edge filter
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+212"/>
        <source>Converts an image from one color space to another.
In case of linear transformations, the range does not matter. But in case of a non-linear transformation,
an input RGB image should be normalized to the proper value range to get the correct results, for example,
for RGB -&gt; L*u*v* transformation. For example, if you have a 32-bit floating-point image directly
converted from an 8-bit image without any scaling, then it will have the 0..255 value range instead of 0..1
assumed by the function. So, before calling cvtColor , you need first to scale the image down

The parameter code defines the conversion:

* RGB &lt;-&gt; GRAY ( CV_BGR2GRAY = 6, CV_RGB2GRAY = 7 , CV_GRAY2BGR = 8, CV_GRAY2RGB = 8)
* RGB &lt;-&gt; CIE XYZ.Rec 709 with D65 white point ( CV_BGR2XYZ = 32, CV_RGB2XYZ = 33, CV_XYZ2BGR = 34, CV_XYZ2RGB = 35)
* RGB &lt;-&gt; YCrCb JPEG (or YCC) ( CV_BGR2YCrCb = 36, CV_RGB2YCrCb = 37, CV_YCrCb2BGR = 38, CV_YCrCb2RGB = 39)
* RGB &lt;-&gt; HSV ( CV_BGR2HSV = 40, CV_RGB2HSV = 41, CV_HSV2BGR = 54, CV_HSV2RGB = 55 )
* RGB &lt;-&gt; HLS ( CV_BGR2HLS = 52, CV_RGB2HLS = 53, CV_HLS2BGR = 60, CV_HLS2RGB = 61)
* RGB &lt;-&gt; CIE L*a*b* ( CV_BGR2Lab = 44, CV_RGB2Lab = 45, CV_Lab2BGR = 56, CV_Lab2RGB = 57)
* RGB &lt;-&gt; CIE L*u*v* ( CV_BGR2Luv = 50, CV_RGB2Luv = 51, CV_Luv2BGR = 58, CV_Luv2RGB = 59)
* Bayer &lt;-&gt; RGB ( CV_BayerBG2BGR = 46, CV_BayerGB2BGR = 47, CV_BayerRG2BGR = 48, CV_BayerGR2BGR = 49, ...
    CV_BayerBG2RGB = 48, CV_BayerGB2RGB = 49, CV_BayerRG2RGB = 46, CV_BayerGR2RGB = 47)

For more details see OpenCV documentation.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+270"/>
        <source>Applies a fixed-level threshold to each array element.. 

The function applies fixed-level thresholding to a multiple-channel array. 
The function is typically used to get a bi-level (binary) image out of a grayscale image (compare could be also used for this purpose)
or for removing a noise, that is, filtering out pixels with too small or too large values. 
There are several types of thresholding supported by the function. They are determined by type parameter.

Also, the special values THRESH_OTSU or THRESH_TRIANGLE may be combined with one of the above values. 
In these cases, the function determines the optimal threshold value using the Otsu&apos;s or Triangle algorithm and uses it instead of the specified thresh.

Note: 
Currently, the Otsu&apos;s and Triangle methods are implemented only for 8-bit single-channel images.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+135"/>
        <source>Calculates an affine matrix of 2D rotation.
The function calculates the following matrix:

| alpha beta  (1 - alpha) * center.x - beta * center.y        |
|- beta alpha beta * center.x        + (1 - alpha) * center.y |

where
alpha = scale * cos(angle), beta = scale * sin(angle)
The transformation maps the rotation center to itself. This is not the target, adjust the shift.
Thr rotation can be applied by using e. g. the cvWarpAffine filter.

Note: 
When you want to use the cvWarpAffine method with this rotation matrix your center coordinates must be in the pixel domain.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+108"/>
        <source>Applies an affine transformation onto a 2D dataObject.
The function warpAffine transforms the source dataObject using the specified matrix:

dst(x,y)=src(M11x+M12y+M13,M21x+M22y+M23):

When the flag WARP_INVERSE_MAP is set.
Otherwise, the transformation is first inverted with invertAffineTransform
and then put in the formula above instead of M.

Note: 
The rotation matrix of the cvGetRotationMatrix2D filter can be used.
The matrix must correspond to the pixel domain.

No metaInformation is set to the destinationObj because the physical units 
of the target object differ from each other depending on the algorithm parameter.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../features2d.cpp" line="-294"/>
        <source>This function uses the nearest search methods to find the best matching points. Matching methods by means of Flann matcher. 
This includes some nearest neighbour algorithms to calculate the distance between two points. 

If desired, this function can also return a filtered list of matches and keypoints (keypoints1 and keypoints2) that only contain matches and keypoints whose matched distances 
are bounded by max_distance. You only need to indicate parameters belonging to the best-matching process if this max_distance parameter is &gt; 0.0.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+145"/>
        <source>Draws keypoints.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+114"/>
        <source>Draw the obtained matches points between two images. 
This function draws matches of keypoints from two images in the output image. Match is a line connecting two keypoints (circles).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location filename="../calib3d.cpp" line="-1265"/>
        <source>Finds circles in a grayscale image using the Hough transform.

This filter is a wrapper for the OpenCV-function cv::HoughCircles.The function finds circles in a grayscale image using a modification of the Hough transform.Based on this filter, circles are identified and located.The result is a dataObject where the number of rows corresponds to the number of found circles, each row is (x,y,r).
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+119"/>
        <source>Finds the positions of internal corners of the chessboard.

This filter is a wrapper for the cv::method cv::findChessboardCorners. 
The openCV-function attempts to determine whether the input image is a view of the chessboard pattern and locate the internal chessboard corners. The function returns a non-zero value if all of the corners are found and they are placed in a certain order (row by row, left to right in every row). Otherwise, if the function fails to find all the corners or reorder them, it returns 0. For example, a regular chessboard has 8 x 8 squares and 7 x 7 internal corners, that is, points where the black squares touch each other. The detected coordinates are approximate, and to determine their positions more accurately, the function calls cornerSubPix(). 

Remark 1: This function gives only a rough estimation of the positions. For a higher resolutions, you should usethe function cornerSubPix() with different parameters if returned coordinates are not accurate enough.This function is wrapped to itom by the filter &apos;cvCornerSubPix&apos;.

Remark 2: The outer frame of the dataObject / the image should not be white but have approximately the same gray value than the bright field.

Remark 3: The bright fields should be free of darker dirt or dust and you should apply a corse shading correction to improve the results. 
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+85"/>
        <source>Renders the detected chessboard corners.

The function draws individual chessboard corners detected either as red circles if the board was not found, or as colored corners connected with lines if the board was found.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+68"/>
        <source>Refines the corner locations e.g. from cvFindChessboardCorners.

This filter is a wrapper for the cv::method cv::cornerSubPix. Check the openCV-doku for more details
</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+81"/>
        <source>Finds the camera intrinsic and extrinsic parameters from several views of a calibration pattern. 

The function estimates the intrinsic camera parameters and extrinsic parameters for each of the views. The coordinates of 3D object points and their corresponding 2D projections in each view must be specified. 
That may be achieved by using an object with a known geometry and easily detectable feature points. Such an object is called a calibration rig or calibration pattern, and OpenCV has built-in support for 
a chessboard as a calibration rig (see cvFindChessboardCorners()). Currently, initialization of intrinsic parameters (when CV_CALIB_USE_INTRINSIC_GUESS is not set) is only implemented for planar 
calibration patterns (where Z-coordinates of the object points must be all zeros). 3D calibration rigs can also be used as long as initial cameraMatrix is provided.

The algorithm performs the following steps: 

1. Compute the initial intrinsic parameters (the option only available for planar calibration patterns) or read them from the input parameters. The distortion coefficients are all set to zeros initially unless some of CV_CALIB_FIX_K? are specified. 
2. Estimate the initial camera pose as if the intrinsic parameters have been already known. This is done using solvePnP() . 
3. Run the global Levenberg-Marquardt optimization algorithm to minimize the reprojection error, that is, the total sum of squared distances between the observed feature points imagePoints and the projected (using the current estimates for camera parameters and the poses) object points objectPoints. See projectPoints() for details. 

If the reprojectionError is NaN, one or both of the matrices objectPoints or imagePoints probabily contains any NaN-value after truncation. Remember that this algorithm truncates objectPoints and imagePoints 
before using it in the way that for each view, the last rows are cut where either the value in the first column of objectPoints or imagePoints is non-finite.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+198"/>
        <source>Computes an optimal affine transformation between two 3D point sets 

The function estimates an optimal 3D affine transformation between two 3D point sets using the RANSAC algorithm. The transformation describes then 
[destination;1] = output * [source;1] for each point in sources and destinations 3D point set.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+73"/>
        <source>Transforms an image to compensate for lens distortion. 

The function transforms an image to compensate radial and tangential lens distortion. 
The function is simply a combination of cvInitUndistortRectifyMap() (with unity R) and cvRemap() (with bilinear interpolation). 
See the former function for details of the transformation being performed. 

Those pixels in the destination image, for which there is no correspondent pixels in the source image, are filled with zeros (black color).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+72"/>
        <source>Computes the ideal point coordinates from the observed point coordinates. 

The function is similar to cvUndistort() and cvInitUndistortRectifyMap() but it operates on a sparse set of points instead of a raster image. Also the function performs a reverse transformation to cvProjectPoints() . 
In case of a 3D object, it does not reconstruct its 3D coordinates, but for a planar object, it does, up to a translation vector, if the proper R is specified.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+81"/>
        <source>Computes the undistortion and rectification transformation map.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+73"/>
        <source>Applies a generic geometrical transformation to an image. 

The function remap transforms the source image using the specified map: 

dst(x,y) = src(map_x(x, y), map_y(x, y)) 

where values of pixels with non-integer coordinates are computed using one of available interpolation methods. map_x and map_y can be encoded as 
separate floating-point maps in map_1 and map_2 respectively, or interleaved floating-point maps of (x,y) in map_1 , 
or fixed-point maps created by using convertMaps() . The reason you might want to convert from floating to fixed-point representations of a map is 
that they can yield much faster (~2x) remapping operations. In the converted case, map_1 contains pairs (cvFloor(x), cvFloor(y)) and map_2 contains 
indices in a table of interpolation coefficients.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+90"/>
        <source>Finds a perspective transformation between two planes. 

The functions find and return the perspective transformation H between the source and the destination planes: 

.. math:: s_i \begin{bmatrix}{x&apos;_i}\\{y&apos;_i}\\{1}\end{bmatrix} \sim H \begin{bmatrix}{x_i}\\{y_i}\\{1}\end{bmatrix} 

so that the back-projection error 

.. math:: \sum _i \left(x&apos;_i- \frac{h_{11} x_i + h_{12} y_i + h_{13}}{h_{31} x_i + h_{32} y_i + h_{33}} \right)^2 + \left(y&apos;_i- \frac{h_{21} x_i + h_{22} y_i + h_{23}}{h_{31} x_i + h_{32} y_i + h_{33}} \right)^2 

is minimized. 

The function is used to find initial intrinsic and extrinsic matrices. Homography matrix is determined up to a scale. Thus, it is normalized so that h_{33}=1.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+75"/>
        <source>Calculates a fundamental matrix from the corresponding points in two images. 

The epipolar geometry is described by the following equation: 

.. math:: [p_2; 1]^T F [p_1; 1] = 0 

where F is a fundamental matrix, p_1 and p_2 are corresponding points in the first and the second images, respectively. 

The function calculates the fundamental matrix using one of four methods listed above and returns the found fundamental matrix. 
Normally just one matrix is found. But in case of the 7-point algorithm, the function may return up to 3 solutions (9 	imes 3 matrix that stores all 3 matrices sequentially).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+88"/>
        <source>For points in an image of a stereo pair, computes the corresponding epilines in the other image. 

For every point in one of the two images of a stereo pair, the function finds the equation of the corresponding epipolar line in the other image. 

From the fundamental matrix definition (see findFundamentalMat()), line l^{(2)}_i in the second image for the point p^{(1)}_i in the first image (when whichImage=1) is computed as: 

.. math:: l^{(2)}_i = F p^{(1)}_i 

And vice versa, when whichImage=2, l^{(1)}_i is computed from p^{(2)}_i as: 

.. math:: l^{(1)}_i = F^T p^{(2)}_i 

Line coefficients are defined up to a scale. They are normalized so that 

.. math:: a_i^2+b_i^2=1.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+63"/>
        <source>Applies a perspective transformation to an image 

The function warpPerspective transforms the source image using the specified matrix H</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <location line="+63"/>
        <source>Project points from object into image space using the given calibration matrices,
distortion coefficients rotation and tralsation vector.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
