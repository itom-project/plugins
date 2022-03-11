/* ********************************************************************
    Plugin "OpenCV-Filter" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.
  
    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "OpenCVFilters.h"
#include "itomCvConversions.h"

#include "DataObject/dataobj.h"
#include "DataObject/dataObjectFuncs.h"
#include <qnumeric.h>

#if (CV_MAJOR_VERSION >= 2) //calib3d only available for OpenCV Version > 2.0
    #include "opencv2/calib3d/calib3d.hpp"

#if (CV_MAJOR_VERSION >= 4)
    #include "opencv2/highgui.hpp"
#else
    #include "opencv/highgui.h"
#endif

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvFindCirclesDoc = QObject::tr("Finds circles in a grayscale image using the Hough transform.\n\
\n\
This filter is a wrapper for the OpenCV-function cv::HoughCircles.\
The function finds circles in a grayscale image using a modification of the Hough transform.\
Based on this filter, circles are identified and located.\
The result is a dataObject where the number of rows corresponds to the number of found circles, each row is (x,y,r).\n\
");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFindCirclesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        // Mandatory Parameters
        ito::Param param = ito::Param("image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("input image of type uint8").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("circles", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("").toLatin1().data());
        paramsMand->append(param);

        // Optional Parameters
        param = ito::Param("dp", ito::ParamBase::Double | ito::ParamBase::In, 1.0, 100.0, 1.0, tr("dp: Inverse ratio of the accumulator resolution to the image resolution.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("min_dist", ito::ParamBase::Double | ito::ParamBase::In, 1.0, 100000.0, 20.0, tr("Minimum center distance of the circles.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("canny_threshold", ito::ParamBase::Double | ito::ParamBase::In, 1.0, 255.0, 200.0, tr("The higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("acc_threshold", ito::ParamBase::Double | ito::ParamBase::In, 1.0, 255.0, 100.0, tr("The accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("min_radius", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, tr("Min Radius in x/y").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("max_radius", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, tr("Max Radius in x/y (if 0: the maximum of the image width or height is taken)").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFindCircles(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
#if TIMEBENCHMARK
    int64 teststart = cv::getTickCount();
#endif

    ito::RetVal retval = ito::retOk;

    // Initialize pointers to the input and output dataObjects
    ito::DataObject input = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "image", ito::Range::all(), ito::Range::all(), retval, -1, 1, ito::tUInt8);
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();


    // Check if destination dataObject exists
    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (!retval.containsError())
    {

        double dp = (*paramsOpt)[0].getVal<double>();
        double MinDist = (*paramsOpt)[1].getVal<double>();
        double Threshold = (*paramsOpt)[2].getVal<double>();
        double AccThreshold = (*paramsOpt)[3].getVal<double>();
        int MinRadius = (*paramsOpt)[4].getVal<int>();
        int MaxRadius = (*paramsOpt)[5].getVal<int>();

        // Copy input image to a cv Mat
        const cv::Mat *cvplaneIn = input.getCvPlaneMat(0);

        // Declare the output vector to hold the circle coordinates and radii
#if (CV_MAJOR_VERSION >= 3)
        std::vector<cv::Vec3f> circles;
        int method = cv::HOUGH_GRADIENT;
#else
        cv::vector<cv::Vec3f> circles;
        int method = CV_HOUGH_GRADIENT;
#endif

        /*    void HoughCircles(InputArray image, OutputArray circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0)
            dp – Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
            param1 – First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
            param2 – Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.*/
        cv::HoughCircles(*cvplaneIn, circles, method, dp, MinDist, Threshold, AccThreshold, MinRadius, MaxRadius);


        // Copy the circles into the output dataObject
        if (circles.size() > 0)
        {
            int sizes[2] = { (int)circles.size(), 3 };
            *dObjDst = ito::DataObject(2, sizes, ito::tFloat32);
            ito::float32 *rowPtr = NULL;

            for (size_t i = 0; i < circles.size(); i++)
            {
                //    This can be used to draw the circles directly into the input image
                /*    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                    int radius = cvRound(circles[i][2]);
                    circle(*cvplaneIn, center, radius, 70, 3, 8, 0);*/

                rowPtr = (ito::float32*)dObjDst->rowPtr(0, i);
                rowPtr[0] = circles[i][0];
                rowPtr[1] = circles[i][1];
                rowPtr[2] = circles[i][2];
            }
        }
    }

#if TIMEBENCHMARK
    int64 testend = cv::getTickCount() - teststart;
    double duration = (double)testend / cv::getTickFrequency();
    std::cout << "cvFindCircles: " << circles.size() << " circles found in " << duration << "ms\n";
#endif

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvFindChessboardCornersDoc = QObject::tr("Finds the positions of internal corners of the chessboard.\n\
\n\
This filter is a wrapper for the cv::method cv::findChessboardCorners. \
\n\
The openCV-function attempts to determine whether the input image is a view of the chessboard pattern and locate the internal chessboard corners. \
The function returns a non-zero value if all of the corners are found and they are placed in a certain order (row by row, left to right in every row). \
Otherwise, if the function fails to find all the corners or reorder them, \
it returns 0. For example, a regular chessboard has 8 x 8 squares and 7 x 7 internal corners, that is, points where the black squares touch each other. \
The detected coordinates are approximate, and to determine their positions more accurately, the function calls cornerSubPix(). \n\
\n\
Remark 1: This function gives only a rough estimation of the positions. For a higher resolutions, you should use\
the function cornerSubPix() with different parameters if returned coordinates are not accurate enough.\
This function is wrapped to itom by the filter 'cvCornerSubPix'.\n\
\n\
Remark 2: The outer frame of the dataObject / the image should not be white but have approximately the same gray value than the bright field.\n\
\n\
Remark 3: The bright fields should be free of darker dirt or dust and you should apply a corse shading correction to improve the results. \n\
");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFindChessboardCornersParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("dataObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "8bit grayscale input image"));
    paramsMand->append(ito::Param("patternSize", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, "Number of inner corners per chessboard row and column (points_per_row, points_per_column)"));
    paramsMand->append(ito::Param("corners", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output: float32-dataObject, [n x 2] with the coordinates of n detected corner points"));

    int allflags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK;
    QString flagsdocs = tr("OR Combination of various flags: \n\n");
    flagsdocs += QString("* CV_CALIB_CB_ADAPTIVE_THRESH (%1) - Use adaptive thresholding to convert the image to black and white, rather than a fixed threshold level (computed from the average image brightness) [default], \n").arg(cv::CALIB_CB_ADAPTIVE_THRESH);
    flagsdocs += QString("* CV_CALIB_CB_NORMALIZE_IMAGE (%1) - Normalize the image gamma with equalizeHist() before applying fixed or adaptive thresholding [default], \n").arg(cv::CALIB_CB_NORMALIZE_IMAGE);
    flagsdocs += QString("* CV_CALIB_CB_FILTER_QUADS (%1) - Use additional criteria (like contour area, perimeter, square-like shape) to filter out false quads extracted at the contour retrieval stage, \n").arg(cv::CALIB_CB_FILTER_QUADS);
    flagsdocs += QString("* CALIB_CB_FAST_CHECK (%1) - Run a fast check on the image that looks for chessboard corners, and shortcut the call if none is found. This can drastically speed up the call in the degenerate condition when no chessboard is observed (recommended to pre-check image).").arg(cv::CALIB_CB_FAST_CHECK);
    paramsOpt->append(ito::Param("flags", ito::ParamBase::Int | ito::ParamBase::In, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE, new ito::IntMeta(0, allflags), flagsdocs.toLatin1().data()));

    paramsOut->append(ito::Param("result", ito::ParamBase::Int | ito::ParamBase::Out, 0, new ito::IntMeta(0,1), "0: detection failed, 1: detection has been successful"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFindChessboardCorners(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    const ito::DataObject *input = (const ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    cv::Size patternSize = itomcv::getCVSizeFromParam(paramsMand->at(1), false, &retval);
    int flags = (*paramsOpt)[0].getVal<int>();
    int result = 0;

    if ((*paramsMand)[2].getVal<ito::DataObject*>() == NULL || input == NULL)
    {
        retval += ito::RetVal(ito::retError,0,"parameters 'dataObject' or 'corners' must not be NULL");
    }

    if (!retval.containsError())
    {
        const cv::Mat *input_ = input->getCvPlaneMat(0);
        cv::Mat corners_;

        try
        {
            result = cv::findChessboardCorners(*input_, patternSize, corners_, flags);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
            result = 0;
        }

        if (result && !retval.containsError())
        {
            corners_ = corners_.reshape(1);
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[2],&corners_);
        }
    }

    (*paramsOut)[0].setVal<int>(result);
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvDrawChessboardCornersDoc = QObject::tr("Renders the detected chessboard corners.\n\
\n\
The function draws individual chessboard corners detected either as red circles if the board was not found, or as colored corners connected with lines if the board was found.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvDrawChessboardCornersParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("image", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("rgba32 input and destination image (must be of type ito::rgba32).").toLatin1().data()));
    paramsMand->append(ito::Param("patternSize", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("Number of inner corners per chessboard row and column (points_per_row, points_per_column)").toLatin1().data()));
    paramsMand->append(ito::Param("corners", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("array of detected corners (n x 2), the output of cvFindChessboardCorners or cvCornerSubPix").toLatin1().data()));
    paramsMand->append(ito::Param("patternWasFound", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("Parameter indicating whether the complete board was found or not.").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvDrawChessboardCorners(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    ito::DataObject *image = paramsMand->at(0).getVal<ito::DataObject*>();
    retval += ito::dObjHelper::verify2DDataObject(image, "image", 1, std::numeric_limits<int>::max(), 1, std::numeric_limits<int>::max(), 1, ito::tRGBA32);

    cv::Size patternSize = itomcv::getCVSizeFromParam((*paramsMand)[1], false, &retval);

    int limits[] = {0, std::numeric_limits<int>::max(),2,2};
    ito::DataObject *corners = apiCreateFromNamedDataObject((*paramsMand)[2].getVal<ito::DataObject*>(),2,ito::tFloat32,"corners",limits,&retval);

    bool patternWasFound = (paramsMand->at(3).getVal<int>() > 0);

    if (!retval.containsError())
    {
        cv::Mat image_ = *(image->getCvPlaneMat(0)); //image_ has four channels, the alpha channel will be set to 0 by drawChessboardCorners. This needs to be corrected to 255 again afterwards.

        try
        {
            cv::drawChessboardCorners(image_, patternSize, corners->getContinuousCvPlaneMat(0), patternWasFound);

            ito::uint8* rowPtr;
            for (int row = 0; row < image_.rows; ++row)
            {
                rowPtr = image_.ptr<ito::uint8>(row);
                for (int col = 3; col < image_.cols * 4; col += 4)
                {
                    rowPtr[col] = 255;
                }
            }

            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[0], &image_);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }
        
    }

    delete corners;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvCornerSubPixDoc = QObject::tr("Refines the corner locations e.g. from cvFindChessboardCorners.\n\
\n\
This filter is a wrapper for the cv::method cv::cornerSubPix. Check the openCV-doku for more details\n\
");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvCornerSubPixParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("8bit grayscale input image").toLatin1().data()));
    paramsMand->append(ito::Param("corners", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("initial coordinates of the input corners and refined coordinates provided for output").toLatin1().data()));
    paramsMand->append(ito::Param("winSize", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("Half of the side length of the search window. Example: (5,5) leads to a (11x11) search window").toLatin1().data()));

    int zeroZone[] = {-1,-1};
    param = ito::Param("zeroZone", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("Half of the size of the dead region in the middle of the search zone over which the summation is not done. (-1,-1) indicates that there is no such a size").toLatin1().data());
    param.setVal<int*>(zeroZone, 2);
    paramsOpt->append (param);

    paramsOpt->append(ito::Param("maxCount", ito::ParamBase::Int | ito::ParamBase::In, 200, new ito::IntMeta(1,100000), tr("position refinement stops after this maximum number of iterations").toLatin1().data()));
    paramsOpt->append(ito::Param("epsilon", ito::ParamBase::Double | ito::ParamBase::In, 0.05, new ito::DoubleMeta(0.0, 10.0), tr("position refinement stops when the corner position moves by less than this value").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvCornerSubPix(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    ito::DataObject *image = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *rawCorners = (*paramsMand)[1].getVal<ito::DataObject*>();

    if (image == NULL || rawCorners == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Parameters 'image' and 'corners' may not be NULL").toLatin1().data());
    }

    int limits[] = {0, std::numeric_limits<int>::max(), 2, 2};
    ito::DataObject *corners = apiCreateFromNamedDataObject(rawCorners, 2, ito::tFloat32, "corners", limits, &retval);

    cv::Size winSize = itomcv::getCVSizeFromParam(paramsMand->at(2), true, &retval);
    
    cv::Size zeroZone = itomcv::getCVSizeFromParam(paramsOpt->at(0), true, &retval);

    cv::TermCriteria criteria = itomcv::getCVTermCriteriaFromParam(paramsOpt->at(1), paramsOpt->at(2), &retval);

    if (!retval.containsError())
    {
        const cv::Mat *image_ = image->getCvPlaneMat(0);
        cv::Mat *corners_ = corners->getCvPlaneMat(0);

        if (corners_->isContinuous() == false)
        {
            retval += ito::RetVal(ito::retError, 0, tr("corners must be a continuous data object. If you passed a subset of columns of a data object, get a region-only copy before calling this function").toLatin1().data());
        }
        else
        {
            try
            {
                cv::cornerSubPix(*image_, *corners_, winSize, zeroZone, criteria);

                retval += itomcv::setOutputArrayToDataObject((*paramsMand)[1], corners_);
            }
            catch (cv::Exception exc)
            {
                retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
            }
        }
    }

    delete corners;
    corners = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvCalibrateCameraDoc = QObject::tr("Finds the camera intrinsic and extrinsic parameters from several views of a calibration pattern. \n\
\n\
The function estimates the intrinsic camera parameters and extrinsic parameters for each of the views. The coordinates of 3D object points and their corresponding 2D projections in each view must be specified. \n\
That may be achieved by using an object with a known geometry and easily detectable feature points. Such an object is called a calibration rig or calibration pattern, and OpenCV has built-in support for \n\
a chessboard as a calibration rig (see cvFindChessboardCorners()). Currently, initialization of intrinsic parameters (when CV_CALIB_USE_INTRINSIC_GUESS is not set) is only implemented for planar \n\
calibration patterns (where Z-coordinates of the object points must be all zeros). 3D calibration rigs can also be used as long as initial cameraMatrix is provided.\n\
\n\
The algorithm performs the following steps: \n\
\n\
1. Compute the initial intrinsic parameters (the option only available for planar calibration patterns) or read them from the input parameters. The distortion coefficients are all set to zeros initially unless some of CV_CALIB_FIX_K? are specified. \n\
2. Estimate the initial camera pose as if the intrinsic parameters have been already known. This is done using solvePnP() . \n\
3. Run the global Levenberg-Marquardt optimization algorithm to minimize the reprojection error, that is, the total sum of squared distances between the observed feature points imagePoints and the projected (using the current estimates for camera parameters and the poses) object points objectPoints. See projectPoints() for details. \n\
\n\
If the reprojectionError is NaN, one or both of the matrices objectPoints or imagePoints probabily contains any NaN-value after truncation. Remember that this algorithm truncates objectPoints and imagePoints \n\
before using it in the way that for each view, the last rows are cut where either the value in the first column of objectPoints or imagePoints is non-finite.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvCalibrateCameraParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("objectPoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("[NrOfViews x NrOfPoints x 3] float32 matrix with the coordinates of all points in object space (coordinate system of calibration pattern).. Non-finite rows at the end of each matrix-plane will be truncated.").toLatin1().data()));
    paramsMand->append(ito::Param("imagePoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("[NrOfViews x NrOfPoints x 2] float32 matrix with the pixel coordinates (u,v) of the corresponding plane in each view. Non-finite rows at the end of each matrix-plane will be truncated.").toLatin1().data()));
    paramsMand->append(ito::Param("imageSize", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("[height,width] of the camera image (in pixels)").toLatin1().data()));

    paramsMand->append(ito::Param("cameraMatrix", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output 3x3 float64 camera patrix. If flags CV_CALIB_USE_INTRINSIC_GUESS and/or CV_CALIB_FIX_ASPECT_RATIO are specified, this matrix must be initialized with right values and is unchanged").toLatin1().data()));
    paramsMand->append(ito::Param("distCoeffs", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output 1x4, 1x5 or 1x8 distortion values (float64). (k1, k2, p1, p2 [,k3 [,k4 ,k5 ,k6]])").toLatin1().data()));
    paramsMand->append(ito::Param("rvecs", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("3 x NrOfViews float64 output vector, where each column is the rotation vector estimated for each pattern view (Rodrigues coordinates)").toLatin1().data()));
    paramsMand->append(ito::Param("tvecs", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("3 x NrOfViews float64 output vector, where each column is the translation vector estimated for each pattern view").toLatin1().data()));

    QString description = tr("Different flags that may be a combination of the following values: ");
    description += QString("CV_CALIB_USE_INTRINSIC_GUESS (%1)").arg(cv::CALIB_USE_INTRINSIC_GUESS);
    description += QString(", CV_CALIB_FIX_PRINCIPAL_POINT (%1)").arg(cv::CALIB_FIX_PRINCIPAL_POINT);
    description += QString(", CV_CALIB_FIX_ASPECT_RATIO (%1)").arg(cv::CALIB_FIX_ASPECT_RATIO);
    description += QString(", CV_CALIB_ZERO_TANGENT_DIST (%1)").arg(cv::CALIB_ZERO_TANGENT_DIST);
    description += QString(", CV_CALIB_FIX_K1 (%1)").arg(cv::CALIB_FIX_K1);
    description += QString(", CV_CALIB_FIX_K2 (%1)").arg(cv::CALIB_FIX_K2);
    description += QString(", CV_CALIB_FIX_K3 (%1)").arg(cv::CALIB_FIX_K3);
    description += QString(", CV_CALIB_FIX_K4 (%1)").arg(cv::CALIB_FIX_K4);
    description += QString(", CV_CALIB_FIX_K5 (%1)").arg(cv::CALIB_FIX_K5);
    description += QString(", CV_CALIB_FIX_K6 (%1)").arg(cv::CALIB_FIX_K6);
    description += QString(", CV_CALIB_RATIONAL_MODEL (%1)").arg(cv::CALIB_RATIONAL_MODEL);
    int maxFlag = cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6 | cv::CALIB_RATIONAL_MODEL;
    paramsOpt->append(ito::Param("flags", ito::ParamBase::Int | ito::ParamBase::In, 0, new ito::IntMeta(0,maxFlag), description.toLatin1().data()));
    paramsOpt->append(ito::Param("maxCounts", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 30, tr("if > 0, maximum number of counts, 0: unlimited number of counts allowed [default: 30]").toLatin1().data()));
    paramsOpt->append(ito::Param("epsilonAccuracy", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<double>::max(), std::numeric_limits<double>::epsilon(), tr("if > 0.0, desired accuracy at which the iterative algorithm stops, 0.0: no epsilon criteria [default: DBL_EPSILON]").toLatin1().data()));
    
    paramsOut->append(ito::Param("reprojectionError", ito::ParamBase::Double | ito::ParamBase::Out, 0.0, std::numeric_limits<double>::max(), 0.0, tr("resulting re-projection error").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvCalibrateCamera(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    //mand-param 0: objectPoints
    int sizeLimits[] = {1,std::numeric_limits<int>::max(),0,std::numeric_limits<int>::max(),3,3};
    const ito::DataObject *objectPoints = apiCreateFromNamedDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), 3, ito::tFloat32, "objectPoints", sizeLimits, &retval);
    
    //mand-param 1: imagePoints
    sizeLimits[4]=sizeLimits[5]=2;
    if (objectPoints)
    {
        sizeLimits[0] = sizeLimits[1] = objectPoints->getSize(0);
    }
    const ito::DataObject *imagePoints = apiCreateFromNamedDataObject(paramsMand->at(1).getVal<ito::DataObject*>(), 3, ito::tFloat32, "imagePoints", sizeLimits, &retval);
    
    //mand-param 2: imageSize
    cv::Size imageSize = itomcv::getCVSizeFromParam(paramsMand->at(2), false, &retval);

    //mand-param 3: cameraMatrix (maybe empty or 3x3)
    ito::RetVal retvalTemp;
    int sizeLimits2[] = {3,3,3,3};
    const ito::DataObject *cameraMatrix = apiCreateFromNamedDataObject(paramsMand->at(3).getVal<ito::DataObject*>(), 2, ito::tFloat64, "cameraMatrix", sizeLimits2, &retvalTemp);

    //mand-param 4: distCoeffs (maybe empty or 1x4, 1x5 or 1x8)
    retvalTemp = ito::retOk;
    int sizeLimits3[] = {1,1,4,8};
    const ito::DataObject *distCoeffs = apiCreateFromNamedDataObject(paramsMand->at(4).getVal<ito::DataObject*>(), 2, ito::tFloat64, "distCoeffs", sizeLimits3, &retvalTemp);

    //mand-param 5+6 are out only and will be written later
    
    //opt-param 0: flags
    int flags = paramsOpt->at(0).getVal<int>();

    //opt-param 1+2: termCriteria
    cv::TermCriteria criteria = itomcv::getCVTermCriteriaFromParam(paramsOpt->at(1), paramsOpt->at(2), &retval);

    //out-param will be written at function call of cv::calibrateCamera

    if (!retval.containsError() && objectPoints && imagePoints)
    {
        std::vector<cv::Mat> objectPoints_ = itomcv::getInputArrayOfArraysFromDataObject(objectPoints, true);
        std::vector<cv::Mat> imagePoints_ = itomcv::getInputArrayOfArraysFromDataObject(imagePoints, true);

        //adjust rois of objectPoints_ and imagePoints_ such that rows with non-finite values in the first column are truncated at the end
        cv::Mat *mat1, *mat2;
        int truncates;

        //truncate imagePoints_ and objectPoints_ (assumption: both have the same size and the last rows in each matrix are truncated whose one of the items in the first column is non-finite)
        for (std::vector<cv::Mat>::size_type i = 0; i < objectPoints_.size(); ++i)
        {
            mat1 = &(objectPoints_[i]);
            mat2 = &(imagePoints_[i]);
            truncates = 0;

            for (int r = mat1->rows - 1; r >= 0; --r)
            {
                if (!qIsFinite(mat1->ptr<ito::float32>(r)[0]) || !qIsFinite(mat2->ptr<ito::float32>(r)[0]))
                {
                    ++truncates;
                }
                else
                {
                    break;
                }
            }

            mat1->adjustROI(0,-truncates,0,0);
            mat2->adjustROI(0,-truncates,0,0);
        }

        std::vector<cv::Mat> rvecs, tvecs;
        cv::Mat cameraMatrix_;
        cv::Mat distCoeffs_;
        if (cameraMatrix)
        {
            cameraMatrix_ = *(cameraMatrix->getCvPlaneMat(0));
        }
        if (distCoeffs)
        {
            distCoeffs_ = *(distCoeffs->getCvPlaneMat(0));
        }

        try
        {
            (*paramsOut)[0].setVal<double>(cv::calibrateCamera(objectPoints_, imagePoints_, imageSize, cameraMatrix_, distCoeffs_, rvecs, tvecs, flags, criteria));
        }
        catch (cv::Exception &exc)
        {
            retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
        }

        if (!retval.containsError())
        {
            //write cameraMatrix back
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[3], &cameraMatrix_);

            //write distCoeffs back
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[4], &distCoeffs_);
        }

        if (!retval.containsError())
        {
            //write rvecs and tvecs back

            //mand-param 5: rvecs (out only)
            *((*paramsMand)[5].getVal<ito::DataObject*>()) = ito::DataObject(3, rvecs.size(), ito::tFloat64);
            ito::float64 *rvecsdata = (ito::float64*)((*paramsMand)[5].getVal<ito::DataObject*>()->rowPtr(0,0));
            
            int s = rvecs.size();
            int sizes[] = {3,s};

            for (int i = 0; i < s; ++i)
            {
                rvecsdata[i] = rvecs[i].at<ito::float64>(0);
                rvecsdata[s*1+i] = rvecs[i].at<ito::float64>(1);
                rvecsdata[s*2+i] = rvecs[i].at<ito::float64>(2);
            }

            //mand-param 6: rvecs (out only)
            *((*paramsMand)[6].getVal<ito::DataObject*>()) = ito::DataObject(3, tvecs.size(), ito::tFloat64);
            ito::float64 *tvecsdata = (ito::float64*)((*paramsMand)[6].getVal<ito::DataObject*>()->rowPtr(0,0));
            s = tvecs.size();
            sizes[1] = s;
            for (int i = 0; i < s; ++i)
            {
                tvecsdata[i] = tvecs[i].at<ito::float64>(0);
                tvecsdata[s*1+i] = tvecs[i].at<ito::float64>(1);
                tvecsdata[s*2+i] = tvecs[i].at<ito::float64>(2);
            }
        }
    }

    delete objectPoints;
    delete imagePoints;
    delete cameraMatrix;
    delete distCoeffs;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvEstimateAffine3DDoc = QObject::tr("Computes an optimal affine transformation between two 3D point sets \n\
\n\
The function estimates an optimal 3D affine transformation between two 3D point sets using the RANSAC algorithm. The transformation describes then \n\
[destination;1] = output * [source;1] for each point in sources and destinations 3D point set.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvEstimateAffine3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("sources", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("[n x 3] array of source points (will be converted to float64).").toLatin1().data()));
    paramsMand->append(ito::Param("destinations", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("[n x 3] array of destination points (must have the same size than sources, will be converted to float64).").toLatin1().data()));
    paramsMand->append(ito::Param("output", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output 3D affine transformation matrix 3x4 (float64)").toLatin1().data()));
    paramsOpt->append(ito::Param("inliers", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output vector indicating which points are inliers (uint8)").toLatin1().data()));
    paramsOpt->append(ito::Param("ransacThreshold", ito::ParamBase::Double | ito::ParamBase::In, 3.0, new ito::DoubleMeta(0.0, std::numeric_limits<double>::max()), tr("Maximum reprojection error in the RANSAC algorithm to consider a point as an inlier (default: 3.0)").toLatin1().data()));
    paramsOpt->append(ito::Param("confidence", ito::ParamBase::Double | ito::ParamBase::In, 0.99, new ito::DoubleMeta(0.0, 1.0), tr("Confidence level, between 0 and 1, for the estimated transformation. Anything between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.").toLatin1().data()));
    paramsOut->append(ito::Param("ret", ito::ParamBase::Int | ito::ParamBase::Out, 0, 1, 0, tr("return value").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvEstimateAffine3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    const ito::DataObject *sources = paramsMand->at(0).getVal<const ito::DataObject*>();
    retval += ito::dObjHelper::verify2DDataObject(sources, "sources", 4, std::numeric_limits<int>::max(), 3, 3, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);

    const ito::DataObject *destinations = paramsMand->at(1).getVal<const ito::DataObject*>();
    if (sources)
    {
        retval += ito::dObjHelper::verify2DDataObject(destinations, "destinations", sources->getSize(1), sources->getSize(1), 3, 3, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    }

    ito::DataObject *inliers = paramsOpt->at(0).getVal<ito::DataObject*>();

    double ransacThreshold = paramsOpt->at(1).getVal<double>();
    double confidence = paramsOpt->at(2).getVal<double>();

    if (!retval.containsError())
    {
        cv::Mat output_, inliers_;

        try
        {
            int ret = cv::estimateAffine3D(sources->getContinuousCvPlaneMat(0), destinations->getContinuousCvPlaneMat(0), output_, inliers_, ransacThreshold, confidence);

            if (ret > 0)
            {
                retval += itomcv::setOutputArrayToDataObject((*paramsMand)[2], &output_);

                if (inliers)
                {
                    retval += itomcv::setOutputArrayToDataObject((*paramsOpt)[0], &inliers_);
                }
            }

            (*paramsOut)[0].setVal<int>(ret);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvUndistortDoc = QObject::tr("Transforms an image to compensate for lens distortion. \n\
\n\
The function transforms an image to compensate radial and tangential lens distortion. \n\
The function is simply a combination of cvInitUndistortRectifyMap() (with unity R) and cvRemap() (with bilinear interpolation). \n\
See the former function for details of the transformation being performed. \n\
\n\
Those pixels in the destination image, for which there is no correspondent pixels in the source image, are filled with zeros (black color).");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvUndistortParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("source", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input (distorted) image (all datatypes)").toLatin1().data()));
    paramsMand->append(ito::Param("destination", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output (corrected) image that has the same size and type as source").toLatin1().data()));
    paramsMand->append(ito::Param("cameraMatrix", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input camera matrix A = [[fx 0 cx];[0 fy cy];[0 0 1]]").toLatin1().data()));
    paramsMand->append(ito::Param("distCoeffs", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input vector of distortion coefficients [1 x 4,5,8] (k1, k2, p1, p2 [, k3[, k4, k5, k6]]) of 4, 5 or 8 elements.").toLatin1().data()));
    paramsOpt->append(ito::Param("newCameraMatrix", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Camera matrix of the distorted image. By default (if not given), it is the same as cameraMatrix but you may additionally scale and shift the result by using a different matrix.").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvUndistort(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    
    ito::DataObject cameraMatrix = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(2).getVal<const ito::DataObject*>(), "cameraMatrix", ito::Range(3,3), ito::Range(3,3), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    ito::DataObject distCoeffs = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(3).getVal<const ito::DataObject*>(), "distCoeffs", ito::Range(1,1), ito::Range(4,8), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    
    ito::DataObject newCameraMatrix = paramsOpt->at(0).getVal<void*>() ? ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsOpt->at(0).getVal<const ito::DataObject*>(), "newCameraMatrix", ito::Range(3,3), ito::Range(3,3), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64) : cameraMatrix;

    ito::DataObject *destination = paramsMand->at(1).getVal<ito::DataObject*>();
    if (!destination)
    {
        retval += ito::RetVal(ito::retError, 0, tr("'destination' must not be NULL").toLatin1().data());
    }

    const ito::DataObject *source = paramsMand->at(0).getVal<const ito::DataObject*>();
    const ito::DataObject sourceSqueezed = source->squeeze();

    if (sourceSqueezed.getDims() != 2)
    {
        retval += ito::RetVal(ito::retError, 0, tr("source data object must be two dimensional").toLatin1().data());
    }
        
    if (!retval.containsError())
    {
        const cv::Mat *src = sourceSqueezed.getCvPlaneMat(0);
        cv::Mat dst;

        try
        {
            cv::undistort(*src, dst, *(cameraMatrix.getCvPlaneMat(0)), *(distCoeffs.getCvPlaneMat(0)), *(newCameraMatrix.getCvPlaneMat(0)));
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[1], &dst);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvUndistortPointsDoc = QObject::tr("Computes the ideal point coordinates from the observed point coordinates. \n\
\n\
The function is similar to cvUndistort() and cvInitUndistortRectifyMap() but it operates on a sparse set of points instead of a raster image. Also the function performs a reverse transformation to cvProjectPoints() . \n\
In case of a 3D object, it does not reconstruct its 3D coordinates, but for a planar object, it does, up to a translation vector, if the proper R is specified.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvUndistortPointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("source", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Observed point coordinates (Nx2) float32").toLatin1().data()));
    paramsMand->append(ito::Param("destination", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output (corrected) image that has the same size and type as source").toLatin1().data()));
    paramsMand->append(ito::Param("cameraMatrix", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input camera matrix A = [[fx 0 cx];[0 fy cy];[0 0 1]]").toLatin1().data()));
    paramsMand->append(ito::Param("distCoeffs", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input vector of distortion coefficients [1 x 4,5,8] (k1, k2, p1, p2 [, k3[, k4, k5, k6]]) of 4, 5 or 8 elements.").toLatin1().data()));
    paramsOpt->append(ito::Param("R", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Rectification transformation in the object space (3x3 matrix). If not given, the identity transformation is used.").toLatin1().data()));
    paramsOpt->append(ito::Param("P", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("New camera matrix (3x3) or new projection matrix (3x4). If not given, the identity new camera matrix is used.").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvUndistortPoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject src = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(),"source", ito::Range(1,INT_MAX), ito::Range(2,2), retval, -1, 1, ito::tFloat32);

    if (!paramsMand->at(1).getVal<ito::DataObject*>())
    {
        retval += ito::RetVal(ito::retError, 0, tr("destination is empty").toLatin1().data());
    }

    ito::DataObject cameraMatrix = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(2).getVal<const ito::DataObject*>(), "cameraMatrix", ito::Range(3,3), ito::Range(3,3), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    ito::DataObject distCoeffs = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(3).getVal<const ito::DataObject*>(), "distCoeffs", ito::Range(1,1), ito::Range(4,8), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);

    ito::DataObject R = paramsOpt->at(0).getVal<void*>() ? ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsOpt->at(0).getVal<const ito::DataObject*>(), "R", ito::Range(3,3), \
        ito::Range(3,3), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64) : ito::DataObject();
    ito::DataObject P = paramsOpt->at(1).getVal<void*>() ? ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsOpt->at(1).getVal<const ito::DataObject*>(), "P", ito::Range(3,4), \
        ito::Range(3,3), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64) : ito::DataObject();

    if (!retval.containsError())
    {
        cv::Mat dst;
        
        try
        {
            cv::Mat src_ = src.getCvPlaneMat(0)->reshape(2,0); //rows remain unchanged, 2 cols become 2 channels.

            if (R.getDims() == 0 && P.getDims() == 0)
            {
                cv::undistortPoints(src_, dst, *(cameraMatrix.getCvPlaneMat(0)), *(distCoeffs.getCvPlaneMat(0)));
            }
            else if (P.getDims() != 0 && R.getDims() != 0)
            {
                cv::undistortPoints(src_, dst, *(cameraMatrix.getCvPlaneMat(0)), *(distCoeffs.getCvPlaneMat(0)), *(R.getCvPlaneMat(0)), *(P.getCvPlaneMat(0)));
            }
            else if (P.getDims() == 0)
            {
                cv::undistortPoints(src_, dst, *(cameraMatrix.getCvPlaneMat(0)), *(distCoeffs.getCvPlaneMat(0)), *(R.getCvPlaneMat(0)));
            }
            else //R.getDims() == 0)
            {
                cv::undistortPoints(src_, dst, *(cameraMatrix.getCvPlaneMat(0)), *(distCoeffs.getCvPlaneMat(0)), cv::noArray(), *(P.getCvPlaneMat(0)));
            }
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[1], &dst);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvInitUndistortRectifyMapDoc = QObject::tr("Computes the undistortion and rectification transformation map.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvInitUndistortRectifyMapParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("cameraMatrix", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input camera matrix A = [[fx 0 cx];[0 fy cy];[0 0 1]]").toLatin1().data()));
    paramsMand->append(ito::Param("distCoeffs", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input vector of distortion coefficients [1 x 4,5,8] (k1, k2, p1, p2 [, k3[, k4, k5, k6]]) of 4, 5 or 8 elements.").toLatin1().data()));

    paramsMand->append(ito::Param("size", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("undistorted image size").toLatin1().data()));
    paramsMand->append(ito::Param("map1", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("The first output map, type is float32.").toLatin1().data()));
    paramsMand->append(ito::Param("map2", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("The second output map, type is float32.").toLatin1().data()));

    paramsOpt->append(ito::Param("R", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Rectification transformation in the object space (3x3 matrix). If not given, the identity transformation is used.").toLatin1().data()));
    paramsOpt->append(ito::Param("newCameraMatrix", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("New camera matrix A'. If not given, the camera matrix is used.").toLatin1().data()));
    
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvInitUndistortRectifyMap(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject cameraMatrix = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(2).getVal<const ito::DataObject*>(), "cameraMatrix", ito::Range(3,3), ito::Range(3,3), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    ito::DataObject distCoeffs = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(3).getVal<const ito::DataObject*>(), "distCoeffs", ito::Range(1,1), ito::Range(4,8), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    
    cv::Size size = itomcv::getCVSizeFromParam((*paramsMand)[2], false, &retval);

    if (paramsMand->at(3).getVal<void*>() == NULL || paramsMand->at(4).getVal<void*>() == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, tr("map1 or map2 are NULL").toLatin1().data());
    }

    ito::DataObject R = paramsOpt->at(0).getVal<void*>() ? ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsOpt->at(0).getVal<const ito::DataObject*>(), "R", ito::Range(3,3), \
        ito::Range(3,3), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64) : ito::DataObject();

    ito::DataObject cameraMatrixNew = (paramsOpt->at(1).getVal<void*>()) ? ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsOpt->at(1).getVal<const ito::DataObject*>(), "cameraMatrixNew", ito::Range(3,3), ito::Range(3,3), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64) : cameraMatrix;
    
    if (!retval.containsError())
    {
        cv::Mat map1, map2;
        
        try
        {
            if (R.getDims() > 0)
            {
                cv::initUndistortRectifyMap(*(cameraMatrix.getCvPlaneMat(0)), *(distCoeffs.getCvPlaneMat(0)), *(R.getCvPlaneMat(0)), *(cameraMatrixNew.getCvPlaneMat(0)), size, CV_32FC1, map1, map2);
            }
            else
            {
                cv::initUndistortRectifyMap(*(cameraMatrix.getCvPlaneMat(0)), *(distCoeffs.getCvPlaneMat(0)), cv::noArray(), *(cameraMatrixNew.getCvPlaneMat(0)), size, CV_32FC1, map1, map2);
            }
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[3], &map1);
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[4], &map2);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvRemapDoc = QObject::tr("Applies a generic geometrical transformation to an image. \n\
\n\
The function remap transforms the source image using the specified map: \n\
\n\
dst(x,y) = src(map_x(x, y), map_y(x, y)) \n\
\n\
where values of pixels with non-integer coordinates are computed using one of available interpolation methods. map_x and map_y can be encoded as \n\
separate floating-point maps in map_1 and map_2 respectively, or interleaved floating-point maps of (x,y) in map_1 , \n\
or fixed-point maps created by using convertMaps() . The reason you might want to convert from floating to fixed-point representations of a map is \n\
that they can yield much faster (~2x) remapping operations. In the converted case, map_1 contains pairs (cvFloor(x), cvFloor(y)) and map_2 contains \n\
indices in a table of interpolation coefficients.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvRemapParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("source", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("source image").toLatin1().data()));
    paramsMand->append(ito::Param("destination", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("destination image. It hast the same size as map1 and the same type as src.").toLatin1().data()));
    paramsMand->append(ito::Param("map1", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("The first map of x values").toLatin1().data()));
    paramsMand->append(ito::Param("map2", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("The second map of y values").toLatin1().data()));

    QString description = tr("Interpolation method. The following values are possible:\n");
    description += QString("INTER_NEAREST (%1)\n").arg(cv::INTER_NEAREST);
    description += QString("INTER_LINEAR (%1)\n").arg(cv::INTER_LINEAR);
    description += QString("INTER_CUBIC (%1)\n").arg(cv::INTER_CUBIC);
    description += QString("INTER_LANCZOS4 (%1)").arg(cv::INTER_LANCZOS4);
    paramsOpt->append(ito::Param("interpolation", ito::ParamBase::Int | ito::ParamBase::In, 0, cv::INTER_LANCZOS4, cv::INTER_LINEAR, description.toLatin1().data()));

    description = tr("Pixel extrapolation method. When boderMode == BORDER_TRANSPARENT (%1), it means that the pixels in the destination image that corresponds to the outliers in the source image are not modified by the function. \nThe following values are possible:\n").arg(cv::BORDER_TRANSPARENT);
    description += QString("BORDER_CONSTANT (%1)\n").arg(cv::BORDER_CONSTANT);
    description += QString("BORDER_REPLICATE (%1)\n").arg(cv::BORDER_REPLICATE);
    description += QString("BORDER_REFLECT (%1)\n").arg(cv::BORDER_REFLECT);
    description += QString("BORDER_WRAP (%1)\n").arg(cv::BORDER_WRAP);
    description += QString("BORDER_REFLECT101 (%1)\n").arg(cv::BORDER_REFLECT101);
    description += QString("BORDER_TRANSPARENT (%1)\n").arg(cv::BORDER_TRANSPARENT);
    description += QString("BORDER_DEFAULT (%1)\n").arg(cv::BORDER_DEFAULT);
    description += QString("BORDER_ISOLATED (%1)").arg(cv::BORDER_ISOLATED);
    paramsOpt->append(ito::Param("borderMode", ito::ParamBase::Int | ito::ParamBase::In, cv::BORDER_CONSTANT, cv::BORDER_ISOLATED, cv::BORDER_CONSTANT, description.toLatin1().data()));
    
    paramsOpt->append(ito::Param("borderValue", ito::ParamBase::Double | ito::ParamBase::In, 0.0, NULL, tr("value used in case of a constant border. By default, it is 0").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvRemap(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject src = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(),"source", ito::Range(1, INT_MAX), ito::Range(1,INT_MAX), retval, -1, 0);

    if (!paramsMand->at(1).getVal<ito::DataObject*>())
    {
        retval += ito::RetVal(ito::retError, 0, tr("destination is empty").toLatin1().data());
    }

    ito::DataObject map1 = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(2).getVal<ito::DataObject*>(),"map1", ito::Range(1,INT_MAX), ito::Range(1,INT_MAX), retval, -1, 2, ito::tUInt16, ito::tFloat32);
    ito::DataObject map2 = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(3).getVal<ito::DataObject*>(),"map2", ito::Range(1,INT_MAX), ito::Range(1,INT_MAX), retval, -1, 2, ito::tUInt16, ito::tFloat32);

    int interpolation = paramsOpt->at(0).getVal<int>();
    int borderType = paramsOpt->at(1).getVal<int>();
    double borderValueDouble = paramsOpt->at(2).getVal<double>();
    cv::Scalar borderValue = borderValueDouble;

    if (!retval.containsError())
    {
        cv::Mat dst;
        
        try
        {
            cv::remap(*(src.getCvPlaneMat(0)), dst, *(map1.getCvPlaneMat(0)), *(map2.getCvPlaneMat(0)), interpolation, borderType, borderValue);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[1], &dst);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvFindHomographyDoc = QObject::tr("Finds a perspective transformation between two planes. \n\
\n\
The functions find and return the perspective transformation H between the source and the destination planes: \n\
\n\
.. math:: s_i \\begin{bmatrix}{x'_i}\\\\{y'_i}\\\\{1}\\end{bmatrix} \\sim H \\begin{bmatrix}{x_i}\\\\{y_i}\\\\{1}\\end{bmatrix} \n\
\n\
so that the back-projection error \n\
\n\
.. math:: \\sum _i \\left(x'_i- \\frac{h_{11} x_i + h_{12} y_i + h_{13}}{h_{31} x_i + h_{32} y_i + h_{33}} \\right)^2 + \\left(y'_i- \\frac{h_{21} x_i + h_{22} y_i + h_{23}}{h_{31} x_i + h_{32} y_i + h_{33}} \\right)^2 \n\
\n\
is minimized. \n\
\n\
The function is used to find initial intrinsic and extrinsic matrices. Homography matrix is determined up to a scale. Thus, it is normalized so that h_{33}=1.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvFindHomographyParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("srcPoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("coordinates of the points in the original plane, a matrix of type [Nx2], float32").toLatin1().data()));
    paramsMand->append(ito::Param("dstPoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("coordinates of the points in the target plane, a matrix of type [Nx2], float32").toLatin1().data()));
    paramsMand->append(ito::Param("homography", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("3x3 homography matrix (output)").toLatin1().data()));

    QString description = tr("Method. The following values are possible: ");
    description += QString("regular method using all points (%1) [default]").arg(0);
    description += QString(", CV_RANSAC (%1)").arg(cv::RANSAC);
    description += QString(", CV_LMEDS (%1)").arg(cv::LMEDS);
    paramsOpt->append(ito::Param("interpolation", ito::ParamBase::Int | ito::ParamBase::In, 0, cv::LMEDS, 0, description.toLatin1().data()));
    
    paramsOpt->append(ito::Param("ransacReprojThreshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<double>::max(), 3.0, tr("maximum allowed reprojection error to treat a point pair as an inlier (used for RANSAC only)").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvFindHomography(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject src = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(),"srcPoints", ito::Range(0,INT_MAX), ito::Range(2,2), retval, ito::tFloat32, 0);
    ito::DataObject dst = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(1).getVal<ito::DataObject*>(),"dstPoints", ito::Range(0,INT_MAX), ito::Range(2,2), retval, ito::tFloat32, 0);

    if (!paramsMand->at(2).getVal<ito::DataObject*>())
    {
        retval += ito::RetVal(ito::retError, 0, tr("homography matrix is empty").toLatin1().data());
    }

    int method = paramsOpt->at(0).getVal<int>();
    double ransacReprojThreshold = paramsOpt->at(1).getVal<int>();

    if (!retval.containsError())
    {
        cv::Mat homography;
        
        try
        {
            homography = cv::findHomography(src.getContinuousCvPlaneMat(0), dst.getContinuousCvPlaneMat(0), method, ransacReprojThreshold);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[2], &homography);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvFindFundamentalMatDoc = QObject::tr("Calculates a fundamental matrix from the corresponding points in two images. \n\
\n\
The epipolar geometry is described by the following equation: \n\
\n\
.. math:: [p_2; 1]^T F [p_1; 1] = 0 \n\
\n\
where F is a fundamental matrix, p_1 and p_2 are corresponding points in the first and the second images, respectively. \n\
\n\
The function calculates the fundamental matrix using one of four methods listed above and returns the found fundamental matrix. \n\
Normally just one matrix is found. But in case of the 7-point algorithm, the function may return up to 3 solutions (9 \times 3 matrix that stores all 3 matrices sequentially).");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvFindFundamentalMatParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("points1", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("coordinates of the points in the first image, a matrix of type [Nx2], float32 or float64").toLatin1().data()));
    paramsMand->append(ito::Param("points2", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("coordinates of the points in the second image, a matrix of type [Nx2], float32 or float64").toLatin1().data()));
    paramsMand->append(ito::Param("F", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("output, fundamental matrix [3x3], float64").toLatin1().data()));

    QString description = tr("Method for computing a fundamental matrix. The following values are possible: ");
    description += QString(", CV_FM_7POINT (%1)").arg(cv::FM_7POINT);
    description += QString(", CV_FM_8POINT (%1) [default]").arg(cv::FM_8POINT);
    description += QString(", CV_FM_RANSAC (%1)").arg(cv::FM_RANSAC);
    description += QString(", CV_FM_LMEDS (%1)").arg(cv::FM_LMEDS);
    paramsOpt->append(ito::Param("method", ito::ParamBase::Int | ito::ParamBase::In, cv::FM_7POINT, std::max(cv::FM_RANSAC, cv::FM_LMEDS), cv::FM_8POINT, description.toLatin1().data()));
    paramsOpt->append(ito::Param("param1", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<double>::max(), 3.0, tr("Parameter used for RANSAC. It is the maximum distance from a point to an epipolar line in pixels, beyond which the point is considered an outlier and is not used for computing the final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the point localization, image resolution, and the image noise.").toLatin1().data()));
    paramsOpt->append(ito::Param("param2", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of confidence (probability) that the estimated matrix is correct.").toLatin1().data()));
    paramsOpt->append(ito::Param("status", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output array of N elements, every element of which is set to 0 for outliers and to 1 for the other points. The array is computed only in the RANSAC and LMedS methods. For other methods, it is set to all 1’s. If not given, no status information is returned.").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvFindFundamentalMat(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject points1 = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(),"points1", ito::Range(0,INT_MAX), ito::Range(2,2), retval, -1, 2, ito::tFloat32, ito::tFloat64);
    ito::DataObject points2 = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(1).getVal<ito::DataObject*>(),"points2", ito::Range(0,INT_MAX), ito::Range(2,2), retval, -1, 2, ito::tFloat32, ito::tFloat64);

    bool sendStatus = (paramsOpt->at(3).getVal<void*>() != NULL);
    double param1 = paramsOpt->at(1).getVal<double>();
    double param2 = paramsOpt->at(2).getVal<double>();
    int method = paramsOpt->at(0).getVal<int>();

    if (method != cv::FM_7POINT && method != cv::FM_8POINT && method != cv::FM_RANSAC && method != cv::FM_LMEDS)
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("method must be either CV_FM_7POINT (%i), CV_FM_8POINT (%i), CV_FM_RANSAC (%i) or CV_FM_LMEDS (%i)").toLatin1().data(), cv::FM_7POINT, cv::FM_8POINT, cv::FM_RANSAC, cv::FM_LMEDS);
    }

    if (!retval.containsError())
    {
        cv::Mat fund;
        cv::Mat status;

        try
        {
            if (sendStatus)
            {
                fund = cv::findFundamentalMat(*(points1.getCvPlaneMat(0)), *(points2.getCvPlaneMat(0)), method, param1, param2, status);
            }
            else
            {
                fund = cv::findFundamentalMat(*(points1.getCvPlaneMat(0)), *(points2.getCvPlaneMat(0)), method, param1, param2);
            }
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[2], &fund);
        }

        if (sendStatus)
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsOpt)[3], &status);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvComputeCorrespondEpilinesDoc = QObject::tr("For points in an image of a stereo pair, computes the corresponding epilines in the other image. \n\
\n\
For every point in one of the two images of a stereo pair, the function finds the equation of the corresponding epipolar line in the other image. \n\
\n\
From the fundamental matrix definition (see findFundamentalMat()), line l^{(2)}_i in the second image for the point p^{(1)}_i in the first image (when whichImage=1) is computed as: \n\
\n\
.. math:: l^{(2)}_i = F p^{(1)}_i \n\
\n\
And vice versa, when whichImage=2, l^{(1)}_i is computed from p^{(2)}_i as: \n\
\n\
.. math:: l^{(1)}_i = F^T p^{(2)}_i \n\
\n\
Line coefficients are defined up to a scale. They are normalized so that \n\
\n\
.. math:: a_i^2+b_i^2=1.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvComputeCorrespondEpilinesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("points", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("coordinates of the image points in the one image, a matrix of type [Nx2], float32").toLatin1().data()));
    paramsMand->append(ito::Param("whichImage", ito::ParamBase::Int | ito::ParamBase::In, 1, 2, 1, tr("Index of the image (1 or 2) that contains the points.").toLatin1().data()));
    paramsMand->append(ito::Param("F", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Fundamental matrix that can be estimated using cvFindFundamentalMat() or cvStereoRectify()").toLatin1().data()));
    paramsMand->append(ito::Param("lines", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output vector of the epipolar lines corresponding to the points in the other image. Each line ax + by + c=0 is encoded by 3 numbers (a, b, c)").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvComputeCorrespondEpilines(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject points = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(),"points", ito::Range(0,INT_MAX), ito::Range(2,2), retval, ito::tFloat32, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    ito::DataObject F = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(2).getVal<ito::DataObject*>(),"F", ito::Range(3,3), ito::Range(3,3), retval, -1, 2, ito::tFloat32, ito::tFloat64);

    int whichImage = paramsMand->at(1).getVal<int>();

    if (!retval.containsError())
    {
        cv::Mat lines;

        try
        {
            cv::computeCorrespondEpilines(points.getContinuousCvPlaneMat(0), whichImage, *(F.getCvPlaneMat(0)), lines);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[3], &lines);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvWarpPerspectiveDoc = QObject::tr("Applies a perspective transformation to an image \n\
\n\
The function warpPerspective transforms the source image using the specified matrix H");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvWarpPerspectiveParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->append(ito::Param("inputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("input image").toLatin1().data()));
    paramsMand->append(ito::Param("outputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("output image that has the size dsize and the same type as input image").toLatin1().data()));
    paramsMand->append(ito::Param("M", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("3x3 transformation matrix").toLatin1().data()));

    QString description = tr("Interpolation method. The following values are possible: ");
    description += QString("INTER_LINEAR (%1)").arg(cv::INTER_LINEAR);
    description += QString(", INTER_NEAREST (%1)").arg(cv::INTER_NEAREST);
    paramsOpt->append(ito::Param("interpolation", ito::ParamBase::Int | ito::ParamBase::In, cv::INTER_NEAREST, cv::INTER_LINEAR, cv::INTER_LINEAR, description.toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvWarpPerspective(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject src = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(),"source", ito::Range(1,INT_MAX), ito::Range(1,INT_MAX), retval, -1, 0);
    ito::DataObject M = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(2).getVal<ito::DataObject*>(),"M", ito::Range(1,INT_MAX), ito::Range(1,INT_MAX), retval, -1, 0);

    if (!paramsMand->at(1).getVal<ito::DataObject*>())
    {
        retval += ito::RetVal(ito::retError, 0, tr("destination is empty").toLatin1().data());
    }

    int interpolation = paramsOpt->at(0).getVal<int>();

    if (!retval.containsError())
    {
        cv::Mat dst;
        
        try
        {
            cv::warpPerspective(*(src.getCvPlaneMat(0)), dst, *(M.getCvPlaneMat(0)), cv::Size(), interpolation);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[1], &dst);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvProjectPointsDoc = QObject::tr("Project points from object into image space using the given calibration matrices,\n\
distortion coefficients rotation and tralsation vector.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvProjectPointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->append(ito::Param("inputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("input image").toLatin1().data()));
    paramsMand->append(ito::Param("outputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("output image that has the size dsize and the same type as input image").toLatin1().data()));
    paramsMand->append(ito::Param("M", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("3x3 camera fundamental matrix").toLatin1().data()));
    paramsMand->append(ito::Param("distCoeff", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("matrix with distortion coefficients").toLatin1().data()));
    paramsMand->append(ito::Param("RVec", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("rotation vector").toLatin1().data()));
    paramsMand->append(ito::Param("TVec", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("translation vector").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvProjectPoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject src = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "source", ito::Range(1, INT_MAX), ito::Range(1, INT_MAX), retval, -1, 1, ito::tFloat32);
    ito::DataObject camMat = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(2).getVal<ito::DataObject*>(), "M", ito::Range(1, INT_MAX), ito::Range(1, INT_MAX), retval, -1, 0);
    ito::DataObject distCoeff = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(3).getVal<ito::DataObject*>(), "distCoeff", ito::Range(1, INT_MAX), ito::Range(1, INT_MAX), retval, -1, 0);
    ito::DataObject rVec = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(4).getVal<ito::DataObject*>(), "RVec", ito::Range(1, INT_MAX), ito::Range(1, INT_MAX), retval, -1, 0);
    ito::DataObject tVec = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(5).getVal<ito::DataObject*>(), "TVec", ito::Range(1, INT_MAX), ito::Range(1, INT_MAX), retval, -1, 0);

    if (!paramsMand->at(1).getVal<ito::DataObject*>())
    {
        retval += ito::RetVal(ito::retError, 0, tr("destination object must not be NULL").toLatin1().data());
    }

    if (!retval.containsError())
    {
        cv::Mat dst;

        try
        {
            cv::projectPoints(*(src.getCvPlaneMat(0)), *(rVec.getCvPlaneMat(0)), *(tVec.getCvPlaneMat(0)), *(camMat.getCvPlaneMat(0)),
                *(distCoeff.getCvPlaneMat(0)), dst);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            ito::DataObject *dstObj = paramsMand->at(1).getVal<ito::DataObject*>();
            *dstObj = ito::DataObject(dst.rows, 2, ito::tFloat32);
            ito::float32 *dPtr = (ito::float32*)(dstObj->rowPtr(0, 0));
            memcpy(dPtr, dst.ptr<float>(0), dst.rows * 2 * sizeof(ito::float32));
            /*
            for (int np = 0; np < dst.rows; np++)
            {
                dPtr[np * 2] = dst.ptr[np * 2];
                dPtr[np * 2 + 1] = dst.ptr[np * 2 + 1];
            }
            */
            //retval += itomcv::setOutputArrayToDataObject((*paramsMand)[1], &dst);
        }
    }

    return retval;
}

////----------------------------------------------------------------------------------------------------------------------------------
///*static*/ const char *OpenCVFilters::cvStereoRectifyDoc = "Computes rectification transforms for each head of a calibrated stereo camera. \n\
//\n\
//The function computes the rotation matrices for each camera that (virtually) make both camera image planes the same plane. Consequently, \n\
//this makes all the epipolar lines parallel and thus simplifies the dense stereo correspondence problem. The function takes the matrices \n\
//computed by stereoCalibrate() as input. As output, it provides two rotation matrices and also two projection matrices in the new coordinates.";
//
///*static*/ ito::RetVal OpenCVFilters::cvStereoRectifyParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
//{
//    ito::Param param;
//    ito::RetVal retval = ito::retOk;
//    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
//    if (retval.containsError()) return retval;
//
//    paramsMand->append(ito::Param("cameraMatrix1", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "First camera matrix A = [[fx 0 cx];[0 fy cy];[0 0 1]]"));
//    paramsMand->append(ito::Param("cameraMatrix2", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "Second camera matrix A = [[fx 0 cx];[0 fy cy];[0 0 1]]"));
//    paramsMand->append(ito::Param("distCoeffs1", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "Input vector of distortion coefficients [1 x 4,5,8] (k1, k2, p1, p2 [, k3[, k4, k5, k6]]) of 4, 5 or 8 elements."));
//    paramsMand->append(ito::Param("distCoeffs2", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "Input vector of distortion coefficients [1 x 4,5,8] (k1, k2, p1, p2 [, k3[, k4, k5, k6]]) of 4, 5 or 8 elements."));
//    paramsMand->append(ito::Param("imageSize", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, "[width,height] of the camera image (in pixels)"));
//
//    paramsOpt->append(ito::Param("flags", ito::ParamBase::Int | ito::ParamBase::In, 0, CV_CALIB_ZERO_DISPARITY, CV_CALIB_ZERO_DISPARITY, "Operation flags that may be zero or CV_CALIB_ZERO_DISPARITY (default). If the flag is set, the function makes the principal points of each camera have the same pixel coordinates in the rectified views. And if the flag is not set, the function may still shift the images in the horizontal or vertical direction (depending on the orientation of epipolar lines) to maximize the useful image area."));
//    paramsOpt->append(ito::Param("alpha", ito::ParamBase::Double | ito::ParamBase::In, -1.0, 1.0, -1.0, "Free scaling parameter. If it is -1 or absent, the function performs the default scaling. Otherwise, the parameter should be between 0 and 1. alpha=0 means that the rectified images are zoomed and shifted so that only valid pixels are visible (no black areas after rectification). alpha=1 means that the rectified image is decimated and shifted so that all the pixels from the original images from the cameras are retained in the rectified images (no source image pixels are lost). Obviously, any intermediate value yields an intermediate result between those two extreme cases."));
//    paramsOpt->append(ito::Param("newImageSize", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, "New image resolution after rectification. The same size should be passed to cvInitUndistortRectifyMap(). When (0,0) is passed (default), it is set to the original imageSize . Setting it to larger value can help you preserve details in the original image, especially when there is a big radial distortion."));
//
//    return retval;
//}
//
///*static*/ ito::RetVal OpenCVFilters::cvStereoRectify(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
//{
//    ito::RetVal retval;
//    ito::DataObject cameraMatrix1 = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<const ito::DataObject*>(), "cameraMatrix1", ito::Range(3,3), ito::Range(3,3), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
//    ito::DataObject cameraMatrix2 = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(1).getVal<const ito::DataObject*>(), "cameraMatrix2", ito::Range(3,3), ito::Range(3,3), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
//    
//    ito::DataObject distCoeffs1 = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(2).getVal<const ito::DataObject*>(), "distCoeffs1", ito::Range(1,1), ito::Range(4,8), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
//    ito::DataObject distCoeffs2 = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(3).getVal<const ito::DataObject*>(), "distCoeffs2", ito::Range(1,1), ito::Range(4,8), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
//
//    cv::Size imageSize = itomcv::getCVSizeFromParam(paramsMand->at(4), false, &retval, false);
//
//    int flags = paramsOpt->at(0).getVal<int>();
//    if (flags != 0 || flags != CV_CALIB_ZERO_DISPARITY)
//    {
//        retval += ito::RetVal::format(ito::retError, 0, "flag must be either 0 or CV_CALIB_ZERO_DISPARITY (%i)", CV_CALIB_ZERO_DISPARITY);
//    }
//
//    double alpha = paramsOpt->at(1).getVal<double>();
//    if (alpha < 0.0 && std::abs(alpha + 1.0) > std::numeric_limits<double>::epsilon())
//    {
//        retval += ito::RetVal(ito::retError, 0, "alpha must be either in the range [0,1] or -1 for default scaling.");
//    }
//
//    cv::Size newImageSize = itomcv::getCVSizeFromParam(paramsOpt->at(2), false, &retval, true);
//
//    if (!retval.containsError())
//    {
//        cv::Mat lines;
//
//        try
//        {
//            cv::stereoRectify(*(points.getCvPlaneMat(0)), whichImage, *(F.getCvPlaneMat(0)), lines);
//        }
//        catch (cv::Exception exc)
//        {
//            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
//        }
//
//        if (!retval.containsError())
//        {
//            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[3], &lines);
//        }
//    }
//
//    return retval;
//
//}

#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)