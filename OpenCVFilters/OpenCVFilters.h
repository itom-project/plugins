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

#ifndef OPENCVFILTERS_H
#define OPENCVFILTERS_H

#include "common/addInInterface.h"

#include "pluginVersion.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class OpenCVFiltersInterface
*   @brief Algorithms used for the processing of images using OpenCV
*
*   AddIn Interface for the OpenCVFilters class s. also \ref OpenCVFilters
*/
class OpenCVFiltersInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        OpenCVFiltersInterface();       /*! <Class constructor */
        ~OpenCVFiltersInterface();      /*! <Class destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*! <Create a new instance of OpenCVFilter-Class */

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*! <Destroy the loaded instance of OpenCVFilter-Class */

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class OpenCVFilters
*   @brief Algorithms used to process images and dataobjects with filters provided by openCV
*
*   In this class the algorithms used for the processing of images are implemented.
*   The filters wrapp openCV-Filters to python interface. Handling of 3D-Objects differs depending on the filter.
*
*/
class OpenCVFilters : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        OpenCVFilters();    /*! <Class constructor */
        ~OpenCVFilters();               /*! <Class destructor */

    public:
        friend class OpenCVFiltersInterface;

        static const QString cvDilateDoc;
        static const QString cvErodeDoc;
        static ito::RetVal cvDilateErodeParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);           /*! This function returns the parameters for the dilate or erode function.*/
        static ito::RetVal cvDilateErode(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, bool erodeNotDilate);    /*! This function executes either the dilate or erodate filter of OpenCV.*/
        static ito::RetVal cvDilate(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut); /*! Function performs a "Dilate-Filter" on the input object*/
        static ito::RetVal cvErode(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);  /*! Function performs a "Erode-Filter" on the input object*/

        static const QString cvMorphologyExDoc;
        static ito::RetVal cvMorphologyExParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);           /*! This function returns the parameters for the morphologyEx function.*/
        static ito::RetVal cvMorphologyEx(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);    /*! This function executes the morphologyEx filter of OpenCV.*/

        static const QString cvBlurDoc;
        static ito::RetVal cvBlur(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs a "Blur-Filter" on the input object*/
        static ito::RetVal cvBlurParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "Blur-Filter"*/

        static const QString cvMedianBlurDoc;
        static ito::RetVal cvMedianBlur(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs a "median Blur-Filter" on the input object*/
        static ito::RetVal cvMedianBlurParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "MedianBlur-Filter"*/

        static const QString cvFFT2DDoc;
        static const QString cvFFT1DDoc;
        static const QString cvIFFT2DDoc;
        static const QString cvIFFT1DDoc;
        static ito::RetVal cvIFFT2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the 2D inverse DFT from the openCV-library on the input object*/
        static ito::RetVal cvIFFT1D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the 2D DFT from the openCV-library on the input object*/
        static ito::RetVal cvFFT2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the linewise inverse DFT from the openCV-library on the input object*/
        static ito::RetVal cvFFT1D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the linewise DFT from the openCV-library on the input object*/
        static ito::RetVal cvFFTParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "FFT-Filter"*/

        //static const QString cvCalcHistDoc;
        //static ito::RetVal cvCalcHist(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function calculates the histogramm for the input object*/
        //static ito::RetVal cvCalcHistParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the parameter for the histogramm function*/

        static const QString cvRemoveSpikesDoc;
        static ito::RetVal cvRemoveSpikes(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function removes spikes using dilateration and erodation filter*/
        static ito::RetVal cvRemoveSpikesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the parameter for the remove spike function*/

        static const QString cvSplitChannelsDoc;
        static ito::RetVal cvSplitChannels(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs a "Blur-Filter" on the input object*/
        static ito::RetVal cvSplitChannelsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "Blur-Filter"*/

        static const QString cvMergeChannelsDoc;
        static ito::RetVal cvMergeChannels(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs a "Blur-Filter" on the input object*/
        static ito::RetVal cvMergeChannelsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "Blur-Filter"*/

        static const QString cvBilateralFilterDoc;
        static ito::RetVal cvBilateralFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut); /*! Function performs a "Bilateral-Filter" on the input object*/
        static ito::RetVal cvBilateralFilterParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut); /*! Function gives back the optional and mandatory parameters for "Bilateral-Filter"*/

        static const QString cvThresholdDoc;
        static ito::RetVal cvThreshold(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut); /*! Function performs a "Bilateral-Filter" on the input object*/
        static ito::RetVal cvThresholdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut); /*! Function gives back the optional and mandatory parameters for "Bilateral-Filter"*/

#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)
        static const QString cvFindCirclesDoc;
        static ito::RetVal cvFindCircles(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvFindCirclesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvFindChessboardCornersDoc;
        static ito::RetVal cvFindChessboardCorners(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvFindChessboardCornersParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvDrawChessboardCornersDoc;
        static ito::RetVal cvDrawChessboardCorners(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvDrawChessboardCornersParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvCornerSubPixDoc;
        static ito::RetVal cvCornerSubPix(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvCornerSubPixParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvCalibrateCameraDoc;
        static ito::RetVal cvCalibrateCamera(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvCalibrateCameraParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvEstimateAffine3DDoc;
        static ito::RetVal cvEstimateAffine3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvEstimateAffine3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvUndistortDoc;
        static ito::RetVal cvUndistort(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvUndistortParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvUndistortPointsDoc;
        static ito::RetVal cvUndistortPoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvUndistortPointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvInitUndistortRectifyMapDoc;
        static ito::RetVal cvInitUndistortRectifyMap(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvInitUndistortRectifyMapParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvRemapDoc;
        static ito::RetVal cvRemap(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvRemapParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvFindHomographyDoc;
        static ito::RetVal cvFindHomography(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvFindHomographyParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvFindFundamentalMatDoc;
        static ito::RetVal cvFindFundamentalMat(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvFindFundamentalMatParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvComputeCorrespondEpilinesDoc;
        static ito::RetVal cvComputeCorrespondEpilines(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvComputeCorrespondEpilinesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        /*static const QString cvStereoRectifyDoc;
        static ito::RetVal cvStereoRectify(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvStereoRectifyParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);*/

        static const QString cvFlannBasedMatcherDoc;
        static ito::RetVal cvFlannBasedMatcher(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvFlannBasedMatcherParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvDrawKeypointsDoc;
        static ito::RetVal cvDrawKeypoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvDrawKeypointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvDrawMatcherDoc;
        static ito::RetVal cvDrawMatcher(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvDrawMatcherParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvResizeDoc;
        static ito::RetVal cvResize(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvResizeParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        
        static const QString cvWarpPerspectiveDoc;
        static ito::RetVal cvWarpPerspective(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvWarpPerspectiveParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvProjectPointsDoc;
        static ito::RetVal cvProjectPoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvProjectPointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)
        static const QString cvCannyEdgeDoc;
        static ito::RetVal cvCannyEdge(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvCannyEdgeParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvCvtColorDoc;
        static ito::RetVal cvCvtColor(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvCvtColorParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString cvFlipUpDownDoc;
        static const QString cvFlipLeftRightDoc;
        static ito::RetVal cvFlipUpDown(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);     /*! Function flips openCV-Mats upside down by executing cvFlip(..., false)*/
        static ito::RetVal cvFlipLeftRight(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function flips openCV-Mats left right by executing cvFlip(..., true)*/

        static const QString cvRotP90Doc;
        static const QString cvRotM90Doc;
        
        static ito::RetVal cvRotP90(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function rotates openCV-Mats cclw for 90° by executing cvRot(..., false)*/
        static ito::RetVal cvRotM90(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function rotates openCV-Mats clw for 90° by executing cvRot(..., false)*/
        
        static const QString cvRot180Doc;
        static ito::RetVal cvRot180(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function rotates openCV-Mats for 180°*/

        static ito::RetVal stdParams2Objects(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

    private:
        static ito::RetVal cvFlip(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, bool colsIfTrue);        /*! Flip upside/down (colsIfTrue == false) or left/right (colsIfTrue == true)*/
        static ito::RetVal cvRotate(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, bool rotClw);        /*! Rotate +90° (rotClw == false) or -90° (rotClw == true) by transpose + flip*/

        static ito::RetVal checkInputOutputEqual(ito::DataObject * p_input, ito::DataObject * p_output, bool * unequal);    /*! <Checks if input and output objects are equal and if the object pointers are valid*/
        static ito::RetVal makeInputOutputEqual(ito::DataObject * p_input, ito::DataObject * p_output);                     /*! <Realloc the output object to size and type of input object */

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> /*val*/, ItomSharedSemaphore * /*waitCond*/) { return ito::retOk; }
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> /*val*/, ItomSharedSemaphore * /*waitCond*/) { return ito::retOk; }
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // OPENCVFILTERS_H
