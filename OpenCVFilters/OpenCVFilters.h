/* ********************************************************************
    Plugin "OpenCV-Filter" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

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
#if QT_VERSION >=  QT_VERSION_CHECK(5,0,0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
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

        static const char * cvDilateDoc;
        static const char * cvErodeDoc;
        static ito::RetVal cvDilateErodeParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);           /*! This function returns the parameters for the dilate or erode function.*/
        static ito::RetVal cvDilateErode(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, bool erodeNotDilate);    /*! This function executes either the dilate or erodate filter of OpenCV.*/
        static ito::RetVal cvDilate(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut); /*! Function performs a "Dilate-Filter" on the input object*/
        static ito::RetVal cvErode(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);  /*! Function performs a "Erode-Filter" on the input object*/

        static const char * cvBlurDoc;
        static ito::RetVal cvBlur(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs a "Blur-Filter" on the input object*/
        static ito::RetVal cvBlurParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "Blur-Filter"*/

        static const char * cvMedianBlurDoc;
        static ito::RetVal cvMedianBlur(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs a "median Blur-Filter" on the input object*/
        static ito::RetVal cvMedianBlurParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "MedianBlur-Filter"*/

        static const char * cvFFT2DDoc;
        static const char * cvFFT1DDoc;
        static const char * cvIFFT2DDoc;
        static const char * cvIFFT1DDoc;
        static ito::RetVal cvIFFT2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the 2D inverse DFT from the openCV-library on the input object*/
        static ito::RetVal cvIFFT1D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the 2D DFT from the openCV-library on the input object*/
        static ito::RetVal cvFFT2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the linewise inverse DFT from the openCV-library on the input object*/
        static ito::RetVal cvFFT1D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the linewise DFT from the openCV-library on the input object*/
        static ito::RetVal cvFFTParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "FFT-Filter"*/

        //static const char * cvCalcHistDoc;
        //static ito::RetVal cvCalcHist(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function calculates the histogramm for the input object*/
        //static ito::RetVal cvCalcHistParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the parameter for the histogramm function*/

        static const char * cvRemoveSpikesDoc;
        static ito::RetVal cvRemoveSpikes(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function removes spikes using dilateration and erodation filter*/
        static ito::RetVal cvRemoveSpikesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the parameter for the remove spike function*/

#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

        static const char *cvFindCirclesDoc;
        static ito::RetVal cvFindCircles(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvFindCirclesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvFindChessboardCornersDoc;
        static ito::RetVal cvFindChessboardCorners(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvFindChessboardCornersParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvDrawChessboardCornersDoc;
        static ito::RetVal cvDrawChessboardCorners(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvDrawChessboardCornersParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvCornerSubPixDoc;
        static ito::RetVal cvCornerSubPix(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvCornerSubPixParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvCalibrateCameraDoc;
        static ito::RetVal cvCalibrateCamera(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvCalibrateCameraParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvEstimateAffine3DDoc;
        static ito::RetVal cvEstimateAffine3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvEstimateAffine3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvUndistortDoc;
        static ito::RetVal cvUndistort(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvUndistortParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvUndistortPointsDoc;
        static ito::RetVal cvUndistortPoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvUndistortPointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvInitUndistortRectifyMapDoc;
        static ito::RetVal cvInitUndistortRectifyMap(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvInitUndistortRectifyMapParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvRemapDoc;
        static ito::RetVal cvRemap(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvRemapParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvFindHomographyDoc;
        static ito::RetVal cvFindHomography(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvFindHomographyParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvFindFundamentalMatDoc;
        static ito::RetVal cvFindFundamentalMat(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvFindFundamentalMatParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvComputeCorrespondEpilinesDoc;
        static ito::RetVal cvComputeCorrespondEpilines(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvComputeCorrespondEpilinesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        /*static const char *cvStereoRectifyDoc;
        static ito::RetVal cvStereoRectify(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvStereoRectifyParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);*/

        static const char *cvFlannBasedMatcherDoc;
        static ito::RetVal cvFlannBasedMatcher(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvFlannBasedMatcherParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvDrawKeypointsDoc;
        static ito::RetVal cvDrawKeypoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvDrawKeypointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *cvDrawMatcherDoc;
        static ito::RetVal cvDrawMatcher(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvDrawMatcherParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

        static const char* cvFlipUpDownDoc;
        static const char* cvFlipLeftRightDoc;
        static ito::RetVal cvFlipUpDown(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);     /*! Function flips openCV-Mats upside down by executing cvFlip(..., false)*/
        static ito::RetVal cvFlipLeftRight(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function flips openCV-Mats left right by executing cvFlip(..., true)*/

        static const char* cvRotP90Doc;
        static const char* cvRotM90Doc;
        
        static ito::RetVal cvRotP90(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function rotates openCV-Mats cclw for 90° by executing cvRot(..., false)*/
        static ito::RetVal cvRotM90(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function rotates openCV-Mats clw for 90° by executing cvRot(..., false)*/
        
        static const char* cvRot180Doc;
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
