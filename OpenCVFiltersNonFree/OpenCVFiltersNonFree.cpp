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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include <math.h>
#include "OpenCVFiltersNonFree.h"
#include "itomCvConversions.h"

#include "DataObject/dataobj.h"
#include "DataObject/dataObjectFuncs.h"

#include <qplugin.h>

#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)
#include "opencv2/nonfree/nonfree.hpp"
#endif


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFiltersNonFreeInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(OpenCVFiltersNonFree)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFiltersNonFreeInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(OpenCVFiltersNonFree)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
OpenCVFiltersNonFreeInterface::OpenCVFiltersNonFreeInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("OpenCV-Filters-Nonfree");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This plugin provides wrappers for various OpenCV algorithms from its section non-free.";

    m_description = QObject::tr("Wrapped algorithms from OpenCV");
    m_detaildescription = QObject::tr(docstring);
    m_author = "M. Gronle, P. Bahar, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL");
    m_aboutThis = QObject::tr("N.A.");
}

//----------------------------------------------------------------------------------------------------------------------------------
OpenCVFiltersNonFreeInterface::~OpenCVFiltersNonFreeInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < QT_VERSION_CHECK(5,0,0)
    Q_EXPORT_PLUGIN2(OpenCVFiltersNonFreeInterface, OpenCVFiltersNonFreeInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
OpenCVFiltersNonFree::OpenCVFiltersNonFree() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
OpenCVFiltersNonFree::~OpenCVFiltersNonFree()
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();
}

#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const char *OpenCVFiltersNonFree::cvSiftDetectorDescriptorExtractorDoc = "Implements the sift algorithm and extracts the corresponding descriptors. \n\
The sift algorithm is a scale invariant feature transform in which image content is transformed into local feature coordinates. \n\
In each octave, the initial image is repeatedly convolved with Gaussians to produce a set of scale space images. Adjacent Gaussians are subtracted to produce the DOG. \n\
After each octave, the Gaussian image is down-sampled by a factor of 2. \n\
Detect maxima and minima of difference-of-Gaussian in scale space. Each point is compared to its 8 neighbours in the current image and 9 neighbours in the scales above and below. \n\
reference: David G. Lowe, \"Distinctive image features from scale-invariant key points,\" International Journal of Computer Vision, 60, 2 (2004), pp. 91-110.";



/*static*/ ito::RetVal OpenCVFiltersNonFree::cvSiftDetectorDescriptorExtractorParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    paramsMand->append( ito::Param("image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "Input parameter - Desired image to extract its descriptor and keypoints by means of SIFT algorithm.") );
    paramsMand->append( ito::Param("descriptor", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "Output parameter - (n x 128) float32 data object with n descriptors") );
    paramsOpt->append( ito::Param("keypoints", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "Optional Output parameter -(n x 7) float32 data object with n keypoints. Every row contains the values (pt_x,pt_y,size,angle,response,octave,id)") );

    return retval;
}



/*static*/ ito::RetVal OpenCVFiltersNonFree::cvSiftDetectorDescriptorExtractor(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    const ito::DataObject *image = paramsMand->at(0).getVal<const ito::DataObject*>();
    ito::DataObject *keypointsOut = paramsOpt->at(0).getVal<ito::DataObject*>();
    retval += ito::dObjHelper::verify2DDataObject(image, "image", 1, std::numeric_limits<int>::max(), 1, std::numeric_limits<int>::max(), 1, ito::tUInt8);

     if (!paramsMand->at(1).getVal<ito::DataObject*>())
    {
        retval += ito::RetVal(ito::retError, 0, "descriptor matrix is empty");
    }

    if (!retval.containsError())
    {
        cv::SIFT detector;
        cv::SiftDescriptorExtractor extractor;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        try
        {
            detector.detect(*(image->getCvPlaneMat(0)), keypoints);
            extractor.compute(*(image->getCvPlaneMat(0)),keypoints, descriptors);

        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str() );
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[1], &descriptors);

            if (keypointsOut != NULL)
            {
                ito::DataObject keypts(keypoints.size(),7,ito::tFloat32);  // DataObject declaration
                ito::float32 *rowPtr = NULL;

                for (int row = 0; row < keypoints.size(); ++row)
                {
                    rowPtr = (ito::float32*)keypts.rowPtr(0,row);
                    rowPtr[0] = keypoints[row].pt.x;
                    rowPtr[1] = keypoints[row].pt.y;
                    rowPtr[2] = keypoints[row].size;  // Diameter
                    rowPtr[3] = keypoints[row].angle;
                    rowPtr[4] = keypoints[row].response;
                    rowPtr[5] = keypoints[row].octave;
                    rowPtr[6] = keypoints[row].class_id;
                }

                *keypointsOut = keypts;
            }

        }
    }

    return retval;
}







////----------------------------------------------------------------------------------------------------------------------------------
///*static*/ const char *OpenCVFiltersNonFree::cvdrawMatcherDoc = "Draw the obtained matches points between two images. \n\ ";
//
///*static*/ ito::RetVal OpenCVFiltersNonFree::cvdrawMatcherParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
//{
//    ito::Param param;
//    ito::RetVal retval = ito::retOk;
//    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
//    if(retval.containsError()) return retval;
//
//
//
//    paramsMand->append( ito::Param("first_image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "Input parameter - first image to draw the matching points") );
//    paramsMand->append( ito::Param("second_image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "Input parameter - second image to draw the matchibg points") );
//    paramsMand->append( ito::Param("first_keypoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "Input parameter - corresponding key points of the first image") );
//    paramsMand->append( ito::Param("second_keypoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "Input parameter - corresponding key points of the second image") );
//    paramsMand->append( ito::Param("matches", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "Input parameter -  Matches from the first image to the second one, which means that keypoints1[i] has a corresponding point in keypoints2[matches[i]]") );
//    paramsMand->append( ito::Param("outImg", ito::ParamBase::DObjPtr | ito::ParamBase::Out, NULL, "Output parameter - Output image") );
//    return retval;
//}
//
///*static*/ ito::RetVal OpenCVFiltersNonFree::cvdrawMatcher(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
//{
//    ito::RetVal retval;
//    const ito::DataObject *first_image = paramsMand->at(0).getVal<const ito::DataObject*>();
//    retval += ito::dObjHelper::verify2DDataObject(first_image, "image", 1, std::numeric_limits<int>::max(), 1, std::numeric_limits<int>::max(), 1);
//    const ito::DataObject *second_image = paramsMand->at(0).getVal<const ito::DataObject*>();
//    retval += ito::dObjHelper::verify2DDataObject(second_image, "image", 1, std::numeric_limits<int>::max(), 1, std::numeric_limits<int>::max(), 1);
//    ito::DataObject first_keypoints = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(2).getVal<ito::DataObject*>(),"first_keypoints", retval, ito::tFloat32, 0);
//    ito::DataObject second_keypoints = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(3).getVal<ito::DataObject*>(),"second_keypoints", retval, ito::tFloat32, 0);
//    ito::DataObject matches = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(4).getVal<ito::DataObject*>(),"matches", retval, ito::tFloat32, 0);
//
//
//
//    if (!paramsMand->at(2).getVal<ito::DataObject*>() || !paramsMand->at(3).getVal<ito::DataObject*>())
//    {
//        retval += ito::RetVal(ito::retError, 0, "keypoint vector error!");
//    }
//
//    if (!paramsMand->at(4).getVal<ito::DataObject*>())
//    {
//        retval += ito::RetVal(ito::retError, 0, "Matches points have not been found!");
//    }
//
//
//    if (!retval.containsError())
//    {
//        cv::Mat outImg;
//        
//        try
//        {
//            cv::drawMatches(*(first_image.getCvPlaneMat(0)), *(first_keypoints.getCvPlaneMat(0)), *(second_image.getCvPlaneMat(0)), *(second_keypoints.getCvPlaneMat(0)), *(matches.getCvPlaneMat(0)), outImg);
//        }
//        catch (cv::Exception exc)
//        {
//            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str() );
//        }
//
//        if (!retval.containsError())
//        {
//            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[2], &outImg);
//        }
//    }
//
//    return retval;
//
//}
//

#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)


//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFiltersNonFree::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

    filter = new FilterDef(OpenCVFiltersNonFree::cvSiftDetectorDescriptorExtractor, OpenCVFiltersNonFree::cvSiftDetectorDescriptorExtractorParams, tr(cvSiftDetectorDescriptorExtractorDoc));
    m_filterList.insert("cvSiftDetectorDescriptorExtractor", filter);

    

    //filter = new FilterDef(OpenCVFiltersNonFree::cvdrawMatcher, OpenCVFiltersNonFree::cvdrawMatcherParams, tr(cvdrawMatcherDoc));
    //m_filterList.insert("cvdrawMatcher", filter);

#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFiltersNonFree::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}