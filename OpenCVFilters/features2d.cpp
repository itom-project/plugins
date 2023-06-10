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
#include <QList>

#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

    #if (CV_MAJOR_VERSION == 2)
        #include "opencv2/features2d/features2d.hpp"
    #else
        #include "opencv2/features2d.hpp"
    #endif

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvFlannBasedMatcherDoc = QObject::tr("This function uses the nearest search methods to find the best matching points. Matching methods by means of Flann matcher. \n\
This includes some nearest neighbour algorithms to calculate the distance between two points. \n\
\n\
If desired, this function can also return a filtered list of matches and keypoints (keypoints1 and keypoints2) that only contain matches and keypoints whose matched distances \n\
are bounded by max_distance. You only need to indicate parameters belonging to the best-matching process if this max_distance parameter is > 0.0.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvFlannBasedMatcherParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        printf(tr("error while executing some methods").toLatin1().data());
        return retval;
    }

    param = ito::Param("first_descriptors", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input parameter - (n x 128) float32 data object of descriptors from first image (queryDescriptors). These descriptors can be computed from sift/surf algorithms.").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("second_descriptors", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input parameter - (n x 128) float32 data object of descriptors from second image (trainDescriptors). These descriptors can be computed from sift/surf algorithms.").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("Matching_descriptor", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output parameter - (n x 4) float32 data object of Matching descriptor vectors using FLANN matcher. Every row contains the values (queryIdx,trainIdx,imgIdx,distance)").toLatin1().data());
    paramsMand->append(param);

    // Optional Parameters
    paramsOpt->append( ito::Param("max_distance", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<double>::max(), 0.0, tr("Maximum distance between two pair of points to calculate the best matching.").toLatin1().data()));
    paramsOpt->append( ito::Param("first_keypoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Optional input parameter - corresponding key points of the first image (n x 7) float32 data object, must have the same number of rows than first_descriptors.").toLatin1().data()));
    paramsOpt->append( ito::Param("second_keypoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Optional input parameter - corresponding key points of the second image (n x 7) float32 data object, must have the same number of rows than second_descriptors.").toLatin1().data()));
    paramsOpt->append( ito::Param("first_best_matches_points", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Optional output parameter - (m x 2) float32 data object of best matching points from first image. each row includes (x and y coordinates), and m is the number of best matching points ").toLatin1().data()));
    paramsOpt->append( ito::Param("second_best_matches_points", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Optional output parameter - (m x 2) float32 data object of best matching points from second image. each row includes (x and y coordinates), and m is the number of best matching points").toLatin1().data()));
    paramsOpt->append( ito::Param("good_matches", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Optional output parameter - (m x 4) float32 data object of good matching descriptor vectors using FLANN matcher. Every row contains the values (queryIdx,trainIdx,imgIdx,distance)").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvFlannBasedMatcher(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    double max_distance = paramsOpt->at(0).getVal<double>(); // Default value is 0.0
    ito::DataObject *matchesOut = paramsMand->at(2).getVal<ito::DataObject*>();
    ito::DataObject descriptor1 = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(),"first_descriptors", ito::Range(0,INT_MAX), ito::Range(128,128), retval, ito::tFloat32, 0);
    ito::DataObject descriptor2 = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(1).getVal<ito::DataObject*>(),"second_descriptors", ito::Range(0,INT_MAX), ito::Range(128,128), retval, ito::tFloat32, 0);

    std::vector<cv::DMatch> Dmatches;

    if (!paramsMand->at(0).getVal<ito::DataObject*>())
    {
        retval += ito::RetVal(ito::retError, 0, tr("The descriptors of the first image is empty").toLatin1().data());
    }

    if (!paramsMand->at(1).getVal<ito::DataObject*>())
    {
        retval += ito::RetVal(ito::retError, 0, tr("The descriptors of the second image is empty").toLatin1().data());
    }

    if (!retval.containsError())
    {
        cv::FlannBasedMatcher matcher;
        typedef std::vector< cv::DMatch >::size_type DMatchSizeType;

        try
        {
            matcher.match(*(descriptor1.getCvPlaneMat(0)), *(descriptor2.getCvPlaneMat(0)),Dmatches);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str() );
        }

        if (!retval.containsError())
        {
            ito::DataObject mtc(Dmatches.size(),4,ito::tFloat32);  // DataObject declaration
            ito::float32 *rowPtr = NULL;

            for (DMatchSizeType row = 0; row < Dmatches.size(); ++row)
            {
                rowPtr = (ito::float32*)mtc.rowPtr(0,row);
                rowPtr[0] = Dmatches[row].queryIdx;
                rowPtr[1] = Dmatches[row].trainIdx;
                rowPtr[2] = Dmatches[row].imgIdx;
                rowPtr[3] = Dmatches[row].distance;
            }
            *matchesOut = mtc;
        }

        if (max_distance > 0.0)
        {
            const ito::DataObject first_keypoints_ = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsOpt->at(1).getVal<const ito::DataObject*>(),"first_keypoints", ito::Range(descriptor1.getSize(0),INT_MAX), ito::Range(7,7), retval, ito::tFloat32, 0);
            const ito::DataObject second_keypoints_ = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsOpt->at(2).getVal<const ito::DataObject*>(),"second_keypoints", ito::Range(descriptor2.getSize(0),INT_MAX), ito::Range(7,7), retval, ito::tFloat32, 0);
            QVector<int> best_matches_idx;

            if (!retval.containsError())
            {
                best_matches_idx.reserve(Dmatches.size());

                for (DMatchSizeType row = 0; row < Dmatches.size(); ++row)
                {
                    if (Dmatches[row].distance <= max_distance)
                    {
                        best_matches_idx.push_back(row);
                    }
                }
            }

            if (!retval.containsError())
            {
                ito::DataObject bestMatches(best_matches_idx.size(), 4, ito::tFloat32);
                ito::DataObject bestKeypoints1(best_matches_idx.size(), 7, ito::tFloat32);
                ito::DataObject bestKeypoints2(best_matches_idx.size(), 7, ito::tFloat32);
                ito::float32 *rowPtrDest;
                ito::float32 *rowPtrSrc;
                int count = 0;

                foreach( const int idx, best_matches_idx)
                {
                    //copy matches line
                    rowPtrSrc = (ito::float32*)(matchesOut->rowPtr(0, idx));
                    rowPtrDest = (ito::float32*)(bestMatches.rowPtr(0, count));
                    memcpy(rowPtrDest, rowPtrSrc, sizeof(ito::float32) * 4);

                    //copy first keypoints line
                    rowPtrSrc = (ito::float32*)(first_keypoints_.rowPtr(0, Dmatches[idx].queryIdx));
                    rowPtrDest = (ito::float32*)(bestKeypoints1.rowPtr(0, count));
                    memcpy(rowPtrDest, rowPtrSrc, sizeof(ito::float32) * 7);

                    //copy second keypoints line
                    rowPtrSrc = (ito::float32*)(second_keypoints_.rowPtr(0, Dmatches[idx].trainIdx));
                    rowPtrDest = (ito::float32*)(bestKeypoints2.rowPtr(0, count));
                    memcpy(rowPtrDest, rowPtrSrc, sizeof(ito::float32) * 7);

                    count++;
                }

                *((*paramsOpt)[3].getVal<ito::DataObject*>()) = bestKeypoints1;
                *((*paramsOpt)[4].getVal<ito::DataObject*>()) = bestKeypoints2;
                *((*paramsOpt)[5].getVal<ito::DataObject*>()) = bestMatches;
            }
        }
    }

    return retval;
}

//------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvDrawKeypointsDoc = QObject::tr("Draws keypoints.");

//------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvDrawKeypointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        printf(tr("error while executing some methods").toLatin1().data());
        return retval;
    }

    param = ito::Param("image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Source image (uint8 or rgba32).").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("keypoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("keypoints of the source image (n x 7) float32 data object").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("outImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output image. Its content depends on the flags value defining what is drawn in the output image. See possible flags bit values below.").toLatin1().data());
    paramsMand->append(param);

    // Optional Parameters
    param = ito::Param("color", ito::ParamBase::Int | ito::ParamBase::In, std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0, tr("color of keypoints (pass a rgba32 value). If 0 or omitted, random colors will be used.").toLatin1().data());
    paramsOpt->append(param);
    int flags_max = int(cv::DrawMatchesFlags::DEFAULT) | int(cv::DrawMatchesFlags::DRAW_OVER_OUTIMG) | int(cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    QString flags_descr = tr("flags for drawing features (bit-combination): \n\
- 0: DEFAULT (Output image matrix will be created (Mat::create), i.e. existing memory of output image may be reused. \
     Two source images, matches, and single keypoints will be drawn. For each keypoint, only the center point will be \
     drawn (without a circle around the keypoint with the keypoint size and orientation). \n\
- 1: DRAW_OVER_OUTIMG: Output image matrix will not be created (using Mat::create). Matches will be drawn \
     on existing content of output image. \n\
- 4: DRAW_RICH_KEYPOINTS: For each keypoint, the circle around keypoint with keypoint size and orientation will be drawn.");
    paramsOpt->append( ito::Param("flags", ito::ParamBase::Int | ito::ParamBase::In, 0, flags_max, int(cv::DrawMatchesFlags::DEFAULT), flags_descr.toLatin1().data()) );
    return retval;
}

//------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvDrawKeypoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    int flags = paramsOpt->at(1).getVal<int>();
    int color = paramsOpt->at(0).getVal<int>();

    const ito::DataObject image = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<const ito::DataObject*>(),"image", ito::Range(0,INT_MAX), ito::Range(0,INT_MAX), retval, -1, 2, ito::tUInt8, ito::tRGBA32);
    cv::Mat image_;

    if (!retval.containsError())
    {
        if (image.getType() == ito::tUInt8)
        {
            image_ = *(image.getCvPlaneMat(0));
        }
        else
        {
            image_ = itomcv::getBGRMatFromRGBA32DataObject(image, &retval);
        }
    }

    std::vector<cv::KeyPoint> keypoints = itomcv::getKeypointsFromParam(paramsMand->at(1), "keypoints", &retval);
    ito::DataObject *output = paramsMand->at(2).getVal<ito::DataObject*>();

    try
    {
        cv::Mat outImage;

        cv::Scalar color__ = cv::Scalar::all(-1);

        if (color != 0)
        {
            ito::Rgba32 color_ = ito::Rgba32::fromUnsignedLong((unsigned int)color);
            color__ = cv::Scalar(color_.b, color_.g, color_.r, 255.0); //alpha will be ignored since opencv only works with three channels (no alpha channel)
        }

        if (flags & int(cv::DrawMatchesFlags::DRAW_OVER_OUTIMG))
        {
            const ito::DataObject outImageIn = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(2).getVal<const ito::DataObject*>(),"outImage", ito::Range(0,INT_MAX), ito::Range(0,INT_MAX), retval, -1, 2, ito::tUInt8, ito::tRGBA32);
            if (!retval.containsError())
            {
                if (outImageIn.getType() == ito::tUInt8)
                {
                    outImage = *(outImageIn.getCvPlaneMat(0));
                }
                else
                {
                    outImage = itomcv::getBGRMatFromRGBA32DataObject(outImageIn, &retval);
                }
            }
        }

        if (!retval.containsError())
        {
#if (CV_MAJOR_VERSION >= 4)
            cv::drawKeypoints(image_, keypoints, outImage, color__, cv::DrawMatchesFlags(flags));
#else
            cv::drawKeypoints(image_, keypoints, outImage, color__, flags);
#endif // (CV_MAJOR_VERSION > 4)


        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[2], &outImage);
        }
    }
    catch (cv::Exception exc)
    {
        retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str() );
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvDrawMatcherDoc = QObject::tr("Draw the obtained matches points between two images. \n\
This function draws matches of keypoints from two images in the output image. Match is a line connecting two keypoints (circles).");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvDrawMatcherParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("first_image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input parameter - first image to draw the matching points").toLatin1().data()));
    paramsMand->append( ito::Param("second_image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input parameter - second image to draw the matchibg points").toLatin1().data()));
    paramsMand->append( ito::Param("first_keypoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("keypoints of the first image (n x 7) float32 data object").toLatin1().data()));
    paramsMand->append( ito::Param("second_keypoints", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("keypoints of the second image (n x 7) float32 data object").toLatin1().data()));
    paramsMand->append( ito::Param("matches", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input parameter -  Matches from the first image to the second one, which means that keypoints1[i] has a corresponding point in keypoints2[matches[i]]").toLatin1().data()));
    paramsMand->append( ito::Param("out_img", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output parameter - Output image").toLatin1().data()));

    param = ito::Param("match_color", ito::ParamBase::Int | ito::ParamBase::In, std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0, tr("color of matches (pass a rgba32 value). If 0 or omitted, random colors will be used.").toLatin1().data());
    paramsOpt->append(param);

    param = ito::Param("single_point_color", ito::ParamBase::Int | ito::ParamBase::In, std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0, tr("color of single keypoints (pass a rgba32 value). If 0 or omitted, random colors will be used.").toLatin1().data());
    paramsOpt->append(param);

    int flags_max = int(cv::DrawMatchesFlags::DEFAULT) | int(cv::DrawMatchesFlags::DRAW_OVER_OUTIMG) | int(cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS) | int(cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    QString flags_descr = tr("flags for drawing features (bit-combination): \n\
- 0: DEFAULT: Output image matrix will be created (Mat::create), i.e. existing memory of output image may be reused. \
     Two source images, matches, and single keypoints will be drawn. For each keypoint, only the center point will be \
     drawn (without a circle around the keypoint with the keypoint size and orientation). \n\
- 1: DRAW_OVER_OUTIMG: Output image matrix will not be created (using Mat::create). Matches will be drawn \
     on existing content of output image. \n\
- 2: NOT_DRAW_SINGLE_POINTS: Single keypoints will not be drawn. \n\
- 4: DRAW_RICH_KEYPOINTS: For each keypoint, the circle around keypoint with keypoint size and orientation will be drawn.");
    paramsOpt->append( ito::Param("flags", ito::ParamBase::Int | ito::ParamBase::In, 0, flags_max, int(cv::DrawMatchesFlags::DEFAULT), flags_descr.toLatin1().data()));

    paramsOpt->append( ito::Param("max_match_distance", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<double>::max(), 0.0, tr("max match distance that should be drawn. If 0, every match is drawn [default]").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvDrawMatcher(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    int flags = paramsOpt->at(2).getVal<int>();
    int matchColor = paramsOpt->at(0).getVal<int>();
    int singlePointColor = paramsOpt->at(1).getVal<int>();
    double max_match_distance = paramsOpt->at(3).getVal<double>();
    const ito::DataObject first_image = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<const ito::DataObject*>(),"first_image", ito::Range(0,INT_MAX), ito::Range(0,INT_MAX), retval, -1, 2, ito::tUInt8, ito::tRGBA32);
    const ito::DataObject second_image = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(1).getVal<const ito::DataObject*>(),"second_image", ito::Range(0,INT_MAX), ito::Range(0,INT_MAX), retval, -1, 2, ito::tUInt8, ito::tRGBA32);

    cv::Mat first_image_, second_image_;

    if (!retval.containsError())
    {
        if (first_image.getType() == ito::tUInt8)
        {
            first_image_ = *(first_image.getCvPlaneMat(0));
        }
        else
        {
            first_image_ = itomcv::getBGRMatFromRGBA32DataObject(first_image, &retval);
        }

        if (second_image.getType() == ito::tUInt8)
        {
            second_image_ = *(second_image.getCvPlaneMat(0));
        }
        else
        {
            second_image_ = itomcv::getBGRMatFromRGBA32DataObject(second_image, &retval);
        }
    }

    std::vector<cv::KeyPoint> first_keypoints = itomcv::getKeypointsFromParam(paramsMand->at(2), "first_keypoints", &retval);
    std::vector<cv::KeyPoint> second_keypoints = itomcv::getKeypointsFromParam(paramsMand->at(3), "second_keypoints", &retval);
    std::vector<cv::DMatch> dmatches = itomcv::getDMatchesFromParam(paramsMand->at(4), "matches", &retval);

    if (!retval.containsError())
    {
        cv::Mat outImg;
        std::vector<char> matchesMask;

        if (max_match_distance > 0.0)
        {
            matchesMask.resize(dmatches.size(), 0);

            for (std::vector<cv::DMatch>::size_type i = 0; i < dmatches.size(); ++i)
            {
                if (dmatches[i].distance <= max_match_distance)
                {
                    matchesMask[i] = 1;
                }
            }
        }

        cv::Scalar matchColor__ = cv::Scalar::all(-1);
        cv::Scalar singlePointColor__ = cv::Scalar::all(-1);

        if (matchColor != 0)
        {
            ito::Rgba32 color_ = ito::Rgba32::fromUnsignedLong((unsigned int)matchColor);
            matchColor__ = cv::Scalar(color_.b, color_.g, color_.r, 255.0); //alpha will be ignored since opencv only works with three channels (no alpha channel)
        }

        if (singlePointColor != 0)
        {
            ito::Rgba32 color_ = ito::Rgba32::fromUnsignedLong((unsigned int)singlePointColor);
            singlePointColor__ = cv::Scalar(color_.b, color_.g, color_.r, 255.0); //alpha will be ignored since opencv only works with three channels (no alpha channel)
        }

        if (flags & int(cv::DrawMatchesFlags::DRAW_OVER_OUTIMG))
        {


            const ito::DataObject outImageIn = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(5).getVal<const ito::DataObject*>(),"out_img", ito::Range(0,INT_MAX), ito::Range(0,INT_MAX), retval, -1, 2, ito::tUInt8, ito::tRGBA32);
            if (!retval.containsError())
            {
                if (outImageIn.getType() == ito::tUInt8)
                {
                    outImg = *(outImageIn.getCvPlaneMat(0));
                }
                else
                {
                    outImg = itomcv::getBGRMatFromRGBA32DataObject(outImageIn, &retval);
                }
            }
        }

        try
        {
#if (CV_MAJOR_VERSION >= 4)
            cv::drawMatches(first_image_, first_keypoints, second_image_, second_keypoints, dmatches, outImg, matchColor__, singlePointColor__, matchesMask, cv::DrawMatchesFlags(flags));
#else
            cv::drawMatches(first_image_, first_keypoints, second_image_, second_keypoints, dmatches, outImg, matchColor__, singlePointColor__, matchesMask, flags);
#endif

        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str() );
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[5], &outImg);
        }
    }

    return retval;
}

#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)
