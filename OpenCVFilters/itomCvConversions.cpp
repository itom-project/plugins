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

#include "itomCvConversions.h"

#include "common/param.h"
#include "DataObject/dataObjectFuncs.h"

namespace itomcv
{

//--------------------------------------------------------------------------------------------------------------------------------------
cv::Size getCVSizeFromParam(const ito::ParamBase &intArrayParam, bool squareSizeIfOneElement /*= false*/, ito::RetVal *retval /*= NULL*/, bool returnEmptySizeIfEmpty /*= false*/)
{
    ito::RetVal ret;
    cv::Size size;

    if (intArrayParam.getType() != (ito::ParamBase::IntArray & ito::paramTypeMask))
    {
        ret += ito::RetVal::format(ito::retError, 0, "Parameter %s must be an integer array", intArrayParam.getName());
    }
    else
    {
        if (intArrayParam.getLen() == 1 && squareSizeIfOneElement)
        {
            size.height = intArrayParam.getVal<int*>()[0];
            size.width = size.height;
        }
        else if (intArrayParam.getLen() == 2)
        {
            size.height = intArrayParam.getVal<int*>()[0];
            size.width = intArrayParam.getVal<int*>()[1];
        }
        else if (intArrayParam.getLen() == 0 && returnEmptySizeIfEmpty)
        {
            size = cv::Size();
        }
        else
        {
            if (squareSizeIfOneElement)
            {
                ret += ito::RetVal::format(ito::retError, 0, "Parameter %s must be an integer array with 1 or 2 values.", intArrayParam.getName());
            }
            else
            {
                ret += ito::RetVal::format(ito::retError, 0, "Parameter %s must be an integer array with 2 values.", intArrayParam.getName());
            }
        }
    }

    if (retval)
    {
        *retval += ret;
    }

    return size;
}

//--------------------------------------------------------------------------------------------------------------------------------------
cv::TermCriteria getCVTermCriteriaFromParam(const ito::ParamBase &intMaxCountParam, const ito::ParamBase &doubleEpsParam, ito::RetVal *retval /*= NULL*/)
{
    //intMaxCountParam > 0: consider it, else: do not consider max count
    //doubleEpsParam > 0.0: consider it, else: do not consider the epsilon value
    ito::RetVal ret;
    cv::TermCriteria criteria;

    if (intMaxCountParam.getType() != (ito::ParamBase::Int & ito::paramTypeMask))
    {
        ret += ito::RetVal::format(ito::retError, 0, "Parameter %s must be an integer", intMaxCountParam.getName());
    }
    else if (doubleEpsParam.getType() != (ito::ParamBase::Double & ito::paramTypeMask))
    {
        ret += ito::RetVal::format(ito::retError, 0, "Parameter %s must be a double", doubleEpsParam.getName());
    }
    else
    {
        criteria.maxCount = intMaxCountParam.getVal<int>();
        criteria.epsilon = doubleEpsParam.getVal<double>();

        criteria.type = (criteria.maxCount > 0) ? cv::TermCriteria::COUNT : 0;
        if (criteria.epsilon > 0.0)
        {
            criteria.type |= cv::TermCriteria::EPS;
        }
    }

    if (retval)
    {
        *retval += ret;
    }

    return criteria;
}

//--------------------------------------------------------------------------------------------------------------------------------------
std::vector<cv::Mat> getInputArrayOfArraysFromDataObject(const ito::DataObject *dObj, bool continuous /*= false*/, ito::RetVal *retval /*= NULL*/)
{
    ito::RetVal ret;
    std::vector<cv::Mat> output;

    if (!dObj)
    {
        ret += ito::RetVal(ito::retError, 0, "dObj must not be NULL");
    }
    else
    {
        if (dObj->getDims() == 2)
        {
            if (!continuous)
            {
                output.push_back( *(dObj->getCvPlaneMat(0)) );
            }
            else
            {
                output.push_back(dObj->getContinuousCvPlaneMat(0));
            }
        }
        else if (dObj->getDims() == 3)
        {
            if (continuous)
            {
                for (int i = 0; i < dObj->getSize(0); ++i)
                {
                    output.push_back( dObj->getContinuousCvPlaneMat(i) );
                }
            }
            else
            {
                for (int i = 0; i < dObj->getSize(0); ++i)
                {
                    output.push_back( *(dObj->getCvPlaneMat(i)) );
                }
            }
        }
        else
        {
            ret += ito::RetVal(ito::retError, 0, "dObj must have either 2 or 3 dimensions");
        }
    }

    if (retval)
    {
        *retval += ret;
    }

    return output;
}

//--------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal setOutputArrayToDataObject(ito::ParamBase &dataObjParam, const cv::Mat* mat)
{
    ito::RetVal retval;

    if (dataObjParam.getType() != (ito::ParamBase::DObjPtr & ito::paramTypeMask))
    {
        retval += ito::RetVal::format(ito::retError, 0, "Parameter %s must be a dataObject", dataObjParam.getName());
    }
    else if ((dataObjParam.getFlags() & ito::ParamBase::Out) == 0)
    {
        retval += ito::RetVal::format(ito::retError, 0, "Parameter %s must be marked with out flag", dataObjParam.getName());
    }
    else if (mat)
    {
        ito::DataObject *dObj = dataObjParam.getVal<ito::DataObject*>();

        if (dObj)
        {
            ito::tDataType cameraMatrixType;
            cv::Mat mat_;

            if (mat->type() == CV_8UC3) //convert it to rgba32 with full alpha channel
            {
                cameraMatrixType = ito::tRGBA32;

                cv::Mat in[] = {*mat, cv::Mat( mat->rows, mat->cols, CV_8UC1 )}; //bgr + alpha (all 255)
                in[1].setTo(255);
                cv::Mat out;
                cv::merge(in, 2, out);
                out.convertTo(mat_, cv::DataType<ito::Rgba32>::type);
            }
            else if (mat->type() == CV_8UC4) //convert it to rgba32
            {
                cameraMatrixType = ito::tRGBA32;
                mat->convertTo(mat_, cv::DataType<ito::Rgba32>::type);
            }
            else
            {
                cameraMatrixType = ito::guessDataTypeFromCVMat(mat, retval);
                mat_ = *mat;
            }

            //check if dataObjParam contains one plane and if the data-pointer of the plane corresponds to the data-pointer of mat. If so, both share memory and we are done!
            if (!retval.containsError() && dObj->calcNumMats() == 1 && (dObj->getDims() == mat_.dims) && dObj->getType() == cameraMatrixType)
            {
                cv::Mat *plane = dObj->getCvPlaneMat(0);
                if ( plane->data == mat_.data && plane->size == mat_.size)
                {
                    return retval;
                }
            }

            if (!retval.containsError())
            {
                if (mat_.dims == 0)
                {
                    *dObj = ito::DataObject();
                }
                else
                {
                    *dObj = ito::DataObject(mat_.dims, mat_.size, cameraMatrixType, &mat_, 1);
                }
            }
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, "DataObject of parameter %s must not be NULL", dataObjParam.getName());
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "mat must not be NULL");
    }

    return retval;
}

//--------------------------------------------------------------------------------------------------------------------------------------
std::vector<cv::KeyPoint> getKeypointsFromParam(const ito::ParamBase &keypointParam, const char* name, ito::RetVal *retval /*= NULL*/)
{
    ito::RetVal retval_;
    std::vector<cv::KeyPoint> kpts;
    const ito::DataObject *input = keypointParam.getVal<const ito::DataObject*>();
    if (input->getDims() > 0)
    {
        const ito::DataObject keypoints = ito::dObjHelper::squeezeConvertCheck2DDataObject(keypointParam.getVal<const ito::DataObject*>(), name, ito::Range(0,INT_MAX), ito::Range(7,7), retval_, ito::tFloat32, 0);
        const ito::float32 *rowPtr = NULL;

        if (!retval_.containsError())
        {
            kpts.reserve(keypoints.getSize(0));

            for (int i = 0; i < keypoints.getSize(0); ++i)
            {
                rowPtr = (const ito::float32*)(keypoints.rowPtr(0, i));
                cv::KeyPoint keyPt1;

                keyPt1.pt.x = rowPtr[0];
                keyPt1.pt.y = rowPtr[1];
                keyPt1.size = rowPtr[2];
                keyPt1.angle = rowPtr[3];
                keyPt1.response = rowPtr[4];
                keyPt1.octave = static_cast<int>(rowPtr[5]);
                keyPt1.class_id = static_cast<int>(rowPtr[6]);

                kpts.push_back(keyPt1);
            }
        }
    }

    if (retval) *retval += retval_;

    return kpts;
}

//--------------------------------------------------------------------------------------------------------------------------------------
std::vector<cv::DMatch> getDMatchesFromParam(const ito::ParamBase &dmatchesParam, const char* name, ito::RetVal *retval /*= NULL*/)
{
    ito::RetVal retval_;
    std::vector<cv::DMatch> matches;
    const ito::DataObject *input = dmatchesParam.getVal<const ito::DataObject*>();
    if (input->getDims() > 0)
    {
        const ito::DataObject matchesArray = ito::dObjHelper::squeezeConvertCheck2DDataObject(dmatchesParam.getVal<const ito::DataObject*>(), name, ito::Range(0,INT_MAX), ito::Range(4,4), retval_, ito::tFloat32, 0);
        const ito::float32 *rowPtr = NULL;

        if (!retval_.containsError())
        {
            matches.reserve(matchesArray.getSize(0));

            for (int i = 0; i < matchesArray.getSize(0); ++i)
            {
                rowPtr = (const ito::float32*)(matchesArray.rowPtr(0, i));
                cv::DMatch m;
                m.queryIdx = static_cast<int>(rowPtr[0]);
                m.trainIdx = static_cast<int>(rowPtr[1]);
                m.imgIdx = static_cast<int>(rowPtr[2]);
                m.distance = rowPtr[3];
                matches.push_back(m);
            }
        }
    }

    if (retval) *retval += retval_;

    return matches;
}

//--------------------------------------------------------------------------------------------------------------------------------------
cv::Mat getBGRMatFromRGBA32DataObject(const ito::DataObject &obj, ito::RetVal *retval /*= NULL*/)
{
    ito::RetVal ret;
    cv::Mat bgr;
    if (obj.getType() != ito::tRGBA32 || obj.getDims() != 2)
    {
        ret += ito::RetVal(ito::retError, 0, "data object must have two dimensions and type RGBA32");
    }
    else
    {
        const cv::Mat_<ito::Rgba32> *rgbaMat = (cv::Mat_<ito::Rgba32>*)(obj.getCvPlaneMat(0));
        cv::Mat bgra;
        rgbaMat->convertTo(bgra, CV_8UC4);
        bgr = cv::Mat( bgra.rows, bgra.cols, CV_8UC3 );

        // forming an array of matrices is a quite efficient operation,
        // because the matrix data is not copied, only the headers
        // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
        // rgba[2] -> bgr[0], rgba[3] -> alpha[0]
        int from_to[] = { 0,0, 1,1, 2,2 };
        cv::mixChannels( &bgra, 1, &bgr, 1, from_to, 3 );
    }

    if (retval)
    {
        *retval += ret;
    }

    return bgr;
}

} //end namespace itomcv
