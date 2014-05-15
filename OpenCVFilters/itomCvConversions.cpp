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

#include "itomCvConversions.h"

#include "common/param.h"

namespace itomcv
{

cv::Size getCVSizeFromParam(const ito::ParamBase &intArrayParam, bool squareSizeIfOneElement /*= false*/, ito::RetVal *retval /*= NULL*/)
{
    ito::RetVal ret;
    cv::Size size(-1,-1);

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


std::vector<cv::Mat> getInputArrayOfArraysFromDataObject(const ito::DataObject *dObj, ito::RetVal *retval /*= NULL*/)
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
            int plane = dObj->seekMat(0);
            output.push_back( *((cv::Mat*)(dObj->get_mdata()[plane])) );
        }
        else if (dObj->getDims() == 3)
        {
            for (int i = 0; i < dObj->getSize(0); ++i)
            {
                int plane = dObj->seekMat(0);
                output.push_back( *(dObj->getCvPlaneMat(i)) );
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
            ito::tDataType cameraMatrixType = ito::guessDataTypeFromCVMat(mat, retval);

            //check if dataObjParam contains one plane and if the data-pointer of the plane corresponds to the data-pointer of mat. If so, both share memory and we are done!
            if (dObj->calcNumMats() == 1 && (dObj->getDims() == mat->dims) && dObj->getType() == cameraMatrixType)
            {
                cv::Mat *plane = dObj->getCvPlaneMat(0);
                if ( plane->data == mat->data && plane->size == mat->size)
                {
                    return retval;
                }
            }

            if (!retval.containsError())
            {
                *(dataObjParam.getVal<ito::DataObject*>()) = ito::DataObject(mat->dims, mat->size, cameraMatrixType, mat, 1);
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

} //end namespace itomcv