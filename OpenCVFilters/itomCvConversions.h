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

#ifndef ITOMCVCONVERSIONS_H
#define ITOMCVCONVERSIONS_H

#include "common/retVal.h"
#include "common/param.h"
#include "DataObject/dataobj.h"

#if (CV_MAJOR_VERSION < 4)
    #include "opencv/cv.h"
#endif

namespace itomcv
{
    cv::Size getCVSizeFromParam(const ito::ParamBase &intArrayParam, bool squareSizeIfOneElement = false, ito::RetVal *retval = NULL, bool returnEmptySizeIfEmpty = false);
    cv::TermCriteria getCVTermCriteriaFromParam(const ito::ParamBase &intMaxCountParam, const ito::ParamBase &doubleEpsParam, ito::RetVal *retval = NULL);
    std::vector<cv::Mat> getInputArrayOfArraysFromDataObject(const ito::DataObject *dObj, bool continuous = false, ito::RetVal *retval = NULL);
    ito::RetVal setOutputArrayToDataObject(ito::ParamBase &dataObjParam, const cv::Mat* mat);
    std::vector<cv::KeyPoint> getKeypointsFromParam(const ito::ParamBase &keypointParam, const char* name, ito::RetVal *retval = NULL);
    std::vector<cv::DMatch> getDMatchesFromParam(const ito::ParamBase &dmatchesParam, const char* name, ito::RetVal *retval = NULL);
    cv::Mat getBGRMatFromRGBA32DataObject(const ito::DataObject &obj, ito::RetVal *retval = NULL);
} //end namespace itomcv

#endif // ITOMCVCONVERSIONS_H
