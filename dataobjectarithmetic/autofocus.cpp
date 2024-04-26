/* ********************************************************************
Plugin "dataobjectarithmetic" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#include "dataobjectarithmetic.h"
#include "common/retVal.h"
#include "common/typeDefs.h"
#include "DataObject/dataObjectFuncs.h"
#include "opencv2/imgproc/imgproc.hpp"

//! creates template defined function table for all supported data types
#define MAKEFUNCLIST(FuncName) static t##FuncName fList##FuncName[] =   \
{                                                                       \
   FuncName<ito::int8>,                                                 \
   FuncName<ito::uint8>,                                                \
   FuncName<ito::int16>,                                                \
   FuncName<ito::uint16>,                                               \
   FuncName<ito::int32>,                                                \
   FuncName<ito::uint32>,                                               \
   FuncName<ito::float32>,                                              \
   FuncName<ito::float64>,                                              \
   0,                                                                   \
   0,                                                                   \
   0                                                                    \
};


template<typename _Tp> ito::RetVal AutoFocusDerivate(const ito::DataObject *src, const QString &method, double *result)
{
    int type1 = (src->getType() == ito::tFloat32) ? CV_32FC1 : CV_64FC1;
    int type2 = (src->getType() == ito::tFloat32) ? CV_32F : CV_64F;

    const cv::Mat *plane = src->getCvPlaneMat(0);
    cv::Mat dstH(plane->rows, plane->cols, type1);
    cv::Mat dstV(plane->rows, plane->cols, type1);
    int numPlanes = src->getNumPlanes();

    int num = plane->cols * plane->rows;

    if (num == 0)
    {
        return ito::RetVal(ito::retError, 0, "no auto focus estimate can be determined for an empty image");
    }

    bool useSobel = false;
    bool useLaplacian = false;
    int ksize;
    cv::Mat kernelH;
    cv::Mat kernelV;

    if (method == "3x3Sobel")
    {
        useSobel = true;
        ksize = 3;
    }
    else if (method == "3x3Scharr")
    {
        useSobel = true;

#if (CV_MAJOR_VERSION >= 4)
        ksize = cv::FILTER_SCHARR;
#else
        ksize = CV_SCHARR;
#endif

    }
    else if (method == "3x3Roberts")
    {
        float dataH[] = { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0 };
        kernelH = cv::Mat(3, 3, CV_32FC1, dataH);
        float dataV[] = { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0 };
        kernelV = cv::Mat(3, 3, CV_32FC1, dataV);
    }
    else if (method == "3x3Prewitt")
    {
        float dataH[] = { -1.0, 0.0, 1.0, -1.0, 0.0, 1.0, -1.0, 0.0, 1.0 };
        kernelH = cv::Mat(3, 3, CV_32FC1, dataH);
        float dataV[] = { -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
        kernelV = cv::Mat(3, 3, CV_32FC1, dataV);
    }
    else if (method == "3x3Diff")
    {
        float data[] = { 1.0, 0.0, -1.0 };
        kernelH = cv::Mat(1, 3, CV_32FC1, data);
        kernelV = cv::Mat(3, 1, CV_32FC1, data);
    }
    else if (method == "Gradient")
    {
        float data[] = { 1.0, -1.0 };
        kernelH = cv::Mat(1, 2, CV_32FC1, data);
        kernelV = cv::Mat(2, 1, CV_32FC1, data);
    }
    else if (method == "5x5Sobel")
    {
        useSobel = true;
        ksize = 5;
    }
    else if (method == "3x3Laplacian")
    {
        useLaplacian = true;
        ksize = 3;
    }
    else if (method == "5x5Laplacian")
    {
        useLaplacian = true;
        ksize = 5;
    }
    else
    {
        return ito::RetVal(ito::retError, 0, "unknown method");
    }

    for (int p = 0; p < numPlanes; ++p)
    {
        plane = src->get_mdata()[src->seekMat(p, numPlanes)];
        if (useSobel)
        {
            cv::Sobel(*plane, dstH, type2, 1, 0, ksize);
            cv::Sobel(*plane, dstV, type2, 0, 1, ksize);
        }
        else if (useLaplacian)
        {
            cv::Laplacian(*plane, dstH, type2, ksize);
            cv::Laplacian(*plane, dstV, type2, ksize);
        }
        else
        {
            cv::filter2D(*plane, dstH, type2, kernelH);
            cv::filter2D(*plane, dstV, type2, kernelV);
        }

        cv::Mat dst;
        cv::sqrt(dstH.mul(dstH) + dstV.mul(dstV), dst);
        result[p] = cv::sum(dst)[0] / (double)num;
    }

    return ito::retOk;
}

typedef ito::RetVal(*tAutoFocusDerivate)(const ito::DataObject *src, const QString &method, double *result);
MAKEFUNCLIST(AutoFocusDerivate)

//------------------------------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectArithmetic::autoFocusEstimateDoc = QObject::tr("Determines an auto focus estimate for every plane in a given 2D or 3D dataObject. \n\
\n\
The estimate is returned in terms of a tuple of double values for each plane. The higher the estimate, the 'sharper' the image. \n\
There are different methods implemented how the auto focus estimate is calculated. \n\
\n\
The methods are partially taken from H. Mir, P. Xu, P. van Beek, 'An extensive empirical evaluation of focus measures for digital photography', Proc. SPIE 9023 (2014). \n\
Many methods are based on linear filters. If so, their horizontal and vertical version is applied and the result is determined by: \n\
\n\
result = sum(sqrt(H*H + V*V)) / numPixelsPerPlane");

//------------------------------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DataObjectArithmetic::autoFocusEstimateParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("source", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D or 3D source image data object (u)int8, (u)int16, int32 only.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("method", ito::ParamBase::String | ito::ParamBase::In, "3x3Sobel", tr("method used to determine the autofocus.").toLatin1().data());
        ito::StringMeta sm(ito::StringMeta::String);
        sm.addItem("3x3Sobel");
        sm.addItem("3x3Scharr");
        sm.addItem("3x3Roberts");
        sm.addItem("3x3Prewitt");
        sm.addItem("5x5Sobel");
        sm.addItem("3x3Diff");
        sm.addItem("3x3Laplacian");
        sm.addItem("5x5Laplacian");
        sm.addItem("Gradient");
        param.setMeta(&sm, false);
        paramsMand->append(param);

        paramsOut->append(ito::Param("result", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("auto focus measure values for every plane in the source image.").toLatin1().data()));
    }

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DataObjectArithmetic::autoFocusEstimate(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    const ito::DataObject *src = paramsMand->at(0).getVal<ito::DataObject*>();
    QString method = paramsMand->at(1).getVal<char*>();
    ito::DataObject src_ = src->squeeze();
    retval += ito::dObjHelper::verifyDataObjectType(&src_, "source", 5, ito::tUInt8, ito::tUInt16, ito::tInt16, ito::tFloat32, ito::tFloat64);
    int dims = src_.getDims();
    if (dims != 2 && dims != 3)
    {
        retval += ito::RetVal(ito::retError, 0, "source must be 2D or 3D");
    }


    if (!retval.containsError())
    {
        int numPlanes = src_.getNumPlanes();
        double *result = new double[numPlanes];

        if (method == "3x3Sobel" || \
            method == "3x3Scharr" || \
            method == "3x3Roberts" || \
            method == "3x3Prewitt" || \
            method == "5x5Sobel" || \
            method == "3x3Diff" || \
            method == "3x3Laplacian" || \
            method == "5x5Laplacian" || \
            method == "Gradient")
        {
            retval += fListAutoFocusDerivate[src_.getType()](&src_, method, result);
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "unknown autofocus method.");
        }

        if (!retval.containsError())
        {
            (*paramsOut)[0].setVal<double*>(result, numPlanes);
        }
    }


    return retval;
}
