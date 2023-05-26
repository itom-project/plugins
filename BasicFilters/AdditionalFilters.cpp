/* ********************************************************************
Plugin "BasicFilters" for itom software
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

/*! \file BasicFilters.cpp
\brief   This file contains the itomflters class and interface definitions.

\author ITO
\date 03.2017
*/

#include "BasicFilters.h"
#include "DataObject/dataObjectFuncs.h"

#if CV_MAJOR_VERSION >= 4
    #include "opencv2/opencv.hpp"
#else
    #include <opencv/cv.h>
#endif

#if CV_MAJOR_VERSION >= 3
    #include <opencv2/imgproc.hpp>
//#include <opencv2/calib3d.hpp>
#endif

#ifdef USEOPENMP
#define useomp 1
#else
#define useomp 0
#endif

extern int NTHREADS;

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::labelingFilterDoc = QObject::tr("Finds connected areas in an image an assigns a label to them. \n\
                                                             In the input image according found regions get painted with the according label.\n\
                                                             The returned list has the format x0,y0,x1,y1 for each label.\n");

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::findEllipsesFilterDoc = QObject::tr("Filter for detecting the centers of ellipses with subpixel accuracy. \n\
                                                          \n");

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal doLabeling(ito::DataObject *img, const double thres, ito::DataObject *labelTable)
{
    ito::int32 xsize = img->getSize(1);
    ito::int32 ysize = img->getSize(0);
    _Tp value;
    ito::float64 minValue, maxValue;
    ito::uint32 minPos[3], maxPos[3];

    ito::dObjHelper::minMaxValue(img, minValue, minPos, maxValue, maxPos, 1);
    if (maxValue >= 1 || minValue < 0)
    {
        *img -= minValue;
        *img *= 1.0 / (maxValue - minValue + 1);
    }

    _Tp *pixPtr = (_Tp*)img->rowPtr(0, 0);
    int lstep = img->getStep(0);
    ito::int32 xmin = 0, ymin = 0, lbl = 0, lbl1 = 1, u = 0;
    ito::int32 **labelPtr = NULL;
    ito::int8 linehadlabel = 0, lastlinehadlabel = 0;

    labelPtr = new ito::int32*[2];
    labelPtr[0] = (ito::int32*)labelTable->rowPtr(0, 0);
    labelPtr[1] = (ito::int32*)labelTable->rowPtr(1, 0);
    for (ito::int32 nl = 0; nl < ysize * xsize; nl++)
    {
        labelPtr[0][nl] = -1;
        labelPtr[1][nl] = -1;
    }

    for (ito::int32 y = 0; y < ysize; y++)
    {
        for (ito::int32 x = 0; x < xsize; x++)
        {
            value = pixPtr[y * lstep + x];

            if (value > thres)
            {
                if (y > 0)
                    u = pixPtr[(y - 1) * lstep + x];
                else
                    u = 0;

                if (x > 0)
                    lbl = pixPtr[y * lstep + x - 1];
                else
                    lbl = 0;

                if (u && !lbl)
                {
                    pixPtr[y * lstep + x] = u;
                }
                else if (lbl && !u)
                {
                    pixPtr[y * lstep + x] = lbl;
                }
                else if (lbl && u)
                {
                    pixPtr[y * lstep + x] = lbl;
                    if (lbl != u)
                    {
                        if (u >= 1 && labelPtr[0][(ito::int32)u] < labelPtr[0][(ito::int32)lbl])
                            xmin = labelPtr[0][(ito::int32)u];
                        else
                            xmin = labelPtr[0][(ito::int32)lbl];
                        if (u >= 1 && labelPtr[1][(ito::int32)u] < labelPtr[1][(ito::int32)lbl])
                            ymin = labelPtr[1][(ito::int32)u];
                        else
                            ymin = labelPtr[1][(ito::int32)lbl];

                        linehadlabel = 0;
                        for (ito::int32 y1 = y; y1 >= ymin; y1--)
                        {
                            lastlinehadlabel = 0;
                            for (ito::int32 x1 = xmin; x1 < xsize; x1++)
                            {
                                if (pixPtr[y1 * lstep + x1] == u)
                                {
                                    pixPtr[y1 * lstep + x1] = lbl;
                                    lastlinehadlabel = 1;
                                }
                            }
                            if (lastlinehadlabel == 0)
                                linehadlabel--;
                            if (linehadlabel < -2)
                                break;
                        }

                        labelPtr[0][(ito::int32)u] = -1;
                        labelPtr[1][(ito::int32)u] = -1;
                        labelPtr[0][(ito::int32)lbl] = xmin;
                        labelPtr[1][(ito::int32)lbl] = ymin;
                    }
                }
                if (!lbl && !u)
                {
                    lbl1++;
                    /*
                    lbl1 = 1;
                    while ((labelPtr[0][(ito::int32)lbl1] != -1) || (labelPtr[1][(ito::int32)lbl1] != -1))
                    {
                        lbl1++;
                    }
                    */
                    pixPtr[y * lstep + x] = lbl1;

                    labelPtr[0][(ito::int32)lbl1] = x;        // Label l1 als verwendet markieren
                    labelPtr[1][(ito::int32)lbl1] = y;
                }
            }
        }
    }

    for (ito::int32 n = 1; n <= lbl1; n++)
    {
        if (labelPtr[0][n] < 1)
        {
            ito::int32 m;
            for (m = n; m < xsize * ysize; m++)
            {
                if ((labelPtr[0][m] != -1) && (labelPtr[1][m] != -1))
                {
                    labelPtr[0][m] = -1;
                    labelPtr[0][n] = 1;
                    break;
                }
            }
            for (ito::int32 y = 0; y < ysize; y++)
            {
                for (ito::int32 x = 0; x < xsize; x++)
                {
                    if (pixPtr[y * lstep + x] == m)
                        pixPtr[y * lstep + x] = n;
                }
            }
        }
    }

    ito::uint32 numLabels = 0;
    for (ito::int32 n = 0; n < xsize * ysize; n++)
    {
        if (labelPtr[0][n] > 0)
        {
            numLabels++;
        }
    }

//    numLabels++;
    if (numLabels > 0)
    {
        ito::DataObject *labelList = new ito::DataObject();
        labelList->zeros(numLabels, 4, ito::tInt32);
        ito::int32 *labelListPtr = (ito::int32*)labelList->rowPtr(0, 0);

        for (ito::int32 n = 0; n < numLabels; n++)
        {
            labelListPtr[n * 4] = xsize + 10;
            labelListPtr[n * 4 + 1] = ysize + 10;
        }

        for (ito::int32 y = 0; y < ysize; y++)
        {
            for (ito::int32 x = 0; x < xsize; x++)
            {
                if ((value = pixPtr[y * lstep + x]) >= 1)
                {
                    if (x < labelListPtr[((ito::int32)value - 1) * 4])
                        labelListPtr[((ito::int32)value - 1) * 4] = x;
                    if (x > labelListPtr[((ito::int32)value - 1) * 4 + 2])
                        labelListPtr[((ito::int32)value - 1) * 4 + 2] = x;
                    if (y < labelListPtr[((ito::int32)value - 1) * 4 + 1])
                        labelListPtr[((ito::int32)value - 1) * 4 + 1] = y;
                    if (y > labelListPtr[((ito::int32)value - 1) * 4 + 3])
                        labelListPtr[((ito::int32)value - 1) * 4 + 3] = y;
                }
            }
        }

        *labelTable = *labelList;
        delete labelList;
    }
    else
    {
        *labelTable = ito::DataObject();
    }

    if (labelPtr)
        delete labelPtr;

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal doLabeling(ito::DataObject *img, const double thres, const double invalid, ito::DataObject *labelTable)
{
    ito::int32 xsize = img->getSize(1);
    ito::int32 ysize = img->getSize(0);
    _Tp value = 0;
    _Tp *pixPtr = (_Tp*)img->rowPtr(0, 0);
    int lstep = img->getStep(0);
    ito::int32 xmin = 0, ymin = 0, lbl = 0, u = 0, lbl1 = 1;
    ito::int32 **labelPtr = NULL;
    ito::int8 linehadlabel = 0, lastlinehadlabel = 0;

    labelPtr = new ito::int32*[2];
    labelPtr[0] = (ito::int32*)labelTable->rowPtr(0, 0);
    labelPtr[1] = (ito::int32*)labelTable->rowPtr(1, 0);
    for (ito::int32 nl = 0; nl < ysize * xsize; nl++)
    {
        labelPtr[0][nl] = -1;
        labelPtr[1][nl] = -1;
    }

    for (ito::int32 y = 0; y < ysize; y++)
    {
        for (ito::int32 x = 0; x < xsize; x++)
        {
            value = pixPtr[y * lstep + x];

            if ((!(value == invalid)) && (value > thres))
            {
                if (y > 0)
                    u = pixPtr[(y - 1) * lstep + x];
                else
                    u = 0;

                if (u == invalid)
                    u = 0;

                if (x > 0)
                    lbl = pixPtr[y * lstep + x - 1];
                else
                    lbl = 0;

                if (lbl == invalid)
                    lbl = 0;

                if (u  && !lbl)
                    pixPtr[y * lstep + x] = u;
                else if (lbl && !u)
                    pixPtr[y * lstep + x] = lbl;
                else if (lbl && u)
                {
                    pixPtr[y * lstep + x] = lbl;
                    if (lbl != u)
                    {
                        if (u > 0 && labelPtr[0][(ito::int32)u] < labelPtr[0][(ito::int32)lbl])
                            xmin = labelPtr[0][(ito::int32)u];
                        else
                            xmin = labelPtr[0][(ito::int32)lbl];
                        if (u > 0 && labelPtr[1][(ito::int32)u] < labelPtr[1][(ito::int32)lbl])
                            ymin = labelPtr[1][(ito::int32)u];
                        else
                            ymin = labelPtr[1][(ito::int32)lbl];

                        linehadlabel = 0;
                        for (ito::int32 y1 = y; y1 >= ymin; y1--)
                        {
                            lastlinehadlabel = 0;
                            for (ito::int32 x1 = xmin; x1 < xsize; x1++)
                            {
                                if (pixPtr[y1 * lstep + x1] == u)
                                {
                                    pixPtr[y1 * lstep + x1] = lbl;
                                    lastlinehadlabel = 1;
                                }
                            }
                            if (lastlinehadlabel == 0)
                                linehadlabel--;
                            if (linehadlabel < -2)
                                break;
                        }

                        labelPtr[0][(ito::int32)u] = -1;
                        labelPtr[1][(ito::int32)u] = -1;
                        labelPtr[0][(ito::int32)lbl] = xmin;
                        labelPtr[1][(ito::int32)lbl] = ymin;
                    }
                }

                if (!lbl && !u)
                {
                    lbl1++;
                    /*
                    lbl1 = 1;
                    while ((labelPtr[0][(ito::int32)lbl1] != -1) || (labelPtr[1][(ito::int32)lbl1] != -1))
                        //                    while (labelPtr[0][(ito::int32)lbl1] != -1)   // Alternative: for(l1=1;labeltable[l1]!=0;l1++);
                        lbl1++;
                    */

                    pixPtr[y * lstep + x] = lbl1;
                    labelPtr[0][(ito::int32)lbl1] = x;
                    labelPtr[1][(ito::int32)lbl1] = y;
                }
            }
        }
    }

    for (ito::int32 n = 1; n <= lbl1; n++)
    {
        if (labelPtr[0][n] < 1)
        {
            ito::int32 m;
            for (m = n; m < xsize * ysize; m++)
            {
                if ((labelPtr[0][m] != -1) && (labelPtr[1][m] != -1))
                {
                    labelPtr[0][m] = -1;
                    labelPtr[0][n] = 1;
                    break;
                }
            }
            for (ito::int32 y = 0; y < ysize; y++)
            {
                for (ito::int32 x = 0; x < xsize; x++)
                {
                    if (pixPtr[y * lstep + x] == m)
                        pixPtr[y * lstep + x] = n;
                }
            }
        }
    }

    ito::uint32 numLabels = 0;
    for (ito::int32 n = 0; n < xsize * ysize; n++)
    {
        if (labelPtr[0][n] > 0)
        {
            numLabels++;
        }
    }

    //numLabels++;
    ito::DataObject *labelList = new ito::DataObject(numLabels, 4, ito::tInt32);
    ito::int32 *labelListPtr = (ito::int32*)labelList->rowPtr(0, 0);
    for (ito::int32 n = 0; n < numLabels; n++)
    {
        labelListPtr[n * 4] = xsize + 10;
        labelListPtr[n * 4 + 1] = ysize + 10;
    }

    for (ito::int32 y = 0; y < ysize; y++)
    {
        for (ito::int32 x = 0; x < xsize; x++)
        {
            if ((value = pixPtr[y * lstep + x]) >= 1)
            {
                if (x < labelListPtr[((ito::int32)value - 1) * 4])
                    labelListPtr[(ito::int32)value - 1] = x;
                if (x > labelListPtr[((ito::int32)value - 1) * 4 + 2])
                    labelListPtr[((ito::int32)value - 1) * 4 + 2] = x;
                if (y < labelListPtr[((ito::int32)value - 1) * 4 + 1])
                    labelListPtr[((ito::int32)value - 1) * 4 + 1] = y;
                if (y > labelListPtr[((ito::int32)value - 1) * 4 + 3])
                    labelListPtr[((ito::int32)value - 1) * 4 + 3] = y;
            }
        }
    }

    *labelTable = *labelList;
    delete labelList;

    if (labelPtr)
        delete labelPtr;

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal doFindLabel(ito::DataObject *img, double label, ito::DataObject *res)
{
    ito::int32 xsize, ysize, x1, y1, x2 = 0, y2, numpix = 0;
    _Tp *pixPtr = (_Tp*)img->rowPtr(0, 0);
    int lstep = img->getStep(0);
    x1 = xsize = img->getSize(1);
    y1 = ysize = img->getSize(0);

    for (ito::int32 y = 0; y < ysize; y++)
    {
        for (ito::int32 x = 0; x < xsize; x++)
        {
            if (pixPtr[y * lstep + x] == label)
            {
                numpix++;
                if (x < x1)
                    x1 = x;
                if (y < y1)
                    y1 = y;
                if (x > x2)
                    x2 = x;
                y2 = y;
            }
        }
        if (y > y2 + 1)
            break;
    }

    *res = ito::DataObject(5, ito::tInt32);
    res->at<ito::int32>(0, 0) = x1;
    res->at<ito::int32>(1, 0) = y1;
    res->at<ito::int32>(2, 0) = x2;
    res->at<ito::int32>(3, 0) = y2;
    res->at<ito::int32>(4, 0) = numpix;

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for labeling filter
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    mand. Params:
*        - image, the image that should be labelled
*        - cthreshold threshold for a label
*        - labelList dataObject holding the found labels
*   opt. Params:
*       - invalids value for invalid pixel
*/
ito::RetVal BasicFilters::labelingParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    ito::Param param;

    if (!retval.containsError())
    {
        param = ito::Param("image", ito::ParamBase::DObjPtr, NULL, tr("image").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("threshold", ito::ParamBase::Double, 0.0, 65535.0, 50.0, tr("threshold for labeling").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("labelList", ito::ParamBase::DObjPtr, NULL, tr("list of found labels").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("invalids", ito::ParamBase::Double, 0.0, 65535.0, 0.0, tr("inavlid pixels").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** labeling filter
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    label adherent areas of an image with a value above threshold with subsequent labels.
*/
ito::RetVal BasicFilters::labeling(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImage = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    if (!dObjImage|| (dObjImage->getDims() != 2))
    {
        return ito::RetVal(ito::retError, 0, tr("image must be a 2D image!").toLatin1().data());
    }
    double thres = (*paramsMand)[1].getVal<double>();
    if (thres < 0)
        thres = 0;
    ito::DataObject *dObjLabels = (ito::DataObject*)(*paramsMand)[2].getVal<void*>();
    dObjLabels->zeros(2, dObjImage->getSize(1), dObjImage->getSize(0), ito::tInt32); // ToDo Check this

    double invalids = (*paramsOpt)[0].getVal<double>();

    switch (dObjImage->getType())
    {
    case ito::tFloat32:
        if (invalids == 0)
        {
            retval += doLabeling<ito::float32>(dObjImage, thres, dObjLabels);
        }
        else
        {
            retval += doLabeling<ito::float32>(dObjImage, thres, invalids, dObjLabels);
        }
        break;

    case ito::tFloat64:
        if (invalids == 0)
        {
            retval += doLabeling<ito::float64>(dObjImage, thres, dObjLabels);
        }
        else
        {
            retval += doLabeling<ito::float64>(dObjImage, thres, invalids, dObjLabels);
        }
        break;

    default:
        return ito::RetVal(ito::retError, 0, tr("input image should be normalized float (32 or 64) object").toLatin1().data());
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for finding a label
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    mand. Params:
*        - image, the labelled image
*        - label, id of the label to be found
*       - res, dataObject with min, max values for surrounding square of the label and number of pixels with the label
*/
ito::RetVal BasicFilters::findLabelParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    ito::Param param;

    if (!retval.containsError())
    {
        param = ito::Param("image", ito::ParamBase::DObjPtr, NULL, tr("image").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("label", ito::ParamBase::Double, 0.0, 65535.0, 50.0, tr("label id").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("res", ito::ParamBase::DObjPtr, NULL, tr("result, i.e. min and max of surrounding square and num of pixels").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** find label filter
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    label adherent areas of an image with a value above threshold with subsequent labels.
*/
ito::RetVal BasicFilters::findLabel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImage = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    if (!dObjImage|| (dObjImage->getDims() != 2))
    {
        return ito::RetVal(ito::retError, 0, tr("image must be a 2D image!").toLatin1().data());
    }
    double label = (*paramsMand)[1].getVal<double>();
    ito::DataObject *res = (ito::DataObject*)(*paramsMand)[2].getVal<void*>();

    switch (dObjImage->getType())
    {
    case ito::tUInt8:
    case ito::tInt8:
        retval += doFindLabel<ito::uint8>(dObjImage, label, res);
        break;

    case ito::tUInt16:
    case ito::tInt16:
        retval += doFindLabel<ito::uint16>(dObjImage, label, res);
        break;

    case ito::tUInt32:
    case ito::tInt32:
        retval += doFindLabel<ito::uint32>(dObjImage, label, res);
        break;

    case ito::tFloat32:
        retval += doFindLabel<ito::float32>(dObjImage, label, res);
        break;

    case ito::tFloat64:
        retval += doFindLabel<ito::float64>(dObjImage, label, res);
        break;

    default:
        return ito::RetVal(ito::retError, 0, tr("invalid image format").toLatin1().data());
        break;
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> inline ito::float64 subPixEdgeRising(_Tp *imgPtr, ito::int32 xsize, ito::int32 startPx, ito::int32 idx, ito::int8 dir, ito::float64 numBorderPix)
{
    ito::float64 MV[] = {0.0, 0.0, 0.0};
    ito::float64 sigma, s, P1;

    // do a subpixel border detection based on the momentum preservation, s. Luhmann, "Nahbereichsphotogrammetrie"
    if (dir == 0)
    {
        for (ito::int32 m = startPx - numBorderPix / 2; m <= startPx + numBorderPix / 2; m++)
        {
            for (ito::int8 q = 1; q <= 3; q++)
            {
                MV[q - 1] += pow((ito::float64)imgPtr[idx * xsize + m], (ito::float64)q) / (ito::float64)(numBorderPix + 1.0);
            }
        }
    }
    else
    {
        for (ito::int32 m = startPx - numBorderPix / 2; m <= startPx + numBorderPix / 2; m++)
        {
            for (ito::int8 q = 1; q <= 3; q++)
            {
                MV[q - 1] += pow((ito::float64)imgPtr[m * xsize + idx], (ito::float64)q) / (ito::float64)(numBorderPix + 1.0);
            }
        }
    }

    sigma = sqrt(MV[1] - MV[0] * MV[0]);
    s = (MV[2] + 2.0 * pow(MV[0], (ito::float64)3.0) - 3.0 * MV[0] * MV[1]) / pow(sigma, (ito::float64)3.0);
    P1 = 0.5 * (1.0 - s * sqrt(1.0 / (4.0 + s * s)));
    return startPx - floor((ito::float64)numBorderPix / 2.0) + P1 * (numBorderPix + 1.0);
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> inline ito::float64 subPixEdgeFalling(_Tp *imgPtr, ito::int32 xsize, ito::int32 startPx, ito::int32 idx, ito::int8 dir, ito::float64 numBorderPix)
{
    ito::float64 MV[] = {0.0, 0.0, 0.0};
    ito::float64 sigma, s, P1;

    // do a subpixel border detection based on the momentum preservation, s. Luhmann, "Nahbereichsphotogrammetrie"
    if (dir == 0)
    {
        for (ito::int32 m = startPx - numBorderPix / 2; m <= startPx + numBorderPix / 2; m++)
        {
            for (ito::int8 q = 1; q <=3 ; q++)
            {
                MV[q - 1] += pow((ito::float64)imgPtr[idx * xsize + m], (ito::float64)q) / (ito::float64)(numBorderPix + 1.0);
            }
        }
    }
    else
    {
        for (ito::int32 m = startPx - numBorderPix / 2; m <= startPx + numBorderPix / 2; m++)
        {
            for (ito::int8 q = 1; q <=3 ; q++)
            {
                MV[q - 1] += pow((ito::float64)imgPtr[m * xsize + idx], (ito::float64)q) / (ito::float64)(numBorderPix + 1.0);
            }
        }
    }

    sigma = sqrt(MV[1] - MV[0] * MV[0]);
    s = (MV[2] + 2.0 * pow(MV[0], (ito::float64)3.0) - 3.0 * MV[0] * MV[1]) / pow(sigma, (ito::float64)3.0);
    P1 = 0.5 * (1.0 + s * sqrt(1.0 / (4.0 + s * s)));
    return startPx - floor((ito::float64)numBorderPix / 2.0) + P1 * (numBorderPix + 1.0) - 1.0;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** find ellipses center filter
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    find centers of ellipses in an image. A previous labeling of the ellipses is neccessary.
*/

ito::RetVal BasicFilters::findEllipsesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    ito::Param param;

    if (!retval.containsError())
    {
        param = ito::Param("image", ito::ParamBase::DObjPtr, NULL, tr("image").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("labelList", ito::ParamBase::DObjPtr, NULL, tr("list of found labels").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("centers", ito::ParamBase::DObjPtr, NULL, tr("detected ellipses' centers").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("contourPts", ito::ParamBase::DObjPtr, NULL, tr("ellipses' contour points").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("numBorderPix", ito::ParamBase::Double, 0.0, 1000.0, 6.0, tr("number of pixels for border detection").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("thresBorder", ito::ParamBase::Double, 0.0, 100.0, 0.04, tr("threshold for identifying a border after differentation").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("minPixEllCenter", ito::ParamBase::Int, 0, 100, 6, tr("minimum number of pixels for ellipses center detection in x and y direction").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** find ellipses center filter
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    find centers of ellipses in an image. A previous labeling of the ellipses is neccessary.
*/
ito::RetVal BasicFilters::findEllipses(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::DataObject *dObjImg = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjLabels = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();
    ito::DataObject matDx, matDy;
    ito::int32 sizex, sizey, numLabels;
    ito::int32 ellCentFound = 0;

    if (!dObjImg|| (dObjImg->getDims() != 2))
    {
        return ito::RetVal(ito::retError, 0, tr("image must be a 2D image!").toLatin1().data());
    }

    if (!dObjLabels|| (dObjLabels->getDims() != 2) || (dObjLabels->getSize(1) != 4))
    {
        return ito::RetVal(ito::retError, 0, tr("labels must be a n x 4 2D dataObject!").toLatin1().data());
    }

    sizex = dObjImg->getSize(1);
    sizey = dObjImg->getSize(0);
    numLabels = dObjLabels->getSize(0);

    // do some smoothing before differentiation
    cv::Mat *imgMat = (cv::Mat*)dObjImg->get_mdata()[dObjImg->seekMat(0)];
    cv::GaussianBlur(*imgMat, *imgMat, cv::Size(3, 3), 1.2, 1.2);

    // make copies of the image for the calculation of the derivatives
    dObjImg->copyTo(matDx);
    dObjImg->copyTo(matDy);
    cv::Mat *dxMat = (cv::Mat*)matDx.get_mdata()[0];
    cv::Mat *dyMat = (cv::Mat*)matDy.get_mdata()[0];

    // do some smoothing before differentiation
    //cv::GaussianBlur(*dxMat, *dxMat, cv::Size(3, 3), 1.2, 1.2);
    //cv::GaussianBlur(*dyMat, *dyMat, cv::Size(3, 3), 1.2, 1.2);

    // do differentiation in x- and y-direction, using sort of improved sobel filter see B. Jaehne "Digital Image Processing"
    cv::Mat dxKrnl = (cv::Mat_<ito::float64>(3, 3) << -3.0 / 32.0, 0.0, 3.0 / 32.0, -10.0 / 32.0, 0.0, 10.0 / 32.0, -3.0 / 32.0, 0.0, 3.0 / 32.0);
    cv::Mat dyKrnl = (cv::Mat_<ito::float64>(3, 3) << -3.0 / 32.0, -10.0 / 32.0, -3.0 / 32.0, 0.0, 0.0, 0.0, 3.0 / 32.0, 10.0 / 32.0, 3.0 / 32.0);
    cv::filter2D(*dxMat, *dxMat, CV_32F, dxKrnl, cv::Point(-1, -1), 0.0, cv::BORDER_REFLECT);
    cv::filter2D(*dyMat, *dyMat, CV_32F, dyKrnl, cv::Point(-1, -1), 0.0, cv::BORDER_REFLECT);
    dxMat->convertTo(*dxMat, CV_64F);
    dyMat->convertTo(*dyMat, CV_64F);

    ito::int32 numBorderPix = (*paramsOpt)[0].getVal<ito::int32>();
    ito::int32 minN, maxN, minM, maxM;
    ito::float64 thresDiff = (*paramsOpt)[1].getVal<ito::float64>();
    ito::int16 minPixEll = (*paramsOpt)[2].getVal<ito::int32>();
    ito::DataObject *centerMatObj = (ito::DataObject*)(*paramsMand)[2].getVal<void*>();

    centerMatObj->zeros(numLabels, 2, ito::tFloat64);
    cv::Mat *centerMat = (cv::Mat*)centerMatObj->get_mdata()[0];
    *centerMat = cv::Mat::zeros(numLabels, 2, CV_64F);
    ito::float64 *centers = (ito::float64*)centerMat->data;

    for (ito::int16 ellnum = 0; ellnum < numLabels; ellnum++)
    {
        minN = dObjLabels->at<ito::int32>(ellnum, 0);
        minM = dObjLabels->at<ito::int32>(ellnum, 1);
        maxN = dObjLabels->at<ito::int32>(ellnum, 2);
        maxM = dObjLabels->at<ito::int32>(ellnum, 3);

        maxN += numBorderPix;
        minN -= numBorderPix;
        maxM += numBorderPix;
        minM -= numBorderPix;
        if (minM < 0)
        {
//            std::cout << "en: " << ellnum << " minM < 0\n" << std::endl;
            continue;
        }
        if (maxM > sizey - 1)
        {
//            std::cout << "en: " << ellnum << " maxM > sizey\n" << std::endl;
            continue;
        }
        if (minN < 0)
        {
//            std::cout << "en: " << ellnum << " minN < 0\n" << std::endl;
            continue;
        }
        if (maxN > sizex - 1)
        {
//            std::cout << "en: " << ellnum << " maxN > sizex\n" << std::endl;
            continue;
        }

        // allocate memory for the lines and columns used within subpixel border detection
        ito::float64 (*StartEndX)[2], (*StartEndY)[2];
        StartEndX = (ito::float64 (*)[2])calloc(maxM - minM + 1, sizeof(ito::float64[2]));
        StartEndY = (ito::float64 (*)[2])calloc(maxN - minN + 1, sizeof(ito::float64[2]));

        ito::float64 *DxPt = (ito::float64*)dxMat->data;
        ito::float64 *DyPt = (ito::float64*)dyMat->data;

        // grab values for border detection into array
        // first for the detection in horizontal direction ...
        for (ito::int32 y = minM; y <= maxM; y++)
        {
            ito::float64 MaxValL, MaxValR;
            MaxValL = MaxValR = 0;
            // in the left half of the ellises region we search for a falling edge ...
            for (ito::int32 x = minN; x <= (minN + maxN) / 2; x++)
            {
                if ((DxPt[y * sizex + x] > MaxValL) && (fabs(DxPt[y * sizex + x]) > thresDiff))
                {
                    MaxValL = DxPt[y * sizex + x];
                    StartEndX[y - minM][0] = x;
                }
            }
            // and in the right half for a rising edge
            for (ito::int32 x = (minN + maxN) / 2; x <= maxN; x++)
            {
                if ((DxPt[y * sizex + x] < MaxValR) && (fabs(DxPt[y * sizex + x]) > thresDiff))
                {
                    MaxValR = DxPt[y * sizex + x];
                    StartEndX[y - minM][1] = x;
                }
            }
        }

        // second for the detection in vertical direction
        for (ito::int32 x = minN; x <= maxN; x++)
        {
            ito::float64 MaxValT, MaxValB;

            MaxValT = MaxValB = 0;
            // this time we search in the upper half for a falling edge ...
            for (ito::int32 y = minM; y <= (minM + maxM) / 2; y++)
            {
                if ((DyPt[y * sizex + x] > MaxValT) && (fabs(DyPt[y * sizex + x]) > thresDiff))
                {
                    MaxValT = DyPt[y * sizex + x];
                    StartEndY[x - minN][0] = y;
                }
            }
            // and in the lower half for a rising edge
            for (ito::int32 y = (minM + maxM) / 2; y <= maxM; y++)
            {
                if ((DyPt[y * sizex + x] < MaxValB) && (fabs(DyPt[y * sizex + x]) > thresDiff))
                {
                    MaxValB = DyPt[y * sizex + x];
                    StartEndY[x - minN][1] = y;
                }
            }
        }

        void *imgPtr = (void*)dObjImg->rowPtr(0, 0);
        ito::int32 PixEllX = 0;
        ito::int32 PixEllY = 0;


        // first we do the subpixel detection of the left and right borders. Therefore we loop over all pixels in y-direction
        for (ito::int32 y = minM; y <= maxM; y++)
        {
            ito::int32 LEdge = (ito::int32)StartEndX[y - minM][0];
            ito::int32 REdge = (ito::int32)StartEndX[y - minM][1];

            // do some sanity checks to reduce calculation time
            if ((LEdge == REdge) || (LEdge == minM) || (REdge == maxM) || (REdge == 0) || (LEdge == 0)
                    || (abs(REdge - LEdge) < (maxN - minN - 10) / 2.0))
            {
                continue;
            }

            // first do detection for left edge ...
            switch (dObjImg->getType())
            {
            case ito::tInt8:
                StartEndX[PixEllY][0] = subPixEdgeFalling<ito::int8>((ito::int8*)imgPtr, sizex, LEdge, y, 0, numBorderPix);
                break;

            case ito::tUInt8:
                StartEndX[PixEllY][0] = subPixEdgeFalling<ito::uint8>((ito::uint8*)imgPtr, sizex, LEdge, y, 0, numBorderPix);
                break;

            case ito::tInt16:
                StartEndX[PixEllY][0] = subPixEdgeFalling<ito::int16>((ito::int16*)imgPtr, sizex, LEdge, y, 0, numBorderPix);
                break;

            case ito::tUInt16:
                StartEndX[PixEllY][0] = subPixEdgeFalling<ito::uint16>((ito::uint16*)imgPtr, sizex, LEdge, y, 0, numBorderPix);
                break;

            case ito::tInt32:
                StartEndX[PixEllY][0] = subPixEdgeFalling<ito::int32>((ito::int32*)imgPtr, sizex, LEdge, y, 0, numBorderPix);
                break;

            case ito::tUInt32:
                StartEndX[PixEllY][0] = subPixEdgeFalling<ito::uint32>((ito::uint32*)imgPtr, sizex, LEdge, y, 0, numBorderPix);
                break;

            case ito::tFloat32:
                StartEndX[PixEllY][0] = subPixEdgeFalling<ito::float32>((ito::float32*)imgPtr, sizex, LEdge, y, 0, numBorderPix);
                break;

            case ito::tFloat64:
                StartEndX[PixEllY][0] = subPixEdgeFalling<ito::float64>((ito::float64*)imgPtr, sizex, LEdge, y, 0, numBorderPix);
                break;
            }

            // then for the right one
            switch (dObjImg->getType())
            {
            case ito::tInt8:
                StartEndX[PixEllY][0] += subPixEdgeRising<ito::int8>((ito::int8*)imgPtr, sizex, REdge, y, 0, numBorderPix);
                break;

            case ito::tUInt8:
                StartEndX[PixEllY][0] += subPixEdgeRising<ito::uint8>((ito::uint8*)imgPtr, sizex, REdge, y, 0, numBorderPix);
                break;

            case ito::tInt16:
                StartEndX[PixEllY][0] += subPixEdgeRising<ito::int16>((ito::int16*)imgPtr, sizex, REdge, y, 0, numBorderPix);
                break;

            case ito::tUInt16:
                StartEndX[PixEllY][0] += subPixEdgeRising<ito::uint16>((ito::uint16*)imgPtr, sizex, REdge, y, 0, numBorderPix);
                break;

            case ito::tInt32:
                StartEndX[PixEllY][0] += subPixEdgeRising<ito::int32>((ito::int32*)imgPtr, sizex, REdge, y, 0, numBorderPix);
                break;

            case ito::tUInt32:
                StartEndX[PixEllY][0] += subPixEdgeRising<ito::uint32>((ito::uint32*)imgPtr, sizex, REdge, y, 0, numBorderPix);
                break;

            case ito::tFloat32:
                StartEndX[PixEllY][0] += subPixEdgeRising<ito::float32>((ito::float32*)imgPtr, sizex, REdge, y, 0, numBorderPix);
                break;

            case ito::tFloat64:
                StartEndX[PixEllY][0] += subPixEdgeRising<ito::float64>((ito::float64*)imgPtr, sizex, REdge, y, 0, numBorderPix);
                break;
            }

            StartEndX[PixEllY][0] /= 2.0;
            StartEndX[PixEllY][1] = y;
            PixEllY++;
        }

        if (PixEllY < minPixEll)
        {
            centers[ellnum * 2] = -1;
            centers[ellnum * 2 + 1] = -1;
            free(StartEndX);
            free(StartEndY);
            StartEndX = NULL;
            StartEndY = NULL;
//            std::cout << "en: " << ellnum << " y < minEY\n" << std::endl;
            continue;
        }

        // then we do the detection of the upper and lower borders. In this case we loop over all pixels in x-direction
        for (ito::int32 x = minN; x <= maxN; x++)
        {
            ito::float64 TEdge = StartEndY[x - minN][0];
            ito::float64 BEdge = StartEndY[x - minN][1];

            // again some sanity checks
            if ((TEdge == BEdge) || (TEdge == minN) || (BEdge == maxN) || (TEdge == 0) || (BEdge == 0)
                    || (std::abs(TEdge - BEdge) < (maxM - minM - 10) / 2.0))
            {
                continue;
            }

            // this time top edge is first
            switch (dObjImg->getType())
            {
            case ito::tInt8:
                StartEndY[PixEllX][1] = subPixEdgeFalling<ito::int8>((ito::int8*)imgPtr, sizex, TEdge, x, 1, numBorderPix);
                break;

            case ito::tUInt8:
                StartEndY[PixEllX][1] = subPixEdgeFalling<ito::uint8>((ito::uint8*)imgPtr, sizex, TEdge, x, 1, numBorderPix);
                break;

            case ito::tInt16:
                StartEndY[PixEllX][1] = subPixEdgeFalling<ito::int16>((ito::int16*)imgPtr, sizex, TEdge, x, 1, numBorderPix);
                break;

            case ito::tUInt16:
                StartEndY[PixEllX][1] = subPixEdgeFalling<ito::uint16>((ito::uint16*)imgPtr, sizex, TEdge, x, 1, numBorderPix);
                break;

            case ito::tInt32:
                StartEndY[PixEllX][1] = subPixEdgeFalling<ito::int32>((ito::int32*)imgPtr, sizex, TEdge, x, 1, numBorderPix);
                break;

            case ito::tUInt32:
                StartEndY[PixEllX][1] = subPixEdgeFalling<ito::uint32>((ito::uint32*)imgPtr, sizex, TEdge, x, 1, numBorderPix);
                break;

            case ito::tFloat32:
                StartEndY[PixEllX][1] = subPixEdgeFalling<ito::float32>((ito::float32*)imgPtr, sizex, TEdge, x, 1, numBorderPix);
                break;

            case ito::tFloat64:
                StartEndY[PixEllX][1] = subPixEdgeFalling<ito::float64>((ito::float64*)imgPtr, sizex, TEdge, x, 1, numBorderPix);
                break;
            }

            // then for the right one
            switch (dObjImg->getType())
            {
            case ito::tInt8:
                StartEndY[PixEllX][1] += subPixEdgeRising<ito::int8>((ito::int8*)imgPtr, sizex, BEdge, x, 1, numBorderPix);
                break;

            case ito::tUInt8:
                StartEndY[PixEllX][1] += subPixEdgeRising<ito::uint8>((ito::uint8*)imgPtr, sizex, BEdge, x, 1, numBorderPix);
                break;

            case ito::tInt16:
                StartEndY[PixEllX][1] += subPixEdgeRising<ito::int16>((ito::int16*)imgPtr, sizex, BEdge, x, 1, numBorderPix);
                break;

            case ito::tUInt16:
                StartEndY[PixEllX][1] += subPixEdgeRising<ito::uint16>((ito::uint16*)imgPtr, sizex, BEdge, x, 1, numBorderPix);
                break;

            case ito::tInt32:
                StartEndY[PixEllX][1] += subPixEdgeRising<ito::int32>((ito::int32*)imgPtr, sizex, BEdge, x, 1, numBorderPix);
                break;

            case ito::tUInt32:
                StartEndY[PixEllX][1] += subPixEdgeRising<ito::uint32>((ito::uint32*)imgPtr, sizex, BEdge, x, 1, numBorderPix);
                break;

            case ito::tFloat32:
                StartEndY[PixEllX][1] += subPixEdgeRising<ito::float32>((ito::float32*)imgPtr, sizex, BEdge, x, 1, numBorderPix);
                break;

            case ito::tFloat64:
                StartEndY[PixEllX][1] += subPixEdgeRising<ito::float64>((ito::float64*)imgPtr, sizex, BEdge, x, 1, numBorderPix);
                break;
            }

            StartEndY[PixEllX][1] /= 2.0;
            StartEndY[PixEllX][0] = x;

            PixEllX++;
        }

        if (PixEllX < minPixEll)
        {
            centers[ellnum * 2] = -1;
            centers[ellnum * 2 + 1] = -1;
            free(StartEndX);
            free(StartEndY);
            StartEndX = NULL;
            StartEndY = NULL;
//            std::cout << "en: " << ellnum << " x < minEX\n" << std::endl;
            continue;
        }

        // now we have computed the center line pixels of the left - right and top - bottom borders
        // the next step will be to do a line fit for the lr and tb pixels
        cv::Mat AMatX(PixEllY, 2, CV_64F);
        AMatX = cv::Mat::zeros(PixEllY, 2, CV_64F);
        cv::Mat YValsX(PixEllY, 1, CV_64F);
        YValsX = cv::Mat::zeros(PixEllY, 1, CV_64F);

        /*
                // As we will use centering and scaling for line fit, we have to determine
                // the extremal values
                ito::float64 xmin = std::numeric_limits<ito::float64>::max();
                ito::float64 ymin = std::numeric_limits<ito::float64>::max();
                ito::float64 xmax = std::numeric_limits<ito::float64>::min();
                ito::float64 ymax = std::numeric_limits<ito::float64>::min();

                for (ito::int16 x = 0; x < PixEllY; x++)
                {
                    if (StartEndX[x][0] < xmin)
                        xmin = StartEndX[x][0];
                    if (StartEndX[x][0] > xmax)
                        xmax = StartEndX[x][0];
                    if (StartEndX[x][1] < ymin)
                        ymin = StartEndX[x][1];
                    if (StartEndX[x][1] > ymax)
                        ymax = StartEndX[x][1];
                }
                xmax /= 2.0; ymax /= 2.0;
                xmin /= xmax; ymin /= ymax;
        */

        for (ito::int16 x = 0; x < PixEllY; x++)
        {
            AMatX.at<ito::float64>(x, 0) = 1.0;
            AMatX.at<ito::float64>(x, 1) = StartEndX[x][1];
            YValsX.at<ito::float64>(x) = StartEndX[x][0];
        }

        cv::SVD svdX = cv::SVD(AMatX);
        cv::Mat WMatX = cv::Mat::zeros(2, 2, CV_64F);
        for (ito::int8 x = 0; x < 2; x++)
        {
            WMatX.at<ito::float64>(x, x) = 1.0 / svdX.w.at<ito::float64>(x);
        }

//        cv::transpose(svdX.u, svdX.u);
//        cv::transpose(svdX.vt, svdX.vt);
        cv::Mat koeffx = svdX.vt.t() * WMatX * svdX.u.t() * YValsX;

        /*
                ito::float64 xmin = std::numeric_limits<ito::float64>::max();
                ito::float64 ymin = std::numeric_limits<ito::float64>::max();
                ito::float64 xmax = std::numeric_limits<ito::float64>::min();
                ito::float64 ymax = std::numeric_limits<ito::float64>::min();

                for (ito::int8 x = 0; x < PixEllX; x++)
                {
                    if (StartEndY[x][0] < xmin)
                        xmin = StartEndY[x][0];
                    if (StartEndY[x][0] > xmax)
                        xmax = StartEndY[x][0];
                    if (StartEndY[x][1] < ymin)
                        ymin = StartEndY[x][1];
                    if (StartEndY[x][1] > ymax)
                        ymax = StartEndY[x][1];
                }
                xmax /= 2; ymax /= 2;
                xmin /= xmax; ymin /= ymax;
        */

        cv::Mat YValsY(PixEllX, 1, CV_64F);
        YValsY = cv::Mat::zeros(PixEllX, 1, CV_64F);
        cv::Mat AMatY(PixEllX, 2, CV_64F);
        AMatY = cv::Mat::zeros(PixEllX, 2, CV_64F);;

        for (ito::int16 x = 0; x < PixEllX; x++)
        {
            AMatY.at<ito::float64>(x, 0) = 1.0;
            AMatY.at<ito::float64>(x, 1) = StartEndY[x][0];
            YValsY.at<ito::float64>(x) = StartEndY[x][1];
        }

        cv::SVD svdY = cv::SVD(AMatY);
        cv::Mat WMatY = cv::Mat::zeros(2, 2, CV_64F);
        for (ito::int8 x = 0; x < 2; x++)
        {
            WMatY.at<ito::float64>(x, x) = 1.0 / svdY.w.at<ito::float64>(x);
        }

//        cv::transpose(svdY.u, svdY.u);
//        cv::transpose(svdY.vt, svdY.vt);
        cv::Mat koeffy = svdY.vt.t() * WMatY * svdY.u.t() * YValsY;

        ito::float64 centerx = (koeffx.at<ito::float64>(1) * koeffy.at<ito::float64>(0) + koeffx.at<ito::float64>(0))
                               / (1.0 - koeffx.at<ito::float64>(1) * koeffy.at<ito::float64>(1));
        ito::float64 centery = koeffy.at<ito::float64>(0) + centerx * koeffy.at<ito::float64>(1);

        if ((centerx <= minN) || (centerx >= maxN) || (centery <= minM) || (centery >= maxM))
//                || (!ito::dObjHelper::isFinite(centerx)) || (!ito::dObjHelper::isFinite(centery)))
//                || (centerx == std::numeric_limits<ito::float64>::infinity()) || (centerx == std::numeric_limits<ito::float64>::signaling_NaN() || (centerx != centerx))
//                || (centery == std::numeric_limits<ito::float64>::infinity()) || (centery == std::numeric_limits<ito::float64>::signaling_NaN()) || (centery != centery))
        {
            centers[ellnum * 2] = -1;
            centers[ellnum * 2 + 1] = -1;
//            std::cout << "en: " << ellnum << " center check cx: " << centerx << " cy: " << centery << "\n" << std::endl;
        }
        else
        {
            centers[ellnum * 2] = centerx;
            centers[ellnum * 2 + 1] = centery;
            ellCentFound++;
        }

        free(StartEndX);
        free(StartEndY);
        StartEndX = NULL;
        StartEndY = NULL;
    }

    ito::DataObject *ellCentFoundObj = new ito::DataObject();
    ellCentFoundObj->zeros(ellCentFound, 2, ito::tFloat64);
    ito::float64 *centersFound = (ito::float64*)ellCentFoundObj->rowPtr(0, 0);
    ito::int32 numEll = 0;
    // we give back only the centers where the detection worked properly
    for (ito::int32 ne = 0; ne < numLabels; ne++)
    {
        if (centers[ne * 2] > 0 && centers[ne * 2] < sizex && centers[ne * 2 + 1] > 0 && centers[ne * 2 + 1] < sizey)
        {
            centersFound[numEll * 2] = centers[ne * 2];
            centersFound[numEll * 2 + 1] = centers[ne * 2 + 1];
            numEll++;
        }
    }
    *centerMatObj = *ellCentFoundObj;
    delete ellCentFoundObj;

    int osizes[2] = {sizey, sizex};
    *dObjImg = ito::DataObject(2, osizes, ito::tFloat64, dxMat, 1);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
