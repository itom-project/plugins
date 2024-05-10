/* ************************************************************************
    This file is part of fftw-plugin for ito's itom measurement software

    The fftw-plugin for itom is a wrapper for the FFTW package.
    The FFTW package was developed at MIT by Matteo Frigo and Steven G.
    Johnson. It was published under GNU General Public License and
    can be downloaded under http://www.fftw.org/.

    The fftw-plugin is a free software: you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation, either version 3 of
    the License, or (at your option) any later version.

    the fftw-plugin is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar. If not, see <http://www.gnu.org/licenses/>.
************************************************************************ */

/*! \file FFTWfilters.cpp
   \brief   This file contains the itomflters class and interface definitions.

   \author ITO, UFAL
   \date 12.2011
*/

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "FFTWfilters.h"

#include "DataObject/dataObjectFuncs.h"
#include "common/numeric.h"
#include "fftw3.h"
#include "qnumeric.h"
#include "qvariant.h"

#include "pluginVersion.h"
#include "gitVersion.h"

#include <QtCore/QtPlugin>
#include <math.h>

#ifdef USEOPENMP
#define useomp 1
#else
#define useomp 0
#endif

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FFTWFiltersInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(FFTWFilters)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FFTWFiltersInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(FFTWFilters)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
FFTWFiltersInterface::FFTWFiltersInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("FFTW-Filter");

    m_description = QObject::tr("Wrapper for the FFTW");

    m_detaildescription = QObject::tr(
"This plugin provides wrappers for Fourier Transforms using the FFTW-library. These are for instance: \n\
- 1D FFT (over an arbitrary axis)\n\
- 1D inverse FFT (over an arbitrary axis)\n\
- 2D FFT (over the last two axes)\n\
- 2D inverse FFT (over the last two axes)\n\
\n\
The FFTW package was developed at MIT by Matteo Frigo and Steven G. Johnson.\
It was published under GNU General Public License and can be downloaded under http://www.fftw.org/ .\n\
\n\
To build this plugin you will need the libs from the fftw.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
FFTWFiltersInterface::~FFTWFiltersInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Parameters for fftw filter
\param[in|out]   paramsMand  Mandatory parameters for the filter function
\param[in|out]   paramsOpt   Optional parameters for the filter function
\param[out]   outVals   Outputvalues, not implemented for this function
\author ITO, Boettcher
\date
*/
ito::RetVal FFTWFilters::xfftshiftParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    ito::Param param = ito::Param("source", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Input object (n-dimensional, (u)int8, (u)int16, int32, float32, float64, complex64, complex128) which is shifted in-place.").toLatin1().data());
    paramsMand->append(param);

    param = ito::Param("axis", ito::ParamBase::Int | ito::ParamBase::In, -1, 1, -1, tr("shift axis: x and y axis (-1, default), only y axis (0), only x axis (1)").toLatin1().data());
    paramsOpt->append(param);

    param = ito::Param("axisIndex", ito::ParamBase::Int | ito::ParamBase::In, -1, 0, tr("shift axis in the case of a > 2D dataObject. (-1, default) the axis parameter is considered, (0) the 0 axis of a 3D dataObject is shifted").toLatin1().data());

    paramsOpt->append(param);
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void calcfftshift0(ito::DataObject *data, bool forward)
{
    /*
    size: even, forward == backward
    |1----2,3----4|    ->  |3----4,1----2|

    msize: odd, forward
    |1----2,3---4|     ->  |3---4,1----2|

    msize odd, backward
    |1---2,3----4|     ->  |3----4,1---2|

    */
    int dims = (*data).getDims();
    int axis0 = dims - 3;
    int axis1 = dims - 2;
    int axis2 = dims - 1;

    int sizeInAxisz = (*data).getSize(axis0);
    int sizeInAxisy = (*data).getSize(axis1);
    int sizeInAxisx = (*data).getSize(axis2);

    int idxData, idxBuf;
    int cntz, cnty;

    _Tp *rowPtrData = NULL;
    _Tp *rowPtrBuf1 = NULL;
    _Tp *rowPtrBuf2 = NULL;

    ito::DataObject buf1;
    ito::DataObject buf2;
    int bufSize = sizeof(_Tp);
    bool even = ((sizeInAxisz % 2) == 0);

    switch (ito::getDataType2<_Tp*>())
    {
    case ito::tUInt8:
        if (even && !forward)
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 , data->getSize(axis1), data->getSize(axis2), ito::tUInt8);
        }
        else
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 + 1, data->getSize(axis1), data->getSize(axis2), ito::tUInt8);
        }
        buf1 = ito::DataObject(data->getSize(axis0) - buf2.getSize(0), data->getSize(axis1), data->getSize(axis2), ito::tUInt8);
        break;
    case ito::tInt8:
        if (even && !forward)
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 , data->getSize(axis1), data->getSize(axis2), ito::tInt8);
        }
        else
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 + 1, data->getSize(axis1), data->getSize(axis2), ito::tInt8);
        }
        buf1 = ito::DataObject(data->getSize(axis0) - buf2.getSize(0), data->getSize(axis1), data->getSize(axis2), ito::tInt8);
        break;
    case ito::tUInt16:
        if (even && !forward)
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 , data->getSize(axis1), data->getSize(axis2), ito::tUInt16);
        }
        else
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 + 1, data->getSize(axis1), data->getSize(axis2), ito::tUInt16);
        }
        buf1 = ito::DataObject(data->getSize(axis0) - buf2.getSize(0), data->getSize(axis1), data->getSize(axis2), ito::tUInt16);
        break;
    case ito::tInt16:
        if (even && !forward)
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 , data->getSize(axis1), data->getSize(axis2), ito::tInt16);
        }
        else
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 + 1, data->getSize(axis1), data->getSize(axis2), ito::tInt16);
        }
        buf1 = ito::DataObject(data->getSize(axis0) - buf2.getSize(0), data->getSize(axis1), data->getSize(axis2), ito::tInt16);
        break;
    case ito::tFloat32:
        if (even && !forward)
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 , data->getSize(axis1), data->getSize(axis2), ito::tFloat32);
        }
        else
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 + 1, data->getSize(axis1), data->getSize(axis2), ito::tFloat32);
        }
        buf1 = ito::DataObject(data->getSize(axis0) - buf2.getSize(0), data->getSize(axis1), data->getSize(axis2), ito::tFloat32);
        break;
    case ito::tFloat64:
        if (even && !forward)
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 , data->getSize(axis1), data->getSize(axis2), ito::tFloat64);
        }
        else
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 + 1, data->getSize(axis1), data->getSize(axis2), ito::tFloat64);
        }
        buf1 = ito::DataObject(data->getSize(axis0) - buf2.getSize(0), data->getSize(axis1), data->getSize(axis2), ito::tFloat64);
        break;
    case ito::tComplex64:
        if (even && !forward)
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2, data->getSize(axis1), data->getSize(axis2), ito::tComplex64);
        }
        else
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 + 1, data->getSize(axis1), data->getSize(axis2), ito::tComplex64);
        }
        buf1 = ito::DataObject(data->getSize(axis0) - buf2.getSize(0), data->getSize(axis1), data->getSize(axis2), ito::tComplex64);
        break;
    case ito::tComplex128:
        if (even && !forward)
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2, data->getSize(axis1), data->getSize(axis2), ito::tComplex128);
        }
        else
        {
            buf2 = ito::DataObject(data->getSize(axis0) / 2 + 1, data->getSize(axis1), data->getSize(axis2), ito::tComplex128);
        }
        buf1 = ito::DataObject(data->getSize(axis0) - buf2.getSize(0), data->getSize(axis1), data->getSize(axis2), ito::tComplex128);
        break;
    }

    int cutSize = buf1.getSize(0);

    for (cntz = 0; cntz < sizeInAxisz; cntz++)
    {
        idxData = (*data).seekMat(cntz);

        if (cntz < cutSize)
        {
            idxBuf = buf1.seekMat(cntz);
        }
        else
        {
            idxBuf = buf2.seekMat(cntz - cutSize);
        }

        for (cnty = 0; cnty < (*data).getSize(1); cnty++)
        {
            rowPtrData = data->rowPtr<_Tp>(idxData, cnty);

            if (cntz < cutSize)//first half
            {
                rowPtrBuf1 = buf1.rowPtr<_Tp>(idxBuf, cnty);
                memcpy(rowPtrBuf1, rowPtrData, bufSize * sizeInAxisx);
            }
            else //second half
            {
                rowPtrBuf2 = buf2.rowPtr<_Tp>(idxBuf, cnty);
                memcpy(rowPtrBuf2, rowPtrData, bufSize * sizeInAxisx);
            }

        }
    }

    if (!even && forward)
    {
        cutSize = cutSize + 1;
    }

    for (cntz = 0; cntz < sizeInAxisz; cntz++)
    {
        idxData = (*data).seekMat(cntz);

        if (cntz < cutSize)
        {
            idxBuf = buf2.seekMat(cntz);
        }
        else
        {
            idxBuf = buf1.seekMat(cntz - cutSize);
        }

        for (cnty = 0; cnty < (*data).getSize(1); cnty++)
        {
            rowPtrData = data->rowPtr<_Tp>(idxData, cnty);

            if (cntz < cutSize) //first half
            {
                rowPtrBuf2 = buf2.rowPtr<_Tp>(idxBuf, cnty);
                memcpy(rowPtrData, rowPtrBuf2, bufSize * sizeInAxisx);
            }
            else //second half
            {
                rowPtrBuf1 = buf1.rowPtr<_Tp>(idxBuf, cnty);
                memcpy(rowPtrData, rowPtrBuf1, bufSize * sizeInAxisx);
            }

        }
    }

    buf1 = ito::DataObject();
    buf2 = ito::DataObject();

    rowPtrData = NULL;
    rowPtrBuf1 = NULL;
    rowPtrBuf2 = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void calcfftshift(uchar *data, int n, int m, int lineStep, int axis, bool forward)
{
    int p;
    bool even;
    int bufSize, bufSize2;

    lineStep *= sizeof(_Tp); //lineStep is in pixel, however we want it in bytes

    if ((axis == 0 || axis == -1) && m > 1)
    {
        /*from... switch to...
        (case: m is even)       (case: m is odd)         (case: m is odd)
        forward = backward      backward                 forward
        ---            ---      ---            ---       ---            ---
         1              3        1              3         1              3
         |              |        |              |         |              |
         |              |        |              |         |              |
         |              |        |              |         |              |
         |              |        2              |         |              4
         2              4       ---             4         2             ---
        ---            ---       3             ---       ---             1
         3 (idx: p)     1        |              1         3              |
         |              |        |              |         |              |
         |              |        |              |         |              |
         |              |        |              |         |              |
         |              |        4              2         4              2
         4              2       ---            ---       ---            ---
        ---            ---
        */
        p = forward ? (m + 1) / 2 : (m - (m + 1) / 2);
        even = ((m % 2) == 0);
        bufSize = sizeof(_Tp) * n;

        _Tp *buf1 = (_Tp*)malloc(bufSize);

        if (even) //simple switch lines
        {
            for (int i = 0; i < p; ++i)
            {
                //switch line i and (p+i)
                memcpy(buf1, data + i * lineStep, bufSize);
                memcpy(data + i * lineStep, data + (p + i) * lineStep, bufSize);
                memcpy(data + (p + i) * lineStep, buf1, bufSize);
            }
        }
        else
        {
            _Tp *buf2 = (_Tp*)malloc(bufSize);

            //a 2nd buffer is required to store another intermediate result
            if (p > 0 && !forward)
            {
                memcpy(buf1, data, bufSize);
                memcpy(data, data + p * lineStep, bufSize);

                for (int i = 1; i <= p; ++i)
                {
                    memcpy(buf2, buf1, bufSize);
                    memcpy(buf1, data + i * lineStep, bufSize);
                    memcpy(data + i * lineStep, data + (p + i) * lineStep, bufSize);
                    memcpy(data + (p + i) * lineStep, buf2, bufSize);
                }
            }
            else if (p > 0 && forward)
            {
                memcpy(buf1, data + (m - 1) * lineStep, bufSize);
                memcpy(data + (m - 1) * lineStep, data + (m - p) * lineStep, bufSize);

                for (int i = 1; i <= (m - p); ++i)
                {
                    memcpy(buf2, buf1, bufSize);
                    memcpy(buf1, data + (m - i - 1) * lineStep, bufSize);
                    memcpy(data + (m - i - 1) * lineStep, data + (m - p - i) * lineStep, bufSize);
                    memcpy(data + (m - p - i) * lineStep, buf2, bufSize);
                }
            }

            free(buf2);
            buf2 = NULL;
        }

        free(buf1);
        buf1 = NULL;
    }

    if ((axis == 1 || axis == -1) && n > 1)
    {
        /*
        m: even, forward == backward
        |1----2,3----4|    ->  |3----4,1----2|

        m: odd, forward
        |1----2,3---4|     ->  |3---4,1----2|

        m: odd, backward
        |1---2,3----4|     ->  |3----4,1---2|

        */
        p = forward ? (n + 1) / 2 : (n - (n + 1) / 2);
        even = ((m % 2) == 0);

        if (even || forward)
        {
            bufSize = sizeof(_Tp) * p; //bufSize >= bufSize2
            bufSize2 = sizeof(_Tp) * (n - p);
        }
        else
        {
            bufSize = sizeof(_Tp) * p; //bufSize < bufSize2
            bufSize2 = sizeof(_Tp) * (n - p);
        }

#if (USEOMP)
#pragma omp parallel num_threads(ito::AddInBase::getMaximumThreadCount())
        {
#endif
        uchar *dataline;

        if (even || forward)
        {
            _Tp *buf1 = (_Tp*)malloc(bufSize);

#if USEOMP
            #pragma omp for schedule(guided)
#endif
            for (int i = 0; i < m; ++i)
            {
                dataline = data + i * lineStep;
                memcpy(buf1, dataline, bufSize);
                memcpy(dataline, dataline + bufSize, bufSize2);
                memcpy(dataline + bufSize2, buf1, bufSize);
            }

            free(buf1);
            buf1 = NULL;
        }
        else //odd and backward
        {
            _Tp *buf1 = (_Tp*)malloc(bufSize2);

#if USEOMP
            #pragma omp for schedule(guided)
#endif
            for (int i = 0; i < m; ++i)
            {
                dataline = data + i * lineStep;
                memcpy(buf1, dataline + bufSize, bufSize2);
                memcpy(dataline + bufSize2, dataline, bufSize);
                memcpy(dataline, buf1, bufSize2);
            }

            free(buf1);
            buf1 = NULL;
        }
#if (USEOMP)
    }
#endif
    }
}

////----------------------------------------------------------------------------------------------------------------------------------
//template<typename _TP> void doifftshift(_TP *field, int sx, int sy, int lineStep)
//{
//    int halfX = sx / 2, halfY = sy / 2;
//    int xodd = 0, yodd = 0;
//    _TP *colbuf = NULL, *rowbuf = NULL, zeroR, zeroC;
//    if (sx / 2 != sx / 2.0)
//    {
//        xodd = 1;
//        colbuf = (_TP*)malloc(sx * sizeof(_TP));
//        for (int n = 0; n < sy; n++)
//            colbuf[n] = field[n * lineStep + sx - 1];
//    }
//    if (sy / 2 != sy / 2.0)
//    {
//        yodd = 1;
//        rowbuf = (_TP*)malloc(sx * sizeof(_TP));
//        memcpy(rowbuf, field + (sy - 1) * lineStep, sx * sizeof(_TP));
//    }
//    if (xodd && yodd)
//    {
//        zeroR = field[(sy - 1) * lineStep + halfX];
//        zeroC = field[halfY * lineStep + sx - 1];
//    }
//#if (USEOMP)
//#pragma omp parallel num_threads(ito::AddInBase::getMaximumThreadCount())
//    {
//#endif
//        _TP buffer;
//
//#if (USEOMP)
//#pragma omp for schedule(guided)
//#endif
//        for (int n = halfY - 1; n >= 0; n--)
//        {
//            for (int m = halfX - 1; m >= 0; m--)
//            {
//                buffer = field[(n) * lineStep + m];
//                field[(n) * lineStep + m] = field[(n + halfY) * lineStep + m + halfX];
//                field[(n + halfY + yodd) * lineStep + m + halfX + xodd] = buffer;
//                buffer = field[(n + halfY) * lineStep + m];
//                field[(n + halfY + yodd) * lineStep + m] = field[(n) * lineStep + m + halfX];
//                field[n * lineStep + m + halfX + xodd] = buffer;
//            }
//        }
//
//        if (xodd)
//        {
//#if (USEOMP)
//#pragma omp for schedule(guided)
//#endif
//            for (int n = 0; n < halfY + yodd; n++)
//            {
//                field[lineStep * n + halfX] = colbuf[n + halfY];
//                field[(n + halfY) * lineStep + halfX] = colbuf[n - yodd];
//            }
//        }
//
//        if (yodd)
//        {
//#if (USEOMP)
//#pragma omp for schedule(guided)
//#endif
//            for (int m = 0; m < halfX + xodd; m++)
//            {
//                field[(halfY)* lineStep + m] = rowbuf[m + halfX];
//                field[(halfY)* lineStep + m + halfX] = rowbuf[m - xodd];
//            }
//        }
//#if (USEOMP)
//    }
//#endif
//
//    if (rowbuf)
//        free(rowbuf);
//    if (colbuf)
//        free(colbuf);
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//template<typename _TP> void dofftshift(_TP *field, int sx, int sy, int lineStep)
//{
//    int halfX = sx / 2, halfY = sy / 2;
//    int xodd = 0, yodd = 0;
//    _TP *colbuf = NULL, *rowbuf = NULL, zeroR, zeroC;
//    if (sx / 2 != sx / 2.0)
//    {
//        xodd = 1;
//        colbuf = (_TP*)malloc(sx * sizeof(_TP));
//        for (int n = 0; n < sy; n++)
//            colbuf[n] = field[n * lineStep];
//    }
//    if (sy / 2 != sy / 2.0)
//    {
//        yodd = 1;
//        rowbuf = (_TP*)malloc(sx * sizeof(_TP));
//        memcpy(rowbuf, field, sx * sizeof(_TP));
//    }
//    if (xodd && yodd)
//    {
//        zeroR = field[halfX];
//        zeroC = field[halfY * lineStep];
//    }
//#if (USEOMP)
//#pragma omp parallel num_threads(ito::AddInBase::getMaximumThreadCount())
//    {
//#endif
//        _TP buffer;
//
//#if (USEOMP)
//        #pragma omp for schedule(guided)
//#endif
//        for (int n = 0; n < halfY; n++)
//        {
//            for (int m = 0; m < halfX; m++)
//            {
//                buffer = field[(n + yodd) * lineStep + m + xodd];
//                field[n * lineStep + m] = field[(n + yodd + halfY) * lineStep + m + halfX + xodd];
//                field[(n + halfY + yodd) * lineStep + m + halfX + xodd] = buffer;
//                buffer = field[(n + yodd + halfY) * lineStep + m + xodd];
//                field[(n + halfY + yodd) * lineStep + m] = field[(n + yodd) * lineStep + m + halfX + xodd];
//                field[n * lineStep + m + halfX + xodd] = buffer;
//            }
//        }
//
//        if (xodd)
//        {
//#if (USEOMP)
//            #pragma omp for schedule(guided)
//#endif
//            for (int n = 0; n < halfY; n++)
//            {
//                field[lineStep * n + halfX] = colbuf[n + halfY + yodd];
//                field[(n + halfY) * lineStep + halfX] = colbuf[n];
//            }
//        }
//
//        if (yodd)
//        {
//#if (USEOMP)
//            #pragma omp for schedule(guided)
//#endif
//            for (int m = 0; m < halfX; m++)
//            {
//                field[(halfY) * lineStep + m] = rowbuf[m + halfX + xodd];
//                field[(halfY) * lineStep + m + halfX] = rowbuf[m];
//            }
//        }
//#if (USEOMP)
//    }
//#endif
//    if (xodd && yodd)
//    {
//        field[halfY * lineStep + sx - 1] = zeroR;
//        field[(sy - 1) * lineStep + halfX] = zeroC;
//    }
//
//    if (rowbuf)
//        free(rowbuf);
//    if (colbuf)
//        free(colbuf);
//}

//----------------------------------------------------------------------------------------------------------------------------------
const QString FFTWFilters::fftshiftDOC = QObject::tr("Perform fftshift as known from Python, Matlab and so on, i.e. make the \n\
zero order of diffraction appear in the center.\n\
\n\
The shift is implemented along the x and y or one of both axes within each plane (inplace) by using the axis parameter.\n\
The axisIndex parameter is used the shift a >2D dataObject in the 0 axis.");

/*static*/ ito::RetVal FFTWFilters::fftshift(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *inField = paramsMand->at(0).getVal<ito::DataObject*>();
    int axis = paramsOpt->at(0).getVal<int>();
    int axisIdx = paramsOpt->at(1).getVal<int>();

    if (!inField)
    {
        return ito::RetVal(ito::retError, 0, "source not available");
    }

    retval += ito::dObjHelper::verifyDataObjectType(inField, "source", 10, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, \
                                                    ito::tUInt32, ito::tFloat32, ito::tFloat64, ito::tComplex64, ito::tComplex128);
    int dims = inField->getDims();
    int numPlanes = inField->getNumPlanes();

    if (dims < 2)
    {
        retval += ito::RetVal(ito::retError, 0, "source must have at least two dimensions");
    }

    if (dims > 3)
    {
        retval += ito::RetVal(ito::retError, 0, "source must have at least maximum 3 dimensions");
    }

    if (!retval.containsError())
    {

        if(dims > 2 && axisIdx == 0) //FFT shift of 3D Object in stack axis (z)
        {
            switch (inField->getType())
            {
            case ito::tInt8:
                calcfftshift0<ito::int8>(inField, true);
                break;
            case ito::tUInt8:
                calcfftshift0<ito::uint8>(inField, true);
                break;
            case ito::tInt16:
                calcfftshift0<ito::int16>(inField, true);
                break;
            case ito::tUInt16:
                calcfftshift0<ito::uint16>(inField, true);
                break;
            case ito::tInt32:
                calcfftshift0<ito::int32>(inField, true);
                break;
            case ito::tUInt32:
                calcfftshift0<ito::uint32>(inField, true);
                break;
            case ito::tFloat32:
                calcfftshift0<ito::float32>(inField, true);
                break;
            case ito::tFloat64:
                calcfftshift0<ito::float64>(inField, true);
                break;
            case ito::tComplex64:
                calcfftshift0<ito::complex64>(inField, true);
                break;
            case ito::tComplex128:
                calcfftshift0<ito::complex128>(inField, true);
                break;
            }
        }
        else
        {
            for (int p = 0; p < numPlanes; ++p)
            {
                switch (inField->getType())
                {
                case ito::tInt8:
                case ito::tUInt8:
                    calcfftshift<ito::uint8>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, true);
                    break;

                case ito::tInt16:
                case ito::tUInt16:
                    calcfftshift<ito::uint16>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, true);
                    break;

                case ito::tInt32:
                case ito::tUInt32:
                    calcfftshift<ito::uint32>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, true);
                    break;

                case ito::tFloat32:
                    calcfftshift<ito::float32>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, true);
                    break;

                case ito::tFloat64:
                    calcfftshift<ito::float64>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, true);
                    break;

                case ito::tComplex64:
                    calcfftshift<ito::complex64>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, true);
                    break;

                case ito::tComplex128:
                    calcfftshift<ito::complex128>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, true);
                    break;
                }
            }
        }
    }

    if (!retval.containsError())
    {
        if (dims > 2 && axisIdx == 0)
        {
            int size = inField->getSize(axisIdx);
            double phys1 = inField->getPixToPhys(axisIdx, 0);
            double phys2 = inField->getPixToPhys(axisIdx, size - 1);
            phys1 = 0.5 * (phys1 + phys2);
            inField->setAxisOffset(axisIdx, phys1 / inField->getAxisScale(axisIdx));
        }
        else
        {
            //shift the axis scales of the last two axis
            int dims = inField->getDims();
            int size;
            double phys1, phys2;
            for (int d = dims - 2; d < dims; ++d)
            {
                if (d >= 0)
                {
                    size = inField->getSize(d);
                    if (size > 0)
                    {
                        phys1 = inField->getPixToPhys(d, 0);
                        phys2 = inField->getPixToPhys(d, size - 1);
                        phys1 = 0.5 * (phys1 + phys2);
                        inField->setAxisOffset(d, phys1 / inField->getAxisScale(d));
                    }
                }
            }
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString FFTWFilters::ifftshiftDOC = QObject::tr("Perform ifftshift as known from Python, Matlab and so on, i.e. move the \n\
zero order of diffraction back to the corner to run the inverse fft correctly.\n\
\n\
The shift is implemented along the x and y or one of both axes within each plane (inplace) by using the axis parameter.\n\
The axisIndex parameter is used the shift a >2D dataObject in the 0 axis.");

/*static*/ ito::RetVal FFTWFilters::ifftshift(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *inField = paramsMand->at(0).getVal<ito::DataObject*>();
    int axis = paramsOpt->at(0).getVal<int>();
    int axisIdx = paramsOpt->at(1).getVal<int>();

    if (!inField)
    {
        return ito::RetVal(ito::retError, 0, "source not available");
    }

    retval += ito::dObjHelper::verifyDataObjectType(inField, "source", 10, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, \
                                                    ito::tUInt32, ito::tFloat32, ito::tFloat64, ito::tComplex64, ito::tComplex128);
    int dims = inField->getDims();
    int numPlanes = inField->getNumPlanes();

    if (dims < 2)
    {
        retval += ito::RetVal(ito::retError, 0, "source must have at least two dimensions");
    }

    if (dims > 3)
    {
        retval += ito::RetVal(ito::retError, 0, "source must have at least maximum 3 dimensions");
    }

    if (!retval.containsError())
    {
        if(dims > 2 && axisIdx == 0)
        {
            switch (inField->getType())
            {
            case ito::tInt8:
                calcfftshift0<ito::int8>(inField, false);
                break;
            case ito::tUInt8:
                calcfftshift0<ito::uint8>(inField, false);
                break;
            case ito::tInt16:
                calcfftshift0<ito::int16>(inField, false);
                break;
            case ito::tUInt16:
                calcfftshift0<ito::uint16>(inField, false);
                break;
            case ito::tInt32:
                calcfftshift0<ito::int32>(inField, false);
                break;
            case ito::tUInt32:
                calcfftshift0<ito::uint32>(inField, false);
                break;
            case ito::tFloat32:
                calcfftshift0<ito::float32>(inField, false);
                break;
            case ito::tFloat64:
                calcfftshift0<ito::float64>(inField, false);
                break;
            case ito::tComplex64:
                calcfftshift0<ito::complex64>(inField, false);
                break;
            case ito::tComplex128:
                calcfftshift0<ito::complex128>(inField, false);
                break;
            }
        }
        else
        {
            for (int p = 0; p < numPlanes; ++p)
            {
                switch (inField->getType())
                {
                case ito::tInt8:
                case ito::tUInt8:
                    calcfftshift<ito::uint8>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, false);
                    break;

                case ito::tInt16:
                case ito::tUInt16:
                    calcfftshift<ito::uint16>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, false);
                    break;

                case ito::tInt32:
                case ito::tUInt32:
                    calcfftshift<ito::uint32>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, false);
                    break;

                case ito::tFloat32:
                    calcfftshift<ito::float32>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, false);
                    break;

                case ito::tFloat64:
                    calcfftshift<ito::float64>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, false);
                    break;

                case ito::tComplex64:
                    calcfftshift<ito::complex64>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, false);
                    break;

                case ito::tComplex128:
                    calcfftshift<ito::complex128>(inField->rowPtr(p, 0), inField->getSize(dims - 1), inField->getSize(dims - 2), inField->getStep(dims - 2), axis, false);
                    break;
                }
            }
        }

    }

    if (!retval.containsError())
    {
        if (dims > 2 && axisIdx == 0)
        {
            int size = inField->getSize(axisIdx);
            double phys1 = inField->getPixToPhys(axisIdx, 0);
            double phys2 = inField->getPixToPhys(axisIdx, size - 1);
            phys1 = 0.5 * (phys1 + phys2);
            inField->setAxisOffset(axisIdx, phys1 / inField->getAxisScale(axisIdx));
        }
        else
        {
            //shift the axis scales of the last two axis
            int dims = inField->getDims();
            int size;
            double phys1, phys2;
            for (int d = dims - 2; d < dims; ++d)
            {
                if (d >= 0)
                {
                    size = inField->getSize(d);
                    if (size > 0)
                    {
                        phys1 = inField->getPixToPhys(d, 0);
                        phys2 = inField->getPixToPhys(d, size - 1);
                        phys1 = 0.5 * (phys1 + phys2);
                        inField->setAxisOffset(d, phys1 / inField->getAxisScale(d));
                    }
                }
            }
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectAxisIterator::DataObjectAxisIterator(const ito::DataObject *objIn, ito::DataObject *objOut, int axis) :
    m_axis(axis)
{
    m_obj[0] = objIn;
    m_obj[1] = objOut;

    m_len = objIn->getDims() - 2;

    int lastSizeBiggerOne = 0;

    m_sizes = new int[m_len];
    for (int i = 0; i < m_len; ++i)
    {
        m_sizes[i] = objIn->getSize(i) - 1;
        if (m_sizes[i] > 0)
        {
            lastSizeBiggerOne = i;
        }
    }
    m_sizes[axis] = 0;

    for (int n = 0; n < 2; ++n)
    {
        m_currentPlaneTreeIndex[n] = -1;
        m_currentSizes[n] = new int[m_len];
        memset(m_currentSizes[n], 0, sizeof(int) *(m_len));

        m_steps[n] = new int[m_len];
        m_steps[n][m_len - 1] = 1;
        int steps = 1;
        for (int i = m_len - 2; i >= 0; --i)
        {
            steps *= m_obj[n]->getOriginalSize(i + 1);
            m_steps[n][i] = steps;
        }

        m_controlIndex[n] = lastSizeBiggerOne;

        m_firstPlaneIndex[n] = m_obj[n]->seekMat(0);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectAxisIterator::~DataObjectAxisIterator()
{
    delete[] m_sizes;
    delete[] m_currentSizes[0];
    delete[] m_currentSizes[1];
    delete[] m_steps[0];
    delete[] m_steps[1];
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DataObjectAxisIterator::nextPlaneTreeIndex(int &planeIndexIn, int &planeIndexOut)
{
    if (m_obj[0] == m_obj[1]) //inplace
    {
        bool ret = nextPlaneTreeIndex(0, planeIndexIn);
        planeIndexOut = planeIndexIn;
        return ret;
    }
    else
    {
        nextPlaneTreeIndex(0, planeIndexIn);
        return nextPlaneTreeIndex(1, planeIndexOut);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool DataObjectAxisIterator::nextPlaneTreeIndex(char idx, int &planeIndex)
{
    if (m_currentPlaneTreeIndex[idx] == -1)
    {
        m_currentPlaneTreeIndex[idx] = m_firstPlaneIndex[idx];
        planeIndex = m_currentPlaneTreeIndex[idx];
        return true;
    }
    else if (m_currentSizes[idx][m_controlIndex[idx]] < m_sizes[m_controlIndex[idx]]) //check if control index can still be incremented:
    {
        //yes it can
        m_currentSizes[idx][m_controlIndex[idx]]++;
        m_currentPlaneTreeIndex[idx] += m_steps[idx][m_controlIndex[idx]];
        planeIndex = m_currentPlaneTreeIndex[idx];
        return true;
    }
    else //one number before will be incremented and currentIndex is set to the last
    {
        //search from the back for the next index that can be incremented
        for (int i = m_controlIndex[idx]; i >= 0; --i)
        {
            if (m_currentSizes[idx][i] < m_sizes[i])
            {
                m_currentSizes[idx][i]++;
                memset(&m_currentSizes[idx][i + 1], 0, sizeof(int) * (m_len - i - 1));

                m_currentPlaneTreeIndex[idx] = m_firstPlaneIndex[idx];
                for (int j = 0; j <= i; ++j)
                {
                    m_currentPlaneTreeIndex[idx] += (m_steps[idx][j] * m_currentSizes[idx][j]);
                }

                planeIndex = m_currentPlaneTreeIndex[idx];
                return true;
            }
        }

        return false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
int DataObjectAxisIterator::getPlaneStepIn() const
{
    return m_steps[0][m_axis];
}

//----------------------------------------------------------------------------------------------------------------------------------
int DataObjectAxisIterator::getPlaneStepOut() const
{
    return m_steps[1][m_axis];
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
FFTWFilters::FFTWFilters() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
FFTWFilters::~FFTWFilters()
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Parameters for fftw filter
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optional parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO, Boettcher
   \date
*/
ito::RetVal FFTWFilters::xfftw1dParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("source", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input object (n-dimensional, (u)int8, (u)int16, int32, float32, float64, complex64, complex128)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destination", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output object (inplace allowed, but only feasible if source is complex64 or complex128). Destination has the same size than the input object, the type is either complex128 (for float64 or complex128 inputs) or complex64 (else).").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("plan_flag", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, \
tr("Method flag, 0: Estimate (default), 1: Measure.  Measure instructs FFTW to run and measure the execution time of several FFTs in order to \
find the best way to compute the transform of size n. This process takes some time (usually a few seconds), depending on your machine and on the \
size of the transform. Estimate, on the contrary, does not run any computation and just builds a reasonable plan that is probably sub-optimal. \
In short, if your program performs many transforms of the same size and initialization time is not important, use Measure; otherwise use Estimate. ").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("axis", ito::ParamBase::Int | ito::ParamBase::In, -1, std::numeric_limits<int>::max(), -1, tr("Axis over which to compute the FFT. If not given, the last axis is used.").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("norm", ito::ParamBase::String | ito::ParamBase::In, "default", tr("Normalization method. no: neither fft nor ifft are scaled, default: direct transform (fft) is not scaled, inverse transform is scaled by 1/n, ortho: both direct and inverse transforms are scaled by 1/sqrt(n)").toLatin1().data());
        ito::StringMeta sm(ito::StringMeta::String);
        sm.addItem("no"); //neither fft nor ifft are scaled
        sm.addItem("default"); //numpy default: fft is not scaled, ifft is scaled with 1/n
        sm.addItem("ortho"); //fft and ifft are scaled with 1/sqrt(n)
        param.setMeta(&sm, false);
        paramsOpt->append(param);

    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Parameters for fftw filter
\param[in|out]   paramsMand  Mandatory parameters for the filter function
\param[in|out]   paramsOpt   Optional parameters for the filter function
\param[out]   outVals   Outputvalues, not implemented for this function
\author ITO, Boettcher
\date
*/
ito::RetVal FFTWFilters::xfftw2dParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("source", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input object (n-dimensional, (u)int8, (u)int16, int32, float32, float64, complex64, complex128)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destination", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output object (inplace allowed, but only feasible if source is complex64 or complex128). Destination has the same size than the input object, the type is either complex128 (for float64 or complex128 inputs) or complex64 (else).").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("plan_flag", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, \
            tr("Method flag, 0: Estimate (default), 1: Measure.  Measure instructs FFTW to run and measure the execution time of several FFTs in order to \
find the best way to compute the transform of size n. This process takes some time (usually a few seconds), depending on your machine and on the \
size of the transform. Estimate, on the contrary, does not run any computation and just builds a reasonable plan that is probably sub-optimal. \
In short, if your program performs many transforms of the same size and initialization time is not important, use Measure; otherwise use Estimate. ").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("norm", ito::ParamBase::String | ito::ParamBase::In, "default", tr("Normalization method. no: neither fft nor ifft are scaled, default: direct transform (fft) is not scaled, inverse transform is scaled by 1/n, ortho: both direct and inverse transforms are scaled by 1/sqrt(n)").toLatin1().data());
        ito::StringMeta sm(ito::StringMeta::String);
        sm.addItem("no"); //neither fft nor ifft are scaled
        sm.addItem("default"); //numpy default: fft is not scaled, ifft is scaled with 1/n
        sm.addItem("ortho"); //fft and ifft are scaled with 1/sqrt(n)
        param.setMeta(&sm, false);
        paramsOpt->append(param);

    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs forward dft by means of fftw row by row (# rows >=1) complex to complex
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in,
                                                        [1]complex Data object out
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
const QString FFTWFilters::fftw1dDOC = QObject::tr("Compute the one-dimensional discrete Fourier Transform. \n\
\n\
This method computes the one-dimensional n-point discrete Fourier Transform (DFT) with the efficient \n\
Fast Fourier Transform (FFT) algorithm using the fast, GPL licensed library FFTW (fftw.org). The transform \n\
is executed over a desired axis. \n\
\n\
This method applies the forward transform, use 'ifft' for the inverse transform. The method works both \n\
inplace as well as out-of-place. The output is a complex64 object of the same size than the input object \n\
if the input is of one of the following types: (u)int8, (u)int16, int32, float32 or complex64. If the \n\
input object has one of the types float64 or complex128, the output is complex128. If a type conversion is necessary, \n\
a new dataObject is always put into the destination object. \n\
\n\
Meta and axes information are copied to the output object. Only properties of the chosen axis are changed: \n\
\n\
* offset: 0.0 \n\
* scaling: 1.0 / (previous-scaling * n), the factor 2pi is not considered here \n\
* unit: inverse of previous-unit, e.g. '1/previous-unit' \n\
\n\
Per default, no value scaling is applied to the result. However, the optional parameter 'norm' influences this. \n\
If 'norm' is set to 'ortho', the values are scaled by 1/sqrt(n). \n\
\n\
The FFTW library comes with two execution strategies: Measure and Estimate. One of both can be chosen by the optional \n\
parameter 'plan_flag'. While Estimate selects a default algorithm, Measure will process some test runs with several \n\
implementations on your machine in order to find out the fastest algorithm for the given type of object. This will \n\
take some time but might speed-up calculations of huge objects.");
ito::RetVal FFTWFilters::fftw1d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    return doFFT1D(paramsMand, paramsOpt, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs backward dft by means of fftw row by row (# rows >=1) complex to complex
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in,
                                                        [1]complex Data object out
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
const QString FFTWFilters::ifftw1dDOC = QObject::tr("Compute the inverse one-dimensional discrete Fourier Transform. \n\
\n\
This method computes the inverse one-dimensional n-point discrete Fourier Transform (DFT) with the efficient \n\
Fast Fourier Transform (FFT) algorithm using the fast, GPL licensed library FFTW (fftw.org). The transform \n\
is executed over a desired axis. \n\
\n\
This method applies the inverse transform, use 'fft' for the forward transform. The method works both \n\
inplace as well as out-of-place. The output is a complex64 object of the same size than the input object \n\
if the input is of one of the following types: (u)int8, (u)int16, int32, float32 or complex64. If the \n\
input object has one of the types float64 or complex128, the output is complex128. If a type conversion is necessary, \n\
a new dataObject is always put into the destination object. \n\
\n\
Meta and axes information are copied to the output object. Only properties of the chosen axis are changed: \n\
\n\
* offset: 0.0 \n\
* scaling: 1.0 / (previous-scaling * n), the factor 2pi is not considered here \n\
* unit: inverse of previous-unit, e.g. '1/previous-unit' \n\
\n\
Per default, the values are scaled by (1/n). However, the optional parameter 'norm' influences this. \n\
If 'norm' is set to 'no', no scaling is applied, if 'norm' is set to 'ortho', the values are scaled by 1/sqrt(n). \n\
\n\
The FFTW library comes with two execution strategies: Measure and Estimate. One of both can be chosen by the optional \n\
parameter 'plan_flag'. While Estimate selects a default algorithm, Measure will process some test runs with several \n\
implementations on your machine in order to find out the fastest algorithm for the given type of object. This will \n\
take some time but might speed-up calculations of huge objects.");

ito::RetVal FFTWFilters::ifftw1d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    return doFFT1D(paramsMand, paramsOpt, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs forward dft of 2D objects complex to complex by means of fftw
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in,
                                                        [1]complex Data object out
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
const QString FFTWFilters::fftw2dDOC = QObject::tr("Compute the two-dimensional discrete Fourier Transform. \n\
\n\
This method computes the two-dimensional n-point discrete Fourier Transform (DFT) with the efficient \n\
Fast Fourier Transform (FFT) algorithm using the fast, GPL licensed library FFTW (fftw.org). The transform \n\
is executed over the last two axes, denoted as planes. \n\
\n\
This method applies the forward transform, use 'ifft2D' for the inverse transform. The method works both \n\
inplace as well as out-of-place. The output is a complex64 object of the same size than the input object \n\
if the input is of one of the following types: (u)int8, (u)int16, int32, float32 or complex64. If the \n\
input object has one of the types float64 or complex128, the output is complex128. If a type conversion is necessary, \n\
a new dataObject is always put into the destination object. \n\
\n\
Meta and axes information are copied to the output object. Only properties of the last two axes are changed: \n\
\n\
* offset: 0.0 \n\
* scaling: 1.0 / (previous-scaling * n), the factor 2pi is not considered here \n\
* unit: inverse of previous-unit, e.g. '1/previous-unit' \n\
\n\
Per default, no value scaling is applied to the result. However, the optional parameter 'norm' influences this. \n\
If 'norm' is set to 'ortho', the values are scaled by 1/sqrt(n). \n\
\n\
The FFTW library comes with two execution strategies: Measure and Estimate. One of both can be chosen by the optional \n\
parameter 'plan_flag'. While Estimate selects a default algorithm, Measure will process some test runs with several \n\
implementations on your machine in order to find out the fastest algorithm for the given type of object. This will \n\
take some time but might speed-up calculations of huge objects.");
ito::RetVal FFTWFilters::fftw2d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    return doFFT2D(paramsMand, paramsOpt, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs backward dft of 2D objects complex to complex by means of fftw
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in,
                                                        [1]complex Data object out
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
const QString FFTWFilters::ifftw2dDOC = QObject::tr("Compute the inverse two-dimensional discrete Fourier Transform. \n\
\n\
This method computes the inverse two-dimensional n-point discrete Fourier Transform (DFT) with the efficient \n\
Fast Fourier Transform (FFT) algorithm using the fast, GPL licensed library FFTW (fftw.org). The transform \n\
is executed over the last two axes, denoted as planes. \n\
\n\
This method applies the inverse transform, use 'fft2D' for the forward transform. The method works both \n\
inplace as well as out-of-place. The output is a complex64 object of the same size than the input object \n\
if the input is of one of the following types: (u)int8, (u)int16, int32, float32 or complex64. If the \n\
input object has one of the types float64 or complex128, the output is complex128. If a type conversion is necessary, \n\
a new dataObject is always put into the destination object. \n\
\n\
Meta and axes information are copied to the output object. Only properties of the last two axes are changed: \n\
\n\
* offset: 0.0 \n\
* scaling: 1.0 / (previous-scaling * n), the factor 2pi is not considered here \n\
* unit: inverse of previous-unit, e.g. '1/previous-unit' \n\
\n\
Per default, the values are scaled by (1/n). However, the optional parameter 'norm' influences this. \n\
If 'norm' is set to 'no', no scaling is applied, if 'norm' is set to 'ortho', the values are scaled by 1/sqrt(n). \n\
\n\
The FFTW library comes with two execution strategies: Measure and Estimate. One of both can be chosen by the optional \n\
parameter 'plan_flag'. While Estimate selects a default algorithm, Measure will process some test runs with several \n\
implementations on your machine in order to find out the fastest algorithm for the given type of object. This will \n\
take some time but might speed-up calculations of huge objects.");

ito::RetVal FFTWFilters::ifftw2d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    return doFFT2D(paramsMand, paramsOpt, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FFTWFilters::doFFT1D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, const bool forward)
{
    ito::RetVal retval = ito::retOk;

    const ito::DataObject *dObjIn = (*paramsMand)[0].getVal<ito::DataObject*>();  //Input object
    ito::DataObject *dObjOut = (*paramsMand)[1].getVal<ito::DataObject*>();  //Output object
    int plan_select = (*paramsOpt)[0].getVal<int>();  //plan selection string for fftw
    unsigned int flags = (plan_select == 0) ? FFTW_ESTIMATE : FFTW_MEASURE;
    int axis = (*paramsOpt)[1].getVal<int>();
    QByteArray norm = paramsOpt->at(2).getVal<char*>();
    int planForwardBackWard = forward ? FFTW_FORWARD : FFTW_BACKWARD;

    retval += ito::dObjHelper::verifyDataObjectType(dObjIn, "source", 9, ito::tInt8, ito::tUInt8,
        ito::tInt16, ito::tUInt16,
        ito::tInt32,
        ito::tFloat32, ito::tFloat64,
        ito::tComplex64, ito::tComplex128);

    if (!retval.containsError())
    {
        int dims = dObjIn->getDims();
        int inType = dObjIn->getType();
        ito::tDataType destType = (inType == ito::tComplex128 || inType == ito::tFloat64) ? ito::tComplex128 : ito::tComplex64;

        if (axis == -1)
        {
            axis = dims - 1;
        }
        else if (axis >= dims)
        {
            retval += ito::RetVal::format(ito::retError, 0, "axis out of range [0,%i]", dObjIn->getDims());
        }

        if (dims == 0 || dObjIn->getTotal() == 0)
        {
            retval += ito::RetVal(ito::retError, 0, "source is empty");
        }

        if (!retval.containsError())
        {
            bool inplace = (dObjIn == dObjOut);
            int numPlanes = dObjIn->getNumPlanes();

            if (inType != destType)
            {
                ito::DataObject destination;
                dObjIn->convertTo(destination, destType);
                dObjIn = dObjOut;
                *dObjOut = destination;
                inType = destType;
                inplace = true;
            }
            else if (!inplace)
            {
                if (dObjOut->getType() != destType || dObjOut->getDims() != dObjIn->getDims() || \
                    dObjOut->getContinuous() != dObjIn->getContinuous() || dObjOut->getSize() != dObjIn->getSize())
                {
                    *dObjOut = ito::DataObject(dObjIn->getDims(), dObjIn->getSize(), destType, dObjIn->getContinuous());
                    dObjIn->copyAxisTagsTo(*dObjOut);
                    dObjIn->copyTagMapTo(*dObjOut);
                }
            }

            int n[] = { 1 };

#if (USEOMP)
            fftw_init_threads();
            fftw_plan_with_nthreads(ito::AddInBase::getMaximumThreadCount());
#endif
            if (axis >= dims - 2) //axis is along x or y, works inplace and not-inplace
            {
                int otherAxis = (axis == (dims - 1)) ? dims - 2 : dims - 1;
                cv::Mat *planeOut = dObjOut->get_mdata()[dObjOut->seekMat(0, numPlanes)];
                const cv::Mat *planeIn = dObjIn->get_mdata()[dObjIn->seekMat(0, numPlanes)];

                n[0] = dObjOut->getSize(axis);
                int howmany = dObjOut->getSize(otherAxis);
                int strideIn = dObjIn->getStep(axis);      //step size from one value to the next value (within one 1d data set)
                int strideOut = dObjOut->getStep(axis);    //step size from one value to the next value (within one 1d data set)
                int distIn = dObjIn->getStep(otherAxis);   //step size from one dft-line to the next dft-line.
                int distOut = dObjOut->getStep(otherAxis); //step size from one dft-line to the next dft-line.

                if (destType == ito::tComplex128)
                {
                    fftw_plan plan;
                    fftw_complex *out = (fftw_complex*)planeOut->ptr<ito::complex128>(0);
                    fftw_complex *in = (fftw_complex*)planeIn->ptr<ito::complex128>(0);
                    if (flags == FFTW_ESTIMATE)
                    {
                        plan = fftw_plan_many_dft(1, n, howmany, in, NULL, strideIn, distIn, out, NULL, strideOut, distOut, planForwardBackWard, flags);
                        fftw_execute(plan);
                    }
                    else //FFTW_MEASURE (in,out will be overwritten during measuring)
                    {
                        if (in == out)
                        {
                            in = (fftw_complex*)fftw_malloc(howmany * distIn * n[0] * strideIn * sizeof(fftw_complex));
                            plan = fftw_plan_many_dft(1, n, howmany, in, NULL, strideIn, distIn, in, NULL, strideOut, distOut, planForwardBackWard, flags);
                            fftw_free(in);
                        }
                        else
                        {
                            in = (fftw_complex*)fftw_malloc(howmany * distIn * n[0] * strideIn * sizeof(fftw_complex));
                            plan = fftw_plan_many_dft(1, n, howmany, in, NULL, strideIn, distIn, out, NULL, strideOut, distOut, planForwardBackWard, flags);
                            fftw_free(in);
                        }

                        in = (fftw_complex*)planeIn->ptr<ito::complex128>(0);
                        fftw_execute_dft(plan, in, out);
                    }

                    for (int z = 1; z < numPlanes; z++)
                    {
                        out = (fftw_complex*)(dObjOut->get_mdata()[dObjOut->seekMat(z, numPlanes)])->ptr<ito::complex128>(0);
                        in = (fftw_complex*)(dObjIn->get_mdata()[dObjIn->seekMat(z, numPlanes)])->ptr<ito::complex128>(0);
                        fftw_execute_dft(plan, in, out);
                    }

                    fftw_destroy_plan(plan);
                }
                else
                {
                    fftwf_plan plan;
                    fftwf_complex *out = (fftwf_complex*)planeOut->ptr<ito::complex64>(0);
                    fftwf_complex *in = (fftwf_complex*)planeIn->ptr<ito::complex64>(0);
                    if (flags == FFTW_ESTIMATE)
                    {
                        plan = fftwf_plan_many_dft(1, n, howmany, in, NULL, strideIn, distIn, out, NULL, strideOut, distOut, planForwardBackWard, flags);
                        fftwf_execute(plan);
                    }
                    else //FFTW_MEASURE (in,out will be overwritten during measuring)
                    {
                        if (in == out)
                        {
                            in = (fftwf_complex*)fftwf_malloc(howmany * distIn * n[0] * strideIn * sizeof(fftwf_complex));
                            plan = fftwf_plan_many_dft(1, n, howmany, in, NULL, strideIn, distIn, in, NULL, strideOut, distOut, planForwardBackWard, flags);
                            fftwf_free(in);
                        }
                        else
                        {
                            in = (fftwf_complex*)fftwf_malloc(howmany * distIn * n[0] * strideIn * sizeof(fftwf_complex));
                            plan = fftwf_plan_many_dft(1, n, howmany, in, NULL, strideIn, distIn, out, NULL, strideOut, distOut, planForwardBackWard, flags);
                            fftwf_free(in);
                        }

                        in = (fftwf_complex*)planeIn->ptr<ito::complex64>(0);
                        fftwf_execute_dft(plan, in, out);
                    }

                    for (int z = 1; z < numPlanes; z++)
                    {
                        out = (fftwf_complex*)(dObjOut->get_mdata()[dObjOut->seekMat(z, numPlanes)])->ptr<ito::complex64>(0);
                        in = (fftwf_complex*)(dObjIn->get_mdata()[dObjIn->seekMat(z, numPlanes)])->ptr<ito::complex64>(0);
                        fftwf_execute_dft(plan, in, out);
                    }

                    fftwf_destroy_plan(plan);
                }
            }
            else if (dObjOut->getContinuous())
            {
                n[0] = dObjOut->getSize(axis);
                int howmany = dObjOut->getSize(dObjOut->getDims() - 1);
                int rowStepIn = dObjIn->getStep(dObjIn->getDims() - 2);
                int rowStepOut = dObjOut->getStep(dObjOut->getDims() - 2);
                int rows = dObjOut->getSize(dObjOut->getDims() - 2);
                int strideIn = dObjIn->getStep(axis); //step size from one value to the next value (within one 1d data set)
                int strideOut = dObjOut->getStep(axis); //step size from one value to the next value (within one 1d data set)
                int dist = 1; //step size from one dft-line to the next dft-line.

                DataObjectAxisIterator it(dObjIn, dObjOut, axis);
                int planeIndexOut, planeIndexIn;

                if (destType == ito::tComplex128)
                {
                    fftw_plan plan;
                    fftw_complex *out = (fftw_complex*)dObjOut->get_mdata()[0]->ptr<ito::complex128>(0);
                    fftw_complex *in = (fftw_complex*)dObjIn->get_mdata()[0]->ptr<ito::complex128>(0);
                    if (flags == FFTW_ESTIMATE)
                    {
                        plan = fftw_plan_many_dft(1, n, howmany, in, NULL, strideIn, dist, out, NULL, strideOut, dist, planForwardBackWard, flags);
                    }
                    else //FFTW_MEASURE (in,out will be overwritten during measuring)
                    {
                        if (in == out)
                        {
                            in = (fftw_complex*)fftw_malloc(howmany * n[0] * strideIn * sizeof(fftw_complex));
                            plan = fftw_plan_many_dft(1, n, howmany, in, NULL, strideIn, dist, in, NULL, strideOut, dist, planForwardBackWard, flags);
                            fftw_free(in);
                        }
                        else
                        {
                            in = (fftw_complex*)fftw_malloc(howmany * n[0] * strideIn * sizeof(fftw_complex));
                            plan = fftw_plan_many_dft(1, n, howmany, in, NULL, strideIn, dist, out, NULL, strideOut, dist, planForwardBackWard, flags);
                            fftw_free(in);
                        }
                    }

                    while (it.nextPlaneTreeIndex(planeIndexIn, planeIndexOut))
                    {
                        //handle first row
                        out = (fftw_complex*)dObjOut->get_mdata()[planeIndexOut]->ptr<ito::complex128>(0);
                        in = (fftw_complex*)dObjIn->get_mdata()[planeIndexIn]->ptr<ito::complex128>(0);
                        fftw_execute_dft(plan, in, out);

                        //handle all other rows
                        for (int r = 1; r < rows; ++r)
                        {
                            out += rowStepOut;
                            in += rowStepIn;
                            fftw_execute_dft(plan, in, out);
                        }
                    }
                    fftw_destroy_plan(plan);
                }
                else
                {
                    fftwf_plan plan;
                    fftwf_complex *out = (fftwf_complex*)dObjOut->get_mdata()[0]->ptr<ito::complex64>(0);
                    fftwf_complex *in = (fftwf_complex*)dObjIn->get_mdata()[0]->ptr<ito::complex64>(0);
                    if (flags == FFTW_ESTIMATE)
                    {
                        plan = fftwf_plan_many_dft(1, n, howmany, in, NULL, strideIn, dist, out, NULL, strideOut, dist, planForwardBackWard, flags);
                    }
                    else //FFTW_MEASURE (in,out will be overwritten during measuring)
                    {
                        if (in == out)
                        {
                            in = (fftwf_complex*)fftwf_malloc(howmany * n[0] * strideIn * sizeof(fftwf_complex));
                            plan = fftwf_plan_many_dft(1, n, howmany, in, NULL, strideIn, dist, in, NULL, strideOut, dist, planForwardBackWard, flags);
                            fftwf_free(in);
                        }
                        else
                        {
                            in = (fftwf_complex*)fftwf_malloc(howmany * n[0] * strideIn * sizeof(fftwf_complex));
                            plan = fftwf_plan_many_dft(1, n, howmany, in, NULL, strideIn, dist, out, NULL, strideOut, dist, planForwardBackWard, flags);
                            fftwf_free(in);
                        }
                    }

                    while (it.nextPlaneTreeIndex(planeIndexIn, planeIndexOut))
                    {
                        //handle first row
                        out = (fftwf_complex*)dObjOut->get_mdata()[planeIndexOut]->ptr<ito::complex64>(0);
                        in = (fftwf_complex*)dObjIn->get_mdata()[planeIndexIn]->ptr<ito::complex64>(0);
                        fftwf_execute_dft(plan, in, out);

                        //handle all other rows
                        for (int r = 1; r < rows; ++r)
                        {
                            out += rowStepOut;
                            in += rowStepIn;
                            fftwf_execute_dft(plan, in, out);
                        }
                    }
                    fftwf_destroy_plan(plan);
                }
            }
            else //axis is within planes, not continuous object
            {
                n[0] = dObjOut->getSize(axis);
                int howmany = dObjOut->getSize(dObjOut->getDims() - 1);
                int rowStepIn = dObjIn->getStep(dObjIn->getDims() - 2);
                int rowStepOut = dObjOut->getStep(dObjOut->getDims() - 2);
                int rows = dObjOut->getSize(dObjOut->getDims() - 2);
                int cols = dObjOut->getSize(dObjOut->getDims() - 1);
                const cv::Mat **mdataIn = dObjIn->get_mdata();
                cv::Mat **mdataOut = dObjOut->get_mdata();



                DataObjectAxisIterator it(dObjIn, dObjOut, axis);
                int planeIndexOut, planeIndexIn;
                int planeStepIn = it.getPlaneStepIn();
                int planeStepOut = it.getPlaneStepOut();

                if (destType == ito::tComplex128)
                {
                    //this is ok for estimate and measure
                    ito::complex128 *out = new ito::complex128[n[0]];
                    fftw_plan plan = fftw_plan_many_dft(1, n, 1, (fftw_complex*)out, NULL, 1, 1, (fftw_complex*)out, NULL, 1, 1, planForwardBackWard, flags);

                    while (it.nextPlaneTreeIndex(planeIndexIn, planeIndexOut))
                    {
                        for (int row = 0; row < rows; ++row)
                        {
                            for (int col = 0; col < cols; ++col)
                            {
                                fListGetComplexLineToComplex128[inType](&mdataIn[planeIndexIn], planeStepIn, n[0], row, col, rowStepIn, out);
                                fftw_execute_dft(plan, (fftw_complex*)out, (fftw_complex*)out);
                                setComplexLine<ito::complex128>(&(mdataOut[planeIndexOut]), planeStepOut, n[0], row, col, rowStepOut, out);
                            }
                        }
                    }
                    fftw_destroy_plan(plan);
                    delete[] out;
                }
                else
                {
                    //this is ok for estimate and measure
                    ito::complex64 *out = new ito::complex64[n[0]];
                    fftwf_plan plan = fftwf_plan_many_dft(1, n, 1, (fftwf_complex*)out, NULL, 1, 1, (fftwf_complex*)out, NULL, 1, 1, planForwardBackWard, flags);

                    while (it.nextPlaneTreeIndex(planeIndexIn, planeIndexOut))
                    {
                        for (int row = 0; row < rows; ++row)
                        {
                            for (int col = 0; col < cols; ++col)
                            {
                                fListGetComplexLineToComplex64[inType](&mdataIn[planeIndexIn], planeStepIn, n[0], row, col, rowStepIn, out);
                                fftwf_execute_dft(plan, (fftwf_complex*)out, (fftwf_complex*)out);
                                setComplexLine<ito::complex64>(&(mdataOut[planeIndexOut]), planeStepOut, n[0], row, col, rowStepOut, out);
                            }
                        }
                    }
                    fftwf_destroy_plan(plan);
                    delete[] out;
                }
            }

            if (forward)
            {
                if (norm == "ortho")
                {
                    *dObjOut *= (1.0 / std::sqrt((double)n[0]));
                    dObjOut->addToProtocol(tr("1D FFT (via FFTW). Scaled by 1/sqrt(n).").toLatin1().data());
                }
                else
                {
                    dObjOut->addToProtocol(tr("unscaled 1D FFT (via FFTW)").toLatin1().data());
                }
            }
            else
            {
                if (norm == "default")
                {
                    *dObjOut *= (1.0 / (double)n[0]);
                    dObjOut->addToProtocol(tr("inverse 1D FFT (via FFTW). Scaled by 1/n.").toLatin1().data());
                }
                else if (norm == "ortho")
                {
                    *dObjOut *= (1.0 / std::sqrt((double)n[0]));
                    dObjOut->addToProtocol(tr("inverse 1D FFT (via FFTW). Scaled by 1/sqrt(n).").toLatin1().data());
                }
                else
                {
                    dObjOut->addToProtocol(tr("unscaled inverse 1D FFT (via FFTW)").toLatin1().data());
                }
            }

            //set unit, scale and offset of transformed axis.
            std::string axisUnit;
            bool test;

            ito::float64 newScale = dObjOut->getAxisScale(axis);
            if (ito::isFinite<ito::float64>(newScale) && ito::isNotZero<ito::float64>(newScale))
            {
                newScale = 1.0 / (newScale * dObjOut->getSize(axis)); //factor of 2pi is missing, but is usually assumed to be part of the unit, e.g. lambda becomes k = 2pi / lambda
                axisUnit = ito::dObjHelper::invertUnit(dObjOut->getAxisUnit(axis, test));
                dObjOut->setAxisUnit(axis, axisUnit);
            }
            else
            {
                newScale = 1.0;
                dObjOut->setAxisUnit(axis, "");
            }
            dObjOut->setAxisScale(axis, newScale);
            dObjOut->setAxisOffset(axis, 0.0);

#if (USEOMP)
            fftw_cleanup_threads();
#endif
        }
    }

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------------------------
template<typename _TpIn, typename _TpOut> /*static*/ void FFTWFilters::getComplexLine(const cv::Mat **mdata, int planeStep, int n, int row, int col, int rowStep, void* linedata)
{
    _TpOut *linedata_ = (_TpOut*)linedata;

    for (int i = 0; i < n; ++i)
    {
        linedata_[i] = static_cast<_TpOut>(((_TpIn*)(mdata[i * planeStep]->data) + row * rowStep)[col]);
    }
}


/*static*/ FFTWFilters::tGetComplexLine FFTWFilters::fListGetComplexLineToComplex64[] =
{
    getComplexLine<ito::int8, ito::complex64>,
    getComplexLine<ito::uint8, ito::complex64>,
    getComplexLine<ito::int16, ito::complex64>,
    getComplexLine<ito::uint16, ito::complex64>,
    getComplexLine<ito::int32, ito::complex64>,
    getComplexLine<ito::uint32, ito::complex64>,
    getComplexLine<ito::float32, ito::complex64>,
    getComplexLine<ito::float64, ito::complex64>,
    getComplexLine<ito::complex64, ito::complex64>,
    getComplexLine<ito::complex128, ito::complex64>,
    NULL
};

/*static*/ FFTWFilters::tGetComplexLine FFTWFilters::fListGetComplexLineToComplex128[] =
{
    getComplexLine<ito::int8, ito::complex128>,
    getComplexLine<ito::uint8, ito::complex128>,
    getComplexLine<ito::int16, ito::complex128>,
    getComplexLine<ito::uint16, ito::complex128>,
    getComplexLine<ito::int32, ito::complex128>,
    getComplexLine<ito::uint32, ito::complex128>,
    getComplexLine<ito::float32, ito::complex128>,
    getComplexLine<ito::float64, ito::complex128>,
    getComplexLine<ito::complex64, ito::complex128>,
    getComplexLine<ito::complex128, ito::complex128>,
    NULL
};

//------------------------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> /*static*/ void FFTWFilters::setComplexLine(cv::Mat **mdata, int planeStep, int n, int row, int col, int rowStep, const _Tp *linedata)
{
    for (int i = 0; i < n; ++i)
    {
        ((_Tp*)(mdata[i * planeStep]->data) + row * rowStep)[col] = linedata[i];
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FFTWFilters::doFFT2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, const bool forward)
{
    ito::RetVal retval = ito::retOk;

    const ito::DataObject *dObjIn = (*paramsMand)[0].getVal<ito::DataObject*>();  //Input object
    ito::DataObject *dObjOut = (*paramsMand)[1].getVal<ito::DataObject*>();  //Output object
    int plan_select = (*paramsOpt)[0].getVal<int>();  //plan selection string for fftw
    unsigned int flags = (plan_select == 0) ? FFTW_ESTIMATE : FFTW_MEASURE;
    QByteArray norm = paramsOpt->at(1).getVal<char*>();
    int planForwardBackWard = forward ? FFTW_FORWARD : FFTW_BACKWARD;
    int sizeX, sizeY;
    int dims;

    retval += ito::dObjHelper::verifyDataObjectType(dObjIn, "source", 9, ito::tInt8, ito::tUInt8,
        ito::tInt16, ito::tUInt16,
        ito::tInt32,
        ito::tFloat32, ito::tFloat64,
        ito::tComplex64, ito::tComplex128);

    if (!retval.containsError())
    {
        dims = dObjIn->getDims();
        if (dims < 2)
        {
            retval += ito::RetVal(ito::retError, 0, "source must have at least two dimensions");
        }
        else
        {
            sizeX = dObjIn->getSize(dims - 1);
            sizeY = dObjIn->getSize(dims - 2);

            if (sizeX < 2 || sizeY < 2)
            {
                retval += ito::RetVal(ito::retError, 0, "minimum plane size of source is 2x2.");
            }
        }
    }

    if (!retval.containsError())
    {
        int inType = dObjIn->getType();
        ito::tDataType destType = (inType == ito::tComplex128 || inType == ito::tFloat64) ? ito::tComplex128 : ito::tComplex64;

        if (!retval.containsError())
        {
#if (USEOMP)
            fftw_init_threads();
            fftw_plan_with_nthreads(ito::AddInBase::getMaximumThreadCount());
#endif
            bool inplace = (dObjIn == dObjOut);

            if (dObjIn->get_mdata()[0]->isContinuous() == false)
            {
                inplace = false;
            }

            int numPlanes = dObjIn->getNumPlanes();

            if (inType != destType)
            {
                ito::DataObject destination;
                dObjIn->convertTo(destination, destType);
                dObjIn = dObjOut;
                *dObjOut = destination;
                inType = destType;
                inplace = true;
            }
            else if (!inplace)
            {
                if (dObjOut->getType() != destType || dObjOut->getDims() != dObjIn->getDims() || \
                    dObjOut->getContinuous() != dObjIn->getContinuous() || dObjOut->getSize() != dObjIn->getSize())
                {
                    *dObjOut = ito::DataObject(dObjIn->getDims(), dObjIn->getSize(), destType, dObjIn->getContinuous());
                    dObjIn->copyAxisTagsTo(*dObjOut);
                    dObjIn->copyTagMapTo(*dObjOut);
                }
            }

            int n[] = { 1 };

            cv::Mat *planeOut = dObjOut->get_mdata()[dObjOut->seekMat(0, numPlanes)];
            const cv::Mat *planeIn = dObjIn->get_mdata()[dObjIn->seekMat(0, numPlanes)];

            if (destType == ito::tComplex128)
            {
                fftw_plan plan;
                fftw_complex *in = (fftw_complex*)(planeIn->ptr<ito::complex128>());;
                fftw_complex *out = (fftw_complex*)(planeOut->ptr<ito::complex128>());
                if (flags == FFTW_ESTIMATE)
                {
                    plan = fftw_plan_dft_2d(sizeY, sizeX, in, out, planForwardBackWard, flags);
                    fftw_execute(plan);
                }
                else //FFTW_MEASURE (in,out will be overwritten during measuring)
                {
                    if (in != out)
                    {
                        in = (fftw_complex*)fftw_malloc(sizeX * sizeY * sizeof(fftw_complex));
                        plan = fftw_plan_dft_2d(sizeY, sizeX, in, out, planForwardBackWard, flags);
                        fftw_free(in);
                    }
                    else
                    {
                        in = (fftw_complex*)fftw_malloc(sizeX * sizeY * sizeof(fftw_complex));
                        plan = fftw_plan_dft_2d(sizeY, sizeX, in, in, planForwardBackWard, flags);
                        fftw_free(in);
                    }

                    in = (fftw_complex*)(planeIn->ptr<ito::complex128>());
                    fftw_execute_dft(plan, in, out);
                }

                for (int z = 1; z < numPlanes; z++)
                {
                    in = (fftw_complex*)(dObjIn->get_mdata()[dObjIn->seekMat(z, numPlanes)]->ptr<ito::complex128>());
                    out = (fftw_complex*)(dObjOut->get_mdata()[dObjOut->seekMat(z, numPlanes)]->ptr<ito::complex128>());
                    fftw_execute_dft(plan, in, out);
                }

                fftw_destroy_plan(plan);
            }
            else
            {
                fftwf_plan plan;
                fftwf_complex *in = (fftwf_complex*)(planeIn->ptr<ito::complex128>());
                fftwf_complex *out = (fftwf_complex*)(planeOut->ptr<ito::complex128>());
                if (flags == FFTW_ESTIMATE)
                {
                    plan = fftwf_plan_dft_2d(sizeY, sizeX, in, out, planForwardBackWard, flags);
                    fftwf_execute(plan);
                }
                else //FFTW_MEASURE (in,out will be overwritten during measuring)
                {
                    if (in != out)
                    {
                        in = (fftwf_complex*)fftwf_malloc(sizeX * sizeY * sizeof(fftwf_complex));
                        plan = fftwf_plan_dft_2d(sizeY, sizeX, in, out, planForwardBackWard, flags);
                        fftwf_free(in);
                    }
                    else
                    {
                        in = (fftwf_complex*)fftwf_malloc(sizeX * sizeY * sizeof(fftwf_complex));
                        plan = fftwf_plan_dft_2d(sizeY, sizeX, in, in, planForwardBackWard, flags);
                        fftwf_free(in);
                    }
                    in = (fftwf_complex*)(planeIn->ptr<ito::complex64>());
                    fftwf_execute_dft(plan, in, out);
                }

                for (int z = 1; z < numPlanes; z++)
                {
                    in = (fftwf_complex*)(dObjIn->get_mdata()[dObjIn->seekMat(z, numPlanes)]->ptr<ito::complex64>());
                    out = (fftwf_complex*)(dObjOut->get_mdata()[dObjOut->seekMat(z, numPlanes)]->ptr<ito::complex64>());
                    fftwf_execute_dft(plan, in, out);
                }

                fftwf_destroy_plan(plan);
            }

            if (forward)
            {
                if (norm == "ortho")
                {
                    *dObjOut *= (1.0 / std::sqrt((double)n[0]));
                    dObjOut->addToProtocol(tr("2D FFT (via FFTW). Scaled by 1/sqrt(n).").toLatin1().data());
                }
                else
                {
                    dObjOut->addToProtocol(tr("unscaled 2D FFT (via FFTW)").toLatin1().data());
                }
            }
            else
            {
                if (norm == "default")
                {
                    *dObjOut *= (1.0 / (double)n[0]);
                    dObjOut->addToProtocol(tr("inverse 2D FFT (via FFTW). Scaled by 1/n.").toLatin1().data());
                }
                else if (norm == "ortho")
                {
                    *dObjOut *= (1.0 / std::sqrt((double)n[0]));
                    dObjOut->addToProtocol(tr("inverse 2D FFT (via FFTW). Scaled by 1/sqrt(n).").toLatin1().data());
                }
                else
                {
                    dObjOut->addToProtocol(tr("unscaled inverse 2D FFT (via FFTW)").toLatin1().data());
                }
            }

            //set unit, scale and offset of last two axes.
            std::string axisUnit;
            bool test;

            for (int axis = dims - 2; axis < dims; ++axis)
            {
                ito::float64 newScale = dObjOut->getAxisScale(axis);
                if (ito::isFinite<ito::float64>(newScale) && ito::isNotZero<ito::float64>(newScale))
                {
                    newScale = 1.0 / (newScale * dObjOut->getSize(axis)); //factor of 2pi is missing, but is usually assumed to be part of the unit, e.g. lambda becomes k = 2pi / lambda
                    axisUnit = ito::dObjHelper::invertUnit(dObjOut->getAxisUnit(axis, test));
                    dObjOut->setAxisUnit(axis, axisUnit);
                }
                else
                {
                    newScale = 1.0;
                    dObjOut->setAxisUnit(axis, "");
                }
                dObjOut->setAxisScale(axis, newScale);
                dObjOut->setAxisOffset(axis, 0.0);
            }
#if (USEOMP)
            fftw_cleanup_threads();
#endif
        }
    }

    return retval;
}

#if 0   // these filteres are not included in the public release
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs forward dft by means of fftw row by row (# rows >=1) real to complex or complex to real
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in,
                                                        [1]complex Data object out
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
ito::RetVal FFTWFilters::realFFTW(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObj_in = reinterpret_cast<ito::DataObject*>((*paramsMand)[0].getVal<void*>());    //Input object
    ito::DataObject *dObj_out = reinterpret_cast<ito::DataObject*>((*paramsMand)[1].getVal<void*>());    //Output object
    char *plan_select_char = (*paramsOpt)[0].getVal<char*>();                                            //plan selection string for fftw
    std::string plan_select = static_cast<std::string>(plan_select_char);
    long int dimensions=0;
    unsigned int plan_sel =0;
    enum Tp_sel {real_type, complex_type};
    Tp_sel Tp_select_in;
    Tp_sel Tp_select_out;

    if (!dObj_in)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObj_out)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    int size;

    if (ito::dObjHelper::isCplxType(dObj_in->getType(), &size))
    {
        Tp_select_in = complex_type;
    }
    else
    {
        Tp_select_in = real_type;
    }

    if (ito::dObjHelper::isCplxType(dObj_out->getType(), &size))
    {
        Tp_select_out = complex_type;
    }
    else
    {
        Tp_select_out = real_type;
    }

    if (Tp_select_in == Tp_select_out)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: one object must be real, the other complex").toLatin1().data());
    }

    if ((plan_select == "measure") || (plan_select == "Measure"))
    {
        plan_sel = (0U);                            // from fftw.h: #define FFTW_MEASURE (0U), #define FFTW_ESTIMATE (1U << 6)
    }
    else
    {
        plan_sel = (1U << 6);                        // estimate as standard, quicker planning, probably slower fft-computing
    }

    free(plan_select_char);
    plan_select_char = NULL;


    dimensions = (*dObj_in).getDims();
    ito::int32 n0= static_cast<ito::int32>(dObj_in->getSize(0));
    ito::int32 n1= static_cast<ito::int32>(dObj_in->getSize(1));


    //real to complex
    //########################
    if (Tp_select_in == real_type)
    {
        if ((int)dObj_out->getSize(1) < (floor(n1/2.)+1) || ((ito::int32)dObj_out->getSize(0) != n0))
        {
            return ito::RetVal(ito::retError, 0, tr("Error: size of output object for r2c does not fit").toLatin1().data());
        }

        if (dimensions == 2)                    // cvMat always have at least 2 dimensions
        {
            ito::float64 *in= (ito::float64*)fftw_malloc(sizeof(ito::float64) * n0 * n1);

            // 1D real to complex
            //#########################
            if (n0 == 1)
            {
                fftw_complex *out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n0 * (floor(n1/2.)+1));

                // calculate plan before initialising in! some keywords destroy in while planning
                fftw_plan plan = fftw_plan_dft_r2c_1d(n1, in, out, plan_sel);

                switch (dObj_in->getType())        //cast all real types to float64 for use of libfftw3-3.dll
                {
                    case ito::tInt8:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::int8>(0,i));
                        }
                       break;
                    case ito::tUInt8:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::uint8>(0,i));
                        }
                       break;
                    case ito::tInt16:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::int16>(0,i));
                        }
                       break;
                    case ito::tUInt16:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::uint16>(0,i));
                        }
                       break;
                    case ito::tInt32:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::int32>(0,i));
                        }
                       break;
                    case ito::tFloat32:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::float32>(0,i));
                        }
                       break;
                    case ito::tFloat64:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    (*dObj_in).at<ito::float64>(0,i);
                        }
                       break;
                }

                fftw_execute(plan);

                switch (dObj_out->getType())
                {
                case ito::tComplex64:
                    for (int i=0; i<(floor(n1/2.)+1); i++)
                    {
                        ito::complex64 c = std::complex<ito::float32>(out[i][0], out[i][1]);
                        ((*dObj_out).at<ito::complex64>(0,i)) = c;
                    }
                break;
                case ito::tComplex128:
                    for (int i=0; i<(floor(n1/2.)+1); i++)
                    {
                        ito::complex128 c = std::complex<ito::float64>(out[i][0], out[i][1]);
                        ((*dObj_out).at<ito::complex128>(0,i)) = c;
                    }
                break;
                }


                fftw_destroy_plan(plan);
                fftw_free(out);
            }        //end of 1D


            // 2D real to complex row by row
            //##############################
            else
            {
                fftw_complex *out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n0 * n1);
                // calculate plan before initialising _in_! some keywords destroy _in_ while planning
                int _n1[]={n1};
                fftw_plan plan = fftw_plan_many_dft_r2c(1, _n1, n0, in, NULL, 1, n1, out, NULL, 1, n1, plan_sel);

                switch (dObj_in->getType())        //cast all real types to float64 for use of libfftw3-3.dll
                {
                    case ito::tInt8:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int8>(j,i));
                            }
                        }
                       break;
                    case ito::tUInt8:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::uint8>(j,i));
                            }
                        }
                       break;
                    case ito::tInt16:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int16>(j,i));
                            }
                        }
                       break;
                    case ito::tUInt16:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::uint16>(j,i));
                            }
                        }
                       break;
                    case ito::tInt32:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int32>(j,i));
                            }
                        }
                       break;
                    case ito::tFloat32:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::float32>(j,i));
                            }
                        }
                       break;
                    case ito::tFloat64:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = (*dObj_in).at<ito::float64>(j,i);
                            }
                        }
                       break;
                }


                fftw_execute(plan);

                switch (dObj_out->getType())
                {
                    case ito::tComplex64:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                ito::complex64 c = std::complex<ito::float32>(static_cast<ito::float32>(out[i*n1+j][0]), static_cast<ito::float32>(out[i*n1+j][1]));
                                ((*dObj_out).at<ito::complex64>(i,j)) = c;
                            }
                        }
                    break;
                    case ito::tComplex128:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                ito::complex128 c = std::complex<ito::float64>(out[i*n1+j][0], out[i*n1+j][1]);
                                ((*dObj_out).at<ito::complex128>(i,j)) = c;
                            }
                        }
                    break;
                }

                fftw_destroy_plan(plan);
                fftw_free(out);
            }        // end of 2D row by row
        }            // end of all 1D and 2D

        else
        {
            retval = ito::RetVal(ito::retError, 0, tr("Error: more than 2 dimensions are not supported").toLatin1().data());
        }

        if (!retval.containsError())
        {
            // Add Protokoll
//            char prot[81] = {0};
//            _snprintf(prot, 80, "FFTW filter forward real to complex (unscaled!)");
//            dObj_out->addToProtocol(std::string(prot));
            QString msg = tr("FFTW filter forward real to complex (unscaled!)");
            dObj_out->addToProtocol(std::string(msg.toLatin1().data()));
        }
    }                // end of real to complex


    //complex to real
    //#################
    else            // Tp_select != real_type
    {
        if (dObj_out->getType() != ito::tFloat64)
        {
            return ito::RetVal(ito::retError, 0, tr("Error: need float64 output for complex input and c2r mode").toLatin1().data());
        }

        fftw_complex *in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n0 * n1);

        if (dimensions == 2)
        {
            // 1D complex to real
            //######################
            if (n0 == 1)
            {
                ito::float64 *out= (ito::float64*)fftw_malloc(sizeof(ito::float64) * n0 * ((n1-1)*2));

                // calculate plan before initialising in! some keywords destroy in while planning
                fftw_plan plan = fftw_plan_dft_c2r_1d(n1, in, out, plan_sel);

                switch (dObj_in->getType())
                {
                case ito::tComplex64:
                    for (int i=0; i<n1; i++)
                    {
                        ito::complex64 x = (*dObj_in).at<ito::complex64>(0,i);
                        in[i][0] = static_cast<ito::float64>(real(x));
                        in[i][1] = static_cast<ito::float64>(imag(x));
                    }
                break;
                case ito::tComplex128:
                    for (int i=0; i<n1; i++)
                    {
                        ito::complex128 x = (*dObj_in).at<ito::complex128>(0,i);
                        in[i][0] = real(x);
                        in[i][1] = imag(x);
                    }
                break;
                }

                fftw_execute(plan);
                fftw_destroy_plan(plan);
                for (int i=0; i<((n1-1)*2); i++)
                    {
                        ((*dObj_out).at<ito::float64>(0,i)) = out[i];
                    }

                fftw_free(out);
            }        // end of 1D

            // 2D complex to real row by row
            //######################################
            else
            {
                ito::float64 *out= (ito::float64*)fftw_malloc(sizeof(ito::float64) *n0 * ((n1-1)*2));
                // calculate plan before initialising _in_! some keywords destroy _in_ while planning
                int _n1[]={n1};
                fftw_plan plan = fftw_plan_many_dft_c2r(1, _n1, n0, in, NULL, 1, n1, out, NULL, 1, n1, plan_sel);

                switch (dObj_in->getType())
                {
                case ito::tComplex64:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            ito::complex64 x = (*dObj_in).at<ito::complex64>(i,j);
                            in[i*n1+j][0] = real(x);
                            in[i*n1+j][1] = imag(x);
                        }
                    }
                break;
                case ito::tComplex128:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            ito::complex128 x = (*dObj_in).at<ito::complex128>(i,j);
                            in[i*n1+j][0] = real(x);
                            in[i*n1+j][1] = imag(x);
                        }
                    }
                break;
                }

                fftw_execute(plan);

                for (int i=0; i<n0; i++)
                {
                    for (int j=0; j<n1; j++)
                    {
                        ((*dObj_out).at<ito::float64>(i,j)) = out[i*n1+j];
                    }
                }

                fftw_destroy_plan(plan);
                fftw_free(out);
            }  // end of 2D row by row

            fftw_free(in);
        }                            // end of all 1D and 2D

        else
        {
            retval = ito::RetVal(ito::retError, 0, tr("Error: more than 2 dimensions are not supported").toLatin1().data());
        }

        if (!retval.containsError())
        {
            // Add Protokoll
//            char prot[81] = {0};
//            _snprintf(prot, 80, "FFTW filter forward complex to real (unscaled!)");
//            dObj_out->addToProtocol(std::string(prot));
            QString msg = tr("FFTW filter forward complex to real (unscaled!)");
            dObj_out->addToProtocol(std::string(msg.toLatin1().data()));
        }
    }

    dObj_in->copyTagMapTo(*dObj_out);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs 2D dft real to complex or complex to real by means of fftw
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in,
                                                        [1]complex Data object out
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
ito::RetVal FFTWFilters::realFFTW2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObj_in = reinterpret_cast<ito::DataObject*>((*paramsMand)[0].getVal<void*>());    //Input object
    ito::DataObject *dObj_out = reinterpret_cast<ito::DataObject*>((*paramsMand)[1].getVal<void*>());    //Output object
    char *plan_select_char = (*paramsOpt)[0].getVal<char*>();                                            //plan selection string for fftw
    std::string plan_select = static_cast<std::string>(plan_select_char);
    long int dimensions=0;
    unsigned int plan_sel =0;
    enum Tp_sel {real_type, complex_type};
    Tp_sel Tp_select_in;
    Tp_sel Tp_select_out;

    if (!dObj_in)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObj_out)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    int size;
    if (ito::dObjHelper::isCplxType(dObj_in->getType(), &size))
    {
        Tp_select_in = complex_type;
    }
    else
    {
        Tp_select_in = real_type;
    }

    if (ito::dObjHelper::isCplxType(dObj_out->getType(), &size))
    {
        Tp_select_out = complex_type;
    }
    else
    {
        Tp_select_out = real_type;
    }

    if (Tp_select_in == Tp_select_out)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: one object must be real, the other complex").toLatin1().data());
    }

    if ((plan_select == "measure") || (plan_select == "Measure"))
    {
        plan_sel = (0U);                            // from fftw.h: #define FFTW_MEASURE (0U), #define FFTW_ESTIMATE (1U << 6)
    }
    else
    {
        plan_sel = (1U << 6);                        // estimate as standard, quicker planning, probably slower fft-computing
    }

    free(plan_select_char);
    plan_select_char = NULL;

    dimensions = (*dObj_in).getDims();
    ito::int32 n0= static_cast<ito::int32>(dObj_in->getSize(0));
    ito::int32 n1= static_cast<ito::int32>(dObj_in->getSize(1));

    //real to complex
    //########################
    if (Tp_select_in == real_type)
    {
        if (dObj_out->getSize(1) < (floor(n1/2.)+1) || ((ito::int32)dObj_out->getSize(0) != n0))
        {
            return ito::RetVal(ito::retError, 0, tr("Error: size of output object for r2c does not fit").toLatin1().data());
        }

        if ((dimensions == 2) && (n0!=1))
        {
            ito::float64 *in= (ito::float64*)fftw_malloc(sizeof(ito::float64) * n0 * n1);
            fftw_complex *out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n0 * n1);

            // calculate plan before initialising _in_! some keywords destroy _in_ while planning
            fftw_plan plan = fftw_plan_dft_r2c_2d(n0, n1, in, out, plan_sel);

            switch (dObj_in->getType())        //cast all real types to float64 for use of libfftw3-3.dll
            {
                case ito::tInt8:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int8>(j,i));
                        }
                    }
                   break;
                case ito::tUInt8:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::uint8>(j,i));
                        }
                    }
                   break;
                case ito::tInt16:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int16>(j,i));
                        }
                    }
                   break;
                case ito::tUInt16:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::uint16>(j,i));
                        }
                    }
                   break;
                case ito::tInt32:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int32>(j,i));
                        }
                    }
                   break;
                case ito::tFloat32:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::float32>(j,i));
                        }
                    }
                   break;
                case ito::tFloat64:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = (*dObj_in).at<ito::float64>(j,i);
                        }
                    }
                   break;
            }

            fftw_execute(plan);

            switch (dObj_out->getType())
            {
                case ito::tComplex64:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            ito::complex64 c = std::complex<ito::float32>(static_cast<ito::float32>(out[i*n1+j][0]), static_cast<ito::float32>(out[i*n1+j][1]));
                            ((*dObj_out).at<ito::complex64>(i,j)) = c;
                        }
                    }
                break;
                case ito::tComplex128:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            ito::complex128 c = std::complex<ito::float64>(out[i*n1+j][0], out[i*n1+j][1]);
                            ((*dObj_out).at<ito::complex128>(i,j)) = c;
                        }
                    }
                break;
            }

            fftw_destroy_plan(plan);
            fftw_free(out);
            fftw_free(in);
        }        //end of 2D complete
        else
        {
            retval = ito::RetVal(ito::retError, 0, tr("Error: this filter is designed for 2D data. As the name says.....").toLatin1().data());
        }

        if (!retval.containsError())
        {
            // Add Protokoll
//            char prot[81] = {0};
//            _snprintf(prot, 80, "FFTW filter 2D real to complex (unscaled!)");
//            dObj_out->addToProtocol(std::string(prot));
            QString msg = tr("FFTW filter 2D real to complex (unscaled!)");
            dObj_out->addToProtocol(std::string(msg.toLatin1().data()));
        }
    }            //end of real to complex

    //complex to real
    //#################
    else            // Tp_select != real_type
    {
        if (dObj_out->getType() != ito::tFloat64)
        {
            return ito::RetVal(ito::retError, 0, tr("Error: need float64 output for complex input and c2r mode").toLatin1().data());
        }

        fftw_complex *in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n0 * n1);

        if ((dimensions == 2) && (n0!=1))
        {
            ito::float64 *out= (ito::float64*)fftw_malloc(sizeof(ito::float64) * n0 * n1);

            // calculate plan before initialising in! some keywords destroy in while planning
            fftw_plan plan = fftw_plan_dft_c2r_2d(n0, n1, in, out, plan_sel);

            switch (dObj_in->getType())
            {
            case ito::tComplex64:
                for (int i=0; i<n0; i++)
                {
                    for (int j=0; j<n1; j++)
                    {
                        ito::complex64 x = (*dObj_in).at<ito::complex64>(i,j);
                        in[i*n1+j][0] = real(x);
                        in[i*n1+j][1] = imag(x);
                    }
                }
            break;
            case ito::tComplex128:
                for (int i=0; i<n0; i++)
                {
                    for (int j=0; j<n1; j++)
                    {
                        ito::complex128 x = (*dObj_in).at<ito::complex128>(i,j);
                        in[i*n1+j][0] = real(x);
                        in[i*n1+j][1] = imag(x);
                    }
                }
            break;
            }

            fftw_execute(plan);
            fftw_destroy_plan(plan);
            for (int i=0; i<n0; i++)
            {
                for (int j=0; j<n1; j++)
                {
                    ((*dObj_out).at<ito::float64>(i,j)) = out[i*n1+j];
                }
            }
            fftw_free(out);
            fftw_free(in);
        }                    // end of 2D standard

        else
        {
            retval = ito::RetVal(ito::retError, 0, tr("Error: this filter is designed for 2D data. As the name says.....").toLatin1().data());
        }

        if (!retval.containsError())
        {
            // Add Protokoll
//            char prot[81] = {0};
//            _snprintf(prot, 80, "FFTW filter 2D complex to real (unscaled!)");
//            dObj_out->addToProtocol(std::string(prot));
            QString msg = tr("FFTW filter 2D complex to real (unscaled!)");
            dObj_out->addToProtocol(std::string(msg.toLatin1().data()));
        }
    }

    dObj_in->copyTagMapTo(*dObj_out);

    return retval;
}

//##############################################################################################################################
//NEW IMPLEMENTATION -- NOT TESTED COMPLETELY
//last modified: 2012/02/20 boettcher
//Known bugs:
//    - flanks to flat? --> use more steps?
//##############################################################################################################################

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail

   \param[in|out]   paramsMand  Mandatory parameters:   Data object in (1 row of height data),
                                                        Data object out (1 row of roughness data)
   \param[in|out]   paramsOpt   Optional parameters:    R_z im mm(lookup table will do the job;
                                                            if 0, at least 1 pair of lambdas must be defined or getR_z flag == 1)
                                                        n: steps for FFT (default 0 ==> #pixels in data object),
                                                        lambda_s,
                                                        lambda_c,
                                                        lambda_f,
                                                        Data object for waviness (default O, if !=0, roughness AND waviness are calculated)

   \param[out]      outVals   Outputvalues

   \author ITO, Boettcher
   \sa
   \date 02.2012
*/
ito::RetVal FFTWFilters::calcGaussianFilterRough1D (QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObj_in = reinterpret_cast<ito::DataObject*>((*paramsMand)[0].getVal<void*>());  //Input object

    if (dObj_in == NULL)    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Source object not defined").toLatin1().data());
    }

    if (dObj_in->getDims() < 1) // Report error of input object is empty
    {
        return ito::RetVal(ito::retError, 0, tr("DataObject is empty").toLatin1().data());
    }

    ito::int32 sizeY = dObj_in->getSize(0);
    ito::int32 sizeX = dObj_in->getSize(1);

    if ((dObj_in->getDims() > 2) || (sizeY > 1))    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Input DataObject must be 1xN").toLatin1().data());
    }

    ito::DataObject *dObj_out = reinterpret_cast<ito::DataObject*>((*paramsMand)[1].getVal<void*>());  //Output object
    if (dObj_out == NULL)    // same here for output object
    {
        return ito::RetVal(ito::retError, 0, tr("Output object not defined").toLatin1().data());
    }

    bool useDstObj = true;
    bool useWaveObj = true;
    bool calcWaveObj = false;

    ito::DataObject *dObj_wavOut = reinterpret_cast<ito::DataObject*>((*paramsOpt)[4].getVal<void*>());  //Output object
    if (dObj_wavOut == NULL)    // same here for output object
    {
        calcWaveObj = false;
    }
    else if (dObj_wavOut == dObj_out)
    {
        return ito::RetVal(ito::retError, 0, tr("Waviness-result must not be equal to roughness output").toLatin1().data());
    }
    else if (dObj_wavOut == dObj_in)
    {
        return ito::RetVal(ito::retError, 0, tr("Waviness-result must not be equal to surface input").toLatin1().data());
    }
    else
    {
        calcWaveObj = true;
    }

    // Check if input and output type are allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObj_in, "dObjIn", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
    {
        return retval;
    }
    ito::DataObject rough_out;
    ito::DataObject wavin_out;

    if (dObj_out == dObj_in)
    {
        if (dObj_out->getType() == ito::tFloat64)
        {
            // inplace is possible
            rough_out = *dObj_out;
        }
        else
        {
            // input must be changed to ito::float64 during filtering
            useDstObj = false;
            rough_out = ito::DataObject(sizeY, sizeX, ito::tFloat64);
        }
    }
    else
    {
        ito::RetVal temp = ito::dObjHelper::verify2DDataObject(dObj_out, "dObjectOut", sizeY, sizeY, sizeX, sizeX, 1, ito::tFloat64);
        if (!temp.containsWarningOrError())
        {
            rough_out = *dObj_out;
        }
        else
        {
            // output must be changed to ito::float64 or right size
            useDstObj = false;
            rough_out = ito::DataObject(sizeY, sizeX, ito::tFloat64);
        }
    }

    if (calcWaveObj)
    {
        ito::RetVal temp = ito::dObjHelper::verify2DDataObject(dObj_wavOut, "dObjectWave", sizeY, sizeY, sizeX, sizeX, 1, ito::tFloat64);
        if (!temp.containsWarningOrError())
        {
            wavin_out = *dObj_wavOut;
        }
        else
        {
            // output must be changed to ito::float64 or right size
            useWaveObj = false;
            wavin_out = ito::DataObject(sizeY, sizeX, ito::tFloat64);
        }
    }

    // Check if filterborders or R_z are set
    ito::float64 R_z = static_cast<ito::float64>((*paramsOpt)[0].getVal<ito::float64>());
    ito::float64 lambda_s = static_cast<ito::float64>((*paramsOpt)[1].getVal<ito::float64>());
    ito::float64 lambda_c = static_cast<ito::float64>((*paramsOpt)[2].getVal<ito::float64>());
    ito::float64 lambda_f = static_cast<ito::float64>((*paramsOpt)[3].getVal<ito::float64>());

    if (!ito::isNotZero(R_z) && !ito::isNotZero(lambda_c))
    {
        return ito::RetVal(ito::retError, 0, tr("Define R_z or 1 pair of lambdas").toLatin1().data());
    }
    else if (ito::isNotZero(R_z)) // search lookuptable for adequate lambdas (in mm!!)
    {
        if (R_z <= 0.1e-3)
        {
                lambda_c = 0.08;
                lambda_s = 2.5e-3;
        }
        else if (R_z <= 0.5e-3)
        {
                lambda_c = 0.25;
                lambda_s = 2.5e-3;
        }
        else if (R_z <= 10.e-3)
        {
                lambda_c = 0.8;
                lambda_s = 2.5e-3;
        }
        else if  (R_z <= 50.e-3)
        {
                lambda_c = 2.5;
                lambda_s = 8e-3;
        }
        else if (R_z <= 200.e-3)
        {
                lambda_c = 8.0;
                lambda_s = 25e-3;
        }
        else if (R_z >= 200.e-3)
        {
            return ito::RetVal(ito::retError, 0, tr("R_z over 200 \u00B5m").toLatin1().data());  // mu m
        }
    }

    cv::Mat *scrMat = ((cv::Mat *)dObj_in->get_mdata()[dObj_in->seekMat(0)]);
    ito::float64 *dstPtr = ((cv::Mat *)rough_out.get_mdata()[rough_out.seekMat(0)])->ptr<ito::float64>();
    ito::float64 *wavPtr = NULL;

    if (calcWaveObj)
    {
        wavPtr = ((cv::Mat *)wavin_out.get_mdata()[wavin_out.seekMat(0)])->ptr<ito::float64>();
    }

    // Allocade buffer for input and output
    ito::int32 sizeXFD = sizeX / 2 + 1;

    ito::float64 *srcInput   = (ito::float64*) fftw_malloc(sizeof(ito::float64) * sizeX);
    fftw_complex *fourDomain = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * sizeXFD);
    fftw_complex *filtRough  = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * sizeXFD);
    fftw_complex *filtWavi   = NULL;

    fftw_plan plan2FD       = fftw_plan_dft_r2c_1d(sizeX, srcInput, fourDomain, FFTW_ESTIMATE);
    fftw_plan planBackRough = fftw_plan_dft_c2r_1d(sizeX, filtRough, srcInput, FFTW_ESTIMATE);
    fftw_plan planBackWavi  = NULL;

    if (calcWaveObj)
    {
        filtWavi     = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * sizeXFD);
        planBackWavi = fftw_plan_dft_c2r_1d(sizeX, filtWavi,  srcInput, FFTW_ESTIMATE);
    }

    ito::float64 dx = dObj_in->getAxisScale(1);

    switch(dObj_in->getType())
    {
        case ito::tUInt8:
            ito::dObjHelper::GetHLineD<ito::uint8>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tInt8:
            ito::dObjHelper::GetHLineD<ito::int8>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tInt16:
            ito::dObjHelper::GetHLineD<ito::int16>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tUInt16:
            ito::dObjHelper::GetHLineD<ito::uint16>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tInt32:
            ito::dObjHelper::GetHLineD<ito::int32>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tFloat32:
            ito::dObjHelper::GetHLineD<ito::float32>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tFloat64:
            ito::dObjHelper::GetHLineD<ito::float64>(scrMat, 0, 0, sizeX, srcInput);
            break;
    }

    fftw_execute(plan2FD);

    //Roughness Filtering
    //filtRough[0][0] = fourDomain[0][0];    //real fourier component (amplitude)

    //const ito::float64 cAlpha =  0.8493218002880191;// 1 / sqrt(2 * ln(2));
    //const ito::float64 cAlpha =  1.2011224087864498;// 1 / sqrt(2 * ln(sqrt(2)));
    const ito::float64 cAlpha =  1.48809737131601248;// 1 / sqrt(2 * ln(sqrt(pi/2)));

    ito::float64 sigmaFS = cAlpha / lambda_s;
    ito::float64 sigmaFC = cAlpha / lambda_c;
    ito::float64 sigmaFF = cAlpha / lambda_f;
    ito::float64 freqInc = 1 / (dx * ((ito::float64)sizeXFD - 1.0));

    ito::DataObject filterFunc = ito::DataObject(2, sizeXFD, ito::tFloat64);
    ito::float64* filterFuncRoughPtr = (ito::float64*)filterFunc.rowPtr(0,0);
    ito::float64* filterFuncWavePtr = (ito::float64*)filterFunc.rowPtr(0,1);

    filterFuncRoughPtr[0] = 0.0;
    filterFuncWavePtr[0] = 0.0;

    ito::DataObject *filterOut = reinterpret_cast<ito::DataObject*>((*paramsOpt)[5].getVal<void*>());  //Output object
    if (filterOut != NULL && filterOut != dObj_wavOut && filterOut != dObj_out && filterOut != dObj_in)    // same here for output object
    {
        *filterOut = filterFunc;
        filterFunc.setAxisScale(1, freqInc);
    }

    if (ito::isNotZero(lambda_s))
    {
        ito::float64 cuExp1 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFS, 2));
        ito::float64 cuExp2 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFC, 2));
        for (ito::int32 i = 0; i < sizeXFD; i++)
        {
            ito::float64 sqrI = pow((ito::float64)i, 2);
            filterFuncRoughPtr[i] = exp(cuExp1 * sqrI) * (1-exp(cuExp2 * sqrI));
        }
    }
    else
    {
        ito::float64 cuExp2 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFC, 2));
        for (ito::int32 i = 0; i < sizeXFD; i++)
        {
            filterFuncRoughPtr[i] = 1-exp(cuExp2 * pow((ito::float64)i, 2));
        }
    }

    for (ito::int32 i = 0; i < sizeXFD; i++)
    {
        filtRough[i][0] = fourDomain[i][0] * filterFuncRoughPtr[i];
        filtRough[i][1] = fourDomain[i][1] * filterFuncRoughPtr[i];
    }

    //Waviness Filterung
    if (calcWaveObj)
    {
        //filtWavi[0][0] = fourDomain[0][0];    //real fourier component (amplitude)

        if (ito::isNotZero(lambda_f))
        {
            ito::float64 cuExp1 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFC, 2));
            ito::float64 cuExp2 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFF, 2));
            for (ito::int32 i = 0; i < sizeXFD; i++)
            {
                ito::float64 sqrI = pow((ito::float64)i, 2);
                filterFuncWavePtr[i] = exp(cuExp1 * sqrI) * (1-exp(cuExp2 * sqrI));
            }
        }
        else
        {
            ito::float64 cuExp2 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFC, 2));
            for (ito::int32 i = 0; i < sizeXFD; i++)
            {
                filterFuncWavePtr[i] = exp(cuExp2 * pow((ito::float64)i, 2));
            }
        }

        for (ito::int32 i = 0; i < sizeXFD; i++)
        {
            filtWavi[i][0] = fourDomain[i][0] * filterFuncWavePtr[i];
            filtWavi[i][1] = fourDomain[i][1] * filterFuncWavePtr[i];
        }

        fftw_execute(planBackWavi);  //destroys filtered?!

        for (ito::int32 i=0; i< sizeX ;i++)
        {
            wavPtr[i] = srcInput[i] / sizeX;
        }
    }
    fftw_execute(planBackRough);  //destroys filtered?!

    for (ito::int32 i=0; i< sizeX ;i++)
    {
        dstPtr[i] = srcInput[i] / sizeX;
    }

    ito::DataObject *fourierOut = reinterpret_cast<ito::DataObject*>((*paramsOpt)[6].getVal<void*>());  //Output object
    if (filterOut != NULL && filterOut != dObj_wavOut && filterOut != dObj_out && filterOut != dObj_in)    // same here for output object
    {
        *fourierOut = ito::DataObject(1, sizeXFD, ito::tComplex128);
        ito::complex128* cRowPtr = (ito::complex128*)fourierOut->rowPtr(0,0);

        memcpy(cRowPtr, fourDomain, sizeof(fourDomain)* sizeXFD);
    }

    //destroy fft stuff
    fftw_destroy_plan(plan2FD);
    fftw_destroy_plan(planBackRough);

    if (planBackWavi)
    {
        fftw_destroy_plan(planBackWavi);
    }

    if (srcInput)
    {
        fftw_free(srcInput);
    }

    if (filtRough)
    {
        fftw_free(filtRough);
    }

    if (filtWavi)
    {
        fftw_free(filtWavi);
    }

    if (fourDomain)
    {
        fftw_free(fourDomain);
    }

    if (!retval.containsError())
    {
        dObj_in->copyTagMapTo(rough_out);
        ito::dObjHelper::dObjCopyLastNAxisTags(*dObj_in, rough_out, 2, true, true);

        QString msg = tr("Roughness after gaussian filter with lambdaS %1 mm and lambdaC %2").arg(QString::number(lambda_s), QString::number(lambda_c));
        rough_out.addToProtocol(std::string(msg.toLatin1().data()));
        if (useDstObj == false)
        {
            *dObj_out = rough_out;
        }
    }

    if (!retval.containsError() && calcWaveObj)
    {
        dObj_in->copyTagMapTo(wavin_out);
        ito::dObjHelper::dObjCopyLastNAxisTags(*dObj_in, wavin_out, 2, true, true);

        QString msg = tr("Waviness after gaussian filter with lambdaC %1 mm and lambdaF %2").arg(QString::number(lambda_c), QString::number(lambda_f));
        wavin_out.addToProtocol(std::string(msg.toLatin1().data()));
        if (useWaveObj == false)
        {
            *dObj_wavOut = wavin_out;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail Get the mandatory parameters "DataObject",
   \param[out]   paramsMand  Mandatory parameters for the filter function
   \param[out]   paramsOpt   Optional parameters for the filter function :
   \author ITO, Boettcher
   \sa  mcppfilters::calcGaussianFilterRough1D
   \date
*/
ito::RetVal FFTWFilters::getGaussianRough1DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::Param param;

    if (!paramsMand)
    {
        retval = ito::RetVal(ito::retError, 0, tr("uninitialized vector for mandatory parameters!").toLatin1().data());
        goto end;
    }
    if (!paramsOpt)
    {
        retval = ito::RetVal(ito::retError, 0, tr("uninitialized vector for optional parameters!").toLatin1().data());
        goto end;
    }

    param = ito::Param("DataObject_in", ito::ParamBase::DObjPtr, NULL, tr("see Algorithm-Doc").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("DataObject_out", ito::ParamBase::DObjPtr, NULL, tr("see Algorithm-Doc").toLatin1().data());
    paramsMand->append(param);

    param = ito::Param("R_z", ito::ParamBase::Double, 0., 0.2, 0., "R_z");
    paramsOpt->append(param);
    param = ito::Param("lambda_s", ito::ParamBase::Double, 0.0, std::numeric_limits<double>::max(), 0.0, tr("Short wavelength to filter").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("lambda_c", ito::ParamBase::Double, 0.0, std::numeric_limits<double>::max(), 0.0, tr("Wavelength to separate between roughness and waviness").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("lambda_f", ito::ParamBase::Double, 0.0, std::numeric_limits<double>::max(), 0.0, tr("Wavelength to separate between waviness and form").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("DataObject_waviness_out", ito::ParamBase::DObjPtr, NULL, tr("see Algorithm-Doc").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("DataObject_filterFunc_out", ito::ParamBase::DObjPtr, NULL, tr("see Algorithm-Doc").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("DataObject_fourier_out", ito::ParamBase::DObjPtr, NULL, tr("see Algorithm-Doc").toLatin1().data());
    paramsOpt->append(param);
end:
    return retval;
}
#endif

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FFTWFilters::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    if (sizeof(fftw_complex) == sizeof(ito::complex128))
    {
        filter = new FilterDef(FFTWFilters::fftw1d, FFTWFilters::xfftw1dParams, fftw1dDOC);
        m_filterList.insert("fftw", filter);
        filter = new FilterDef(FFTWFilters::ifftw1d, FFTWFilters::xfftw1dParams, ifftw1dDOC);
        m_filterList.insert("ifftw", filter);
        filter = new FilterDef(FFTWFilters::fftw2d, FFTWFilters::xfftw2dParams, fftw2dDOC);
        m_filterList.insert("fftw2D", filter);
        filter = new FilterDef(FFTWFilters::ifftw2d, FFTWFilters::xfftw2dParams, ifftw2dDOC);
        m_filterList.insert("ifftw2D", filter);
        filter = new FilterDef(FFTWFilters::fftshift, FFTWFilters::xfftshiftParams, fftshiftDOC);
        m_filterList.insert("fftshift", filter);
        filter = new FilterDef(FFTWFilters::ifftshift, FFTWFilters::xfftshiftParams, ifftshiftDOC);
        m_filterList.insert("ifftshift", filter);
    }
    else
    {
        retval += ito::RetVal(ito::retWarning, 0, tr("Warning: compatibility error between fftw_complex and ito::complex128").toLatin1().data());
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FFTWFilters::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
