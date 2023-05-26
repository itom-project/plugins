/* ********************************************************************
    Plugin "DIC" for itom software
    URL: http://lccv.ufal.br/
    Copyright (C) 2016, Universidade Federal de Alagoas (UFAL), Brazil

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

#include "dic.h"
#include "dicPrivate.h"
#include "dicInterpolation.h"
#include "dicInterpolation.hu"

#include "DataObject/dataObjectFuncs.h"
#include "opencv2/imgproc/imgproc.hpp"

extern int NTHREADS;
extern bool hasCuda;

extern struct MatT AMatBiCu, AMatBiQu, AMatBiHe;

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DIC::DICInterpolationDoc = QObject::tr("subpixel intensity interpolation");

/*static*/ ito::RetVal DIC::DICInterpolationParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::Param param;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("dObjIn", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d input data field"));
    paramsMand->append(ito::Param("dObjOut", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "2d output vector with intensities and in case derivatives at given positions"));
    paramsMand->append(ito::Param("PosXY", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "interpolation positions"));

    paramsOpt->append(ito::Param("algorithm", ito::ParamBase::Int | ito::ParamBase::In, 0, 5, 0, "0: bilinear, 1: bicubic, 2: ..."));
    paramsOpt->append(ito::Param("degree", ito::ParamBase::Int | ito::ParamBase::In, 0, 200, 1, "polynomial degree in case algorithm is of type ..."));
    paramsOpt->append(ito::Param("withDer", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, "also calculate derivates (1) or do not(0)"));
    paramsOpt->append(ito::Param("keepImage", ito::ParamBase::Int | ito::ParamBase::In, -1, 1, 1, "keep image stored for further interpolations, -1 means a new image is pushed for further processing (reset image)"));
    paramsOpt->append(ito::Param("useCuda", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, "use Cuda calculation if available"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
// flag: Bit Dez   Func
//        0   1     calculate derivatives
//        1   2     keep image
//        2   4     reset image
//        3   8
//        4   16    interpolation type LoByte: Bi-Linear, Bi-Cubic, Bi-Quintic, Bi-Heptic
//        5   32    interpolation type HighByte
//        6   64    Reserved for other interpolation types
//        7   128

ito::RetVal doInterpolate(ito::DataObject *inFieldPtr, ito::DataObject *positionsdPtr, ito::DataObject *outFieldPtr, int algo, int flag, int useCuda)
{
    ito::RetVal retval;

    int sizex = inFieldPtr->getSize(inFieldPtr->getDims() - 1);
    int sizey = inFieldPtr->getSize(inFieldPtr->getDims() - 2);

    if (positionsdPtr->getType() != ito::tFloat32)
    {
        ito::DataObject positionsPtrTmp;
        positionsdPtr->convertTo(positionsPtrTmp, ito::tFloat32);
        *positionsdPtr = positionsPtrTmp;
    }

    if (algo == 0)
    {
//        if (outFieldPtr->getSize(1) != 2 || outFieldPtr->getSize(0) != numPts || outFieldPtr->getType() != inFieldPtr->getType())
//            *outFieldPtr = ito::DataObject(numPts, 1, inFieldPtr->getType());
        if (outFieldPtr->getSize(1) != 2 || outFieldPtr->getSize(0) != positionsdPtr->getSize(0) || outFieldPtr->getType() != inFieldPtr->getType())
            outFieldPtr->zeros(positionsdPtr->getSize(0), 3, ito::tFloat32);

        ito::DataObject inpObj = *inFieldPtr;
        int dtype = inFieldPtr->getType();

#ifdef USECUDA
        if (hasCuda && useCuda)
        {
            switch (dtype)
            {
                case ito::tUInt8:
                    retval += h_interpolBiLi<unsigned char>(inpObj.rowPtr<unsigned char>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<ito::float32>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tInt8:
                    retval += h_interpolBiLi<signed char>(inpObj.rowPtr<signed char>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<ito::float32>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tUInt16:
                    retval += h_interpolBiLi<unsigned short>(inpObj.rowPtr<unsigned short>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<ito::float32>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tInt16:
                    retval += h_interpolBiLi<signed short>(inpObj.rowPtr<signed short>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<ito::float32>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tUInt32:
                    retval += h_interpolBiLi<unsigned long>(inpObj.rowPtr<unsigned long>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<ito::float32>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tInt32:
                    retval += h_interpolBiLi<signed long>(inpObj.rowPtr<signed long>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<ito::float32>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tFloat32:
                    retval += h_interpolBiLi<float>(inpObj.rowPtr<float>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<ito::float32>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tFloat64:
                    retval += h_interpolBiLi<double>(inpObj.rowPtr<double>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<ito::float32>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                default:
                    return ito::RetVal(ito::retError, 0, QObject::tr("invalid data type / not implemeneted yet").toLatin1().data());
            }
        }
        else
#endif
        {
            switch (dtype)
            {
                case ito::tUInt8:
                    retval += doInterpolateLinear<ito::uint8>(inpObj.rowPtr<ito::uint8>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<float>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tInt8:
                    retval += doInterpolateLinear<ito::int8>(inpObj.rowPtr<ito::int8>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<float>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tUInt16:
                    retval += doInterpolateLinear<ito::uint16>(inpObj.rowPtr<ito::uint16>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<float>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tInt16:
                    retval += doInterpolateLinear<ito::int16>(inpObj.rowPtr<ito::int16>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<float>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tUInt32:
                    retval += doInterpolateLinear<ito::uint32>(inpObj.rowPtr<ito::uint32>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<float>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tInt32:
                    retval += doInterpolateLinear<ito::int32>(inpObj.rowPtr<ito::int32>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<float>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tFloat32:
                    retval += doInterpolateLinear<ito::float32>(inpObj.rowPtr<ito::float32>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<float>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                case ito::tFloat64:
                    retval += doInterpolateLinear<ito::float64>(inpObj.rowPtr<ito::float64>(0, 0), sizex, sizey, inpObj.getStep(0), positionsdPtr->rowPtr<float>(0, 0),
                        positionsdPtr->getSize(0), outFieldPtr->rowPtr<ito::float32>(0, 0), outFieldPtr->getStep(0), flag);
                break;

                default:
                    return ito::RetVal(ito::retError, 0, QObject::tr("invalid data type / not implemeneted yet").toLatin1().data());
            }
        }
    }
    else if (algo >= 1)
    {
        // beware this implementation is inteded for dense point field interpolations. For 'sparse', i.e. interpolation points
        // have a large spacing between them the calculation of the fullfield A matrix does not make sense and the code
        // should be adapted accordingly
        if (outFieldPtr->getSize(1) != 2 || outFieldPtr->getSize(0) != positionsdPtr->getSize(0) || outFieldPtr->getType() != inFieldPtr->getType())
            outFieldPtr->zeros(positionsdPtr->getSize(0), 3, ito::tFloat32);

        if (!(hasCuda && useCuda)
            && ((flag & 2) != 2
            || (flag & 4) == 4
            || ((algo == 1) && (AMatBiCu.sizex != sizex || AMatBiCu.sizey != sizey))
            || ((algo == 2) && (AMatBiQu.sizex != sizex || AMatBiQu.sizey != sizey))
            || ((algo == 3) && (AMatBiHe.sizex != sizex || AMatBiHe.sizey != sizey))))
        {
            ito::DataObject imgFP;
            inFieldPtr->convertTo(imgFP, ito::tFloat32);
            retval += doCalcAMat((ito::float32*)imgFP.rowPtr(0, 0), sizex, sizey, imgFP.getStep(0), algo, flag);
        }

        if (algo == 1)
        {
#ifdef USECUDA
            if (hasCuda && useCuda)
            {
                if ((flag & 2) != 2 || (flag & 4) == 4)
                {
                    ito::DataObject imgFP;
                    inFieldPtr->convertTo(imgFP, ito::tFloat32);
                    retval += h_interpolAMat((ito::float32*)imgFP.rowPtr(0, 0), sizex, sizey, imgFP.getStep(0),
                        (float*)positionsdPtr->rowPtr(0, 0), positionsdPtr->getSize(0), (float*)outFieldPtr->rowPtr(0, 0), algo, flag);
                }
                else
                {
                    retval += h_interpolAMat(NULL, sizex, sizey, 0,
                        (float*)positionsdPtr->rowPtr(0, 0), positionsdPtr->getSize(0), (float*)outFieldPtr->rowPtr(0, 0), algo, flag);
                }
            }
            else
#endif
            {
                retval += doInterpolateCubic((float*)positionsdPtr->rowPtr(0, 0), positionsdPtr->getSize(0), (float*)outFieldPtr->rowPtr(0, 0), flag);
            }
        }
        else if (algo == 2)
        {
            retval += doInterpolateQuintic((float*)positionsdPtr->rowPtr(0, 0), positionsdPtr->getSize(0), (float*)outFieldPtr->rowPtr(0, 0), flag);
        }
        else if (algo == 3)
        {
            retval += doInterpolateHeptic((float*)positionsdPtr->rowPtr(0, 0), positionsdPtr->getSize(0), (float*)outFieldPtr->rowPtr(0, 0), flag);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DIC::DICInterpolation(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);
	ito::DataObject *inFieldPtr = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *outFieldPtr = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::DataObject *positionsdPtr = paramsMand->at(2).getVal<ito::DataObject*>();
    int algo = paramsOpt->at(0).getVal<int>();
    int degree = paramsOpt->at(1).getVal<int>();
    int withDer = paramsOpt->at(2).getVal<int>();
    int keepImage = paramsOpt->at(3).getVal<int>();
    int useCuda = paramsOpt->at(4).getVal<int>();

    ito::DataObject positions;
    int numPts;
    if (positionsdPtr->getSize(0) == 2 && positionsdPtr->getSize(1) != 2)
    {
        positions = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)positionsdPtr, "PosXY", ito::Range(0, 2), ito::Range::all(), retVal, ito::tFloat64, 0);
        *positionsdPtr = positionsdPtr->trans();
    }
    else
    {
        positions = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)positionsdPtr, "PosXY", ito::Range::all(), ito::Range(0, 2), retVal, ito::tFloat64, 0);
        numPts = positionsdPtr->getSize(0);
    }

    if (numPts < 1)
        return ito::RetVal(ito::retError, 0, tr("at least one point must be passed for interpolation").toLatin1().data());

    int flag = 0;
    if (keepImage == -1)
        flag = 4 + withDer + 2;
    else if (keepImage == 1)
        flag = withDer + 2;
    retVal += doInterpolate(inFieldPtr, &positions, outFieldPtr, algo, flag, useCuda);

    // do some clean up
    clearFilterCache();

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
