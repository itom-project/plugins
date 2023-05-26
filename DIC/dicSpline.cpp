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
#include "DataObject/dataObjectFuncs.h"
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
extern int NTHREADS;

//----------------------------------------------------------------------------------------------------------------------------------
const QString DIC::DICSplineCoeffsDoc = QObject::tr("calculate spline coefficients from given data points");

ito::RetVal DIC::DICSplineCoeffsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::Param param;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("dObjIn", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input vector with data points"));
    paramsMand->append(ito::Param("dObjOut", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output vector with calculated spline coefficients"));

    paramsOpt->append(ito::Param("algorithm", ito::ParamBase::Int | ito::ParamBase::In, 0, 5, 0, "maybe we have different algos in future"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal doCalcSplineCoeffs(const _Tp *inpPts, const int numPt, const int colWise,
    const int stp, const double xMin, const double xMax, Eigen::VectorXd &xVec, Eigen::VectorXd &yVec)
{
    ito::RetVal retval(ito::retOk);
    double scale;
    if (xMax - xMin != 0)
        scale = 1.0 / (xMax - xMin);
    else
        scale = 1.0;

    if (!colWise)
    {
        for (int np = 0; np < numPt; np++)
        {
            xVec(np) = (inpPts[stp * np] - xMin) * scale;
            yVec(np) = inpPts[stp * np + 1];
        }
    }
    else
    {
        for (int np = 0; np < numPt; np++)
        {
            xVec(np) = (inpPts[np] - xMin) * scale;
            yVec(np) = inpPts[stp + np];
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DIC::DICSplineCoeffs(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);
    ito::DataObject *inFieldPtr = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *outFieldPtr = paramsMand->at(1).getVal<ito::DataObject*>();

    if (!inFieldPtr || !outFieldPtr || inFieldPtr->getDims() > 2 || (inFieldPtr->getSize(0) != 2 && inFieldPtr->getSize(1) != 2))
        return ito::RetVal(ito::retWarning, 0, tr("input and output vector must be initialized").toLatin1().data());

    int rowWise = 1, npts;
    if (inFieldPtr->getSize(0) > inFieldPtr->getSize(1))
        rowWise = 0;
    double xMin, xMax;
    if (rowWise)
    {
        ito::uint32 pos[3];
        ito::DataObject xpts = inFieldPtr->at(ito::Range(0, 1), ito::Range::all());
        ito::dObjHelper::minValue(&xpts, xMin, pos);
        ito::dObjHelper::maxValue(&xpts, xMax, pos);
        npts = inFieldPtr->getSize(1);
    }
    else
    {
        ito::uint32 pos[3];
        ito::DataObject xpts = inFieldPtr->at(ito::Range::all(), ito::Range(0, 1));
        ito::dObjHelper::minValue(&xpts, xMin, pos);
        ito::dObjHelper::maxValue(&xpts, xMax, pos);
        npts = inFieldPtr->getSize(0);
    }

    Eigen::VectorXd eigenXVals(npts);
    Eigen::VectorXd eigenYVals(npts);

    int stp = inFieldPtr->getStep(0);
    switch (inFieldPtr->getType())
    {
        case ito::tInt8:
            doCalcSplineCoeffs<ito::int8>((ito::int8*)inFieldPtr->rowPtr(0, 0), npts, rowWise, stp, xMin, xMax, eigenXVals, eigenYVals);
        break;
        case ito::tUInt8:
            doCalcSplineCoeffs<ito::uint8>((ito::uint8*)inFieldPtr->rowPtr(0, 0), npts, rowWise, stp, xMin, xMax, eigenXVals, eigenYVals);
        break;
        case ito::tInt16:
            doCalcSplineCoeffs<ito::int16>((ito::int16*)inFieldPtr->rowPtr(0, 0), npts, rowWise, stp, xMin, xMax, eigenXVals, eigenYVals);
        break;
        case ito::tUInt16:
            doCalcSplineCoeffs<ito::uint16>((ito::uint16*)inFieldPtr->rowPtr(0, 0), npts, rowWise, stp, xMin, xMax, eigenXVals, eigenYVals);
        break;
        case ito::tInt32:
            doCalcSplineCoeffs<ito::int32>((ito::int32*)inFieldPtr->rowPtr(0, 0), npts, rowWise, stp, xMin, xMax, eigenXVals, eigenYVals);
        break;
        case ito::tUInt32:
            doCalcSplineCoeffs<ito::uint32>((ito::uint32*)inFieldPtr->rowPtr(0, 0), npts, rowWise, stp, xMin, xMax, eigenXVals, eigenYVals);
        break;
        case ito::tFloat32:
            doCalcSplineCoeffs<ito::float32>((ito::float32*)inFieldPtr->rowPtr(0, 0), npts, rowWise, stp, xMin, xMax, eigenXVals, eigenYVals);
        break;
        case ito::tFloat64:
            doCalcSplineCoeffs<ito::float64>((ito::float64*)inFieldPtr->rowPtr(0, 0), npts, rowWise, stp, xMin, xMax, eigenXVals, eigenYVals);
        break;
        default:
            retVal += ito::RetVal(ito::retError, 0, tr("Input data type not supported").toLatin1().data());
    }

    Eigen::Spline<double, 1> sf = Eigen::SplineFitting<Eigen::Spline<double, 1> >::Interpolate(
        eigenYVals.transpose(), // No more than cubic spline, but accept short vectors.
        std::min<int>(eigenXVals.rows() - 1, 3),
        eigenXVals);
/*
    Eigen::Spline<double, 1> sf = Eigen::SplineFitting<Eigen::Spline<double, 1> >::InterpolateWithDerivatives(
        eigenYVals.transpose(), // No more than cubic spline, but accept short vectors.
        const PointArrayType &  	derivatives,
        const IndexArray &  	derivativeIndices,
        const unsigned int  	degree
        );
*/
    Eigen::VectorXd knots, ctrlPts;
    knots = sf.knots();
    ctrlPts = sf.ctrls();

    *outFieldPtr = ito::DataObject(2, knots.rows(), ito::tFloat64);
    double *pOut = (double*)outFieldPtr->rowPtr(0, 0);
    for (int np = 0; np < knots.rows(); np++)
    {
        pOut[np] = knots(np);
    }
    for (int np = 0; np < ctrlPts.rows(); np++)
    {
        pOut[knots.rows() + np] = ctrlPts(np);
    }
    pOut[knots.rows() + ctrlPts.rows()] = xMin;
    pOut[knots.rows() + ctrlPts.rows() + 1] = xMax;

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal doSplineInterpolate(const Eigen::Spline<double, 1> &sf, const _Tp *pInPts, const int npts, const int stp,
    const double xmin, const double scale, double *pOutPts, const int withDer = 0)
{
    ito::RetVal retVal(ito::retOk);

    if (withDer)
    {
        Eigen::Array<double, 1, -1, 1, 1, -1> val;
        for (int np = 0; np < npts; np++)
        {
            //pOutPts[np * 2] = sf((pInPts[np * stp] - xmin) * scale)(0);
            val = sf.derivatives((pInPts[np * stp] - xmin) * scale, 1);
            pOutPts[np * 2] = val[0];
            pOutPts[np * 2 + 1] = val[1] * scale;
        }
    }
    else
    {
        for (int np = 0; np < npts; np++)
        {
            pOutPts[np] = sf((pInPts[np * stp] - xmin) * scale)(0);
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString DIC::DICSplineValsDoc = QObject::tr("calculate spline interpolation at given points from coefficients");

ito::RetVal DIC::DICSplineValsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::Param param;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("dCoeffs", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "vector with spline coefficients"));
    paramsMand->append(ito::Param("dPts", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "vector with interpolation positions"));
    paramsMand->append(ito::Param("dObjOut", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output vector with interpolated values"));

    paramsOpt->append(ito::Param("algorithm", ito::ParamBase::Int | ito::ParamBase::In, 0, 5, 0, "maybe we have different algos in future"));
    paramsOpt->append(ito::Param("derivatives", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, "calculate derivatives 1 or not 0"));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DIC::DICSplineVals(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);
    ito::DataObject *inCoeffs = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *inPts = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::DataObject *outPts = paramsMand->at(2).getVal<ito::DataObject*>();
    int withDer = paramsOpt->at(1).getVal<int>();

    if (!inCoeffs || !inPts || !outPts || inCoeffs->getDims() > 2 || (inCoeffs->getSize(0) != 2 && inCoeffs->getSize(1) != 2) || inPts->getSize(0) < 1)
        return ito::RetVal(ito::retWarning, 0, tr("input and output vectors must be initialized").toLatin1().data());

    int rowWise = 1, npts;
    if (inCoeffs->getSize(0) > inCoeffs->getSize(1))
        rowWise = 0;

    Eigen::VectorXd knots;
    Eigen::VectorXd ctrlPts;
    double *pCoeffs = (double*)inCoeffs->rowPtr(0, 0);

    if (rowWise)
    {
        npts = inCoeffs->getSize(1);
        knots = Eigen::VectorXd(npts);
        ctrlPts = Eigen::VectorXd(npts - 4);
        for (int np = 0; np < npts; np++)
        {
            knots(np) = pCoeffs[np];
        }
        for (int np = 0; np < npts - 4; np++)
        {
            ctrlPts(np) = pCoeffs[npts + np];
        }
    }
    else
    {
        npts = inCoeffs->getSize(0);
        knots = Eigen::VectorXd(npts);
        ctrlPts = Eigen::VectorXd(npts - 4);
        for (int np = 0; np < npts; np++)
        {
            knots(np) = pCoeffs[np * 2];
        }
        for (int np = 0; np < npts - 4; np++)
        {
            ctrlPts(np) = pCoeffs[np * 2 + 1];
        }
    }

    Eigen::Spline<double, 1> sf(knots, ctrlPts);
    double xmin = pCoeffs[npts + npts - 4];
    double xmax = pCoeffs[npts + npts - 3];
    double scale = 1.0 / (xmax - xmin);
    int stp = 0;
    if (inPts->getSize(0) > inPts->getSize(1))
    {
        npts = inPts->getSize(0);
        stp = inPts->getStep(0);
    }
    else
    {
        npts = inPts->getSize(1);
        stp = inPts->getStep(1);
    }

    if (withDer)
        outPts->zeros(npts, 2, ito::tFloat64);
    else
        outPts->zeros(npts, 1, ito::tFloat64);
    double *pOutPts = (double*)outPts->rowPtr(0, 0);
    double *pInPts = (double*)inPts->rowPtr(0, 0);

    switch (inPts->getType())
    {
    case ito::tInt8:
        doSplineInterpolate<ito::int8>(sf, (ito::int8*)inPts->rowPtr(0, 0), npts, stp, xmin, scale, pOutPts, withDer);
        break;
    case ito::tUInt8:
        doSplineInterpolate<ito::uint8>(sf, (ito::uint8*)inPts->rowPtr(0, 0), npts, stp, xmin, scale, pOutPts, withDer);
        break;
    case ito::tInt16:
        doSplineInterpolate<ito::int16>(sf, (ito::int16*)inPts->rowPtr(0, 0), npts, stp, xmin, scale, pOutPts, withDer);
        break;
    case ito::tUInt16:
        doSplineInterpolate<ito::uint16>(sf, (ito::uint16*)inPts->rowPtr(0, 0), npts, stp, xmin, scale, pOutPts, withDer);
        break;
    case ito::tInt32:
        doSplineInterpolate<ito::int32>(sf, (ito::int32*)inPts->rowPtr(0, 0), npts, stp, xmin, scale, pOutPts, withDer);
        break;
    case ito::tUInt32:
        doSplineInterpolate<ito::uint32>(sf, (ito::uint32*)inPts->rowPtr(0, 0), npts, stp, xmin, scale, pOutPts, withDer);
        break;
    case ito::tFloat32:
        doSplineInterpolate<ito::float32>(sf, (ito::float32*)inPts->rowPtr(0, 0), npts, stp, xmin, scale, pOutPts, withDer);
        break;
    case ito::tFloat64:
        doSplineInterpolate<ito::float64>(sf, (ito::float64*)inPts->rowPtr(0, 0), npts, stp, xmin, scale, pOutPts, withDer);
        break;
    default:
        retVal += ito::RetVal(ito::retError, 0, tr("Input data type not supported").toLatin1().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
