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

#define _USE_MATH_DEFINES
#include <math.h>
#include "dicPrivate.h"
#include "dic.h"

#include "DataObject/dataobj.h"
#include "DataObject/dataObjectFuncs.h"
#include "opencv2/imgproc/imgproc.hpp"

#include <QtCore/QtPlugin>

#include "pluginVersion.h"

extern int NTHREADS;
extern bool hasCuda;

//----------------------------------------------------------------------------------------------------------------------------------
int doCalcCoord(cell *thisCell, cv::Mat *coeffs, ito::DataObject *pts)
{
    if (!coeffs)
        return 0;
    ito::float32 *cptr = (ito::float32*)coeffs->data;
    ito::float32 *ptsPtr = (ito::float32*)pts->rowPtr(0, 0);
    int nrows = thisCell->y1 - thisCell->y0;
    int ncols = thisCell->x1 - thisCell->x0;

    switch (coeffs->rows)
    {
        case 12:
        {
#if (USEOMP)
        #pragma omp parallel num_threads(NTHREADS)
        {
        #pragma omp for schedule(guided)
#endif
            for (int cy = 0; cy < nrows; cy++)
            {
//                ito::float64 dy = (cy - nrows / 2) / (nrows / 2.0), dy2 = dy * dy * 0.5;
                ito::float64 dy = (cy - nrows / 2), dy2 = dy * dy * 0.5;
                for (int cx = 0; cx < ncols; cx++)
                {
//                    ito::float64 dx = (cx - ncols / 2) / (ncols / 2.0), dx2 = dx * dx * 0.5;
                    ito::float64 dx = (cx - ncols / 2), dx2 = dx * dx * 0.5;
                    ptsPtr[(cy * ncols + cx) * 2] = cx + thisCell->x0 + cptr[0] + cptr[2] * dx
                        + cptr[4] * dy + cptr[6] * dx2 + cptr[8] * dy2 + cptr[10] * dx * dy;
                    ptsPtr[(cy * ncols + cx) * 2 + 1] = cy + thisCell->y0 + cptr[1] + cptr[3] * dx
                        + cptr[5] * dy + cptr[7] * dx2 + cptr[9] * dy2 + cptr[11] * dx * dy;
                }
            }
#if (USEOMP)
        }
#endif
        }
        break;

        default:
            return -1;
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal doCalcIntDiff(ito::DataObject *imgInRef, ito::DataObject *imgInDef, ito::DataObject *intDiff, ito::DataObject *gradients, cell *thisCell, cv::Mat *coeffs, int algo, int mode, ito::float64 *intSqrSum = NULL)
{
    ito::RetVal retVal(ito::retOk);
    int csx = thisCell->x1 - thisCell->x0;
    int csy = thisCell->y1 - thisCell->y0;

    ito::DataObject ptVec(csx * csy, 2, ito::tFloat32);
    ito::DataObject intInterpol(csx * csy, 3, ito::tFloat32);

    doCalcCoord(thisCell, coeffs, &ptVec);

    // always calculate interpolation including the derivatives
    retVal += doInterpolate(imgInDef, &ptVec, &intInterpol, algo & 14, mode, 0);
    // set ROI as intInterpol also includes the gradients
    ito::DataObject imgDef;
    intInterpol.at(ito::Range(0, intInterpol.getSize(0)), ito::Range(0, 1)).copyTo(imgDef);
    retVal += normalizeInt(imgDef, imgDef, intSqrSum);

    *intDiff = *imgInRef - imgDef;
    if (mode & 1)
        *gradients = intInterpol;
    else
        *gradients = ito::DataObject();

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
int doCalcJacobian(ito::DataObject *imgInRef, ito::DataObject *imgInDef, ito::DataObject *jacobian, ito::DataObject *intDiff, cell *thisCell, cv::Mat *coeffs, int algo)
{
    ito::RetVal retVal(ito::retOk);
    int csx = thisCell->x1 - thisCell->x0;
    int csy = thisCell->y1 - thisCell->y0;
    ito::float64 intDefSqr;

    ito::DataObject ptVec(csx * csy, 2, ito::tFloat32);
    ito::DataObject intInterpol(csx * csy, 3, ito::tFloat32);

    doCalcIntDiff(imgInRef, imgInDef, intDiff, &intInterpol, thisCell, coeffs, algo, 3, &intDefSqr);

    ito::float32 *intDiffPtr = (ito::float32*)intDiff->rowPtr(0, 0);

    jacobian->zeros(12, csx * csy, ito::tFloat32);
    ito::float32 *jacPtr = (ito::float32*)jacobian->rowPtr(0, 0);
    ito::float32 *intGradPtr = (ito::float32*)intInterpol.rowPtr(0, 0);

    int jacLineStp = csx * csy;
#if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #pragma omp for schedule(guided)
#endif
    for (int cy = 0; cy < csy; cy++)
    {
//        ito::float64 dy = (cy - csy / 2) / (csy / 2.0);
        ito::float64 dy = (cy - csy / 2), dy2 = dy * dy * 0.5;
        for (int cx = 0; cx < csx; cx++)
        {
//            ito::float64 dx = (cx - csx / 2) / (csx / 2.0);
            ito::float64 dx = (cx - csx / 2), dx2 = dx * dx * 0.5;
            int lineStp = csx * cy;
            ito::float64 intFact;
            if (intDefSqr != 0)
                intFact = -2.0 / intDefSqr; // *intDiffPtr[lineStp + cx];
            else
                intFact = -2.0; // *intDiffPtr[lineStp + cx];

            jacPtr[lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 1];
            jacPtr[jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 2];
            jacPtr[2 * jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 1] * dx;
            jacPtr[3 * jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 2] * dx;
            jacPtr[4 * jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 1] * dy;
            jacPtr[5 * jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 2] * dy;
            jacPtr[6 * jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 1] * dx2;
            jacPtr[7 * jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 2] * dx2;
            jacPtr[8 * jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 1] * dy2;
            jacPtr[9 * jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 2] * dy2;
            jacPtr[10 * jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 1] * dx * dy;
            jacPtr[11 * jacLineStp + lineStp + cx] = intFact * intGradPtr[(lineStp + cx) * 3 + 2] * dx * dy;
        }
    }
#if (USEOMP)
    }
#endif

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DIC::DICDisplacementDoc = QObject::tr("calculate displacement fields between two images");

/*static*/ ito::RetVal DIC::DICDisplacementParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::Param param;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("dObjInRef", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d input reference data field"));
    paramsMand->append(ito::Param("dObjInDef", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d output data field"));
    paramsMand->append(ito::Param("PosXY", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "displacement positions and subfield sizes"));
    paramsMand->append(ito::Param("disCoeff", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output vector with polynomial coefficients for each subfield"));
    paramsMand->append(ito::Param("correlCoeff", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output vector with correlation coefficients for each subfield"));

    paramsOpt->append(ito::Param("interpolAlgo", ito::ParamBase::Int | ito::ParamBase::In, 0, 5, 1, "Algorithm used for interpolation 0: bilinear, 1: bicubic, 2: ..."));
    paramsOpt->append(ito::Param("maxIter", ito::ParamBase::Int | ito::ParamBase::In, 0, 10000, 300, "Maximum number of iterations for Levenberg-Marquardt optimization"));
    paramsOpt->append(ito::Param("threshold", ito::ParamBase::Double | ito::ParamBase::In, 1.0, std::numeric_limits<ito::float64>::epsilon(), 1.0e-5, "convergence threshold"));
    paramsOpt->append(ito::Param("initialGuessType", ito::ParamBase::Int | ito::ParamBase::In, 0, 4, 4, "initial guess of displacement field: 0 fill with zeros, 1 use coarse fine search, 2 use coarse fine search with refinement, 3 fft, 4 multi-scale fft"));
    paramsOpt->append(ito::Param("initialGuessRange", ito::ParamBase::Int | ito::ParamBase::In, 0, 10, 5, "range of initial guess search"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DIC::DICDisplacement(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);

    ito::DataObject inFieldRef;
    paramsMand->at(0).getVal<ito::DataObject*>()->copyTo(inFieldRef);
    ito::DataObject *inFieldDef = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::DataObject *positionsPtr = paramsMand->at(2).getVal<ito::DataObject*>();
    ito::DataObject *distCoeff = paramsMand->at(3).getVal<ito::DataObject*>();
    ito::DataObject *correlCoeffVec = paramsMand->at(4).getVal<ito::DataObject*>();

    int interpAlgo = paramsOpt->at(0).getVal<int>();
    int maxIter = paramsOpt->at(1).getVal<int>();
    ito::float64 threshold = paramsOpt->at(2).getVal<ito::float64>();
    int guessType = paramsOpt->at(3).getVal<int>();
    int guessRange= paramsOpt->at(4).getVal<int>();
    ito::float64 guessCenter[2] = {0, 0};

    if (inFieldRef.getDims() != 2 || inFieldDef->getDims() != 2
        || inFieldRef.getSize(0) != inFieldDef->getSize(0) || inFieldDef->getSize(0) == 0
        || inFieldRef.getSize(1) != inFieldDef->getSize(1) || inFieldDef->getSize(1) == 0)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Reference and deformed field must have the same size and 2 dimensions!").toLatin1().data());
        return retVal;
    }

    inFieldRef = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)&inFieldRef, "dObjInRef", ito::Range::all(), ito::Range::all(), retVal, ito::tFloat32, 0);
    if (retVal.containsError())
        return retVal;

    *inFieldDef = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)inFieldDef, "dObjInDef", ito::Range::all(), ito::Range::all(), retVal, ito::tFloat32, 0);
    if (retVal.containsError())
        return retVal;

    ito::DataObject positions = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)positionsPtr, "PosXY", ito::Range::all(), ito::Range(4, 4), retVal, ito::tFloat64, 0);
    if (retVal.containsError())
        return retVal;

    distCoeff->zeros(positions.getSize(0), 12, ito::tFloat32);
    correlCoeffVec->zeros(positions.getSize(0), ito::tFloat32);

    // do a dummy interpolation to cache AMat for proceeding calls
    ito::DataObject dummyPos, dummyRes;
    dummyPos.zeros(1, 2, ito::tFloat64);
    retVal += doInterpolate(inFieldDef, &dummyPos, &dummyRes, interpAlgo & 14, 0, hasCuda);

    for (int nPos = 0; nPos < positions.getSize(0); nPos++)
    {
        ito::DataObject jacobian;
        int startx = positions.at<ito::float64>(nPos, 0) - positions.at<ito::float64>(nPos, 2) / 2 < 0 ?
            0 : positions.at<ito::float64>(nPos, 0) - positions.at<ito::float64>(nPos, 2) / 2;
        int endx = positions.at<ito::float64>(nPos, 0) + positions.at<ito::float64>(nPos, 2) / 2 + 1 >= inFieldRef.getSize(1) - 1 ?
            inFieldRef.getSize(1) - 1 : positions.at<ito::float64>(nPos, 0) + positions.at<ito::float64>(nPos, 2) / 2 + 1;
        int starty = positions.at<ito::float64>(nPos, 1) - positions.at<ito::float64>(nPos, 3) / 2 < 0 ?
            0 : positions.at<ito::float64>(nPos, 1) - positions.at<ito::float64>(nPos, 3) / 2;
        int endy = positions.at<ito::float64>(nPos, 1) + positions.at<ito::float64>(nPos, 3) / 2 + 1 >= inFieldRef.getSize(0) - 1?
            inFieldRef.getSize(0) - 1 : positions.at<ito::float64>(nPos, 1) + positions.at<ito::float64>(nPos, 3) / 2 + 1;
        cell thisCell(positions.at<ito::float64>(nPos, 0), positions.at<ito::float64>(nPos, 1), startx, endx, starty, endy);

        if (startx < 0 || endx < 0 || startx >= inFieldRef.getSize(1) - 1 || endx >= inFieldRef.getSize(1) - 1
            || starty < 0 || endy < 0 || starty >= inFieldRef.getSize(0) - 1 || endy >= inFieldRef.getSize(0) - 1)
        {
            retVal += ito::RetVal(ito::retWarning, 0, tr("Skipped cell #%1 due to invalid dimensions").arg(nPos).toLatin1().data());
            continue;
        }
        if (startx > endx || starty > endy)
        {
            retVal += ito::RetVal(ito::retError, 0, tr("Error calculating cell coordinates ...").toLatin1().data());
            return retVal;
        }

        ito::float64 correlCoeff;
        ito::DataObject intensDiff(endy - starty, endx - startx, ito::tFloat32);
        int sizey = intensDiff.getSize(0), sizex = intensDiff.getSize(1);

        // extract reference patch and normalize its intensity values
        ito::DataObject inPatchRef;
        inFieldRef.at(ito::Range(starty, endy), ito::Range(startx, endx)).copyTo(inPatchRef);

        cv::Mat distCoeffCell(12, 1, CV_32F);
        memcpy(distCoeffCell.data, distCoeff->rowPtr(0, nPos), 12 * sizeof(ito::float32));

        if (guessType > 0)
        {
            int guessRangeArr[2] = { guessRange, guessRange };
            if (guessType < 3)
            {
                retVal += normalizeInt(inPatchRef, inPatchRef);
                retVal += doCalcInitialGuess(&inPatchRef, inFieldDef, &thisCell, guessRangeArr, guessType, guessCenter, correlCoeff);
            }
            else if (guessType == 3)
            {
                retVal += doCalcInitialGuessFFT(&inPatchRef, inFieldDef, &thisCell, guessType, guessCenter, correlCoeff);
                retVal += normalizeInt(inPatchRef, inPatchRef);
            }
            else
            {
                float maxxy[2] = { 128.0, 128.0 };
                retVal += doCalcInitialGuessFFTMScale(&inFieldRef, inFieldDef, &thisCell, guessType, maxxy, 16.0, guessCenter, correlCoeff);
                retVal += normalizeInt(inPatchRef, inPatchRef);
            }

            if (retVal.containsWarningOrError())
                return retVal;
            ((ito::float32*)distCoeffCell.data)[0] = guessCenter[0];
            ((ito::float32*)distCoeffCell.data)[1] = guessCenter[1];
        }
        else
        {
            retVal += normalizeInt(inPatchRef, inPatchRef);
        }

        int nSizes[2] = { sizex * sizey, 1 };
        inPatchRef = inPatchRef.reshape(2, nSizes);

        doCalcJacobian(&inPatchRef, inFieldDef, &jacobian, &intensDiff, &thisCell, &distCoeffCell, interpAlgo);

        ito::DataObject Hesse = jacobian * jacobian.trans() * 0.5;
        ito::DataObject Gradient = jacobian * intensDiff;

        intensDiff = intensDiff.mul(intensDiff);
        retVal += ito::dObjHelper::meanValue(&intensDiff, correlCoeff, 0);
        correlCoeff *= sizex * sizey;
        ito::float64 mu = 0;
        for (int ni = 0; ni < Hesse.getSize(0); ni++)
        {
            if (Hesse.at<ito::float32>(ni, ni) > mu)
                mu = Hesse.at<ito::float32>(ni, ni);
        }

        cv::Mat dCoeff = cv::Mat::zeros(12, 1, CV_32F);
        ito::DataObject imat;
        cv::Mat disCoeffNew;
        ito::float64 correlCoeffNew;
        imat.eye(Hesse.getSize(0), ito::tFloat32);
        bool convergence = 0;
        int niter = 0;
        ito::float64 damping = 2.0;
        do
        {
            Hesse += imat * mu;
            // maybe revise calculation method used for matrix inversion
            cv::Mat HesseInv = ((cv::Mat*)Hesse.get_mdata()[0])->inv(cv::DECOMP_CHOLESKY);
            dCoeff = HesseInv * (*(cv::Mat*)Gradient.get_mdata()[0]) * -1.0;

            if ((*cv::sum(cv::abs(dCoeff)).val) < threshold)
            {
                convergence = 1;
            }
            else
            {
                distCoeffCell.copyTo(disCoeffNew);
                disCoeffNew += dCoeff;
                ito::DataObject gradDummy;
                doCalcIntDiff(&inPatchRef, inFieldDef, &intensDiff, &gradDummy, &thisCell, &disCoeffNew, interpAlgo, 2);

                intensDiff = intensDiff.mul(intensDiff);
                retVal += ito::dObjHelper::meanValue(&intensDiff, correlCoeffNew, 0);
                correlCoeffNew *= sizex * sizey;

                cv::Mat glin = dCoeff.t() * (dCoeff * mu - (*(cv::Mat*)Gradient.get_mdata()[0])) * 0.5;
                ito::float64 ro = (correlCoeff - correlCoeffNew) / glin.at<ito::float32>(0);
                if (ro > 0)
                {
                    disCoeffNew.copyTo(distCoeffCell);
                    doCalcJacobian(&inPatchRef, inFieldDef, &jacobian, &intensDiff, &thisCell, &distCoeffCell, interpAlgo);

                    Hesse = jacobian * jacobian.trans();
                    Gradient = jacobian * intensDiff;

                    intensDiff = intensDiff.mul(intensDiff);
                    retVal += ito::dObjHelper::meanValue(&intensDiff, correlCoeff, 0);
                    correlCoeff *= sizex * sizey;

                    mu = 1 / 3.0 > 1.0 - pow(2.0 * ro - 1.0, 3.0) ? mu * 1 / 3.0 : mu * (1.0 - pow(2.0 * ro - 1.0, 3.0));
                    damping = 2.0;
                }
                else
                {
                    mu *= damping;
                    damping *= 2.0;
                    // is it a good choice to grow mu geometrically? Maybe linear growth is better, needs checking
//                    mu *= 2.0;
                }
            }

            niter++;
        } while (!convergence && niter < maxIter);
        memcpy(distCoeff->rowPtr(0, nPos), distCoeffCell.data, 12 * sizeof(ito::float32));
        ((ito::float32*)correlCoeffVec->rowPtr(0, 0))[nPos] = correlCoeff;
    }

    // clean up cached AMat to free memory
    clearFilterCache();

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
