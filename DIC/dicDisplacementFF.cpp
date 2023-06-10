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

#include "dicPrivate.h"
#include "dic.h"
#include <qvector.h>
#include "DataObject/dataObjectFuncs.h"

#include "Eigen/SparseCore"
#include "Eigen/SparseLU"

extern int NTHREADS;

//----------------------------------------------------------------------------------------------------------------------------------
int doCalcCoordFF(cell *thisCell, Eigen::VectorXd *coeffMat, ito::DataObject *pts, ito::DataObject *coeffsDer, const int numcoeff)
{
    if (!coeffMat || !thisCell || !pts)
        return 0;
    ito::float64 *cptr = (ito::float64*)coeffMat->data();
    ito::float64 *ptsPtr = (ito::float64*)pts->rowPtr(0, 0);
    ito::float64 *cdPtr = (ito::float64*)coeffsDer->rowPtr(0, 0);
    int nrows = thisCell->y1 - thisCell->y0;
    int ncols = thisCell->x1 - thisCell->x0;

    switch (numcoeff)
    {
        // Q4 Element
        case 4:
        {
#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
        {
#pragma omp for schedule(guided)
#endif
            for (int cy = 0; cy < nrows; cy++)
            {
//                ito::float64 dy = (cy - nrows / 2) / nrows;
                for (int cx = 0; cx < ncols; cx++)
                {
                    // 1.0 * makes calculation float othwerwise it is int
//                    ito::float64 dx = (cx - ncols / 2) / ncols;
                    ito::float64 n1 = 1.0 * (ncols - 1 - cx) * (nrows - 1 - cy) / (nrows - 1) / (ncols - 1);
                    ito::float64 n2 = 1.0 * cx * (nrows - 1 - cy) / (nrows - 1) / (ncols - 1);
                    ito::float64 n3 = 1.0 * (ncols - 1 - cx) * cy / (nrows - 1) / (ncols - 1);
                    ito::float64 n4 = 1.0 * cx * cy / nrows / ncols;

                    cdPtr[(cy * ncols + cx) * 8] = n1;
                    cdPtr[(cy * ncols + cx) * 8 + 2] = n2;
                    cdPtr[(cy * ncols + cx) * 8 + 4] = n3;
                    cdPtr[(cy * ncols + cx) * 8 + 6] = n4;
                    cdPtr[(cy * ncols + cx) * 8 + 1] = n1;
                    cdPtr[(cy * ncols + cx) * 8 + 3] = n2;
                    cdPtr[(cy * ncols + cx) * 8 + 5] = n3;
                    cdPtr[(cy * ncols + cx) * 8 + 7] = n4;

                    ptsPtr[(cy * ncols + cx) * 2] = cx + thisCell->x0
                        + cdPtr[(cy * ncols + cx) * 8] * cptr[thisCell->nodes[0] * 2]
                        + cdPtr[(cy * ncols + cx) * 8 + 2] * cptr[thisCell->nodes[1] * 2]
                        + cdPtr[(cy * ncols + cx) * 8 + 4] * cptr[thisCell->nodes[2] * 2]
                        + cdPtr[(cy * ncols + cx) * 8 + 6] * cptr[thisCell->nodes[3] * 2];
                    ptsPtr[(cy * ncols + cx) * 2 + 1] = cy + thisCell->y0
                        + cdPtr[(cy * ncols + cx) * 8 + 1] * cptr[thisCell->nodes[0] * 2 + 1]
                        + cdPtr[(cy * ncols + cx) * 8 + 3] * cptr[thisCell->nodes[1] * 2 + 1]
                        + cdPtr[(cy * ncols + cx) * 8 + 5] * cptr[thisCell->nodes[2] * 2 + 1]
                        + cdPtr[(cy * ncols + cx) * 8 + 7] * cptr[thisCell->nodes[3] * 2 + 1];

                    /*
                    ptsPtr[(cy * ncols + cx) * 2] = cx + thisCell->x0
                        + cptr[thisCell->nodes[0]] * n1 + cptr[thisCell->nodes[1]] * n2
                        + cptr[thisCell->nodes[2]] * n3 + cptr[thisCell->nodes[3]] * n4;
                    ptsPtr[(cy * ncols + cx) * 2 + 1] = cy + thisCell->y0
                        + cptr[thisCell->nodes[4]] * n1 + cptr[thisCell->nodes[5]] * n2
                        + cptr[thisCell->nodes[6]] * n3 + cptr[thisCell->nodes[7]] * n4;
                    */
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
// mode:
//      Bit  Dez
//        0   1     calculate derivatives
//        1   2     keep image
ito::RetVal doCalcIntDiffFF(ito::DataObject *imgInDef, ito::DataObject *intDiff, ito::DataObject *gradients,
    ito::DataObject *coeffsDer, cell *thisCell, Eigen::VectorXd *coeffs, const int flags, const int mode, const int numcoeff, ito::float64 *intSqrSum = NULL)
{
    ito::RetVal retVal(ito::retOk);
    int csx = thisCell->x1 - thisCell->x0;
    int csy = thisCell->y1 - thisCell->y0;

    ito::DataObject ptVec(csx * csy, 2, ito::tFloat64);
    ito::DataObject intInterpol(csx * csy, 3, ito::tFloat64);
    ito::DataObject coeffDerivatives;

    doCalcCoordFF(thisCell, coeffs, &ptVec, coeffsDer, numcoeff);

    // always calculate interpolation including the derivatives
    retVal += doInterpolate(imgInDef, &ptVec, &intInterpol, flags & 14, mode, 0);
    // set ROI as intInterpol also includes the gradients
    ito::DataObject imgDef;
    intInterpol.at(ito::Range(0, intInterpol.getSize(0)), ito::Range(0, 1)).copyTo(imgDef);
    retVal += normalizeInt(imgDef, imgDef, intSqrSum);

    *intDiff = thisCell->refInt - imgDef;
//    imgDef.copyTo(*intDiff);
//    thisCell->refInt.copyTo(*intDiff);
//    *intDiff = ptVec;
//    *intDiff = imgDef - thisCell->refInt;
//    *intDiff = ptVec;
    if (mode & 1)
        *gradients = intInterpol;

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
int doCalcSMatRVecFF(ito::DataObject *imgInRef, ito::DataObject *imgInDef, Eigen::SparseMatrix<double> &SMat, Eigen::VectorXd &RVec,
    QVector<cell> *cells, Eigen::VectorXd *coeffs, ito::float64 *correlCoeff, const int totNumNodes, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    ito::float64 intDefSqr;
    ito::uint8 nnodes;
    *correlCoeff = 0;

    if ((flags >> 4) == 0)          // Q4 elements -> 4 nodes per cell
    {
        nnodes = 4;
    }
    else if ((flags >> 4) == 1)     // Q8 elements -> 8 nodes per cell
    {
        nnodes = 8;
    }
    else if ((flags >> 4) == 2)     // ANCF elements -> 4 nodes, 2 center coords per cell
    {
        nnodes = 6;
    }
    else if ((flags >> 4) == 2)     // 3D elements -> 4 nodes, center and defocus per cell
    {
        nnodes = 5;
    }

    SMat = Eigen::SparseMatrix<double>(2 * totNumNodes, 2 * totNumNodes);
    SMat.reserve(2 * 4 * totNumNodes);
    RVec = Eigen::SparseVector<double>(2 * totNumNodes);
    RVec = Eigen::VectorXd::Zero(2 * totNumNodes);
//    RVec->zeros(2 * totNumNodes, 1, ito::tFloat64);
//    SMat->zeros(2 * totNumNodes, 2 * totNumNodes, ito::tFloat64);
//    ito::float64 *RVecPtr = (ito::float64*)RVec->rowPtr(0, 0);
//    ito::float64 *SMatPtr = (ito::float64*)SMat->rowPtr(0, 0);

    // iterating over all cells
    for (int nc = 0; nc < cells->length(); nc++)
    {
        int csy = (*cells)[nc].y1 - (*cells)[nc].y0;
        int csx = (*cells)[nc].x1 - (*cells)[nc].x0;
        ito::DataObject ptVec(csy * csx, 2, ito::tFloat64);
        ito::DataObject intInterpol(csy * csx, 3, ito::tFloat64);
        ito::DataObject coeffsDer, intDiff;
        coeffsDer.zeros(csy * csx, nnodes * 2, ito::tFloat64);
        int rsizes[2];
        intDiff.zeros(csy, csx, ito::tFloat64);

        rsizes[0] = csy * csx;
        rsizes[1] = 1;

        doCalcIntDiffFF(imgInDef, &intDiff,
            &intInterpol, &coeffsDer, &(*cells)[nc], coeffs, flags, 3, nnodes, &intDefSqr);

        ito::float32 *intDiffPtr = (ito::float32*)intDiff.rowPtr(0, 0);
        ito::float32 *intGradPtr = (ito::float32*)intInterpol.rowPtr(0, 0);
        ito::float64 *coeffsDerPtr = (ito::float64*)coeffsDer.rowPtr(0, 0);

        for (int ny = 0; ny < csy; ny++)
        {
            int lineStp = csx * ny;
            for (int nx = 0; nx < csx; nx++)
            {
                for (int nk = 0; nk < nnodes; nk++)
                {
                    // dG(x, y) / da_mk = -2.0 * (dG(x', y') / dx' + dG(x', y') / dy') N_k(xsi, eta)
                    RVec((*cells)[nc].nodes[nk] * 2) += -2.0 / intDefSqr * intDiffPtr[lineStp + nx] * intGradPtr[(lineStp + nx) * 3 + 1]
                            * coeffsDerPtr[(lineStp + nx) * nnodes * 2 + nk * 2];
                    RVec((*cells)[nc].nodes[nk] * 2 + 1) += -2.0 / intDefSqr * intDiffPtr[lineStp + nx] * intGradPtr[(lineStp + nx) * 3 + 2]
                            * coeffsDerPtr[(lineStp + nx) * nnodes * 2 + nk * 2 + 1];

//                    RVecPtr[(*cells)[nc].nodes[nk] * 2] += -2.0 / intDefSqr * intDiffPtr[lineStp + nx] * intGradPtr[(lineStp + nx) * 3 + 1]
//                            * coeffsDerPtr[(lineStp + nx) * nnodes * 2 + nk * 2];
//                    RVecPtr[(*cells)[nc].nodes[nk] * 2 + 1] += -2.0 / intDefSqr * intDiffPtr[lineStp + nx] * intGradPtr[(lineStp + nx) * 3 + 2]
//                            * coeffsDerPtr[(lineStp + nx) * nnodes * 2 + nk * 2 + 1];

                }
            }
        }

//        int smlsize = SMat->getSize(1);
        int smlsize = SMat.cols();
        for (int ny = 0; ny < csy; ny++)
        {
            int lineStp = csx * ny;
            for (int nx = 0; nx < csx; nx++)
            {
                for (int nh = 0; nh < nnodes * 2; nh++)
                {
                    int acthnode = (*cells)[nc].nodes[nh / 2] * 2;
                    for (int nk = 0; nk < nnodes * 2; nk++)
                    {
                        int actknode = (*cells)[nc].nodes[nk / 2] * 2;
                        // dG(x, y) / da_mk = (dG(x', y') / dx' + dG(x', y') / dy') N_k(xsi, eta)
                        SMat.coeffRef((acthnode + nh % 2), actknode + nk % 2) // xx
                            += 2.0 / (intDefSqr * intDefSqr) * (intGradPtr[(lineStp + nx) * 3 + 1 + nh % 2] * coeffsDerPtr[(lineStp + nx) * nnodes * 2 + nh])
                            * (intGradPtr[(lineStp + nx) * 3 + 1 + nk % 2] * coeffsDerPtr[(lineStp + nx) * nnodes * 2 + nk]);
//                        SMatPtr[(acthnode + nh % 2) * smlsize + actknode + nk % 2] // xx
//                            += 2.0 / (intDefSqr * intDefSqr) * (intGradPtr[(lineStp + nx) * 3 + 1 + nh % 2] * coeffsDerPtr[(lineStp + nx) * nnodes * 2 + nh])
//                            * (intGradPtr[(lineStp + nx) * 3 + 1 + nk % 2] * coeffsDerPtr[(lineStp + nx) * nnodes * 2 + nk]);
                    }
                }
            }
        }

        ito::float64 correlCoeffTmp;
        intDiff = intDiff.mul(intDiff);
        retVal += ito::dObjHelper::meanValue(&intDiff, correlCoeffTmp, 0);
        correlCoeffTmp *= csx * csy;
        *correlCoeff += correlCoeffTmp;
    }

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DIC::DICDisplacementFFDoc = QObject::tr("calculate full field displacement between two images, \
in synthesis from: 'Finite element formulation for a digital image correlation method', \
Yaofeng Sun, John H.L.Pang, Chee Khuen Wong and Fei Su, \
Applied Optics 44(34), 2005 \
and \
'Mesh-based digital image correlation method using higher order isoparametric elements', \
Shaopeng Ma, Zilong Zhao and Xian Wang, \
Journal of Strain Analysis 47(3), 2012");

/*static*/ ito::RetVal DIC::DICDisplacementFFParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
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
    paramsOpt->append(ito::Param("cellType", ito::ParamBase::Int | ito::ParamBase::In, 0, 3, 0, "cell type used for calculation, 0: Q4, 1: Q8, 2: ANCF, 3: 3D"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DIC::DICDisplacementFF(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
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
    int guessRange = paramsOpt->at(4).getVal<int>();
    int cellType = paramsOpt->at(5).getVal<int>();
    ito::float64 guessCenter[2] = { 0, 0 };
    Eigen::VectorXd distCoeffVec, distCoeffVecNew;

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

    // get number of cells per line
    int ncellsx = 1, ncellsy = 0, np = 0;
    while (np + 1 < positions.getSize(0) && positions.at<ito::float64>(np, 1) == positions.at<ito::float64>(np + 1, 1))
    {
        ncellsx++;
        np++;
    }

    ncellsy = positions.getSize(0) / ncellsx;
    if (ncellsx * ncellsy != positions.getSize(0))
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Error determining cell grid size").toLatin1().data());
        return retVal;
    }

    int numcoeff = 0, flags = interpAlgo;
    if (cellType == 0) // Q4
    {
        numcoeff = 4;
        flags |= 0 << 4;
        distCoeff->zeros((ncellsx + 1) * (ncellsy + 1) * 2, 1, ito::tFloat64);
        distCoeffVec = Eigen::VectorXd::Zero((ncellsx + 1) * (ncellsy + 1) * 2, 1);
    }
    else if (cellType == 1) // Q8
    {
        numcoeff = 8;
        flags |= 1 << 4;
        distCoeff->zeros((ncellsx + 1) * (ncellsy + 1) * 2, 1, ito::tFloat64);
        distCoeffVec = Eigen::VectorXd::Zero((ncellsx + 1) * (ncellsy + 1) * 2, 1);
    }
    else if (cellType == 2) // ANCF
    {
        numcoeff = 6;
        flags |= 2 << 4;
        distCoeff->zeros((ncellsx + 1) * (ncellsy + 1) * 2, 1, ito::tFloat64);
        distCoeffVec = Eigen::VectorXd::Zero((ncellsx + 1) * (ncellsy + 1) * 2, 1);
    }
    else if (cellType == 3) // 3D
    {
        numcoeff = 5;
        flags |= 3 << 4;
        distCoeff->zeros((ncellsx + 1) * (ncellsy + 1) * 2, 1, ito::tFloat64);
        distCoeffVec = Eigen::VectorXd::Zero((ncellsx + 1) * (ncellsy + 1) * 2, 1);
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, tr("unknown field type!").toLatin1().data());
        return retVal;
    }

    correlCoeffVec->zeros((ncellsx + 1) * (ncellsy + 1), ito::tFloat64);

    // do a dummy interpolation to cache AMat for preceeding calls
    ito::DataObject dummyPos, dummyRes;
    dummyPos.zeros(1, 2, ito::tFloat64);
    retVal += doInterpolate(inFieldDef, &dummyPos, &dummyRes, flags & 14, 1, 0);

//    ito::DataObject SMat, RVec;
    Eigen::SparseMatrix<double> SMat;
    Eigen::VectorXd RVec, dCoeffVec;

    // we could run the check on the node positions here or we put it into the calculation method of the Jacobian matrix ...
    // need to define this
    QVector<cell> cells;
//    ito::float64 *distCoeffPtr = (ito::float64*)distCoeff->rowPtr(0, 0);
    cells.resize(ncellsx * ncellsy);
    for (int nPos = 0; nPos < positions.getSize(0); nPos++)
    {
        int startx = positions.at<ito::float64>(nPos, 0) - positions.at<ito::float64>(nPos, 2) / 2 < 0 ?
            0 : positions.at<ito::float64>(nPos, 0) - positions.at<ito::float64>(nPos, 2) / 2;
        int endx = positions.at<ito::float64>(nPos, 0) + positions.at<ito::float64>(nPos, 2) / 2 + 1 >= inFieldRef.getSize(1) - 1 ?
            inFieldRef.getSize(1) - 1 : positions.at<ito::float64>(nPos, 0) + positions.at<ito::float64>(nPos, 2) / 2 + 1;
        int starty = positions.at<ito::float64>(nPos, 1) - positions.at<ito::float64>(nPos, 3) / 2 < 0 ?
            0 : positions.at<ito::float64>(nPos, 1) - positions.at<ito::float64>(nPos, 3) / 2;
        int endy = positions.at<ito::float64>(nPos, 1) + positions.at<ito::float64>(nPos, 3) / 2 + 1 >= inFieldRef.getSize(0) - 1 ?
            inFieldRef.getSize(0) - 1 : positions.at<ito::float64>(nPos, 1) + positions.at<ito::float64>(nPos, 3) / 2 + 1;
        cells[nPos] = cell(positions.at<ito::float64>(nPos, 0), positions.at<ito::float64>(nPos, 1), startx, endx, starty, endy);
        if (cellType == 0) // Q4
        {
            cells[nPos].nodes[0] = nPos / ncellsx + nPos;
            cells[nPos].nodes[1] = nPos / ncellsx + nPos + 1;
            cells[nPos].nodes[2] = nPos / ncellsx + nPos + (ncellsx + 1);
            cells[nPos].nodes[3] = nPos / ncellsx + nPos + (ncellsx + 1) + 1;
        }

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

        // normalize intensities of ZOEs
        ito::DataObject inPatchRef;
        // TODO: using shallow copies of reference image did not work, needs checking
        inFieldRef.at(ito::Range(starty, endy), ito::Range(startx, endx)).copyTo(cells[nPos].refInt);
        int newdim[2] = {(endy - starty) * (endx - startx), 1};

        if (guessType > 0)
        {
            int guessRangeArr[2] = { guessRange, guessRange };
            ito::float64 correlCoeff;
            if (guessType < 3)
            {
                retVal += normalizeInt(cells[nPos].refInt, cells[nPos].refInt);
                retVal += doCalcInitialGuess(&cells[nPos].refInt, inFieldDef, &cells[nPos], guessRangeArr, guessType, guessCenter, correlCoeff);
            }
            else if (guessType == 3)
            {
                retVal += doCalcInitialGuessFFT(&cells[nPos].refInt, inFieldDef, &cells[nPos], guessType, guessCenter, correlCoeff);
                retVal += normalizeInt(cells[nPos].refInt, cells[nPos].refInt);
            }
            else
            {
                float maxxy[2] = { 128.0, 128.0 };
                retVal += doCalcInitialGuessFFTMScale(&inFieldRef, inFieldDef, &cells[nPos], guessType, maxxy, 16, guessCenter, correlCoeff);
                retVal += normalizeInt(cells[nPos].refInt, cells[nPos].refInt);
            }
            if (retVal.containsWarningOrError())
                return retVal;

            // Q4
            if (cellType == 0)
            {
                for (int nn = 0; nn < numcoeff; nn++)
                {
                    // last or first line of nodes
                    if (cells[nPos].nodes[nn] < (ncellsx + 1) || cells[nPos].nodes[nn] >= ncellsy * (ncellsx + 1))
                    {
                        // first or last node in line
                        if (cells[nPos].nodes[nn] % (ncellsx + 1) == 0 || cells[nPos].nodes[nn] % (ncellsx + 1) == ncellsx)
                        {
                            distCoeffVec(cells[nPos].nodes[nn] * 2) = guessCenter[0];
                            distCoeffVec(cells[nPos].nodes[nn] * 2 + 1) = guessCenter[1];
//                            distCoeffPtr[cells[nPos].nodes[nn] * 2] = guessCenter[0];
//                            distCoeffPtr[cells[nPos].nodes[nn] * 2 + 1] = guessCenter[1];
                        }
                        else
                        {
                            distCoeffVec(cells[nPos].nodes[nn] * 2) += guessCenter[0] / 2.0;
                            distCoeffVec(cells[nPos].nodes[nn] * 2 + 1) += guessCenter[1] / 2.0;
//                            distCoeffPtr[cells[nPos].nodes[nn] * 2] += guessCenter[0] / 2.0;
//                            distCoeffPtr[cells[nPos].nodes[nn] * 2 + 1] += guessCenter[1] / 2.0;
                        }
                    }
                    else
                    {
                        // first or last node in line
                        if (cells[nPos].nodes[nn] % (ncellsx + 1) == 0 || cells[nPos].nodes[nn] % (ncellsx + 1) == ncellsx)
                        {
                            distCoeffVec(cells[nPos].nodes[nn] * 2) += guessCenter[0] / 2.0;
                            distCoeffVec(cells[nPos].nodes[nn] * 2 + 1) += guessCenter[1] / 2.0;
//                            distCoeffPtr[cells[nPos].nodes[nn] * 2] += guessCenter[0] / 2.0;
//                            distCoeffPtr[cells[nPos].nodes[nn] * 2 + 1] += guessCenter[1] / 2.0;
                        }
                        else
                        {
                            distCoeffVec(cells[nPos].nodes[nn] * 2) += guessCenter[0] / 4.0;
                            distCoeffVec(cells[nPos].nodes[nn] * 2 + 1) += guessCenter[1] / 4.0;
//                            distCoeffPtr[cells[nPos].nodes[nn] * 2] += guessCenter[0] / 4.0;
//                            distCoeffPtr[cells[nPos].nodes[nn] * 2 + 1] += guessCenter[1] / 4.0;
                        }
                    }
                }
            }
//            ((ito::float64*)distCoeffCell.data)[0] = guessCenter[0];
//            ((ito::float64*)distCoeffCell.data)[1] = guessCenter[1];
        }
        else
        {
            retVal += normalizeInt(cells[nPos].refInt, cells[nPos].refInt);
        }
        cells[nPos].refInt = cells[nPos].refInt.reshape(2, newdim);
    }

    ito::float64 correlCoeff;
    int sizey = inFieldRef.getSize(0), sizex = inFieldRef.getSize(1);

    doCalcSMatRVecFF(&inFieldRef, inFieldDef, SMat, RVec, &cells, &distCoeffVec, &correlCoeff, (ncellsx + 1) * (ncellsy + 1), flags);
/*
    ito::DataObject *tmp1 = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *tmp2 = paramsMand->at(1).getVal<ito::DataObject*>();
    *tmp1 = SMat;
    *tmp2 = RVec;
    return ito::retOk;
*/
    ito::float64 mu = 0;
/*
    for (int ni = 0; ni < SMat.getSize(0); ni++)
    {
        if (SMat.at<ito::float64>(ni, ni) > mu)
            mu = SMat.at<ito::float64>(ni, ni);
    }
*/
    for (int ni = 0; ni < SMat.rows(); ni++)
    {
        if (SMat.coeffRef(ni, ni) > mu)
            mu = SMat.coeffRef(ni, ni);
    }

    ito::float64 correlCoeffNew;
//    ito::DataObject imat;
//    imat.eye(SMat.getSize(0), ito::tFloat64);
    bool convergence = 0;
    int niter = 0;
    ito::float64 damping = 2.0;
//    cv::Mat dCoeff, disCoeffNew;
    do
    {
        for (int nc = 0; nc < SMat.rows(); nc++)
        {
            SMat.coeffRef(nc, nc) += mu;
        }
        //SMat += imat * mu;
        // maybe revise calculation method used for matrix inversion
//        cv::Mat SMatInv = ((cv::Mat*)SMat.get_mdata()[0])->inv(cv::DECOMP_CHOLESKY);
//        dCoeff = SMatInv * (*(cv::Mat*)RVec.get_mdata()[0]) * -1.0;

        // fill b
        // solve Ax = b
        Eigen::SparseLU<Eigen::SparseMatrix<double> > solver;
        solver.compute(SMat);
        /*
        if (solver.info() != Success) {
            // decomposition failed
            return;
        }
        */
        dCoeffVec = solver.solve(RVec * -1.0);

        if (dCoeffVec.array().abs().sum() < threshold)
//        if ((*cv::sum(cv::abs(dCoeff)).val) < threshold)
        {
            convergence = 1;
        }
        else
        {
//            distCoeff->get_mdata()[0]->copyTo(disCoeffNew);
//            disCoeffNew += dCoeff;
            distCoeffVecNew = distCoeffVec + dCoeffVec;
            // iterating over all cells
            correlCoeffNew = 0;
            for (int nc = 0; nc < cells.length(); nc++)
            {
                ito::float64 tmpIntSqrSum, correlCoeffTmp;
                int csy = cells[nc].y1 - cells[nc].y0;
                int csx = cells[nc].x1 - cells[nc].x0;
                ito::DataObject intInterpol(csy * csx, 3, ito::tFloat64);
                ito::DataObject intDiff(csy, csx, ito::tFloat64);
                ito::DataObject coeffsDer(csy * csx, numcoeff * 2, ito::tFloat64);

                doCalcIntDiffFF(inFieldDef, &intDiff, &intInterpol, &coeffsDer,
                    &cells[nc], &distCoeffVecNew, flags, 2, numcoeff, &tmpIntSqrSum);

/*
                ito::DataObject *tmp1 = paramsMand->at(0).getVal<ito::DataObject*>();
                ito::DataObject *tmp2 = paramsMand->at(1).getVal<ito::DataObject*>();
                *tmp1 = intDiff;
//                *tmp2 = RVec;
                return ito::retOk;
*/
                intDiff = intDiff.mul(intDiff);

                retVal += ito::dObjHelper::meanValue(&intDiff, correlCoeffTmp, 0);
                correlCoeffTmp *= csy * csx;
                correlCoeffNew += correlCoeffTmp;
            }

//            cv::Mat glin = dCoeff.t() * (dCoeff * mu - (*(cv::Mat*)RVec.get_mdata()[0])) * 0.5;
//            cv::Mat glin = dCoeff.t() * (dCoeff * mu - RVec(0)) * 0.5;
            ito::float64 glin = dCoeffVec.dot(((dCoeffVec.array() * mu - RVec(0)) * 0.5).matrix());
            ito::float64 ro = (correlCoeff - correlCoeffNew) / glin;
            if (ro > 0)
            {
                //disCoeffNew.copyTo(*distCoeff->get_mdata()[0]);
                distCoeffVec = distCoeffVecNew;
                doCalcSMatRVecFF(&inFieldRef, inFieldDef, SMat, RVec, &cells, &distCoeffVec, &correlCoeff, (ncellsx + 1) * (ncellsy + 1), flags);

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

    // clean up cached AMat to free memory
    clearFilterCache();

    ito::DataObject distCoeffRes(distCoeff->getSize(0) / 2, 4, distCoeff->getType());
    ito::float64 *pDcr = (ito::float64*)distCoeffRes.rowPtr(0, 0);
    //ito::float64 *pScr = (ito::float64*)distCoeff->rowPtr(0, 0);
    ito::float64 *pScr = (ito::float64*)distCoeffVec.data();
    // blindly fill the node coordinates ... nodes get overwritten several times, but the strategy is simple ;-)
    for (int nc = 0; nc < cells.length(); nc++)
    {
        pDcr[cells[nc].nodes[0] * 4] = cells[nc].x0;
        pDcr[cells[nc].nodes[0] * 4 + 1] = cells[nc].y0;
        pDcr[cells[nc].nodes[0] * 4 + 2] = pScr[cells[nc].nodes[0] * 2];
        pDcr[cells[nc].nodes[0] * 4 + 3] = pScr[cells[nc].nodes[0] * 2 + 1];
        pDcr[cells[nc].nodes[1] * 4] = cells[nc].x1;
        pDcr[cells[nc].nodes[1] * 4 + 1] = cells[nc].y0;
        pDcr[cells[nc].nodes[1] * 4 + 2] = pScr[cells[nc].nodes[1] * 2];
        pDcr[cells[nc].nodes[1] * 4 + 3] = pScr[cells[nc].nodes[1] * 2 + 1];
        pDcr[cells[nc].nodes[2] * 4] = cells[nc].x0;
        pDcr[cells[nc].nodes[2] * 4 + 1] = cells[nc].y1;
        pDcr[cells[nc].nodes[2] * 4 + 2] = pScr[cells[nc].nodes[2] * 2];
        pDcr[cells[nc].nodes[2] * 4 + 3] = pScr[cells[nc].nodes[2] * 2 + 1];
        pDcr[cells[nc].nodes[3] * 4] = cells[nc].x1;
        pDcr[cells[nc].nodes[3] * 4 + 1] = cells[nc].y1;
        pDcr[cells[nc].nodes[3] * 4 + 2] = pScr[cells[nc].nodes[3] * 2];
        pDcr[cells[nc].nodes[3] * 4 + 3] = pScr[cells[nc].nodes[3] * 2 + 1];
    }
    *distCoeff = distCoeffRes;
    /*
    ito::float64 correlCoeff;
    ito::DataObject intensDiff(endy - starty, endx - startx, ito::tFloat64);
    int sizey = intensDiff.getSize(0), sizex = intensDiff.getSize(1);
    */

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
