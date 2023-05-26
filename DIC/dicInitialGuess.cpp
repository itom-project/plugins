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
#include <qvector.h>

#include "common/param.h"
#include "common/apiFunctionsInc.h"
#include "DataObject/dataObjectFuncs.h"
#include "dic.h"

extern int NTHREADS;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal doCalcInitialGuessFFTMScale(ito::DataObject *imgInRef, ito::DataObject *imgInDef, cell *thisCell, const int guessType, const float maxxy[2], const float minsize, ito::float64 estCenter[2], ito::float64 &correlCoeffRet)
{
    ito::RetVal retval(ito::retOk);

    QVector<ito::ParamBase> paramsMand, paramsOpt, paramsOut;
    if (ito::ITOM_API_FUNCS)
        retval = apiFilterParamBase("fftw2D", &paramsMand, &paramsOpt, &paramsOut);
    else
        retval += ito::RetVal(ito::retError, 0, "Api function pointer not initialized");
    if (retval.containsWarningOrError())
        return retval;

    int maxx = maxxy[0];
    int maxy = maxxy[1];
    int minSize = minsize;

    int sizex = abs(thisCell->x1 - thisCell->x0) < 16 ? 16 : abs(thisCell->x1 - thisCell->x0);
    int sizey = abs(thisCell->y1 - thisCell->y0) < 16 ? 16 : abs(thisCell->y1 - thisCell->y0);
    int imgsx = imgInDef->getSize(1);
    int imgsy = imgInDef->getSize(0);

    int maxpx = ceil(log(maxx) / log(2.0));
    int maxpy = ceil(log(maxy) / log(2.0));
    std::vector<int> sizesx, sizesy;

    if (ceil(log(sizex) / log(2.0)) != log(sizex) / log(2.0))
        sizesx.push_back(sizex);
    for (int n = ceil(log(sizex) / log(2.0)); n <= maxpx; n++)
    {
        if (thisCell->cx - pow(2.0, n) / 2.0 >= 0 && thisCell->cx + pow(2.0, n) / 2.0 < imgsx)
            sizesx.push_back(pow(2.0, n));
    }

    if (ceil(log(sizey) / log(2.0)) != log(sizey) / log(2.0))
        sizesy.push_back(sizey);
    for (int n = ceil(log(sizey) / log(2.0)); n <= maxpy; n++)
    {
        if (thisCell->cy - pow(2.0, n) / 2.0 >= 0 && thisCell->cy + pow(2.0, n) / 2.0 < imgsy)
            sizesy.push_back(pow(2.0, n));
    }

    int nruns = std::max(sizesx.size(), sizesy.size());
    std::vector<float> cccoeffs;

    cccoeffs.resize(nruns * 4);
    float meanx = 0, meany = 0;
    for (int nr = 0; nr < nruns; nr++)
    {
        int idxx = std::min<int>(nr, sizesx.size());
        int idxy = std::min<int>(nr, sizesy.size());
        int x0 = ceil(thisCell->cx - sizesx[idxx] / 2.0);
        int x1 = ceil(thisCell->cx + sizesx[idxx] / 2.0);
        int y0 = ceil(thisCell->cy - sizesy[idxy] / 2.0);
        int y1 = ceil(thisCell->cy + sizesy[idxy] / 2.0);

        ito::DataObject imgDef, imgRef;
        imgInDef->at(ito::Range(y0, y1), ito::Range(x0, x1)).copyTo(imgDef);
        imgInRef->at(ito::Range(y0, y1), ito::Range(x0, x1)).copyTo(imgRef);

        paramsMand[0].setVal<void*>(&imgRef);
        paramsMand[1].setVal<void*>(&imgRef);
        paramsOpt[1].setVal<const char*>("no");
        retval = apiFilterCall("fftw2D", &paramsMand, &paramsOpt, &paramsOut);
        if (retval.containsWarningOrError())
            return retval;

        paramsMand[0].setVal<void*>(&imgDef); // image stack
        paramsMand[1].setVal<void*>(&imgDef); // MaxIntPos, need to recalculate
        retval = apiFilterCall("fftw2D", &paramsMand, &paramsOpt, &paramsOut);
        if (retval.containsWarningOrError())
            return retval;

        imgDef.conj();
        imgRef = imgRef.mul(imgDef);

        paramsMand[0].setVal<void*>(&imgRef); // image stack
        paramsMand[1].setVal<void*>(&imgRef); // MaxIntPos, need to recalculate
        retval = apiFilterCall("ifftw2D", &paramsMand, &paramsOpt, &paramsOut);
        if (retval.containsWarningOrError())
            return retval;

        ito::uint32 position[3];
        ito::float64 maxVal;
        cv::Mat *values = imgRef.getCvPlaneMat(0);
        *values = cv::abs(*values);
        cv::Scalar fsum = cv::sum(*values);
        ito::dObjHelper::maxValue(&imgRef, maxVal, position);

        if (position[1] > ((x1 - x0) / 2.0))
            //estCenter[0] = -1.0 * (position[1] - (thisCell->x1 - thisCell->x0));
            cccoeffs[nr * 4 + 1] = -1.0 * ((int)position[1] - (x1 - x0));
        else
            //estCenter[0] = -1.0 * position[1];
            cccoeffs[nr * 4 + 1] = -1.0 * (int)position[1];

        if (position[2] > ((y1 - y0) / 2.0))
            //estCenter[1] = -1.0 * (position[2] - (thisCell->y1 - thisCell->y0));
            cccoeffs[nr * 4 + 2] = -1.0 * ((int)position[2] - (y1 - y0));
        else
            //estCenter[1] = -1.0 * position[2];
            cccoeffs[nr * 4 + 2] = -1.0 * (int)position[2];

        //correlCoeffRet = sqrt(maxVal);
        float divisor = fsum[0] / (imgRef.getSize(0) * imgRef.getSize(1));
        if (divisor != 0)
            cccoeffs[nr * 4] = maxVal / divisor;
        else
            cccoeffs[nr * 4] = 0;
        meanx += cccoeffs[nr * 4 + 1] / nruns;
        meany += cccoeffs[nr * 4 + 2] / nruns;
        cccoeffs[nr * 4 + 3] = maxVal;
    }

    int bestidx = -1;
    float maxcorr = 0;
    for (int nr = 0; nr < nruns; nr++)
    {
        if (fabs(cccoeffs[nr * 4 + 1] - meanx) > fabs(meanx * 0.1)
            || fabs(cccoeffs[nr * 4 + 2] - meany) > fabs(meany * 0.1))
            cccoeffs[nr * 4] = 0;
        else if (cccoeffs[nr * 4] > maxcorr)
        {
            maxcorr = cccoeffs[nr * 3];
            bestidx = nr;
        }
    }

    // emergency check for few diverging elements
    if (bestidx < 0)
        bestidx = 0;

    estCenter[0] = cccoeffs[bestidx * 4 + 1];
    estCenter[1] = cccoeffs[bestidx * 4 + 2];
    correlCoeffRet = sqrt(cccoeffs[bestidx * 4 + 3]);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal doCalcInitialGuessFFT(ito::DataObject *imgInRef, ito::DataObject *imgInDef, cell *thisCell, const int guessType, ito::float64 estCenter[2], ito::float64 &correlCoeffRet)
{
    ito::RetVal retval(ito::retOk);

    QVector<ito::ParamBase> paramsMand, paramsOpt, paramsOut;
    if (ito::ITOM_API_FUNCS)
        retval = apiFilterParamBase("fftw2D", &paramsMand, &paramsOpt, &paramsOut);
    else
        retval += ito::RetVal(ito::retError, 0, "Api function pointer not initialized");
    if (retval.containsWarningOrError())
        return retval;

    ito::DataObject imgDef, imgRef;
    imgInDef->at(ito::Range(thisCell->y0, thisCell->y1), ito::Range(thisCell->x0, thisCell->x1)).copyTo(imgDef);
    imgInRef->copyTo(imgRef);
//    retval += normalizeInt(imgDef, imgDef);

    paramsMand[0].setVal<void*>(&imgRef);
    paramsMand[1].setVal<void*>(&imgRef);
    paramsOpt[1].setVal<const char*>("no");
    retval = apiFilterCall("fftw2D", &paramsMand, &paramsOpt, &paramsOut);
    if (retval.containsWarningOrError())
        return retval;
//    imgRef = (*(DataObject*)paramsMand[1].getVal<void*>());

    paramsMand[0].setVal<void*>(&imgDef); // image stack
    paramsMand[1].setVal<void*>(&imgDef); // MaxIntPos, need to recalculate
    retval = apiFilterCall("fftw2D", &paramsMand, &paramsOpt, &paramsOut);
    if (retval.containsWarningOrError())
        return retval;
//    imgDef = (*((DataObject*)(paramsMand[1].getVal<void*>())));

    imgDef.conj();
    imgRef = imgRef.mul(imgDef);

    paramsMand[0].setVal<void*>(&imgRef); // image stack
    paramsMand[1].setVal<void*>(&imgRef); // MaxIntPos, need to recalculate
    retval = apiFilterCall("ifftw2D", &paramsMand, &paramsOpt, &paramsOut);
    if (retval.containsWarningOrError())
        return retval;
//    imgRef = (*(DataObject*)paramsMand[1].getVal<void*>());

    ito::uint32 position[3];
    ito::float64 maxVal;
    ito::dObjHelper::maxValue(&imgRef, maxVal, position);

    /*
    if (position[1] > ((thisCell->x1 - thisCell->x0) / 2.0))
        estCenter[0] = thisCell->cx - (position[1] - (thisCell->x1 - thisCell->x0));
    else
        estCenter[0] = thisCell->cx - position[1];

    if (position[2] > ((thisCell->y1 - thisCell->y0) / 2.0))
        estCenter[1] = thisCell->cy - (position[2] - (thisCell->y1 - thisCell->y0));
    else
        estCenter[1] = thisCell->cy - position[2];
    */
    if (position[1] > ((thisCell->x1 - thisCell->x0) / 2.0))
        estCenter[0] = -1.0 * (position[1] - (thisCell->x1 - thisCell->x0));
    else
        estCenter[0] = -1.0 * position[1];

    if (position[2] > ((thisCell->y1 - thisCell->y0) / 2.0))
        estCenter[1] = -1.0 * (position[2] - (thisCell->y1 - thisCell->y0));
    else
        estCenter[1] = -1.0 * position[2];

    correlCoeffRet = sqrt(maxVal);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal doCalcInitialGuess(ito::DataObject *imgInRef, ito::DataObject *imgInDef, cell *thisCell, const int range[2], const int guessType, ito::float64 estCenter[2], ito::float64 &correlCoeffRet)
{
    ito::RetVal retval(ito::retOk);
    ito::DataObject testPatchDef;
    ito::float64 correlCoeff = 1.0e208, correlCoeffNew;
    ito::float64 cy = -100;
    ito::float64 cx = -100;
    int sizey = imgInRef->getSize(0), sizex = imgInRef->getSize(1);
    ito::DataObject correlCoeffMat(range[0] * 2 + 1, range[1] * 2 + 1, ito::tFloat64);

    // imgInRef is the already normalized reference patch
    for (int n = 0; n <= abs(range[1]); n++)
    {
        int starty = thisCell->y0 + n;
        int endy = thisCell->y1 + n;
        for (int m = 0; m <= abs(range[0]); m++)
        {
            int startx = thisCell->x0 + m;
            int endx = thisCell->x1 + m;
            imgInDef->at(ito::Range(starty, endy), ito::Range(startx, endx)).copyTo(testPatchDef);
            retval += normalizeInt(testPatchDef, testPatchDef);
            ito::DataObject intDiff = testPatchDef - *imgInRef;
            intDiff = intDiff.mul(intDiff);

            retval += ito::dObjHelper::meanValue(&intDiff, correlCoeffNew, 0);
            correlCoeffNew *= sizex * sizey;
            if (correlCoeffNew < correlCoeff)
            {
                cy = n;
                cx = m;
                correlCoeff = correlCoeffNew;
            }
            ((ito::float64*)correlCoeffMat.rowPtr(0, n + abs(range[1])))[m + abs(range[0])] = correlCoeffNew;

            if (m > 0)
            {
                ((ito::float64*)correlCoeffMat.rowPtr(0, n + abs(range[1])))[-m + abs(range[0])] = correlCoeffNew;
                startx = thisCell->x0 - m;
                endx = thisCell->x1 - m;
                imgInDef->at(ito::Range(starty, endy), ito::Range(startx, endx)).copyTo(testPatchDef);
                retval += normalizeInt(testPatchDef, testPatchDef);
                ito::DataObject intDiff = testPatchDef - *imgInRef;
                intDiff = intDiff.mul(intDiff);

                retval += ito::dObjHelper::meanValue(&intDiff, correlCoeffNew, 0);
                correlCoeffNew *= sizex * sizey;
                if (correlCoeffNew < correlCoeff)
                {
                    cy = n;
                    cx = -m;
                    correlCoeff = correlCoeffNew;
                }
                ((ito::float64*)correlCoeffMat.rowPtr(0, n + abs(range[1])))[-m + abs(range[0])] = correlCoeffNew;
            }
        }
        if (n > 0)
        {
            starty = thisCell->y0 - n;
            endy = thisCell->y1 - n;
            for (int m = 0; m <= abs(range[0]); m++)
            {
                int startx = thisCell->x0 + m;
                int endx = thisCell->x1 + m;
                imgInDef->at(ito::Range(starty, endy), ito::Range(startx, endx)).copyTo(testPatchDef);
                retval += normalizeInt(testPatchDef, testPatchDef);
                ito::DataObject intDiff = testPatchDef - *imgInRef;
                intDiff = intDiff.mul(intDiff);

                retval += ito::dObjHelper::meanValue(&intDiff, correlCoeffNew, 0);
                correlCoeffNew *= sizex * sizey;
                if (correlCoeffNew < correlCoeff)
                {
                    cy = -n;
                    cx = m;
                    correlCoeff = correlCoeffNew;
                }
                ((ito::float64*)correlCoeffMat.rowPtr(0, -n + abs(range[1])))[m + abs(range[0])] = correlCoeffNew;

                if (m > 0)
                {
                    startx = thisCell->x0 - m;
                    endx = thisCell->x1 - m;
                    imgInDef->at(ito::Range(starty, endy), ito::Range(startx, endx)).copyTo(testPatchDef);
                    retval += normalizeInt(testPatchDef, testPatchDef);
                    ito::DataObject intDiff = testPatchDef - *imgInRef;
                    intDiff = intDiff.mul(intDiff);

                    retval += ito::dObjHelper::meanValue(&intDiff, correlCoeffNew, 0);
                    correlCoeffNew *= sizex * sizey;
                    if (correlCoeffNew < correlCoeff)
                    {
                        cy = -n;
                        cx = -m;
                        correlCoeff = correlCoeffNew;
                    }
                    ((ito::float64*)correlCoeffMat.rowPtr(0, -n + abs(range[1])))[-m + abs(range[0])] = correlCoeffNew;
                }
            }
        }
    }

    if (guessType == 2)
    {
        // detected position is on an edge, so this type of optimization will not work ... skip it
        if (cy - 1 + abs(range[1]) >= 0 && cx - 1 + abs(range[0]) >= 0 &&
            cy + 1 + abs(range[1]) < range[1] * 2 + 1 && cx + 1 + abs(range[0]) < range[0] * 2 + 1)
        {
            // TODO: review optimization of initial guess; Pan, B. 2006
            ito::DataObject AMat(6, 6, ito::tFloat64);
            ito::float64 *AMatPtr = (ito::float64*)AMat.rowPtr(0, 0);

            // A = [1  x      y - 1             x*x               x*(y - 1)        (y - 1)*(y - 1);
            //      1  x      y                 x*x               x*y              y*y;
            //      1  x      y + 1             x*x               x*(y + 1)        (y + 1)*(y + 1);
            //      1  x - 1  y                 y(x - 1)*(x - 1)  (x - 1)*(y)      y*y;
            //      1  x + 1  y                 y(x + 1)*(x + 1)  (x + 1)*y        y*y
            //      1  x - 1  y - 1             (x - 1)*(x - 1)   (x - 1)*(y - 1)  (y - 1)*(y - 1)];

            AMatPtr[0] = 1.0;  AMatPtr[1] = cx;      AMatPtr[2] = cy - 1;  AMatPtr[3] = cx * cx;              AMatPtr[4] = cx * (cy - 1);        AMatPtr[5] = (cy - 1) * (cy - 1);
            AMatPtr[6] = 1.0;  AMatPtr[7] = cx;      AMatPtr[8] = cy;      AMatPtr[9] = cx * cx;              AMatPtr[10] = cx * cy;             AMatPtr[11] = cy * cy;
            AMatPtr[12] = 1.0; AMatPtr[13] = cx;     AMatPtr[14] = cy + 1; AMatPtr[15] = cx * cx;             AMatPtr[16] = cx * (cy + 1);       AMatPtr[17] = (cy + 1) * (cy + 1);
            AMatPtr[18] = 1.0; AMatPtr[19] = cx - 1; AMatPtr[20] = cy;     AMatPtr[21] = (cx - 1) * (cx - 1); AMatPtr[22] = (cx - 1) * cy;       AMatPtr[23] = cy * cy;
            AMatPtr[24] = 1.0; AMatPtr[25] = cx + 1; AMatPtr[26] = cy;     AMatPtr[27] = (cx + 1) * (cx + 1); AMatPtr[28] = (cx + 1) * cy;       AMatPtr[29] = cy * cy;
            AMatPtr[30] = 1.0; AMatPtr[31] = cx - 1; AMatPtr[32] = cy - 1; AMatPtr[33] = (cx - 1) * (cx - 1); AMatPtr[34] = (cx - 1) * (cy - 1); AMatPtr[35] = (cy - 1) * (cy - 1);

            ito::DataObject BVec(6, 1, ito::tFloat64);
            ito::float64 *BVecPtr = (ito::float64*)BVec.rowPtr(0, 0);

            BVecPtr[0] = correlCoeffMat.at<ito::float64>(cy - 1 + abs(range[1]), cx + abs(range[0]));
            BVecPtr[1] = correlCoeffMat.at<ito::float64>(cy + abs(range[1]), cx + abs(range[0]));
            BVecPtr[2] = correlCoeffMat.at<ito::float64>(cy + 1 + abs(range[1]), cx + abs(range[0]));
            BVecPtr[3] = correlCoeffMat.at<ito::float64>(cy + abs(range[1]), cx - 1 + abs(range[0]));
            BVecPtr[4] = correlCoeffMat.at<ito::float64>(cy + abs(range[1]), cx + 1 + abs(range[0]));
            BVecPtr[5] = correlCoeffMat.at<ito::float64>(cy - 1 + abs(range[1]), cx - 1 + abs(range[0]));

            cv::Mat AMatInv = ((cv::Mat*)AMat.get_mdata()[0])->inv(cv::DECOMP_SVD);
            cv::Mat coeffs = AMatInv * (*(cv::Mat*)BVec.get_mdata()[0]);

            // x
            estCenter[0] = (2.0 * coeffs.at<ito::float64>(1) * coeffs.at<ito::float64>(5) - coeffs.at<ito::float64>(2) * coeffs.at<ito::float64>(4))
                / (coeffs.at<ito::float64>(4) * coeffs.at<ito::float64>(4) - 4.0 * coeffs.at<ito::float64>(3) * coeffs.at<ito::float64>(5));
            // y
            estCenter[1] = (2.0 * coeffs.at<ito::float64>(2) * coeffs.at<ito::float64>(3) - coeffs.at<ito::float64>(1) * coeffs.at<ito::float64>(4))
                / (coeffs.at<ito::float64>(4) * coeffs.at<ito::float64>(4) - 4.0 * coeffs.at<ito::float64>(3) * coeffs.at<ito::float64>(5));

            cv::Mat ccVec(1, 6, CV_64F);
            ((ito::float64*)ccVec.data)[0] = 1.0;
            ((ito::float64*)ccVec.data)[1] = estCenter[0];
            ((ito::float64*)ccVec.data)[2] = estCenter[1];
            ((ito::float64*)ccVec.data)[3] = estCenter[0] * estCenter[0];
            ((ito::float64*)ccVec.data)[4] = estCenter[0] * estCenter[1];
            ((ito::float64*)ccVec.data)[5] = estCenter[1] * estCenter[1];

            cv::Mat res = ccVec * coeffs;
            // why can we have negative values here ....
            correlCoeffRet = fabs(res.at<ito::float64>(0));
        }
        else
        {
            correlCoeffRet = correlCoeff + 1;
        }

        // seems "optimization" of initial guess was not a good idea, so revert it
        if (correlCoeffRet > correlCoeff)
        {
            correlCoeffRet = correlCoeff;
            estCenter[0] = cx;
            estCenter[1] = cy;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString DIC::DICInitialGuessDoc = QObject::tr("calculate initial guess of deformation with pixel accuracy");

ito::RetVal DIC::DICInitialGuessParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::Param param;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("dObjInRef", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d input reference data field"));
    paramsMand->append(ito::Param("dObjInDef", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d output data field"));
    paramsMand->append(ito::Param("PosXY", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "displacement positions and subfield sizes"));
    paramsMand->append(ito::Param("correlCoeff", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output vector with correlation coefficients for each subfield"));

    paramsOpt->append(ito::Param("initialGuessType", ito::ParamBase::Int | ito::ParamBase::In, 0, 4, 2, "initial guess of displacement field: 0 fill with zeros, 1 use coarse fine search, 2 use coarse fine search with refinement, 3 multi-scale fft"));
    paramsOpt->append(ito::Param("initialGuessRange", ito::ParamBase::Int | ito::ParamBase::In, 0, 10, 5, "range of initial guess search"));
    paramsOpt->append(ito::Param("maxSizeX", ito::ParamBase::Int | ito::ParamBase::In, 0, 16384, 128, "maximum region size in x-direction"));
    paramsOpt->append(ito::Param("maxSizeY", ito::ParamBase::Int | ito::ParamBase::In, 0, 16384, 128, "maximum region size in y-direction"));
    paramsOpt->append(ito::Param("minSize", ito::ParamBase::Int | ito::ParamBase::In, 0, 16384, 16, "minimum region size"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DIC::DICInitialGuess(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);

    ito::DataObject *inFieldRef = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *inFieldDef = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::DataObject *positionsPtr = paramsMand->at(2).getVal<ito::DataObject*>();
    ito::DataObject *distCoeff = paramsMand->at(3).getVal<ito::DataObject*>();

    int guessType = paramsOpt->at(0).getVal<int>();
    int guessRange = paramsOpt->at(1).getVal<int>();
    ito::float64 guessCenter[2] = { 0, 0 };

    if (inFieldRef->getDims() != 2 || inFieldDef->getDims() != 2
        || inFieldRef->getSize(0) != inFieldDef->getSize(0) || inFieldDef->getSize(0) == 0
        || inFieldRef->getSize(1) != inFieldDef->getSize(1) || inFieldDef->getSize(1) == 0)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Reference and deformed field must have the same size and 2 dimensions!").toLatin1().data());
        return retVal;
    }

    *inFieldRef = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)inFieldRef, "dObjInRef", ito::Range::all(), ito::Range::all(), retVal, ito::tFloat32, 0);
    if (retVal.containsError())
        return retVal;

    *inFieldDef = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)inFieldDef, "dObjInDef", ito::Range::all(), ito::Range::all(), retVal, ito::tFloat32, 0);
    if (retVal.containsError())
        return retVal;

    ito::DataObject positions = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)positionsPtr, "PosXY", ito::Range::all(), ito::Range(4, 4), retVal, ito::tFloat64, 0);
    if (retVal.containsError())
        return retVal;

    distCoeff->zeros(positions.getSize(0), 2, ito::tFloat64);

    for (int nPos = 0; nPos < positions.getSize(0); nPos++)
    {
        int startx = positions.at<ito::float64>(nPos, 0) - positions.at<ito::float64>(nPos, 2) / 2 < 0 ?
            0 : positions.at<ito::float64>(nPos, 0) - positions.at<ito::float64>(nPos, 2) / 2;
        int endx = positions.at<ito::float64>(nPos, 0) + positions.at<ito::float64>(nPos, 2) / 2 + 1 >= inFieldRef->getSize(1) - 1 ?
            inFieldRef->getSize(1) - 1 : positions.at<ito::float64>(nPos, 0) + positions.at<ito::float64>(nPos, 2) / 2 + 1;
        int starty = positions.at<ito::float64>(nPos, 1) - positions.at<ito::float64>(nPos, 3) / 2 < 0 ?
            0 : positions.at<ito::float64>(nPos, 1) - positions.at<ito::float64>(nPos, 3) / 2;
        int endy = positions.at<ito::float64>(nPos, 1) + positions.at<ito::float64>(nPos, 3) / 2 + 1 >= inFieldRef->getSize(0) - 1 ?
            inFieldRef->getSize(0) - 1 : positions.at<ito::float64>(nPos, 1) + positions.at<ito::float64>(nPos, 3) / 2 + 1;
        cell thisCell(positions.at<ito::float64>(nPos, 0), positions.at<ito::float64>(nPos, 1), startx, endx, starty, endy);

        if (startx < 0 || endx < 0 || startx >= inFieldRef->getSize(1) - 1 || endx >= inFieldRef->getSize(1) - 1
            || starty < 0 || endy < 0 || starty >= inFieldRef->getSize(0) - 1 || endy >= inFieldRef->getSize(0) - 1)
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
        ito::DataObject intensDiff(endy - starty, endx - startx, ito::tFloat64);
        int sizey = intensDiff.getSize(0), sizex = intensDiff.getSize(1);

        // extract reference patch and normalize its intensity values
        ito::DataObject inPatchRef;
        inFieldRef->at(ito::Range(starty, endy), ito::Range(startx, endx)).copyTo(inPatchRef);

        cv::Mat distCoeffCell(12, 1, CV_64F);
        memcpy(distCoeffCell.data, distCoeff->rowPtr(0, nPos), 12 * sizeof(ito::float64));

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
            }
            else
            {
                float maxxy[2];
                maxxy[0] = (*paramsOpt)[2].getVal<int>();
                maxxy[1] = (*paramsOpt)[3].getVal<int>();
                float minsize = (*paramsOpt)[4].getVal<int>();
                retVal += doCalcInitialGuessFFTMScale(&inPatchRef, inFieldDef, &thisCell, guessType, maxxy, minsize, guessCenter, correlCoeff);
            }

            if (retVal.containsWarningOrError())
                return retVal;
            ((ito::float64*)distCoeffCell.data)[0] = guessCenter[0];
            ((ito::float64*)distCoeffCell.data)[1] = guessCenter[1];
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
