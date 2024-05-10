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

extern int NTHREADS;

//----------------------------------------------------------------------------------------------------------------------------------
const QString DIC::DICDeformationDoc = QObject::tr("calculate deformation from displacement");

ito::RetVal DIC::DICDeformationParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::Param param;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("Displacement", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input data vector with displacement field"));
    paramsMand->append(ito::Param("NodesX", ito::ParamBase::Int | ito::ParamBase::In, 0, 10000, 10, "number of nodes in x-direction"));
    paramsMand->append(ito::Param("NodesY", ito::ParamBase::Int | ito::ParamBase::In, 0, 10000, 10, "number of nodes in y-direction"));
    paramsMand->append(ito::Param("Deformation", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "2d output vector with calculated deformations"));

    paramsOpt->append(ito::Param("algorithm", ito::ParamBase::Int | ito::ParamBase::In, 0, 5, 0, "maybe we have different algos in future"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DIC::DICDeformation(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);

    ito::DataObject *defCoeffs = paramsMand->at(0).getVal<ito::DataObject*>();
    int sizex = paramsMand->at(1).getVal<int>();
    int sizey = paramsMand->at(2).getVal<int>();
    ito::DataObject *defField = paramsMand->at(3).getVal<ito::DataObject*>();
    int rowWise = 1;

    if (defCoeffs->getSize(0) != sizey * sizex || defCoeffs->getSize(1) < 2 &&
        defCoeffs->getSize(1) != sizey * sizex || defCoeffs->getSize(0) < 2)
        return ito::RetVal(ito::retError, 0, tr("Input data object size mismatch (sizex * sizey x 2 || 2 x sizex * sizey").toLatin1().data());

    if (defCoeffs->getSize(1) < defCoeffs->getSize(0))
        rowWise = 0;

    /*
    // osize test code
    ito::DataObject d1, d2;
    int sizes[2] = { 9, 11 };
    d1.rand(99, 4, ito::tFloat64);
    d1.at(ito::Range::all(), ito::Range(0, 1)).copyTo(d2, 1);
    d2 = d2.reshape(2, sizes);
    d2 = d2.at(ito::Range(0, 1), ito::Range::all());
    d2 = d2.trans();
    */

    ito::DataObject xvals, yvals, dx, dy, posx, posy;
    int nSizes[2] = { sizey, sizex };
    if (sizex < 4 || sizey < 4)
        return ito::RetVal(ito::retError, 0, tr("Currently only spline interpolation implemented, requiring at least 3 x 3 cells.").toLatin1().data());


    if (rowWise)
    {
        defCoeffs->at(ito::Range(2, 3), ito::Range::all()).copyTo(dx, 1);
        defCoeffs->at(ito::Range(3, 4), ito::Range::all()).copyTo(dy, 1);
        defCoeffs->at(ito::Range(0, 1), ito::Range::all()).copyTo(posx, 1);
        defCoeffs->at(ito::Range(1, 2), ito::Range::all()).copyTo(posy, 1);
        posx = posx.reshape(2, nSizes);
        posx = posx.at(ito::Range(0, 1), ito::Range::all());
        posx = posx.trans();
        posy = posy.reshape(2, nSizes);
        posy = posy.at(ito::Range::all(), ito::Range(0, 1));
    }
    else
    {
        defCoeffs->at(ito::Range::all(), ito::Range(2, 3)).copyTo(dx, 1);
        defCoeffs->at(ito::Range::all(), ito::Range(3, 4)).copyTo(dy, 1);
        defCoeffs->at(ito::Range::all(), ito::Range(0, 1)).copyTo(posx, 1);
        defCoeffs->at(ito::Range::all(), ito::Range(1, 2)).copyTo(posy, 1);
        posx = posx.reshape(2, nSizes);
        posx = posx.at(ito::Range(0, 1), ito::Range::all());
        posx = posx.trans();
        posy = posy.reshape(2, nSizes);
        posy = posy.at(ito::Range::all(), ito::Range(0, 1));
    }

    dx = dx.reshape(2, nSizes);
    dy = dy.reshape(2, nSizes);

    cv::Mat *dxMat = dx.get_mdata()[0];
    cv::Mat *dyMat = dy.get_mdata()[0];
    cv::Mat ux, vx, wx;
    dxMat->convertTo(*dxMat, CV_64F);
    cv::SVDecomp(*dxMat, wx, ux, vx);

    int numcoeff = 0;
    for (int nc = 0; nc < wx.rows; nc++)
    {
        if (wx.at<double>(nc) > 1)
            numcoeff++;
        else
            break;
    }

    if (numcoeff == 0 && fabs(wx.at<double>(0)) < 1e-15)
        return ito::RetVal(ito::retError, 0, tr("No principal component for analysis found, aborting!").toLatin1().data());
    else if (numcoeff == 0 && wx.at<double>(0) != 0)
        numcoeff = 1;

    cv::Mat xdefVals = *dxMat * vx(cv::Range(0, numcoeff), cv::Range::all()).t();
    cv::Mat ydefVals = dxMat->t() * ux(cv::Range::all(), cv::Range(0, numcoeff));

    ito::DataObject inpObj;
    ito::DataObject outpObj, interpVals;

    QVector<ito::ParamBase> fParamsMand, fParamsOpt, fParamsOut;
    fParamsMand.append(ito::Param("dObjIn", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input vector with data points"));
    fParamsMand.append(ito::Param("dObjOut", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output vector with calculated spline coefficients"));
    fParamsMand.append(ito::Param("dObjOut", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output vector with interpolated values"));
    fParamsOpt.append(ito::Param("algorithm", ito::ParamBase::Int | ito::ParamBase::In, 0, 5, 0, "maybe we have different algos in future"));
    fParamsOpt.append(ito::Param("derivatives", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, "calculate derivatives 1 or not 0"));

    ito::DataObject derVecXY(posy.getSize(0), numcoeff, ito::tFloat64);
    ito::DataObject derVecXX(posx.getSize(0), numcoeff, ito::tFloat64);
    for (int nc = 0; nc < numcoeff; nc++)
    {
        ito::DataObject dObj, dObj2;
        inpObj = ito::DataObject(sizey, 2, ito::tFloat64);
        dObj = inpObj.at(ito::Range::all(), ito::Range(0, 1));
        retVal += posy.copyTo(dObj, 1);
        cv::Mat tmpMat = ux(cv::Range::all(), cv::Range(0, numcoeff));
        ito::DataObject tmpObj(tmpMat);
        dObj = inpObj.at(ito::Range::all(), ito::Range(1, 2));
        retVal += tmpObj.copyTo(dObj, 1);
        fParamsMand[0].setVal<void*>(&inpObj);
        fParamsMand[1].setVal<void*>(&outpObj);
        retVal += DICSplineCoeffs(&fParamsMand, &fParamsOpt, &fParamsOut);

        fParamsMand[0].setVal<void*>(&outpObj);
        fParamsMand[1].setVal<void*>(&posy);
        fParamsMand[2].setVal<void*>(&interpVals);
        retVal += DICSplineVals(&fParamsMand, &fParamsOpt, &fParamsOut);
        dObj = interpVals.at(ito::Range::all(), ito::Range(1, 2));
        dObj2 = derVecXY.at(ito::Range::all(), ito::Range(nc, nc + 1));
        retVal += dObj.copyTo(dObj2, 1);

        inpObj = ito::DataObject(sizex, 2, ito::tFloat64);
        dObj = inpObj.at(ito::Range::all(), ito::Range(0, 1));
        retVal += posx.copyTo(dObj, 1);
        tmpMat = vx(cv::Range(0, numcoeff), cv::Range::all()).t();
        tmpObj = ito::DataObject(tmpMat);
        dObj = inpObj.at(ito::Range::all(), ito::Range(1, 2));
        retVal += tmpObj.copyTo(dObj, 1);
        fParamsMand[0].setVal<void*>(&inpObj);
        fParamsMand[1].setVal<void*>(&outpObj);
        retVal += DICSplineCoeffs(&fParamsMand, &fParamsOpt, &fParamsOut);

        fParamsMand[0].setVal<void*>(&outpObj);
        fParamsMand[1].setVal<void*>(&posx);
        fParamsMand[2].setVal<void*>(&interpVals);
        retVal += DICSplineVals(&fParamsMand, &fParamsOpt, &fParamsOut);
        dObj = interpVals.at(ito::Range::all(), ito::Range(1, 2));
        dObj2 = derVecXX.at(ito::Range::all(), ito::Range(nc, nc + 1));
        retVal += dObj.copyTo(dObj2, 1);
    }
    ito::DataObject xxdefValsMat(xdefVals);
    ito::DataObject xydefValsMat(ydefVals.t());
    xxdefValsMat = xxdefValsMat * derVecXX.trans();
    xydefValsMat = derVecXY * xydefValsMat;

    cv::Mat uy, vy, wy;
    dyMat->convertTo(*dyMat, CV_64F);
    cv::SVDecomp(*dyMat, wy, uy, vy);

    numcoeff = 0;
    for (int nc = 0; nc < wy.rows; nc++)
    {
        if (wy.at<double>(nc) > 1)
            numcoeff++;
        else
            break;
    }

    if (numcoeff == 0 && fabs(wy.at<double>(0)) < 1e-15)
        return ito::RetVal(ito::retError, 0, tr("No principal component for analysis found, aborting!").toLatin1().data());
    else if (numcoeff == 0 && wy.at<double>(0) != 0)
        numcoeff = 1;

    xdefVals = *dyMat * vy(cv::Range(0, numcoeff), cv::Range::all()).t();
    ydefVals = dyMat->t() * uy(cv::Range::all(), cv::Range(0, numcoeff));

    ito::DataObject derVecYX(posx.getSize(0), numcoeff, ito::tFloat64);
    ito::DataObject derVecYY(posy.getSize(0), numcoeff, ito::tFloat64);
    for (int nc = 0; nc < numcoeff; nc++)
    {
        ito::DataObject dObj, dObj2;
        inpObj = ito::DataObject(sizey, 2, ito::tFloat64);
        dObj = inpObj.at(ito::Range::all(), ito::Range(0, 1));
        retVal += posy.copyTo(dObj, 1);
        cv::Mat tmpMat = uy(cv::Range::all(), cv::Range(0, numcoeff));
        ito::DataObject tmpObj(tmpMat);
        dObj = inpObj.at(ito::Range::all(), ito::Range(1, 2));
        retVal += tmpObj.copyTo(dObj, 1);
        fParamsMand[0].setVal<void*>(&inpObj);
        fParamsMand[1].setVal<void*>(&outpObj);
        retVal += DICSplineCoeffs(&fParamsMand, &fParamsOpt, &fParamsOut);
        fParamsMand[0].setVal<void*>(&outpObj);
        fParamsMand[1].setVal<void*>(&posy);
        fParamsMand[2].setVal<void*>(&interpVals);

        retVal += DICSplineVals(&fParamsMand, &fParamsOpt, &fParamsOut);
        dObj = interpVals.at(ito::Range::all(), ito::Range(1, 2));
        dObj2 = derVecYY.at(ito::Range::all(), ito::Range(nc, nc + 1));
        retVal += dObj.copyTo(dObj2, 1);

        inpObj = ito::DataObject(sizex, 2, ito::tFloat64);
        dObj = inpObj.at(ito::Range::all(), ito::Range(0, 1));
        retVal += posx.copyTo(dObj, 1);
        tmpMat = vy(cv::Range(0, numcoeff), cv::Range::all()).t();
        tmpObj = ito::DataObject(tmpMat);
        dObj = inpObj.at(ito::Range::all(), ito::Range(1, 2));
        retVal += tmpObj.copyTo(dObj, 1);
        fParamsMand[0].setVal<void*>(&inpObj);
        fParamsMand[1].setVal<void*>(&outpObj);
        retVal += DICSplineCoeffs(&fParamsMand, &fParamsOpt, &fParamsOut);
        fParamsMand[0].setVal<void*>(&outpObj);
        fParamsMand[1].setVal<void*>(&posx);
        fParamsMand[2].setVal<void*>(&interpVals);

        retVal += DICSplineVals(&fParamsMand, &fParamsOpt, &fParamsOut);
        dObj = interpVals.at(ito::Range::all(), ito::Range(1, 2));
        dObj2 = derVecYX.at(ito::Range::all(), ito::Range(nc, nc + 1));
        retVal += dObj.copyTo(dObj2, 1);
    }
    ito::DataObject yxdefValsMat(xdefVals);
    ito::DataObject yydefValsMat(ydefVals.t());
    yxdefValsMat = yxdefValsMat * derVecYX.trans();
    yydefValsMat = derVecYY * yydefValsMat;

    ito::DataObject *deform = paramsMand->at(3).getVal<ito::DataObject*>();
    //xydefValsMat = xydefValsMat * 0.5 + yxdefValsMat * 0.5;
    nSizes[0] = sizex * sizey;
    nSizes[1] = 1;
    xxdefValsMat = xxdefValsMat.reshape(2, nSizes);
    yydefValsMat = yydefValsMat.reshape(2, nSizes);
    xydefValsMat = xydefValsMat.reshape(2, nSizes);
    yxdefValsMat = yxdefValsMat.reshape(2, nSizes);
    *deform = ito::DataObject(xxdefValsMat.getSize(0), 3, ito::tFloat64);
    ito::DataObject dObj;
    dObj = deform->at(ito::Range::all(), ito::Range(0, 1));
    retVal += xxdefValsMat.copyTo(dObj, 1);
    dObj = deform->at(ito::Range::all(), ito::Range(1, 2));
    retVal += yydefValsMat.copyTo(dObj, 1);
    dObj = deform->at(ito::Range::all(), ito::Range(2, 3));
    ito::DataObject tempVal = (xydefValsMat + yxdefValsMat) * 0.5;
    retVal += tempVal.copyTo(dObj, 1);

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
