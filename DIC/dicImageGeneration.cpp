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
#include "dic.h"
#include "dicPrivate.h"
#include "DataObject/dataObjectFuncs.h"
#include <qstring.h>

extern int NTHREADS;

//----------------------------------------------------------------------------------------------------------------------------------
// calculate "area weighted" intensity interpolation, i.e. sum up all intensities of the input's image pixels that fall into the new
// output's image pixel.
// We diference between two cases:
//      - 6 parameters given - we are dealing with a linear deformation, that means points' coordinates
//        can only vary along their carthesian coordinate axes, one at a time
//      - 10 parameters given - general deformation, 'bounding points' defining the bounding quadrangle can vary in
//        x and y axes together, resulting in skewed pixels
ito::RetVal doAreaAveragedInterpol(ito::DataObject *imgIn, ito::DataObject *imgOut, ito::DataObject *pts)
{
    ito::RetVal retval(ito::retOk);

    imgOut->zeros(imgOut->getSize(0), imgOut->getSize(1), imgOut->getType());
    ito::float32 *outPtr = (ito::float32*)imgOut->rowPtr(0, 0);
    int stepX = imgOut->getStep(0);
    int sizex = imgIn->getSize(1);
    int sizey = imgIn->getSize(0);
    if (pts->getSize(1) == 6)
    {
        // 6 points
        for (int np = 0; np < pts->getSize(0); np++)
        {
            ito::float64 x0 = pts->at<ito::float32>(np, 2);
            ito::float64 y0 = pts->at<ito::float32>(np, 3);
            ito::float64 x1 = pts->at<ito::float32>(np, 4);
            ito::float64 y1 = pts->at<ito::float32>(np, 5);
            ito::float64 val = 0, totArea = 0;
            for (int py = 0; py < fabs(ceil(y1) - floor(y0)); py++)
            {
                if (floor(y0) + py >= sizey || floor(y0) + py < 0)
                    continue;
                ito::float64 yfact = 1;
                if (py < 1 && ceil(y0) != y0)
                {
                    yfact = ceil(y0) - y0;
                }
                else if (py > fabs(y1 - y0) - 1)
                {
                    yfact = y1 - floor(y1);
                }
                else if (fabs(y1 - y0) < 1 && floor(y0) != floor(y1))
                {
                    yfact = fabs(y1 - y0);
                }

                ito::float32 *inPtr = (ito::float32*)imgIn->rowPtr(0, floor(y0) + py);
                for (int px = 0; px < fabs(ceil(x1) - floor(x0)); px++)
                {
                    if (floor(x0) + px >= sizex || floor(x0) + px < 0)
                        continue;
                    ito::float64 xfact = 1;

                    if (px < 1 && ceil(x0) != x0)
                    {
                        xfact = ceil(x0) - x0;
                    }
                    else if (px > fabs(x1 - x0) - 1)
                    {
                        xfact = x1 - floor(x1);
                    }
                    else if (fabs(x1 - x0) < 1 && floor(x1) != floor(x0))
                    {
                        xfact = fabs(x1 - x0);
                    }

                    val += xfact * yfact * inPtr[px + (int)floor(x0)];
                    totArea += xfact * yfact;
                }
            }
            if (totArea)
                val /= totArea;
            else
                val = std::numeric_limits<ito::float32>::quiet_NaN();
            outPtr[(int)(pts->at<ito::float32>(np, 1) * stepX + pts->at<ito::float32>(np, 0))] += val;
        }
    }
    else if (pts->getSize(1) == 10)
    {
        //   P2  l3  P3
        //   x-------x
        //   |       |
        // l4|       |l2
        //   |       |
        //   x-------x
        //   P0  l1  P1
        for (int np = 0; np < pts->getSize(0); np++)
        {
            ito::float64 x0 = pts->at<ito::float32>(np, 2), x1 = pts->at<ito::float32>(np, 4),
                x2 = pts->at<ito::float32>(np, 6), x3 = pts->at<ito::float32>(np, 8);
            ito::float64 y0 = pts->at<ito::float32>(np, 3), y1 = pts->at<ito::float32>(np, 5),
                y2 = pts->at<ito::float32>(np, 7), y3 = pts->at<ito::float32>(np, 9);

            ito::float64 l1dx = (x1 - x0) / (y1 - y0), l1dy = (y1 - y0) / (x1 - x0);
            ito::float64 l2dx = (x3 - x1) / (y3 - y1), l2dy = (y3 - y1) / (x3 - x1);
            ito::float64 l3dx = (x3 - x2) / (y3 - y2), l3dy = (y3 - y2) / (x3 - x2);
            ito::float64 l4dx = (x2 - x0) / (y2 - y0), l4dy = (y2 - y0) / (x2 - x0);

            // finding bounding quadrangle
            ito::float64 ymin = floor(y0), xmin = floor(x0);
            ito::float64 ymax = ceil(y0), xmax = ceil(x0);
            for (int ny = 0; ny < 4; ny++)
            {
                if (floor(pts->at<ito::float32>(np, 3 + ny * 2)) < ymin)
                    ymin = floor(pts->at<ito::float32>(np, 3 + ny * 2));
                if (ceil(pts->at<ito::float32>(np, 3 + ny * 2)) > ymax)
                    ymax = ceil(pts->at<ito::float32>(np, 3 + ny * 2));
                if (floor(pts->at<ito::float32>(np, 2 + ny * 2)) < xmin)
                    xmin = floor(pts->at<ito::float32>(np, 2 + ny * 2));
                if (ceil(pts->at<ito::float32>(np, 2 + ny * 2)) > xmax)
                    xmax = ceil(pts->at<ito::float32>(np, 2 + ny * 2));
            }

            ito::float64 val = 0, totArea = 0;
            for (int py = 0; py < (ymax - ymin); py++)
            {
                if ((ymin + py) < 0 || (ymin + py) >= sizey)
                    continue;
                ito::float32 *inPtr = (ito::float32*)imgIn->rowPtr(0, floor(ymin) + py);
                ito::float64 l1x = x0 + l4dx * (ymin + py);
                ito::float64 l3x = x1 + l2dx * (ymin + py);
                for (int px = 0; px < fabs(xmax - xmin); px++)
                {
                    if ((xmin + px) < 0 || (xmin + px) >= sizex)
                        continue;
                    ito::float64 xfact = 1, yfact = 1;

                    ito::float64 l2y = y0 + l1dy * (xmin + px);
                    ito::float64 l4y = y2 + l3dy * (xmin + px);

                    // maybe use more elaborated numerical integration here ...
                    //ito::float64 xcoord = xmin + px;
                    if (px == floor(l1x - xmin) && floor(l3x - xmin) != px)
                    {
                        xfact = ceil(l1x) - l1x;
                    }
                    else if (px == floor(l3x - xmin) && floor(l1x - xmin) != px)
                    {
                        xfact = l3x - floor(l3x);
                    }
                    else if (fabs(l3x - l1x) < 1 && floor(l1x) != floor(l3x))
                    {
                        xfact = fabs(l3x - l1x);
                    }

                    //ito::float64 ycoord = ymin + py;
                    if (py == floor(l2y - ymin) && floor(l4y - ymin) != py)
                    {
                        yfact = ceil(l2y) - l2y;
                    }
                    else if (py == floor(l4y - ymin) && floor(l2y - ymin) != py)
                    {
                        yfact = l4y - floor(l4y);
                    }
                    else if (fabs(l4y - l2y) < 1 && floor(l4y) != floor(l2y))
                    {
                        yfact = fabs(l4y - l2y);
                    }

                    val += xfact * yfact * inPtr[px + (int)floor(xmin)];
                    totArea += xfact * yfact;
                }
            }
            if (totArea)
                val /= totArea;
            else
                val = std::numeric_limits<ito::float32>::quiet_NaN();
            outPtr[(int)(pts->at<ito::float32>(np, 1) * stepX + pts->at<ito::float32>(np, 0))] = val;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DIC::DICGenerateImageDoc = QObject::tr("calculate displacement fields between two images");

/*static*/ ito::RetVal DIC::DICGenerateImageParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::Param param;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("dObjIn", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d input image"));
    paramsMand->append(ito::Param("dObjOut", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "2d output image"));
    paramsMand->append(ito::Param("type", ito::ParamBase::Int | ito::ParamBase::In, 0, 5, 1, "type of deformed image to generate: 0 rigid body movement, 1: uniaxile stress, 2: pure bending, 3: axile symmetric compression, 4: diametral compression"));
    paramsMand->append(ito::Param("disCoeff", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "output vector with polynomial coefficients for each subfield"));

    paramsOpt->append(ito::Param("nu", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 0.3, "Poisson module"));
    paramsOpt->append(ito::Param("E", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 0.15, "Materials E-module in kN/mm²"));
    paramsOpt->append(ito::Param("T", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 0.01, "Tensile stress in kN/mm"));
    paramsOpt->append(ito::Param("M", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 7.0e6, "bending moment in kN mm"));
    paramsOpt->append(ito::Param("I", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 1.0e12, "geometrical moment of inertia mm^4"));
    paramsOpt->append(ito::Param("te", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 1.0e3, "disc thickness in mm"));
    paramsOpt->append(ito::Param("tr", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 200.0, "cylinder diameter / pile width in mm"));
    paramsOpt->append(ito::Param("Pa", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 5.0e-6, "areal load in kN/mm²"));
    paramsOpt->append(ito::Param("Pc", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 500.0, "central load in kN"));

    paramsOpt->append(ito::Param("useLPFilter", ito::ParamBase::Int | ito::ParamBase::In, 0, 100, 0, "use low pass filtering of generated image - size of lp filter"));
    paramsOpt->append(ito::Param("useOversampling", ito::ParamBase::Int | ito::ParamBase::In, 0, 10, 0, "use image oversampling"));
    paramsOpt->append(ito::Param("displacementOnly", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, "calculate vector of displacements only, no image generation"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DIC::DICGenerateImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::DataObject *imgIn = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *imgOut = paramsMand->at(1).getVal<ito::DataObject*>();
    int type = paramsMand->at(2).getVal<int>();
    ito::DataObject *distCoeff = paramsMand->at(3).getVal<ito::DataObject*>();
    if (distCoeff->getType() != ito::tFloat32)
    {
        ito::DataObject distCoeffTmp;
        distCoeff->convertTo(distCoeffTmp, ito::tFloat32);
        *distCoeff = distCoeffTmp;
    }

//    int interpAlgo = paramsOpt->at(0).getVal<int>();

    if (imgIn->getDims() != 2)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Reference must have 2 dimensions!").toLatin1().data());
        return retval;
    }

    if (imgOut->getDims() != 2
        || imgIn->getSize(0) != imgOut->getSize(0) || imgIn->getSize(0) == 0
        || imgIn->getSize(1) != imgOut->getSize(1) || imgIn->getSize(1) == 0
        || imgIn->getType() != imgOut->getType())
    {
        *imgOut = ito::DataObject(imgIn->getSize(0), imgIn->getSize(1), imgIn->getType());
    }

    *imgIn = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)imgIn, "dObjIn", ito::Range::all(), ito::Range::all(), retval, ito::tFloat32, 0);
    if (retval.containsError())
        return retval;

    *imgOut = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)imgOut, "dObjOut", ito::Range::all(), ito::Range::all(), retval, ito::tFloat32, 0);
    if (retval.containsError())
        return retval;

    int sizey = imgIn->getSize(0);
    int sizex = imgIn->getSize(1);
    ito::DataObject ptsVec;
    cell imgCell(sizex / 2, sizey / 2, 0, sizex, 0, sizey);
    ito::float64 nu = paramsOpt->at(0).getVal<ito::float64>();
    ito::float64 E = paramsOpt->at(1).getVal<ito::float64>();
    ito::float64 T = paramsOpt->at(2).getVal<ito::float64>();
    ito::float64 I = paramsOpt->at(4).getVal<ito::float64>();
    ito::float64 M = paramsOpt->at(3).getVal<ito::float64>();
    ito::float64 te = paramsOpt->at(5).getVal<ito::float64>();
    ito::float64 Pa = paramsOpt->at(7).getVal<ito::float64>();
    ito::float64 Pc = paramsOpt->at(8).getVal<ito::float64>();
    ito::float64 scale = sizex / paramsOpt->at(6).getVal<ito::float64>();

    // we can run the image generation with the displacement only flag. In this case we return the displacement at
    // the coordinates passed in the imgIn data object. But therefore imgIn must be 2xn or nx2
    int displOnly = (*paramsOpt)[11].getVal<int>();
    if (displOnly && (sizey != 2 || sizex != 2))
        return ito::RetVal(ito::retError, 0, tr("Displacement vector request, the input object must be a coordinate list of either 2xn or nx2").toLatin1().data());

    //if (type == 0 || 5 && displOnly)
    // 04/08/18 changed ck, don't remember why we check for type 0, though it delivers reasonable results ...
    if (5 && displOnly)
        return ito::RetVal(ito::retError, 0, tr("Calculation of displacement vector for polynomial currently not possible").toLatin1().data());

    switch (type)
    {
        // rigid body movement and distortion coefficients
        case 0:
        case 5:
        {
            ptsVec = ito::DataObject(sizey * sizex, 2, ito::tFloat32);
            if (distCoeff->get_mdata())
                doCalcCoord(&imgCell, ((cv::Mat**)distCoeff->get_mdata())[0], &ptsVec);
            else
                doCalcCoord(&imgCell, NULL, &ptsVec);
            retval += doInterpolate(imgIn, &ptsVec, imgOut, 0, 0, 0);
            //doInterpolate(imgIn, &ptsVec, imgOut, sizex * sizey, 0, 0);
            int sizes[2] = {sizey, sizex};
            *imgOut = imgOut->at(ito::Range::all(), ito::Range(0, 1)).reshape(2, sizes);
            ito::DataObject tmp;
            imgOut->convertTo(tmp, imgIn->getType());
            *imgOut = tmp;
        }
        break;

        // uniaxile tensile stress
        case 1:
        {
//            ptsVec = ito::DataObject(sizey * sizex, 6, ito::tFloat64);
            if (displOnly)
            {
                ito::float64 dx, dy;
                if (imgOut->getSize(1) != imgIn->getSize(1) || imgOut->getSize(0) != imgIn->getSize(0))
                    imgOut->zeros(imgIn->getSize(0), imgIn->getSize(1), ito::tFloat32);
                ito::float32 *imgoutPtr = (ito::float32*)imgOut->rowPtr(0, 0);
                for (int n = 0; n < sizey; n++)
                {
                    for (int m = 0; m < sizex; m++)
                    {
                        dx = (m - sizex / 2) / scale;
                        dy = (n - sizey / 2) / scale;

                        imgoutPtr[(n * sizex + m) * 2] = m - scale * (T / E) * dx;
                        imgoutPtr[(n * sizex + m) * 2 + 1] = n + scale * nu * (T / E) * dy;

                    }
                }
            }
            else
            {
                ptsVec = ito::DataObject(sizey * sizex, 10, ito::tFloat32);
                ito::float32 *ptsVecPtr = (ito::float32*)ptsVec.rowPtr(0, 0);
                ito::float64 dx, dy;
                for (int n = 0; n < sizey; n++)
                {
                    for (int m = 0; m < sizex; m++)
                    {
                        ptsVecPtr[(n * sizex + m) * 10 + 0] = m;
                        ptsVecPtr[(n * sizex + m) * 10 + 1] = n;
                        for (int np = 0; np < 4; np++)
                        {
                            dx = ((m - sizex / 2) + np % 2) / scale;
                            dy = ((n - sizey / 2) + np / 2) / scale;
                            ptsVecPtr[(n * sizex + m) * 10 + 2 + np * 2] = m + np % 2 - scale * (T / E) * dx;
                            ptsVecPtr[(n * sizex + m) * 10 + 2 + np * 2 + 1] = n + np / 2 + scale * nu * (T / E) * dy;
                        }

                    }
                }
                doAreaAveragedInterpol(imgIn, imgOut, &ptsVec);
            }
        }
        break;

        // bending moment
        case 2:
        {
            if (displOnly)
            {
                if (imgOut->getSize(1) != imgIn->getSize(1) || imgOut->getSize(0) != imgIn->getSize(0))
                    imgOut->zeros(imgIn->getSize(0), imgIn->getSize(1), ito::tFloat32);
                ito::float32 *imgoutPtr = (ito::float32*)imgOut->rowPtr(0, 0);

                ito::float64 dx, dy, ux, uy, sign;
                for (int n = 0; n < sizey; n++)
                {
                    for (int m = 0; m < sizex; m++)
                    {
                        dx = (m - sizex / 2) / scale;
                        dy = (n - sizey / 2) / scale;
                        if (dx == 0) dx = 1e-4;
                        if (dy == 0) dy = 1e-4;
                        sign = -(dx * dy) / abs(dx * dy);
                        ux = M * (dy * dy + nu * dx * dx) / (2.0 * E * I);
                        uy = M * dx * dy / (E * I);
                        imgoutPtr[(n * sizex + m) * 2] = m - scale * sign * ux;
                        imgoutPtr[(n * sizex + m) * 2 + 1] = n + scale * sign * uy;
                    }
                }
            }
            else
            {
                ptsVec = ito::DataObject(sizey * sizex, 10, ito::tFloat32);
                ito::float32 *ptsVecPtr = (ito::float32*)ptsVec.rowPtr(0, 0);
                ito::float64 dx, dy, ux, uy, sign;
                for (int n = 0; n < sizey; n++)
                {
                    for (int m = 0; m < sizex; m++)
                    {
                        ptsVecPtr[(n * sizex + m) * 10 + 0] = m;
                        ptsVecPtr[(n * sizex + m) * 10 + 1] = n;
                        // we always calculate four edge points for each pixel thus we use the np loop
                        for (int np = 0; np < 4; np++)
                        {
                            dx = ((m - sizex / 2) + np % 2) / scale;
                            dy = ((n - sizey / 2) + np / 2) / scale;
                            if (dx == 0) dx = 1e-4;
                            if (dy == 0) dy = 1e-4;
                            sign = -(dx * dy) / abs(dx * dy);
                            ux = M * (dy * dy + nu * dx * dx) / (2.0 * E * I);
                            uy = M * dx * dy / (E * I);
                            ptsVecPtr[(n * sizex + m) * 10 + 2 + np * 2] = m + np % 2 - scale * sign * ux;
                            ptsVecPtr[(n * sizex + m) * 10 + 2 + np * 2 + 1] = n + np / 2 + scale * sign * uy;
                        }
                    }
                }
                doAreaAveragedInterpol(imgIn, imgOut, &ptsVec);
            }
        }
        break;

        // axial symmetric compression
        case 3:
        {
            if (displOnly)
            {
                ito::float64 dx, dy;
                if (imgOut->getSize(1) != imgIn->getSize(1) || imgOut->getSize(0) != imgIn->getSize(0))
                    imgOut->zeros(imgIn->getSize(0), imgIn->getSize(1), ito::tFloat32);
                ito::float32 *imgoutPtr = (ito::float32*)imgOut->rowPtr(0, 0);

                for (int n = 0; n < sizey; n++)
                {
                    for (int m = 0; m < sizex; m++)
                    {
                        dx = (m - sizex / 2) / scale;
                        dy = (n - sizey / 2) / scale;
                        ito::float64 theta = atan2(dx, dy);
                        ito::float64 ro = sqrt(dx * dx + dy * dy);
                        ito::float64 ur = -((1.0 - nu) / E) * Pa * ro;
                        ito::float64 ux = ur * sin(theta);
                        ito::float64 uy = ur * cos(theta);
                        imgoutPtr[(n * sizex + m) * 2] = m - scale * ux;
                        imgoutPtr[(n * sizex + m) * 2 + 1] = n - scale * uy;
                    }
                }
            }
            else
            {
                ptsVec = ito::DataObject(sizey * sizex, 10, ito::tFloat32);
                ito::float32 *ptsVecPtr = (ito::float32*)ptsVec.rowPtr(0, 0);
                ito::float64 dx, dy;
                for (int n = 0; n < sizey; n++)
                {
                    for (int m = 0; m < sizex; m++)
                    {
                        ptsVecPtr[(n * sizex + m) * 10 + 0] = m;
                        ptsVecPtr[(n * sizex + m) * 10 + 1] = n;
                        // we always calculate four edge points for each pixel thus we use the np loop
                        for (int np = 0; np < 4; np++)
                        {
                            dx = ((m - sizex / 2) + np % 2) / scale;
                            dy = ((n - sizey / 2) + np / 2) / scale;
                            ito::float64 theta = atan2(dx, dy);
                            ito::float64 ro = sqrt(dx * dx + dy * dy);
                            ito::float64 ur = -((1.0 - nu) / E) * Pa * ro;
                            ito::float64 ux = ur * sin(theta);
                            ito::float64 uy = ur * cos(theta);
                            ptsVecPtr[(n * sizex + m) * 10 + 2 + np * 2] = m + np % 2 - scale * ux;
                            ptsVecPtr[(n * sizex + m) * 10 + 2 + np * 2 + 1] = n + np / 2 - scale * uy;
                        }
                    }
                }
                doAreaAveragedInterpol(imgIn, imgOut, &ptsVec);
            }
        }
        break;

        // diametral compression
        case 4:
        {
            if (displOnly)
            {
                ito::float64 dx, dy;
                if (imgOut->getSize(1) != imgIn->getSize(1) || imgOut->getSize(0) != imgIn->getSize(0))
                    imgOut->zeros(imgIn->getSize(0), imgIn->getSize(1), ito::tFloat32);
                ito::float32 *imgoutPtr = (ito::float32*)imgOut->rowPtr(0, 0);

                ito::float64 R = sizex / 2.0;
                for (int n = 0; n < sizey; n++)
                {
                    for (int m = 0; m < sizex; m++)
                    {
                        dy = (n - sizey / 2);
                        dx = (m - sizex / 2);
                        //ito::float64 theta = atan(dx / dy);
                        ito::float64 theta = atan2(dx, dy);
                        ito::float64 ro = sqrt(dx * dx + dy * dy) / R;
                        ito::float64 ro2 = ro * ro;
                        ito::float64 ro4 = ro2 * ro2;
                        ito::float64 ur = -2.0 * Pc / E / M_PI / te * (-ro * (2.0 + ro2 + ro4 - (1.0 + 3.0 * ro2)
                            * cos(2.0 * theta)) / (1.0 + ro4 - 2.0 * ro2 * cos(2.0 * theta)) + sin(theta)
                            * (atan(ro * sin(theta) / (1.0 + ro * cos(theta)))
                                + atan(ro * sin(theta) / (1.0 - ro * cos(theta)))) + cos(theta)
                            * log(abs((1.0 + ro2 + 2.0 * ro * cos(theta)) / (1.0 + ro2 - 2.0 * ro * cos(theta))))
                            + nu * (-ro * (1.0 - ro2) * (ro2 - cos(2.0 * theta)) / (1.0 + ro4
                                - 2.0 * ro2 * cos(2.0 * theta)) - sin(theta) * (atan(ro * sin(theta)
                                    / (1.0 + ro * cos(theta))) + atan(ro * sin(theta) / (1.0 - ro * cos(theta))))));
                        ito::float64 ut = -2.0 * Pc / E / M_PI / te * (cos(theta) * (atan(ro * sin(theta)
                            / (1.0 + ro * cos(theta))) + atan(ro * sin(theta) / (1.0 - ro * cos(theta))))
                            - sin(theta) * (2.0 * ro * (1.0 - ro2) * cos(theta) / (1.0 + ro4 - 2.0 * ro2 * cos(2.0 * theta))
                                + log(abs((1.0 + ro2 + 2.0 * ro * cos(theta)) / (1.0 + ro2 - 2.0 * ro * cos(theta)))))
                            - nu * cos(theta) * (2.0 * ro * (1.0 - ro2) * sin(theta) / (1.0 + ro4 - 2.0 * ro2 * cos(2.0 * theta))
                                + atan(ro * sin(theta) / (1.0 + ro * cos(theta))) + atan(ro * sin(theta) / (1.0 - ro * cos(theta)))));

                        ito::float64 ux = -(ur * sin(theta) + ut * cos(theta));
                        ito::float64 uy = -(ur * cos(theta) - ut * sin(theta));

                        imgoutPtr[(n * sizex + m) * 2] = m + scale * ux;
                        imgoutPtr[(n * sizex + m) * 2 + 1] = n + scale * uy;
                    }
                }
            }
            else
            {
                ptsVec = ito::DataObject(sizey * sizex, 10, ito::tFloat32);
                ito::float32 *ptsVecPtr = (ito::float32*)ptsVec.rowPtr(0, 0);
                ito::float64 dx, dy;

                ito::float64 R = sizex / 2.0;
                for (int n = 0; n < sizey; n++)
                {
                    for (int m = 0; m < sizex; m++)
                    {
                        ptsVecPtr[(n * sizex + m) * 10 + 0] = m;
                        ptsVecPtr[(n * sizex + m) * 10 + 1] = n;
                        // we always calculate four edge points for each pixel thus we use the np loop
                        for (int np = 0; np < 4; np++)
                        {
                            dy = (n - sizey / 2) + np / 2;
                            dx = (m - sizex / 2) + np % 2;
                            //ito::float64 theta = atan(dx / dy);
                            ito::float64 theta = atan2(dx, dy);
                            ito::float64 ro = sqrt(dx * dx + dy * dy) / R;
                            ito::float64 ro2 = ro * ro;
                            ito::float64 ro4 = ro2 * ro2;
                            ito::float64 ur = -2.0 * Pc / E / M_PI / te * (-ro * (2.0 + ro2 + ro4 - (1.0 + 3.0 * ro2)
                                * cos(2.0 * theta)) / (1.0 + ro4 - 2.0 * ro2 * cos(2.0 * theta)) + sin(theta)
                                * (atan(ro * sin(theta) / (1.0 + ro * cos(theta)))
                                    + atan(ro * sin(theta) / (1.0 - ro * cos(theta)))) + cos(theta)
                                * log(abs((1.0 + ro2 + 2.0 * ro * cos(theta)) / (1.0 + ro2 - 2.0 * ro * cos(theta))))
                                + nu * (-ro * (1.0 - ro2) * (ro2 - cos(2.0 * theta)) / (1.0 + ro4
                                    - 2.0 * ro2 * cos(2.0 * theta)) - sin(theta) * (atan(ro * sin(theta)
                                        / (1.0 + ro * cos(theta))) + atan(ro * sin(theta) / (1.0 - ro * cos(theta))))));
                            ito::float64 ut = -2.0 * Pc / E / M_PI / te * (cos(theta) * (atan(ro * sin(theta)
                                / (1.0 + ro * cos(theta))) + atan(ro * sin(theta) / (1.0 - ro * cos(theta))))
                                - sin(theta) * (2.0 * ro * (1.0 - ro2) * cos(theta) / (1.0 + ro4 - 2.0 * ro2 * cos(2.0 * theta))
                                    + log(abs((1.0 + ro2 + 2.0 * ro * cos(theta)) / (1.0 + ro2 - 2.0 * ro * cos(theta)))))
                                - nu * cos(theta) * (2.0 * ro * (1.0 - ro2) * sin(theta) / (1.0 + ro4 - 2.0 * ro2 * cos(2.0 * theta))
                                    + atan(ro * sin(theta) / (1.0 + ro * cos(theta))) + atan(ro * sin(theta) / (1.0 - ro * cos(theta)))));

                            ito::float64 ux = -(ur * sin(theta) + ut * cos(theta));
                            ito::float64 uy = -(ur * cos(theta) - ut * sin(theta));

                            ptsVecPtr[(n * sizex + m) * 10 + 2 + np * 2] = m + np % 2 + scale * ux;
                            ptsVecPtr[(n * sizex + m) * 10 + 2 + np * 2 + 1] = n + np / 2 + scale * uy;
                        }
                    }
                }
                doAreaAveragedInterpol(imgIn, imgOut, &ptsVec);
            }
        }
        break;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DIC::DICGenerateProjImageDoc = QObject::tr("calculate intensity distribution of images after a 3D projection using a pinhole camera model");

/*static*/ ito::RetVal DIC::DICGenerateProjImageParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::Param param;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("dObjIn", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d input image"));
    paramsMand->append(ito::Param("dObjOut", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "2d output image"));
    paramsMand->append(ito::Param("ObjSizeX", ito::ParamBase::Double| ito::ParamBase::In, 0.0, 10000.0, 100.0, "object x-size in mm"));
    paramsMand->append(ito::Param("ObjSizeY", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 10000.0, 150.0, "object y-size in mm"));
    paramsMand->append(ito::Param("ZVal", ito::ParamBase::Double | ito::ParamBase::In, -10000.0, 10000.0, 0.0, "Z-Value for plane objects in mm"));
    paramsMand->append(ito::Param("CamMat", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "camera fundamental matrix"));
    paramsMand->append(ito::Param("DistCoeff", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "camera distortion coefficients"));
    paramsMand->append(ito::Param("RVec", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "camera rotation vector"));
    paramsMand->append(ito::Param("TVec", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "camera translationvector"));

    paramsOpt->append(ito::Param("nu", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 0.3, "Poisson module"));
    paramsOpt->append(ito::Param("E", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 0.15, "Materials E-module in kN/mm²"));
    paramsOpt->append(ito::Param("te", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 1.0e3, "disc thickness in mm"));
    paramsOpt->append(ito::Param("tr", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 200.0, "cylinder diameter / pile width in mm"));
    paramsOpt->append(ito::Param("shapeType", ito::ParamBase::Int | ito::ParamBase::In, 0, 2, 0, "shape used for 3d point calculation: 0: plane, 1: cylinder, 2: square pile"));
    paramsOpt->append(ito::Param("Pc", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e208, 500.0, "central load in kN"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void swapAB(ito::float64 &a, ito::float64 &b)
{
    if (b < a)
    {
        ito::float64 tb = b;
        b = a;
        a = tb;
    }
}

ito::float64 getSrcInt(ito::DataObject *src, int xpos, int ypos, ito::float64 weight)
{
    switch (src->getType())
    {
        case ito::tUInt8:
            return weight * src->at<ito::uint8>(ypos, xpos);
        break;

        case ito::tUInt16:
            return weight * src->at<ito::uint16>(ypos, xpos);
        break;

        case ito::tUInt32:
            return weight * src->at<ito::uint32>(ypos, xpos);
        break;

        case ito::tFloat32:
            return weight * src->at<ito::float32>(ypos, xpos);
        break;

        case ito::tFloat64:
            return weight * src->at<ito::float64>(ypos, xpos);
        break;
    }

    return 0.0;
}

/*static*/ ito::RetVal DIC::DICGenerateProjImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::DataObject *imgIn = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *imgOut = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::float64 objSX = paramsMand->at(2).getVal<ito::float64>();
    ito::float64 objSY = paramsMand->at(3).getVal<ito::float64>();
    ito::float64 zval = paramsMand->at(4).getVal<ito::float64>();
    ito::DataObject *camMat = paramsMand->at(5).getVal<ito::DataObject*>();
    ito::DataObject *distCoeff = paramsMand->at(6).getVal<ito::DataObject*>();
    ito::DataObject *rVec = paramsMand->at(7).getVal<ito::DataObject*>();
    ito::DataObject *tVec = paramsMand->at(8).getVal<ito::DataObject*>();
    ito::float32 *imgOutPtr = NULL;
    ito::float32 *ptsVecOutPtr = NULL;
    ito::float32 bt = 0, lr = 0;
    int inPlace = 0;

    if (imgIn->getDims() != 2)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Reference must have 2 dimensions!").toLatin1().data());
        return retval;
    }
    /*
    if (imgOut->getDims() != 2
        || imgIn->getSize(0) != imgOut->getSize(0) || imgIn->getSize(0) == 0
        || imgIn->getSize(1) != imgOut->getSize(1) || imgIn->getSize(1) == 0
        || imgIn->getType() != imgOut->getType())
    {
        *imgOut = ito::DataObject(imgIn->getSize(0), imgIn->getSize(1), imgIn->getType());
    }
    */

    ito::DataObject imgInTmp = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)imgIn, "dObjIn", ito::Range::all(), ito::Range::all(), retval, ito::tFloat64, 0);
    if (retval.containsError())
        return retval;

    /*
    *imgOut = ito::dObjHelper::squeezeConvertCheck2DDataObject((const ito::DataObject*)imgOut, "dObjOut", ito::Range::all(), ito::Range::all(), retval, ito::tFloat64, 0);
    if (retval.containsError())
        return retval;
    */

    int sizey = imgInTmp.getSize(0);
    int sizex = imgInTmp.getSize(1);
    ito::DataObject ptsVecIn;

    ptsVecIn.zeros(sizey * sizex, 3, ito::tFloat32);
    ito::float32 *ptsVecInPtr = (ito::float32*)ptsVecIn.rowPtr(0, 0);
    int shapeType = paramsOpt->at(4).getVal<int>();

    switch (shapeType)
    {
        case 0:
            for (int sy = 0; sy < sizey; sy++)
            {
                for (int sx = 0; sx < sizex; sx++)
                {
                    ptsVecInPtr[(sy * sizex + sx) * 3] = objSX / sizex * sx;
                    ptsVecInPtr[(sy * sizex + sx) * 3 + 1] = objSY / sizey * sy;
                    ptsVecInPtr[(sy * sizex + sx) * 3 + 2] = zval;
                }
            }
        break;

        // 3D compression
        case 1:
        {
            ito::float64 nu = paramsOpt->at(0).getVal<ito::float64>();
            ito::float64 E = paramsOpt->at(1).getVal<ito::float64>();
            ito::float64 Pc = paramsOpt->at(5).getVal<ito::float64>();

            ito::float64 R = objSX / 2.0;
            if (Pc != 0)
            {
                ito::float64 dR = nu * Pc / (E * R * R * M_PI);
                ito::float64 dh = Pc / (E * R * R * M_PI);
                R += dR;
                objSY -= dh;
            }
            ito::float64 A = R * R * M_PI;
            for (int sy = 0; sy < sizey; sy++)
            {
                ito::float64 *inPtr = (ito::float64*)imgInTmp.rowPtr(0, sy);
                for (int sx = 0; sx < sizex; sx++)
                {
                    ptsVecInPtr[(sy * sizex + sx) * 3 + 0] = objSX / sizex * sx;
                    ptsVecInPtr[(sy * sizex + sx) * 3 + 1] = objSY / sizey * sy;
                    ptsVecInPtr[(sy * sizex + sx) * 3 + 2] = sqrt(R * R - (ptsVecInPtr[(sy * sizex + sx) * 3 + 0] - R) * (ptsVecInPtr[(sy * sizex + sx) * 3 + 0] - R));
                }
            }
        }
        break;

        // square pillar - not yet implemented
        case 2:
            retval += ito::RetVal(ito::retError, 0, tr("not yet implemented").toLatin1().data());
        break;
    }

    QVector<ito::ParamBase> fparamsMand, fparamsOpt, fparamsOut;
    ito::DataObject ptsVecOut;
    retval += apiFilterParamBase("cvProjectPoints", &fparamsMand, &fparamsOpt, &fparamsOut);
    if (retval.containsWarningOrError())
        goto end;

    fparamsMand[0].setVal<ito::DataObject*>(&ptsVecIn);
    fparamsMand[1].setVal<ito::DataObject*>(&ptsVecOut);
    fparamsMand[2].setVal<ito::DataObject*>(camMat);
    fparamsMand[3].setVal<ito::DataObject*>(distCoeff);
    fparamsMand[4].setVal<ito::DataObject*>(rVec);
    fparamsMand[5].setVal<ito::DataObject*>(tVec);

    retval += apiFilterCall("cvProjectPoints", &fparamsMand, &fparamsOpt, &fparamsOut);
    if (retval.containsWarningOrError())
        goto end;

    ptsVecOutPtr = (ito::float32*)ptsVecOut.rowPtr(0, 0);

    ito::int16 xmin, ymin, xmax, ymax;
    // plane object, using simple search
    if (shapeType == 0)
    {
        if (ptsVecOutPtr[0] < ptsVecOutPtr[(sizex - 1) * 2]
            && ptsVecOutPtr[0] < ptsVecOutPtr[(sizey - 1) * sizex * 2]
            && ptsVecOutPtr[0] < ptsVecOutPtr[sizey * sizex * 2 - 2])
            xmin = ptsVecOutPtr[0];
        else if (ptsVecOutPtr[(sizex - 1) * 2] < ptsVecOutPtr[(sizey - 1) * sizex * 2]
            && ptsVecOutPtr[(sizex - 1) * 2] < ptsVecOutPtr[sizey * sizex * 2 - 2])
            xmin = ptsVecOutPtr[(sizex - 1) * 2];
        else if (ptsVecOutPtr[(sizey - 1) * sizex * 2] < ptsVecOutPtr[sizey * sizex * 2 - 2])
            xmin = ptsVecOutPtr[(sizey - 1) * sizex * 2];
        else
            xmin = ptsVecOutPtr[sizey * sizex * 2 - 2];

        if (ptsVecOutPtr[0] > ptsVecOutPtr[(sizex - 1) * 2]
            && ptsVecOutPtr[0] > ptsVecOutPtr[(sizey - 1) * sizex * 2]
            && ptsVecOutPtr[0] > ptsVecOutPtr[sizey * sizex * 2 - 2])
            xmax = ptsVecOutPtr[0];
        else if (ptsVecOutPtr[(sizex - 1) * 2] > ptsVecOutPtr[sizey * sizex * 2]
            && ptsVecOutPtr[(sizex - 1) * 2] > ptsVecOutPtr[sizey * sizex * 2 - 2])
            xmax = ptsVecOutPtr[(sizex - 1) * 2];
        else if (ptsVecOutPtr[(sizey - 1) * sizex * 2] > ptsVecOutPtr[sizey * sizex * 2 - 2])
            xmax = ptsVecOutPtr[(sizey - 1) * sizex * 2];
        else
            xmax = ptsVecOutPtr[sizey * sizex * 2 - 2];

        if (ptsVecOutPtr[1] < ptsVecOutPtr[(sizex - 1) * 2 + 1]
            && ptsVecOutPtr[1] < ptsVecOutPtr[(sizey - 1) * sizex * 2 + 1]
            && ptsVecOutPtr[1] < ptsVecOutPtr[sizey * sizex * 2 - 2 + 1])
            ymin = ptsVecOutPtr[1];
        else if (ptsVecOutPtr[(sizex - 1) * 2 + 1] < ptsVecOutPtr[(sizey - 1) * sizex * 2 + 1]
            && ptsVecOutPtr[(sizex - 1) * 2 + 1] < ptsVecOutPtr[sizey * sizex * 2 - 2 + 1])
            ymin = ptsVecOutPtr[(sizex - 1) * 2 + 1];
        else if (ptsVecOutPtr[(sizey - 1) * sizex * 2 + 1] < ptsVecOutPtr[sizey * sizex * 2 - 2 + 1])
            ymin = ptsVecOutPtr[(sizey - 1) * sizex * 2 + 1];
        else
            ymin = ptsVecOutPtr[sizey * sizex * 2 - 2 + 1];

        if (ptsVecOutPtr[1] > ptsVecOutPtr[(sizex - 1) * 2 + 1]
            && ptsVecOutPtr[1] > ptsVecOutPtr[(sizey - 1) * sizex * 2 + 1]
            && ptsVecOutPtr[1] > ptsVecOutPtr[sizey * sizex * 2 - 2 + 1])
            ymax = ptsVecOutPtr[1];
        else if (ptsVecOutPtr[(sizex - 1) * 2 + 1] > ptsVecOutPtr[(sizey - 1) * sizex * 2 + 1]
            && ptsVecOutPtr[(sizex - 1) * 2 + 1] > ptsVecOutPtr[sizey * sizex * 2 - 2 + 1])
            ymax = ptsVecOutPtr[(sizex - 1) * 2 + 1];
        else if (ptsVecOutPtr[(sizey - 1) * sizex * 2 + 1] > ptsVecOutPtr[sizey * sizex * 2 - 2 + 1])
            ymax = ptsVecOutPtr[(sizey - 1) * sizex * 2 + 1];
        else
            ymax = ptsVecOutPtr[sizey * sizex * 2 - 2 + 1];
    }
    else // not plane object we have to do a full search
    {
        ymin = xmin = 32767;
        ymax = xmax = -32767;
        for (int pt = 0; pt < sizex * sizey; pt++)
        {
            if (ceil(ptsVecOutPtr[pt * 2]) > xmax)
                xmax = ceil(ptsVecOutPtr[pt * 2]);
            if (floor(ptsVecOutPtr[pt * 2]) < xmin)
                xmin = ptsVecOutPtr[pt * 2];
            if (ceil(ptsVecOutPtr[pt * 2 + 1]) > ymax)
                ymax = ceil(ptsVecOutPtr[pt * 2 + 1]);
            if (floor(ptsVecOutPtr[pt * 2 + 1]) < ymin)
                ymin = floor(ptsVecOutPtr[pt * 2 + 1]);
        }
    }
    // enlarge by one pixel in each direction. This is necessary as the interpolation could use these values
    xmax++; ymax++; ymin--; xmin--;

    if (imgOut == imgIn)
    {
        inPlace = 1;
        imgOut = new ito::DataObject();
    }
    else
        *imgOut = ito::DataObject();
    imgOut->zeros(ymax - ymin + 1, xmax - xmin + 1, ito::tFloat32);
    imgOutPtr = (ito::float32*)imgOut->rowPtr(0, 0);

    // determine order of projected points for calculation loop
    bt = ptsVecOutPtr[sizex * 2 + 1] - ptsVecOutPtr[1];
    bt /= abs(bt);
    lr = ptsVecOutPtr[(sizex - 1) * 2] - ptsVecOutPtr[0];
    lr /= abs(lr);

#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
    {
#pragma omp for schedule(guided)
#endif
    for (int pt = 0; pt < sizex * sizey; pt++)
    {
        ito::float64 ptxmin, ptxmax, ptymin, ptymax;
        // determine actual pixel dimensions and check if we hit some border
        if (pt % sizex == 0) // left border of original image
        {
            if (lr > 0)
            {
                ptxmax = 0.5 * (ptsVecOutPtr[pt * 2] + ptsVecOutPtr[(pt + 1) * 2]);
                //                ptxmin = floor(ptsVecOutPtr[pt * 2]);
                ptxmin = ptsVecOutPtr[pt * 2] - (ptxmax - ptsVecOutPtr[pt * 2]);
                if (ptxmin < floor(ptsVecOutPtr[pt * 2]))
                    ptxmin = floor(ptsVecOutPtr[pt * 2]);
            }
            else
            {
                ptxmin = 0.5 * (ptsVecOutPtr[pt * 2] + ptsVecOutPtr[(pt + 1) * 2]);
                //                ptxmax = ceil(ptsVecOutPtr[pt * 2]);
                ptxmax = ptsVecOutPtr[pt * 2] + (ptsVecOutPtr[pt * 2] - ptxmin);
                if (ptxmax > ceil(ptsVecOutPtr[pt * 2]))
                    ptxmax = ceil(ptsVecOutPtr[pt * 2]);
            }
        }
        else if (pt % sizex == sizex - 1) // right border of original image
        {
            if (lr > 0)
            {
                ptxmin = 0.5 * (ptsVecOutPtr[(pt - 1) * 2] + ptsVecOutPtr[pt * 2]);
                ptxmax = ptsVecOutPtr[pt * 2] + (ptsVecOutPtr[pt * 2] - ptxmin);
                //                ptxmax = ceil(ptsVecOutPtr[pt * 2]);
                if (ptxmax > ceil(ptsVecOutPtr[pt * 2]))
                    ptxmax = ceil(ptsVecOutPtr[pt * 2]);
            }
            else
            {
                ptxmax = 0.5 * (ptsVecOutPtr[(pt - 1) * 2] + ptsVecOutPtr[pt * 2]);
                ptxmin = ptsVecOutPtr[pt * 2] - (ptxmax - ptsVecOutPtr[pt * 2]);
                //                ptxmin = floor(ptsVecOutPtr[pt * 2]);
                if (ptxmin < floor(ptsVecOutPtr[pt * 2]))
                    ptxmin = floor(ptsVecOutPtr[pt * 2]);
            }
        }
        else
        {
            if (lr > 0)
            {
                ptxmin = 0.5 * (ptsVecOutPtr[(pt - 1) * 2] + ptsVecOutPtr[pt * 2]);
                ptxmax = 0.5 * (ptsVecOutPtr[pt * 2] + ptsVecOutPtr[(pt + 1) * 2]);
            }
            else
            {
                ptxmax = 0.5 * (ptsVecOutPtr[(pt - 1) * 2] + ptsVecOutPtr[pt * 2]);
                ptxmin = 0.5 * (ptsVecOutPtr[pt * 2] + ptsVecOutPtr[(pt + 1) * 2]);
            }
        }

        if (pt < sizex) // top border of original image
        {
            if (bt > 0)
            {
                ptymax = 0.5 * (ptsVecOutPtr[pt * 2 + 1] + ptsVecOutPtr[(pt + sizex) * 2 + 1]);
                //                ptymin = floor(ptsVecOutPtr[pt * 2 + 1]);
                ptymin = ptsVecOutPtr[pt * 2 + 1] - (ptymax - ptsVecOutPtr[pt * 2 + 1]);
                if (ptymin < floor(ptsVecOutPtr[pt * 2 + 1]))
                    ptymin = floor(ptsVecOutPtr[pt * 2 + 1]);
            }
            else
            {
                ptymin = 0.5 * (ptsVecOutPtr[pt * 2 + 1] + ptsVecOutPtr[(pt + sizex) * 2 + 1]);
                //                ptymax = ceil(ptsVecOutPtr[pt * 2 + 1]);
                ptymax = ptsVecOutPtr[pt * 2 + 1] + (ptsVecOutPtr[pt * 2 + 1] - ptymin);
                if (ptymax > ceil(ptsVecOutPtr[pt * 2 + 1]))
                    ptymax = ceil(ptsVecOutPtr[pt * 2 + 1]);
            }
        }
        else if (pt / sizex == sizey - 1) // bottom border of original image
        {
            if (bt > 0)
            {
                ptymin = 0.5 * (ptsVecOutPtr[(pt - sizex) * 2 + 1] + ptsVecOutPtr[pt * 2 + 1]);
                //                ptymax = ceil(ptsVecOutPtr[pt * 2 + 1]);
                ptymax = ptsVecOutPtr[pt * 2 + 1] + (ptsVecOutPtr[pt * 2 + 1] - ptymin);
                if (ptymax > ceil(ptsVecOutPtr[pt * 2 + 1]))
                    ptymax = ceil(ptsVecOutPtr[pt * 2 + 1]);
            }
            else
            {
                ptymax = 0.5 * (ptsVecOutPtr[(pt - sizex) * 2 + 1] + ptsVecOutPtr[pt * 2 + 1]);
                //                ptymin = floor(ptsVecOutPtr[pt * 2 + 1]);
                ptymin = ptsVecOutPtr[pt * 2 + 1] - (ptymax - ptsVecOutPtr[pt * 2 + 1]);
                if (ptymin < floor(ptsVecOutPtr[pt * 2 + 1]))
                    ptymin = floor(ptsVecOutPtr[pt * 2 + 1]);
            }
        }
        else
        {
            if (bt > 0)
            {
                ptymin = 0.5 * (ptsVecOutPtr[(pt - sizex) * 2 + 1] + ptsVecOutPtr[pt * 2 + 1]);
                ptymax = 0.5 * (ptsVecOutPtr[pt * 2 + 1] + ptsVecOutPtr[(pt + sizex) * 2 + 1]);
            }
            else
            {
                ptymax = 0.5 * (ptsVecOutPtr[(pt - sizex) * 2 + 1] + ptsVecOutPtr[pt * 2 + 1]);
                ptymin = 0.5 * (ptsVecOutPtr[pt * 2 + 1] + ptsVecOutPtr[(pt + sizex) * 2 + 1]);
            }
        }

        if (floor(ptxmax) == floor(ptxmin) && floor(ptymax) == floor(ptymin))
        {
            imgOutPtr[(int)(floor(ptymin) - ymin) * (xmax - xmin + 1) + (int)floor(ptxmin) - xmin]
                += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, (ptymax - ptymin) * (ptxmax - ptxmin));
        }
        else if (floor(ptymax) == floor(ptymin))
        {
            // lower, left edge pixel
            imgOutPtr[(int)(floor(ptymin) - ymin) * (xmax - xmin + 1) + (int)floor(ptxmin) - xmin]
                += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, (ptymax - ptymin) * (ceil(ptxmin) - ptxmin));

            // complete border pixels
            for (int px = floor(ptxmin) + 1; px < ceil(ptxmax) - 1; px++)
            {
                imgOutPtr[(int)(floor(ptymin) - ymin) * (xmax - xmin + 1) + px - xmin]
                    += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, ptymax - ptymin);
            }

            // lower right edge pixel
            imgOutPtr[(int)(floor(ptymax) - ymin) * (xmax - xmin + 1) + (int)floor(ptxmax) - xmin]
                += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, (ptymax - ptymin) * (ptxmax - floor(ptxmax)));
        }
        else if (floor(ptxmax) == floor(ptxmin))
        {
            // lower, left edge pixel
            imgOutPtr[(int)(floor(ptymin) - ymin) * (xmax - xmin + 1) + (int)floor(ptxmin) - xmin]
                += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, (ceil(ptymin) - ptymin) * (ptxmax - ptxmin));

            // complete border pixels
            for (int py = floor(ptymin) + 1; py < ceil(ptymax) - 1; py++)
            {
                imgOutPtr[(py - ymin) * (xmax - xmin + 1) + (int)floor(ptxmin) - xmin]
                    += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, ptxmax - ptxmin);
            }

            // lower right edge pixel
            imgOutPtr[(int)(floor(ptymax) - ymin) * (xmax - xmin + 1) + (int)floor(ptxmax) - xmin]
                += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, (ptymax - floor(ptymax)) * (ptxmax - ptxmin));
        }
        else
        {
            // lower, left edge pixel
            imgOutPtr[(int)(floor(ptymin) - ymin) * (xmax - xmin + 1) + (int)floor(ptxmin) - xmin]
                += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, (ceil(ptymin) - ptymin) * (ceil(ptxmin) - ptxmin));

            // incomplete lower border pixels
            for (int px = floor(ptxmin) + 1; px < ceil(ptxmax) - 1; px++)
            {
                imgOutPtr[(int)(floor(ptymin) - ymin) * (xmax - xmin + 1) + px - xmin]
                    += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, ceil(ptymin) - ptymin);
            }

            // lower right edge pixel
            imgOutPtr[(int)(floor(ptymin) - ymin) * (xmax - xmin + 1) + (int)floor(ptxmax) - xmin]
                += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, (ceil(ptymin) - ptymin) * (ptxmax - floor(ptxmax)));

            // fully covered pixels in y
            for (int py = floor(ptymin) + 1; py < ceil(ptymax) - 1; py++)
            {
                // left border pixel
                imgOutPtr[(py - ymin) * (xmax - xmin + 1) + (int)floor(ptxmin) - xmin]
                    += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, ceil(ptxmin) - ptxmin);

                // fully covered pixels in y and x
                for (int px = floor(ptxmin) + 1; px < ceil(ptxmax) - 1; px++)
                {
                    imgOutPtr[(py - ymin) * (xmax - xmin + 1) + px - xmin]
                        += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, 1.0);
                }

                // right border pixel
                imgOutPtr[(py - ymin) * (xmax - xmin + 1) + (int)floor(ptxmax) - xmin]
                    += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, ptxmax - floor(ptxmax));
            }

            // upper left edge pixel
            imgOutPtr[(int)(floor(ptymax) - ymin) * (xmax - xmin + 1) + (int)floor(ptxmin) - xmin]
                += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, (ptymax - floor(ptymax)) * (ceil(ptxmin) - ptxmin));

            // incomplete upper border pixel treatment
            for (int px = floor(ptxmin) + 1; px < ceil(ptxmax) - 1; px++)
            {
                imgOutPtr[(int)(floor(ptymax) - ymin) * (xmax - xmin + 1) + px - xmin]
                    += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, ptymax - floor(ptymax));
            }

            // upper right edge pixel
            imgOutPtr[(int)(floor(ptymax) - ymin) * (xmax - xmin + 1) + (int)floor(ptxmax) - xmin]
                += getSrcInt(&imgInTmp, pt % sizex, pt / sizex, (ptymax - floor(ptymax)) * (ptxmax - floor(ptxmax)));
        }
    }
#if (USEOMP)
    }
#endif
    imgOut->setAxisOffset(0, -ymin);
    imgOut->setAxisOffset(1, -xmin);

    if (inPlace)
        *imgIn = *imgOut;
end:
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
