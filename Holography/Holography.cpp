/* ********************************************************************
    Plugin "Holography" for itom software
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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include <math.h>
#include "Holography.h"

#include "DataObject/dataobj.h"
#include "DataObject/dataObjectFuncs.h"

#include <QtCore/QtPlugin>

#include "pluginVersion.h"
#include "gitVersion.h"


#ifdef USEOPENMP
    #define useomp 1
#else
    #define useomp 0
#endif

int NTHREADS = 2;

/*
    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif
    int b = 0, bitmask = 0, i = 0, invert = 0;

    #if (USEOMP)
    #pragma omp for schedule(guided)
    #endif
    for (int g = 0; g <= (1 << maxBits); g++)
    {
    }
*/

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal HolographyInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(Holography)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal HolographyInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(Holography)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
HolographyInterface::HolographyInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("Holography");

    m_description = QObject::tr("Algorithms used for holographic optical systems");
    m_detaildescription = QObject::tr("This DLL contains several algorithms for holography.");
    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    NTHREADS = QThread::idealThreadCount();
}

//----------------------------------------------------------------------------------------------------------------------------------
HolographyInterface::~HolographyInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
Holography::Holography() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
Holography::~Holography()
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
/** initialize filter functions within this addIn
*    @param [in]    paramsMand    mandatory parameters that have to passed to the addIn on initialization
*    @param [in]    paramsOpt    optional parameters that can be passed to the addIn on initialization
*    @return                    retError in case of an error
*
*    Here are the filter functions defined that are available through this addIn.
*    These are:
*        - FresnelCalcProp  Calculate propagator used for a subsequent Fresnel propagation
*        - FresnelDoProp    Apply / propagate provided wavefield using Fresnel propagation and the previously calculated propagator
*        - RSCalcProp       Calculate propagator used for a subsequent Rayleigh-Sommerfeld propagation
*        - RSDoProp         Calculate propagator used for a subsequent Rayleigh-Sommerfeld propagation
*/
ito::RetVal Holography::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(Holography::FresnelCalcProp, Holography::FresnelCalcPropParams, tr("Calculate phasemasks used for a subsequent Fresnel propagation"));
    m_filterList.insert("FresnelCalcProp", filter);
    filter = new FilterDef(Holography::FresnelDoProp, Holography::FresnelDoPropParams, tr("perform Fresnel propagation using the before calculated phasemasks and 1 Fourier transform"));
    m_filterList.insert("FresnelDoProp", filter);

    filter = new FilterDef(Holography::RSCalcProp, Holography::RSCalcPropParams, tr("Calculate phasemask used for a subsequent Rayleigh-Sommerfeld propagation"));
    m_filterList.insert("RSCalcProp", filter);
    filter = new FilterDef(Holography::RSDoProp, Holography::RSDoPropParams, tr("perform Rayleigh-Sommerfeld propagation using the before calculated phasemask and 2 Fourier transforms"));
    m_filterList.insert("RSDoProp", filter);

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Holography::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void calcFieldXY(_Tp *dPtrX, _Tp *dPtrY, int sx, int sy)
{
#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
    {
#endif
    double ofsX = sx / 2;
    double ofsY = sy / 2;
#if (USEOMP)
    #pragma omp for schedule(guided)
#endif
        for (int ny = 0; ny < sy; ny++)
        {
            double yval = ny - ofsY;
            for (int nx = 0; nx < sx; nx++)
            {
                double xval = nx - ofsX;
                dPtrX[ny * sx + nx] = xval * xval;
                dPtrY[ny * sx + nx] = yval * yval;
            }
        }

#if (USEOMP)
    }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _T1, typename _T2> void RScalcPhaseMask(_T1 *PM, _T2 *X, _T2 *Y, int sx, int sy, double dist, double px, double wavelen)
{
#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
    {
#endif
        double pha1 = -2.0 * CV_PI * dist;
        double preFact = 1.0 / (wavelen * wavelen);
        double scalex = sx * sx * px * px;
        double scaley = sy * sy * px * px;
#if (USEOMP)
#pragma omp for schedule(guided)
#endif
        for (int ny = 0; ny < sy; ny++)
        {
            for (int nx = 0; nx < sx; nx++)
            {
                double pha = pha1 * sqrt(preFact - X[ny * sx + nx] / scalex - Y[ny * sx + nx] / scaley);
                PM[ny * sx + nx].real(cos(pha));
                PM[ny * sx + nx].imag(sin(pha));
            }
        }

#if (USEOMP)
    }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _T1, typename _T2> void FresnelcalcPhaseMasks(_T1 *H1, _T1 *H2, _T1 *H3, _T2 *X, _T2 *Y, int sx, int sy, double dist, double px, double wavelen)
{
    H1->real(cos(2.0 * CV_PI / wavelen * dist) / (wavelen * dist));
    H1->imag(sin(2.0 * CV_PI / wavelen * dist) / (wavelen * dist));
#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
    {
#endif
    double pha1 = CV_PI * wavelen * dist;
    double pha2 = CV_PI / wavelen / dist * px * px ;
    double scalex = sx * sx * px * px;
    double scaley = sy * sy * px * px;
#if (USEOMP)
#pragma omp for schedule(guided)
#endif
        for (int ny = 0; ny < sy; ny++)
        {
            for (int nx = 0; nx < sx; nx++)
            {
                double pha = pha1 * (X[ny * sx + nx] / scalex + Y[ny * sx + nx] / scaley);
                H2[ny * sx + nx].real(cos(pha));
                H2[ny * sx + nx].imag(sin(pha));
                pha = pha2 * (X[ny * sx + nx] + Y[ny * sx + nx]);
                H3[ny * sx + nx].real(cos(pha));
                H3[ny * sx + nx].imag(sin(pha));
            }
        }

#if (USEOMP)
    }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString Holography::FresnelCalcPropDoc = QObject::tr("A simple free space Fresnel propagation using 1 fft");

/*static*/ ito::RetVal Holography::FresnelCalcPropParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::Param param;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("dObjOut", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "2d complex output data field"));
    paramsMand->append(ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::In, 0, 100000, 1000, "field size in x-direction in mu"));
    paramsMand->append(ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::In, 0, 100000, 1000, "field size in y-direction in mu"));
    paramsMand->append( ito::Param("dist", ito::ParamBase::Double | ito::ParamBase::In, -1.0e64, 1.0e64, 1000.0, "Propagation distance in mu") );
    paramsMand->append( ito::Param("pixelsize", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 100000.0, 5.0, "pixel i.e. sampling spacing of input object, returns sampling spacing after propagation in mu") );
    paramsMand->append( ito::Param("wavelen", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e10, 0.6328, "wavelength used for propagation in mu") );

    paramsOpt->append(ito::Param("makeSquare", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, "make square propagators only"));
    paramsOpt->append(ito::Param("dtype", ito::ParamBase::Int | ito::ParamBase::In, ito::tComplex64, ito::tComplex128, ito::tComplex128, "data type for phase masks"));
    paramsOpt->append(ito::Param("h1Re", ito::ParamBase::Double | ito::ParamBase::In | ito::ParamBase::Out, -1.0e208, 1.0e208, 0.0, "Real part of 'h1' phase factor"));
    paramsOpt->append(ito::Param("h1Im", ito::ParamBase::Double | ito::ParamBase::In | ito::ParamBase::Out, -1.0e208, 1.0e208, 0.0, "Imag part of 'h1' phase factor"));

    paramsOut->append(ito::Param("objSampleSize", ito::ParamBase::Double | ito::ParamBase::Out, 0.0, 1.0e208, 0.001, "Sampling step size in object plane"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal Holography::FresnelCalcProp(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal;
    ito::DataObject *outFieldPtr = paramsMand->at(0).getVal<ito::DataObject*>(); //0, 1
    ito::DataObject outField;
    int sizex = paramsMand->at(1).getVal<int>();
    int sizey = paramsMand->at(2).getVal<int>();
    double dist = paramsMand->at(3).getVal<double>();
    double sampling = paramsMand->at(4).getVal<double>();
    double wavelen = paramsMand->at(5).getVal<double>();
    int dtype = paramsOpt->at(1).getVal<int>();

    ito::DataObject fXY;
    ito::complex64 H164;
    ito::complex128 H1128;
    if (sizex != sizey && paramsOpt->at(0).getVal<int>())
    {
        if (sizex > sizey)
            sizey = sizex;
        else
            sizex = sizey;
    }
    switch (dtype)
    {
        case ito::tComplex64:
            outField = ito::DataObject(2, sizey, sizex, ito::tComplex64);
            fXY = ito::DataObject(2, sizey, sizex, ito::tFloat32);
            calcFieldXY<ito::float32>(fXY.rowPtr<ito::float32>(0, 0), fXY.rowPtr<ito::float32>(1, 0), sizex, sizey);
            FresnelcalcPhaseMasks<ito::complex64, ito::float32>(&H164, (ito::complex64*)outField.rowPtr(0, 0), (ito::complex64*)outField.rowPtr(1, 0),
                (ito::float32*)fXY.rowPtr(0, 0), (ito::float32*)fXY.rowPtr(1, 0),
                sizex, sizey, dist, sampling, wavelen);
            (*paramsOpt)[2].setVal<double>(H164.real());
            (*paramsOpt)[3].setVal<double>(H164.imag());
        break;

        case ito::tComplex128:
            outField = ito::DataObject(2, sizey, sizex, ito::tComplex128);
            fXY = ito::DataObject(2, sizey, sizex, ito::tFloat64);
            calcFieldXY<ito::float64>(fXY.rowPtr<ito::float64>(0, 0), fXY.rowPtr<ito::float64>(1, 0), sizex, sizey);
            FresnelcalcPhaseMasks<ito::complex128, ito::float64>(&H1128, (ito::complex128*)outField.rowPtr(0, 0), (ito::complex128*)outField.rowPtr(1, 0),
                (ito::float64*)fXY.rowPtr(0, 0), (ito::float64*)fXY.rowPtr(1, 0),
                sizex, sizey, dist, sampling, wavelen);
            (*paramsOpt)[2].setVal<double>(H1128.real());
            (*paramsOpt)[3].setVal<double>(H1128.imag());
        break;
    }

    if (sizex != sizey)
        retVal += ito::RetVal(ito::retWarning, 0, tr("warning using non square propagators will result in non square pixels in object plane").toLatin1().data());
    (*paramsOut)[0].setVal<double>((fabs(dist) * wavelen) / (sampling * sizex));

    *outFieldPtr = outField;

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString Holography::FresnelDoPropDoc = QObject::tr("A simple free space Fresnel propagation using 1 fft");

/*static*/ ito::RetVal Holography::FresnelDoPropParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);
    ito::Param param;
    retVal += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retVal.containsError()) return retVal;

    paramsMand->append(ito::Param("inpField", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d complex input data field"));
    paramsMand->append(ito::Param("outpField", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "2d complex output data field"));
    paramsMand->append(ito::Param("propagators", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d complex data field with propagator phase masks"));

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal Holography::FresnelDoProp(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);
    ito::DataObject *inpObjPtr = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *outpObjPtr = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::DataObject *pMasksPtr = paramsMand->at(2).getVal<ito::DataObject*>();
    ito::DataObject squareField, tmpObj;;
    int sx0 = 0, sy0 = 0, isizex = 0, isizey = 0;
    int sdiffy = 0, sdiffx = 0;
    int roi1[4], roi2[4];
    int limits[6] = { -1, 0, 0, 0, 0, 0 };

    QVector<ito::ParamBase> filterParamsMand0(0), filterParamsMand1(0);
    QVector<ito::ParamBase> filterParamsOpt0(0), filterParamsOpt1(0);
    QVector<ito::ParamBase> filterParamsOut0(0), filterParamsOut1(0);

    if (!inpObjPtr || !pMasksPtr || inpObjPtr->getSize(0) == -1 && inpObjPtr->getSize(1) == -1)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("input object empty").toLatin1().data());
        goto end;
    }
    if (inpObjPtr->getDims() > 2)
    {
        *inpObjPtr = inpObjPtr->squeeze();
        if (inpObjPtr->getDims() > 2)
        {
            retVal += ito::RetVal(ito::retError, 0, tr("input object must be two dimensional").toLatin1().data());
            goto end;
        }
    }

    retVal += ito::dObjHelper::verify2DDataObject(inpObjPtr, "inpField", pMasksPtr->getSize(1), pMasksPtr->getSize(1),
        pMasksPtr->getSize(2), pMasksPtr->getSize(2), 0);
    if (retVal.containsWarningOrError())
    {
        if (inpObjPtr->getSize(0) != inpObjPtr->getSize(1) && pMasksPtr->getSize(2) >= inpObjPtr->getSize(1) && pMasksPtr->getSize(1) >= inpObjPtr->getSize(0))
        {
            retVal = ito::retOk;
            squareField.zeros(pMasksPtr->getSize(1), pMasksPtr->getSize(2), inpObjPtr->getType());
            isizex = pMasksPtr->getSize(1);
            isizey = inpObjPtr->getSize(0);
            sdiffy = pMasksPtr->getSize(1) - inpObjPtr->getSize(0);
            sdiffx = pMasksPtr->getSize(2) - inpObjPtr->getSize(1);
            if (pMasksPtr->getSize(1) > inpObjPtr->getSize(0))
                sy0 = sdiffy / 2;
            if (pMasksPtr->getSize(2) > inpObjPtr->getSize(1))
                sx0 = sdiffx / 2;
            roi1[0] = -sy0;
            roi1[1] = -(sdiffy - sy0);
            roi1[2] = -sx0;
            roi1[3] = -(sdiffx - sx0);
            squareField.adjustROI(2, roi1);
            retVal += inpObjPtr->deepCopyPartial(squareField);
            if (retVal.containsError())
                goto end;
            roi2[0] = sy0;
            roi2[1] = (sdiffy - sy0);
            roi2[2] = sx0;
            roi2[3] = (sdiffx - sx0);
            squareField.adjustROI(2, roi2);
            inpObjPtr = &squareField;
        }
        else
        {
            goto end;
        }
    }

    retVal += apiFilterParamBase("fftw2D", &filterParamsMand0, &filterParamsOpt0, &filterParamsOut0);
    if (retVal.containsError())
        goto end;
    retVal += apiFilterParamBase("fftshift", &filterParamsMand1, &filterParamsOpt1, &filterParamsOut1);
    if (retVal.containsError())
        goto end;

    retVal += inpObjPtr->copyTo(tmpObj);
    if (outpObjPtr != inpObjPtr)
    {
        retVal += tmpObj.convertTo(*outpObjPtr, pMasksPtr->getType());
    }
    else if (inpObjPtr->getType() != pMasksPtr->getType())
    {
        retVal += tmpObj.convertTo(*outpObjPtr, pMasksPtr->getType());
    }

    if (retVal.containsError())
        goto end;

    pMasksPtr->adjustROI(3, limits);
    *outpObjPtr = outpObjPtr->mul(*pMasksPtr);
    filterParamsMand1[0].setVal<ito::DataObject*>(outpObjPtr);
    retVal += apiFilterCall("ifftshift", &filterParamsMand1, &filterParamsOpt1, &filterParamsOut1);
    filterParamsMand0[0].setVal<ito::DataObject*>(outpObjPtr);
    filterParamsMand0[1].setVal<ito::DataObject*>(outpObjPtr);
    retVal += apiFilterCall("fftw2D", &filterParamsMand0, &filterParamsOpt0, &filterParamsOut0);
    if (retVal.containsError())
        goto end;
    filterParamsMand1[0].setVal<ito::DataObject*>(outpObjPtr);
    retVal += apiFilterCall("fftshift", &filterParamsMand1, &filterParamsOpt1, &filterParamsOut1);
    limits[0] = 1;
    limits[1] = -1;
    pMasksPtr->adjustROI(3, limits);
    *outpObjPtr = outpObjPtr->mul(*pMasksPtr);
    limits[0] = 0;
    limits[1] = 1;
    pMasksPtr->adjustROI(3, limits);

    if (sx0 != 0 || sy0 != 0)
    {
        squareField = ito::DataObject(outpObjPtr->getSize(0), outpObjPtr->getSize(1), outpObjPtr->getType());
        retVal += outpObjPtr->deepCopyPartial(squareField);
        if (retVal.containsError())
            goto end;
        roi1[0] = -sy0;
        roi1[1] = -(sdiffy - sy0);
        roi1[2] = -sx0;
        roi1[3] = -(sdiffx - sx0);
        squareField.adjustROI(2, roi1);
        *outpObjPtr = ito::DataObject(isizey, isizex, outpObjPtr->getType());
        *outpObjPtr = squareField;
    }

end:
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString Holography::RSCalcPropDoc = QObject::tr("A simple free space Rayleigh-Sommerfeld propagation using 2 fft");

/*static*/ ito::RetVal Holography::RSCalcPropParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval(ito::retOk);
    ito::Param param;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("dObjOut", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "2d complex output data field"));
    paramsMand->append(ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::In, 0, 10000, 1000, "field size in x-direction in mu"));
    paramsMand->append(ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::In, 0, 10000, 1000, "field size in y-direction in mu"));
    paramsMand->append(ito::Param("dist", ito::ParamBase::Double | ito::ParamBase::In, -1.0e64, 1.0e64, 1000.0, "Propagation distance in mu"));
    paramsMand->append(ito::Param("pixelsize", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 100000.0, 5.0, "pixel i.e. sampling spacing of input object, returns sampling spacing after propagation in mu"));
    paramsMand->append(ito::Param("wavelen", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0e10, 0.6328, "wavelength used for propagation in mu"));

    paramsOpt->append(ito::Param("dtype", ito::ParamBase::Int | ito::ParamBase::In, ito::tComplex64, ito::tComplex128, ito::tComplex128, "data type for phase masks"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal Holography::RSCalcProp(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal;
    ito::DataObject *outFieldPtr = paramsMand->at(0).getVal<ito::DataObject*>(); //0, 1
    ito::DataObject outField;
    int sizex = paramsMand->at(1).getVal<int>();
    int sizey = paramsMand->at(2).getVal<int>();
    double dist = paramsMand->at(3).getVal<double>();
    double sampling = paramsMand->at(4).getVal<double>();
    double wavelen = paramsMand->at(5).getVal<double>();
    int dtype = paramsOpt->at(0).getVal<int>();

    ito::DataObject fXY;
    switch (dtype)
    {
        case ito::tComplex64:
            outField = ito::DataObject(sizey, sizex, ito::tComplex64);
            fXY = ito::DataObject(2, sizey, sizex, ito::tFloat32);
            calcFieldXY<ito::float32>(fXY.rowPtr<ito::float32>(0, 0), fXY.rowPtr<ito::float32>(1, 0), sizex, sizey);
            RScalcPhaseMask<ito::complex64, ito::float32>((ito::complex64*)outField.rowPtr(0, 0),
                (ito::float32*)fXY.rowPtr(0, 0), (ito::float32*)fXY.rowPtr(1, 0),
                sizex, sizey, dist, sampling, wavelen);
            break;

        case ito::tComplex128:
            outField = ito::DataObject(sizey, sizex, ito::tComplex128);
            fXY = ito::DataObject(2, sizey, sizex, ito::tFloat64);
            calcFieldXY<ito::float64>(fXY.rowPtr<ito::float64>(0, 0), fXY.rowPtr<ito::float64>(1, 0), sizex, sizey);
            RScalcPhaseMask<ito::complex128, ito::float64>((ito::complex128*)outField.rowPtr(0, 0),
                (ito::float64*)fXY.rowPtr(0, 0), (ito::float64*)fXY.rowPtr(1, 0),
                sizex, sizey, dist, sampling, wavelen);
        break;
    }

    *outFieldPtr = outField;

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString Holography::RSDoPropDoc = QObject::tr("A simple free space Rayleigh-Sommerfeld propagation using 2 fft");

/*static*/ ito::RetVal Holography::RSDoPropParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);
    ito::Param param;
    retVal += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retVal.containsError()) return retVal;

    paramsMand->append(ito::Param("inpField", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d complex input data field"));
    paramsMand->append(ito::Param("outpField", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "2d complex output data field"));
    paramsMand->append(ito::Param("propagator", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "2d complex data field with propagator phase mask"));

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal Holography::RSDoProp(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);
    ito::DataObject *inpObjPtr = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *outpObjPtr = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::DataObject *pMasksPtr = paramsMand->at(2).getVal<ito::DataObject*>();

    QVector<ito::ParamBase> fPMand1(0), fPMand2(0);
    QVector<ito::ParamBase> fPOpt1(0), fPOpt2(0);
    QVector<ito::ParamBase> fPOut1(0), fPOut2(0);

    if (!inpObjPtr || !pMasksPtr || inpObjPtr->getSize(0) == -1 && inpObjPtr->getSize(1) == -1)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("input object empty").toLatin1().data());
        goto end;
    }
    retVal += ito::dObjHelper::verify2DDataObject(pMasksPtr, "propagators", inpObjPtr->getSize(0), inpObjPtr->getSize(0),
        inpObjPtr->getSize(1), inpObjPtr->getSize(1), 2, ito::tComplex64, ito::tComplex128);

    retVal += apiFilterParamBase("fftw2D", &fPMand1, &fPOpt1, &fPOut1);
    if (retVal.containsError())
        goto end;

    if (outpObjPtr != inpObjPtr)
    {
        ito::DataObject tmpObj;
        retVal += inpObjPtr->copyTo(tmpObj);
        retVal += tmpObj.convertTo(*outpObjPtr, pMasksPtr->getType());
    }
    else if (inpObjPtr->getType() != pMasksPtr->getType())
    {
        ito::DataObject tmpObj;
        retVal += inpObjPtr->copyTo(tmpObj);
        retVal += tmpObj.convertTo(*outpObjPtr, pMasksPtr->getType());
    }
    if (retVal.containsError())
        goto end;

//    *outpObjPtr = outpObjPtr->mul(pMasksPtr->adjustROI(3, limits));
    fPMand1[0].setVal<ito::DataObject*>(outpObjPtr);
    fPMand1[1].setVal<ito::DataObject*>(outpObjPtr);
    retVal += apiFilterCall("fftw2D", &fPMand1, &fPOpt1, &fPOut1);
    if (retVal.containsError())
        goto end;
    retVal += apiFilterParamBase("fftshift", &fPMand2, &fPOpt2, &fPOut2);
    if (retVal.containsError())
        goto end;
    fPMand2[0].setVal<ito::DataObject*>(outpObjPtr);
    retVal += apiFilterCall("fftshift", &fPMand2, &fPOpt2, &fPOut2);
    if (retVal.containsError())
        goto end;
    *outpObjPtr = outpObjPtr->mul(*pMasksPtr);
    fPMand2[0].setVal<ito::DataObject*>(outpObjPtr);
    retVal += apiFilterCall("ifftshift", &fPMand2, &fPOpt2, &fPOut2);
    if (retVal.containsError())
        goto end;
    fPMand1[0].setVal<ito::DataObject*>(outpObjPtr);
    fPMand1[1].setVal<ito::DataObject*>(outpObjPtr);
    retVal += apiFilterCall("ifftw2D", &fPMand1, &fPOpt1, &fPOut1);
    if (retVal.containsError())
        goto end;
end:
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
