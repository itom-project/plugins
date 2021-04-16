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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "dic.h"
#include "DataObject/dataObjectFuncs.h"
#include "dicInterpolation.hu"
#include "pluginVersion.h"


#ifdef USEOPENMP
    #define useomp 1
#else
    #define useomp 0
#endif
#ifdef USECUDA
    bool hasCuda = 1;
#else
    bool hasCuda = 0;
#endif
int NTHREADS = 2;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DICInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(DIC)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DICInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(DIC)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
DICInterface::DICInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("DIC");

    m_description = QObject::tr("Algorithms used for digital image correlation");
    m_detaildescription = QObject::tr("This DLL contains several algorithms for digital image correlation.");
    m_author = "UFAL, Universidade Federal de Alagoas";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL");
    m_aboutThis = QObject::tr("N.A.");

    NTHREADS = QThread::idealThreadCount();
}

//----------------------------------------------------------------------------------------------------------------------------------
DICInterface::~DICInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(DICInterface, DICInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
DIC::DIC() : AddInAlgo(), m_lastDevice(0)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
DIC::~DIC()
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
*/
ito::RetVal DIC::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(DIC::DICInitialGuess, DIC::DICInitialGuessParams, tr("Initial guess of displacement (pixel accurate)"));
    m_filterList.insert("DICInitialGuess", filter);
    filter = new FilterDef(DIC::DICInterpolation, DIC::DICInterpolationParams, tr("Subpixel interpolation of intensity values"));
    m_filterList.insert("DICInterpolation", filter);
    filter = new FilterDef(DIC::DICDisplacement, DIC::DICDisplacementParams, tr("Calculation of displacement fields"));
    m_filterList.insert("DICDisplacement", filter);
    filter = new FilterDef(DIC::DICDisplacementFF, DIC::DICDisplacementFFParams, tr("Calculation of full field displacement"));
    m_filterList.insert("DICDisplacementFF", filter);
    filter = new FilterDef(DIC::DICGenerateImage, DIC::DICGenerateImageParams, tr("Generation of test images for DIC"));
    m_filterList.insert("DICGenImages", filter);
    filter = new FilterDef(DIC::DICGenerateProjImage, DIC::DICGenerateProjImageParams, tr("Generation of test images for DIC"));
    m_filterList.insert("DICGenProjectedImages", filter);
    filter = new FilterDef(DIC::DICSplineCoeffs, DIC::DICSplineCoeffsParams, tr("Calculation of coefficients / cubic spline interpolation"));
    m_filterList.insert("DICSplineCoeffs", filter);
    filter = new FilterDef(DIC::DICSplineVals, DIC::DICSplineValsParams, tr("Calculation of interpolated values of cubic spline interpolation"));
    m_filterList.insert("DICSplineVals", filter);
    filter = new FilterDef(DIC::DICDeformation, DIC::DICDeformationParams, tr("Calculation of deformation field from given displacement field"));
    m_filterList.insert("DICDeformation", filter);

#ifdef USECUDA
    if (hasCuda)
    {
        retval += InitCudaDevice(m_cudaDevices);
    }
#endif

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DIC::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal normalizeInt(ito::DataObject &dObjIn, ito::DataObject &dObjOut, ito::float64 *intSqr = NULL)
{
    ito::RetVal retval(ito::retOk);
    ito::float64 intAvg = 0, intSqrSum = 0;
    retval += ito::dObjHelper::meanValue(&dObjIn, intAvg, 0);

    if (&dObjIn != &dObjOut)
        retval += dObjIn.copyTo(dObjOut);
    dObjOut -= intAvg;

    ito::DataObject dObjSqr = dObjOut.mul(dObjOut);

    // maybe we need to redo this part as we actually want to compute the square root of the sums of the intensities squares ...
    // we abuse here the average function to do so

    retval += ito::dObjHelper::meanValue(&dObjSqr, intSqrSum, 0);
    intSqrSum = sqrt(intSqrSum * dObjOut.getSize(1) * dObjOut.getSize(0));
    if (intSqr)
        *intSqr = intSqrSum;
    dObjOut *= 1.0 / intSqrSum;

    // alternative version using maximum value for normalization, somewhat not working this way, needs checking
    /*
    ito::float64 maxVal;
    ito::uint32 loc[3];
    retval += ito::dObjHelper::maxValue(&dObjOut, maxVal, loc);
    if (maxVal != 0)
    dObjOut *= 1.0 / maxVal;
    */

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------