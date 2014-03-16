/* ********************************************************************
    Plugin "BasicFilters" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

/*! \file BasicFilters.cpp
   \brief   This file contains the itomflters class and interface definitions.
   
   \author ITO 
   \date 12.2011
*/

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "BasicFilters.h"
#include <QtCore/QtPlugin>
#include "pluginVersion.h"

int NTHREADS = 2;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFiltersInterface::getAddInInst(ito::AddInBase **addInInst)
{
    BasicFilters* newInst = new BasicFilters();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    //fill basePlugin-pointer of every registered filter
    QHashIterator<QString, ito::AddInAlgo::FilterDef *> i(newInst->m_filterList);
    while (i.hasNext()) 
    {
        i.next();
        i.value()->m_pBasePlugin = this;
    }

    //fill basePlugin-pointer of every registered algo widget
    QHashIterator<QString, ito::AddInAlgo::AlgoWidgetDef *> j(newInst->m_algoWidgetList);
    while (j.hasNext()) 
    {
        j.next();
        j.value()->m_pBasePlugin = this;
    }
    
    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFiltersInterface::closeThisInst(ito::AddInBase **addInInst)
{
    if (*addInInst)
    {
        delete ((BasicFilters *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
BasicFiltersInterface::BasicFiltersInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("BasicFilters");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This plugin provides several basic filter calculations for itom::dataObject. These are for instance: \n\
- merging of planes\n\
- swap byte order of objects \n\
- resample slices from dataObjects \n\
- mean value filter along axis \n\
\n\
This plugin does not have any unusual dependencies.";

    m_description = QObject::tr("ITO developed filter-functions for dateObjects");
    m_detaildescription = QObject::tr(docstring);
    m_author = "W. Lyda, T. Boettcher, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL");
    m_aboutThis = QObject::tr("N.A.");       
    
    NTHREADS  = QThread::idealThreadCount();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
BasicFiltersInterface::~BasicFiltersInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(BasicFiltersInterface, BasicFiltersInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
BasicFilters::BasicFilters() : AddInAlgo()
{   
}

//----------------------------------------------------------------------------------------------------------------------------------
BasicFilters::~BasicFilters() 
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();
}

const char* BasicFilters::replaceInfAndNaNDoc= "replaces infinite and/or nan-values by values of second matrix. \n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
const char* BasicFilters::flaten3Dto2DDoc = "Flattens a z-Stack of Mx1xN or MxNx1 matrixes to NxM or MxN. \n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
const char* BasicFilters::swapByteOrderDoc= "Swap byte order for input image. \n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
const char* BasicFilters::mergeColorPlaneDoc= "Merge 3 or 4 color planes to a single tRGBA32 or tInt32-plane. \n\
\n\
If second object is tInt32 and of right size in x and y, the stack object will be convertet to tInt32. In all other cases the object will be tRGBA32 \n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
const char* BasicFilters::calcMeanOverZDoc= "Calculate meanValue of a 3D-Object stack in z-directon. \n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
const char* BasicFilters::calcObjSliceDoc= "Interpolate 1D-slice from along the defined line from a 2D-Object. \n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
const char* BasicFilters::clipValueDoc = "clip values outside or inside of minValue and maxValue to newValue (default = 0) \n\
\n\
Depending on the parameter 'insideFlag', this filter sets all values within (1) or outside (0) of the range (minValue, maxValue) to \
the value given by 'newValue'. In both cases the range boundaries are not clipped and replaced. If clipping is executed outside of range, \
NaN and Inf values are replaced as well (floating point data objects only). This filter supports only real value data types.";
//----------------------------------------------------------------------------------------------------------------------------------
const char* BasicFilters::clipAbyBDoc = "clip values of image A to newValue (default = 0) outside or inside of minValue and maxValue in image B \n\
\n\
Depending on the parameter 'insideFlag', this filter sets all values in image A depending on image B within (1) or outside (0) of the range (minValue, maxValue) to \
the value given by 'newValue'. In both cases the range boundaries are not clipped and replaced. If clipping is executed outside of range, \
NaN and Inf values are replaced as well (floating point data objects only). This filter supports only real value data types.";
//----------------------------------------------------------------------------------------------------------------------------------
const char *BasicFilters::calcHistDoc = "calculates histgram of real input data object";
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(BasicFilters::replaceInfAndNaN, BasicFilters::replaceInfAndNaNParams, tr(replaceInfAndNaNDoc));
    m_filterList.insert("replaceInfAndNaN", filter);
    filter = new FilterDef(BasicFilters::flaten3Dto2D, BasicFilters::stdParams2Objects, tr(flaten3Dto2DDoc));
    m_filterList.insert("flatten3Dto2D", filter);
    filter = new FilterDef(BasicFilters::swapByteOrder, BasicFilters::stdParams2Objects, tr(swapByteOrderDoc));
    m_filterList.insert("swapByteOrder", filter);
    filter = new FilterDef(BasicFilters::mergeColorPlane, BasicFilters::mergeColorPlanesParams, tr(mergeColorPlaneDoc));
    m_filterList.insert("mergeColorPlane", filter);

    filter = new FilterDef(BasicFilters::calcMeanOverZ, BasicFilters::calcMeanOverZParams, tr(calcMeanOverZDoc));
    m_filterList.insert("calcMeanZ", filter);
    filter = new FilterDef(BasicFilters::calcObjSlice, BasicFilters::calcObjSliceParams, tr(calcObjSliceDoc));
    m_filterList.insert("calcObjSlice", filter);

    filter = new FilterDef(BasicFilters::calcHistFilter, BasicFilters::calcHistParams, tr(calcHistDoc));
    m_filterList.insert("calcHist", filter);

    filter = new FilterDef(BasicFilters::clipValueFilter, BasicFilters::clipValueFilterParams, tr(clipValueDoc));
    m_filterList.insert("clipValues", filter);

    filter = new FilterDef(BasicFilters::clipAbyBFilter, BasicFilters::clipAbyBFilterParams, tr(clipAbyBDoc));
    m_filterList.insert("clipAbyB", filter);

    filter = new FilterDef(BasicFilters::genericLowValueFilter, BasicFilters::genericStdParams, tr("Set each pixel to the lowest value within the kernel (x ,y) using the generic mcpp filter engine"));
    m_filterList.insert("lowValueFilter", filter);
    filter = new FilterDef(BasicFilters::genericHighValueFilter, BasicFilters::genericStdParams, tr("Set each pixel to the highest value within the kernel (x ,y) using the generic mcpp filter engine"));
    m_filterList.insert("highValueFilter", filter);
    filter = new FilterDef(BasicFilters::genericMedianFilter, BasicFilters::genericStdParams, tr("Performs a median filter with kernelsize (x ,y) using the generic mcpp filter engine"));
    m_filterList.insert("medianFilter", filter);
    filter = new FilterDef(BasicFilters::genericLowPassFilter, BasicFilters::genericStdParams, tr("Performs a low-pass filter with kernelsize (x ,y) using the generic mcpp filter engine"));
    m_filterList.insert("lowPassFilter", filter);

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \date
*/
ito::RetVal BasicFilters::stdParams2Objects(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("scrImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input image").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output image").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------