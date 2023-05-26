/* ********************************************************************
    Plugin "BasicFilters" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

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
#include "gitVersion.h"

//----------------------------------------------------------------------------------------------------------------------------------
BasicFiltersInterface::BasicFiltersInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("BasicFilters");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"This plugin provides several basic filter calculations for itom::dataObject. These are for instance: \n\
\n\
* merging of planes\n\
* swap byte order of objects \n\
* resample slices from dataObjects \n\
* mean value filter along axis \n\
\n\
This plugin does not have any unusual dependencies.";
*/
    m_description = QObject::tr("ITO developed filter-functions for data objects");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"This plugin provides several basic filter calculations for itom::dataObject. These are for instance: \n\
\n\
* merging of planes\n\
* swap byte order of objects \n\
* resample slices from dataObjects \n\
* mean value filter along axis \n\
\n\
This plugin does not have any unusual dependencies.");

    m_author = "W. Lyda, T. Boettcher, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL");
    m_aboutThis = QObject::tr(GITVERSION);
}

//----------------------------------------------------------------------------------------------------------------------------------
BasicFiltersInterface::~BasicFiltersInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFiltersInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(BasicFilters)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFiltersInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(BasicFilters)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------


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

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::replaceInfAndNaNDoc= QObject::tr("replaces infinite and/or nan-values by values of second matrix. \n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::flaten3Dto2DDoc = QObject::tr("Flattens a z-Stack of Mx1xN or MxNx1 matrixes to NxM or MxN. \n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::swapByteOrderDoc= QObject::tr("Swap byte order for input image. \n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::mergeColorPlaneDoc= QObject::tr("Merge 3 or 4 color planes to a single tRGBA32 or tInt32-plane. \n\
\n\
If second object is tInt32 and of right size in x and y, the stack object will be convertet to tInt32. In all other cases the object will be tRGBA32 \n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::calcMeanOverZDoc= QObject::tr("Calculate mean value (and optional standard deviation) of a 3D data object in z-direction. \n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::calcObjSliceDoc= QObject::tr("Interpolate 1D-slice from along the defined line from a 2D-Object. \n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::clipValueDoc = QObject::tr("clip values outside or inside of minValue and maxValue to newValue (default = 0) \n\
\n\
Depending on the parameter 'insideFlag', this filter sets all values within (1) or outside (0) of the range (minValue, maxValue) to \
the value given by 'newValue'. In both cases the range boundaries are not clipped and replaced. If clipping is executed outside of range, \
NaN and Inf values are replaced as well (floating point data objects only). This filter supports only real value data types.");

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::clipAbyBDoc = QObject::tr("clip values of image A to newValue (default = 0) outside or inside of minValue and maxValue in image B \n\
\n\
Depending on the parameter 'insideFlag', this filter sets all values in image A depending on image B within (1) or outside (0) of the range (minValue, maxValue) to \
the value given by 'newValue'. In both cases the range boundaries are not clipped and replaced. If clipping is executed outside of range, \
NaN and Inf values are replaced as well (floating point data objects only). This filter supports only real value data types.");

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::fillGeometricDoc = QObject::tr("fills a ROI, which defined by a geometric primitive, of the given dataObject with a defined value\n\
\n\
Depending on the parameter 'insideFlag', this filter sets all values of the dataObject depending on the geometric primitiv within (1) or outside (2) or both (3) to \
the value given by 'newValueInside' and 'newValueOutside'. The 'edgeFlag' is currently not used but shall manage the edge handling of primitive.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(BasicFilters::replaceInfAndNaN, BasicFilters::replaceInfAndNaNParams, replaceInfAndNaNDoc);
    m_filterList.insert("replaceInfAndNaN", filter);

    filter = new FilterDef(BasicFilters::flaten3Dto2D, BasicFilters::stdParams2Objects, flaten3Dto2DDoc);
    m_filterList.insert("flatten3Dto2D", filter);

    filter = new FilterDef(BasicFilters::swapByteOrder, BasicFilters::stdParams2Objects, swapByteOrderDoc);
    m_filterList.insert("swapByteOrder", filter);

    filter = new FilterDef(BasicFilters::mergeColorPlane, BasicFilters::mergeColorPlanesParams, mergeColorPlaneDoc);
    m_filterList.insert("mergeColorPlane", filter);

    filter = new FilterDef(BasicFilters::calcMeanOverZ, BasicFilters::calcMeanOverZParams, calcMeanOverZDoc);
    m_filterList.insert("calcMeanZ", filter);

    filter = new FilterDef(BasicFilters::calcObjSlice, BasicFilters::calcObjSliceParams, calcObjSliceDoc);
    m_filterList.insert("calcObjSlice", filter);

    filter = new FilterDef(BasicFilters::calcHistFilter, BasicFilters::calcHistParams, calcHistDoc);
    m_filterList.insert("calcHist", filter);

    filter = new FilterDef(BasicFilters::calcRadialMeanFilter, BasicFilters::calcRadialMeanFilterParams, calcRadialMeanFilterDoc);
    m_filterList.insert("calcRadialMean", filter);

    filter = new FilterDef(BasicFilters::clipValueFilter, BasicFilters::clipValueFilterParams, clipValueDoc);
    m_filterList.insert("clipValues", filter);

    filter = new FilterDef(BasicFilters::clipAbyBFilter, BasicFilters::clipAbyBFilterParams, clipAbyBDoc);
    m_filterList.insert("clipAbyB", filter);

    filter = new FilterDef(BasicFilters::genericLowValueFilter, BasicFilters::genericStdParams, genericLowValueFilterDoc);
    m_filterList.insert("lowValueFilter", filter);

    filter = new FilterDef(BasicFilters::genericHighValueFilter, BasicFilters::genericStdParams, genericHighValueFilterDoc);
    m_filterList.insert("highValueFilter", filter);

    filter = new FilterDef(BasicFilters::genericMedianFilter, BasicFilters::genericStdParams, genericMedianFilterDoc);
    m_filterList.insert("medianFilter", filter);

    filter = new FilterDef(BasicFilters::genericLowPassFilter, BasicFilters::genericStdParams, genericLowPassFilterDoc);
    m_filterList.insert("lowPassFilter", filter);

    filter = new FilterDef(BasicFilters::genericSobelOptFilter, BasicFilters::genericSobelOptParams, tr("Performs a Sobel filtering with kernelsize (3 ,3) using the generic mcpp filter engine"));
    m_filterList.insert("sobelOpt", filter);

    filter = new FilterDef(BasicFilters::genericGaussianEpsilonFilter, BasicFilters::genericGaussianEpsilonParams, genericGaussianEpsilonFilterDoc);
    m_filterList.insert("gaussianFilterEpsilon", filter);

    filter = new FilterDef(BasicFilters::genericGaussianFilter, BasicFilters::genericGaussianParams, genericGaussianFilterDoc);
    m_filterList.insert("gaussianFilter", filter);

    filter = new FilterDef(BasicFilters::spikeMedianFilter, BasicFilters::spikeCompFilterStdParams, spikeMedianFilterDoc);
    m_filterList.insert("spikeMedianFilter", filter);

    filter = new FilterDef(BasicFilters::spikeMeanFilter, BasicFilters::spikeCompFilterStdParams, spikeMeanFilterDoc);
    m_filterList.insert("spikeMeanFilter", filter);

    filter = new FilterDef(BasicFilters::fillGeometricPrimitiv, BasicFilters::fillGeometricParams, fillGeometricDoc);
    m_filterList.insert("fillObject", filter);

    filter = new FilterDef(BasicFilters::labeling, BasicFilters::labelingParams, labelingFilterDoc);
    m_filterList.insert("labeling", filter);

    filter = new FilterDef(BasicFilters::findEllipses, BasicFilters::findEllipsesParams, findEllipsesFilterDoc);
    m_filterList.insert("findEllipses", filter);

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
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("scrImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input image").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output image").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
