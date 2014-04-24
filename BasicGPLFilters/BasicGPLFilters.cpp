/* ********************************************************************
    Plugin "BasicGPLFilters" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.
  
    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public Licence as published by
    the Free Software Foundation; either version 3 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

/*! \file BasicGPLFilters.cpp
   \brief   This file contains the itomflters class and interface definitions.
   
   \author ITO 
   \date 12.2011
*/



#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "BasicGPLFilters.h"
#include <QtCore/QtPlugin>
#include "pluginVersion.h"
#include "DataObject/dataObjectFuncs.h"
int NTHREADS = 2;

ito::int32 BasicGPLFilters::histLess = 0;
ito::int32 BasicGPLFilters::histMore = 0;
ito::int32 BasicGPLFilters::histRemain = 0;
ito::int32 BasicGPLFilters::blackLevel = 0;
ito::int32 BasicGPLFilters::whiteLevel = 0;

//----------------------------------------------------------------------------------------------------------------------------------
BasicGPLFiltersInterface::BasicGPLFiltersInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("BasicGPLFilters");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This plugin provides several basic filter calculations for itom::dataObject. These filteres are derived from other open-source projects under GPL and hence this plugin is also GPL. These are for instance: \n\
\n\
* despeckle via histogramm and median derived from the GIMP 2.8 despeckle-filter\n\
\n\
This plugin does not have any unusual dependencies.";

    m_description = QObject::tr("filter-functions for dateObjects");
    m_detaildescription = QObject::tr(docstring);
    m_author = "W. Lyda, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("GPL 3.0");
    m_aboutThis = QObject::tr("N.A.");       
    
    NTHREADS  = QThread::idealThreadCount();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
BasicGPLFiltersInterface::~BasicGPLFiltersInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicGPLFiltersInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(BasicGPLFilters)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicGPLFiltersInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(BasicGPLFilters)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(BasicGPLFiltersInterface, BasicGPLFiltersInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
BasicGPLFilters::BasicGPLFilters() : AddInAlgo()
{   
}

//----------------------------------------------------------------------------------------------------------------------------------
BasicGPLFilters::~BasicGPLFilters() 
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();
}
//----------------------------------------------------------------------------------------------------------------------------------
const char* BasicGPLFilters::despeckle_adapted= "The filter was implemented based on despeckle filter in gimp 2.8. The filter remove speckle noise from the image \n\
This plug-in selectively performs a median or adaptive box filter on an image. \n\
Michael Sweet <mike@easysw.com>, Copyright 1997-1998 by Michael Sweet \n\
\n";

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicGPLFilters::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(BasicGPLFilters::despeckleAdapted, BasicGPLFilters::despeckleAdaptedParams, tr(despeckle_adapted));
    m_filterList.insert("despeckleAdapted", filter);

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicGPLFilters::close(ItomSharedSemaphore * /*waitCond*/)
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
ito::RetVal BasicGPLFilters::despeckleAdaptedParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("scrImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input image").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output image").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("lowestValue", ito::ParamBase::Int | ito::ParamBase::In, -1, 256, -1, tr("Only values > lowestValue are used").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("highestValue", ito::ParamBase::Int | ito::ParamBase::In, -1, 256, 256, tr("Only values < lowestValue are used").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("radius", ito::ParamBase::Int | ito::ParamBase::In , 1, 30, 1, tr("Radius of filter kernel").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("adapt", ito::ParamBase::Int | ito::ParamBase::In , 0, 1, 1, tr("Adapted used radius between 1 and radius").toLatin1().data());
        paramsOpt->append(param);

    }

    return retval;
}

template<typename _Tp> void BasicGPLFilters::despeckleAdaptedFilterBlock(const cv::Mat* cvMatIn, cv::Mat* cvMatOut, const ito::int32 radius, const bool adaptRadius)
{
    ito::int32 curRadius = radius;
    ito::int32 x, y;
    ito::int32 pos, ymin, ymax, xmin, xmax;

    DespeckleHistogram histogram;

    memset (&histogram, 0, sizeof(histogram));

    const ito::uint8* src;
    ito::uint8* dst;

    src = cvMatIn->ptr<const ito::uint8>(0);
    dst = cvMatOut->ptr<ito::uint8>(0);

    for (y = 0; y < cvMatIn->rows; y++)
    {


        x = 0;
        ymin = std::max<int>(0, y - curRadius);
        ymax = std::min<int>(cvMatIn->rows - 1, y + curRadius);
        xmin = std::max<int>(0, x - curRadius);
        xmax = std::min<int>(cvMatIn->cols - 1, x + curRadius);
        BasicGPLFilters::histLess = 0;
        BasicGPLFilters::histMore = 0;
        BasicGPLFilters::histRemain = 0;
        histogram_clean (&histogram);
        histogram.xmin = xmin;
        histogram.ymin = ymin;
        histogram.xmax = xmax;
        histogram.ymax = ymax;
        add_vals<_Tp>(&histogram, src, cvMatIn->cols, histogram.xmin, histogram.ymin, histogram.xmax, histogram.ymax);

        for (x = 0; x < cvMatIn->cols; x++)
        {
            const ito::uint8 *pixel;

            ymin = std::max<int>(0, y - curRadius);
            ymax = std::min<int>(cvMatIn->rows - 1, y + curRadius);
            xmin = std::max<int>(0, x - curRadius);
            xmax = std::min<int>(cvMatIn->cols - 1, x + curRadius);

            update_histogram<_Tp>(&histogram, src, cvMatIn->cols, xmin, ymin, xmax, ymax);

            pos = (x + y * cvMatIn->cols)* sizeof(_Tp);
            pixel = histogram_get_median(&histogram, src + pos);

            memcpy((dst + pos), pixel, sizeof(_Tp));

            if (adaptRadius)
            {
                if (BasicGPLFilters::histLess >= curRadius || BasicGPLFilters::histMore >= curRadius)
                {
                    if (curRadius < radius)
                        curRadius++;
                }
                else if (curRadius > 1)
                {
                    curRadius--;
                }
            }
        }
    }
}

ito::RetVal BasicGPLFilters::despeckleAdapted(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjIn = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();

    ito::DataObject dObjOut;

    bool createdNewObject = false;

    retval += ito::dObjHelper::verify2DDataObject(dObjIn, "dObjIn", 1, std::numeric_limits<ito::uint16>::max(), 1, std::numeric_limits<ito::uint16>::max(), 
                                                                    3, ito::tUInt8, ito::tUInt16, ito::tRGBA32);
    if(retval.containsError())
    {
        return retval;
    }
    if (dObjIn->getDims() != 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source is not a 2d matrix").toLatin1().data());
    }

    int xsize = dObjIn->getSize(1);
    int ysize = dObjIn->getSize(0);

    ito::RetVal rettemp = ito::dObjHelper::verify2DDataObject(dObjDst, "dObjDst", ysize, ysize, xsize, xsize, 1, dObjIn->getType());
    if (rettemp.containsError() || dObjIn == dObjDst)
    {
        if(dObjDst == NULL) return rettemp;
        createdNewObject = true;

        dObjOut = ito::DataObject(ysize, xsize, dObjIn->getType());
        dObjIn->copyAxisTagsTo(dObjOut);
        dObjIn->copyTagMapTo(dObjOut);
    }
    else
    {
        createdNewObject = false;
        dObjOut = *dObjDst;
        dObjIn->copyAxisTagsTo(dObjOut);
        dObjIn->copyTagMapTo(dObjOut);
    }


    if( (*paramsOpt)[0].getVal<double>() > (*paramsOpt)[1].getVal<double>())
    {
        return ito::RetVal(ito::retError, 0, tr("Error: minValue must be smaller than maxValue").toLatin1().data());
    }
    bool adapt_radius  = (*paramsOpt)[3].getVal<int>() == 1 ? true : false;
    ito::int32 radius  = (*paramsOpt)[2].getVal<int>();
    blackLevel  = (*paramsOpt)[0].getVal<double>();
    whiteLevel  = (*paramsOpt)[1].getVal<double>();


    int z_length = dObjIn->calcNumMats();
    const cv::Mat *cvMatIn  = (cv::Mat *)(dObjIn->get_mdata()[dObjIn->seekMat(0)]);
    cv::Mat *cvMatObj = (cv::Mat *)(dObjOut.get_mdata()[dObjOut.seekMat(0)]);

    switch(dObjIn->getType())
    {
        case ito::tUInt8:
            BasicGPLFilters::despeckleAdaptedFilterBlock<ito::uint8>(cvMatIn, cvMatObj, radius, adapt_radius);
            break;
        case ito::tUInt16:
            BasicGPLFilters::despeckleAdaptedFilterBlock<ito::uint16>(cvMatIn, cvMatObj, radius, adapt_radius);
            break;
        case ito::tRGBA32:
            BasicGPLFilters::despeckleAdaptedFilterBlock<ito::Rgba32>(cvMatIn, cvMatObj, radius, adapt_radius);
            break;
        default:
            return ito::RetVal(ito::retError, 0, tr("DataType not supported for this filter").toLatin1().data());
    }

    if(createdNewObject)
    {
        *dObjDst = dObjOut;
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if(!retval.containsError())
    {
        // Add Protokoll

        //        char prot[81] = {0};
        QString msg = tr("despeckle median with ]%1, %2[, r = %3, adaption = %4").arg(QString::number(blackLevel), QString::number(whiteLevel), QString::number(radius), QString::number((int)adapt_radius));
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------