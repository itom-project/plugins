/* ********************************************************************
    Plugin "BasicGPLFilters" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public Licence as published by
    the Free Software Foundation; either version 3 of the Licence, or (at
    your option) any later version.

    This plugin contains code and algorithms inspired or copied from other open
    source projects under GNU General Public Licence, e.g.

        - GIMP 2.8 under GPL version 3.0 or higher

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

/*! \file BasicGPLFilters.cpp
   \brief   This file contains the BasicGPLFilters class and interface definitions.

   \author W. Lyda, twip-os
   \date 04.2014
*/

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "BasicGPLFilters.h"
#include <QtCore/QtPlugin>
#include "pluginVersion.h"
#include "gitVersion.h"
#include "DataObject/dataObjectFuncs.h"
int NTHREADS = 2;

//----------------------------------------------------------------------------------------------------------------------------------
BasicGPLFiltersInterface::BasicGPLFiltersInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("BasicGPLFilters");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"This plugin provides several basic filter calculations for itom::dataObject. These filteres are derived from other open-source projects under GPL and hence this plugin is also GPL. These are for instance: \n\
\n\
* despeckle via histogramm and median derived from the GIMP 2.8 despeckle-filter by Michael Sweet\n\
\n\
This plugin does not have any unusual dependencies.";
*/
    m_description = QObject::tr("filter-functions for dateObjects");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"This plugin provides several basic filter calculations for itom::dataObject. These filteres are derived from other open-source projects under GPL and hence this plugin is also GPL. These are for instance: \n\
\n\
* despeckle via histogramm and median derived from the GIMP 2.8 despeckle-filter by Michael Sweet\n\
\n\
This plugin does not have any unusual dependencies.");

    m_author = "W. Lyda, twip optical solutions GmbH (plugin)";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("GPL 3.0");
    m_aboutThis = QObject::tr(GITVERSION);

    NTHREADS  = QThread::idealThreadCount();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
\brief Basic interface function
\sa ito::AbstractInterface
\author ITO
\date 04.2014
*/
BasicGPLFiltersInterface::~BasicGPLFiltersInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
\brief Basic interface function
\sa ito::AbstractInterface
\author ITO
\date 04.2014
*/
ito::RetVal BasicGPLFiltersInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(BasicGPLFilters)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
\brief Basic interface function
\sa ito::AbstractInterface
\author ITO
\date 04.2014
*/
ito::RetVal BasicGPLFiltersInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(BasicGPLFilters)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
/*!
\brief Basic interface function
\sa ito::AbstractInterface
\author ITO
\date 04.2014
*/
BasicGPLFilters::BasicGPLFilters() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
\brief Basic interface function
\sa ito::AbstractInterface
\author ITO
\date 04.2014
*/
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
/*!
\brief Variable for filter help string
*/
const char* BasicGPLFilters::despeckle_adapted= "The filter was implemented based on despeckle filter in gimp 2.8. The filter remove speckle noise from the image \n\
This plugin selectively performs a median or adaptive box filter on an image. \n\
Michael Sweet <mike@easysw.com>, Copyright 1997-1998 by Michael Sweet \n\
\n";

//----------------------------------------------------------------------------------------------------------------------------------
/*!
\brief Basic interface function
\sa ito::AbstractInterface
\author ITO
\date 04.2014
*/
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
/*!
\brief Basic interface function
\sa ito::AbstractInterface
\author ITO
\date 04.2014
*/
ito::RetVal BasicGPLFilters::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail Filter parameter for the despeckle algorithm
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author W. Lyda, twip-os
   \date 04.2014
*/
ito::RetVal BasicGPLFilters::despeckleAdaptedParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
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

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail The filter block realizing the despeckle-Algorithm
   \param[in]       filterSettings  The filter settings of type DespeckleSettings
   \param[in]       cvMatIn         The input matrix which shall not be equal to the output matrix
   \param[out]      cvMatOut        The filtered output matrix of same type than input matrix
   \author W. Lyda, twip-os
   \date 04.2014
*/
template<typename _Tp> void BasicGPLFilters::despeckleAdaptedFilterBlock(DespeckleSettings &filterSettings, const cv::Mat* cvMatIn, cv::Mat* cvMatOut)
{
    ito::int32 curRadius = filterSettings.radiusMax;
    ito::int32 x, y;
    ito::int32 pos, ymin, ymax, xmin, xmax;

    DespeckleHistogram histogram;

    memset (&histogram, 0, sizeof(histogram));

    const ito::uint8* src;
    ito::uint8* dst;

    // get the first pixel within the column to be able to deal with ROI-settings
    src = cvMatIn->ptr<const ito::uint8>(0);
    dst = cvMatOut->ptr<ito::uint8>(0);

    // iterate through all rows
    for (y = 0; y < cvMatIn->rows; y++)
    {
        x = 0;
        ymin = std::max<int>(0, y - curRadius);
        ymax = std::min<int>(cvMatIn->rows - 1, y + curRadius);
        xmin = std::max<int>(0, x - curRadius);
        xmax = std::min<int>(cvMatIn->cols - 1, x + curRadius);

        filterSettings.histLess = 0;
        filterSettings.histMore = 0;
        filterSettings.histRemain = 0;

        histogram_clean (histogram);
        histogram.xmin = xmin;
        histogram.ymin = ymin;
        histogram.xmax = xmax;
        histogram.ymax = ymax;
        add_vals<_Tp>(filterSettings, histogram, src, cvMatIn->cols, histogram.xmin, histogram.ymin, histogram.xmax, histogram.ymax);

        // iterate through all colums
        for (x = 0; x < cvMatIn->cols; x++)
        {
            const ito::uint8 *pixel;

            // move kernel
            ymin = std::max<int>(0, y - curRadius);
            ymax = std::min<int>(cvMatIn->rows - 1, y + curRadius);
            xmin = std::max<int>(0, x - curRadius);
            xmax = std::min<int>(cvMatIn->cols - 1, x + curRadius);

            // update currently used histogramm to new kernel
            update_histogram<_Tp>(filterSettings, histogram, src, cvMatIn->cols, xmin, ymin, xmax, ymax);

            // get the current source and destination position relativ to first pixel within the row
            pos = (x + y * cvMatIn->cols)* sizeof(_Tp);
            //pos = x * sizeof(_Tp);

            // retrieve median filtered data for the current pixel
            pixel = histogram_get_median(filterSettings.histRemain, histogram, src + pos);

            // write data output
            memcpy(dst + pos, pixel, sizeof(_Tp));

            // Adapt the kernel size to the current local histogramm
            if (filterSettings.adaptRadius)
            {
                if (filterSettings.histLess >= curRadius || filterSettings.histMore >= curRadius)
                {
                    if (curRadius < filterSettings.radiusMax)
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

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail Filter function wrapper for the despeckle algorithm
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]      paramsOut   Outputvalues, not implemented for this function
   \author W. Lyda, twip-os
   \date 04.2014
*/
ito::RetVal BasicGPLFilters::despeckleAdapted(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    ito::RetVal retval = ito::retOk;

    // Get pointer to input and out put dataObject
    ito::DataObject *dObjIn = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();

    // Define a temporary data object
    ito::DataObject dObjOut;

    // If input is equal to output, we need a temporary dataobject
    bool createdNewObject = false;

    // Check wether source object is of right type and size abd a 2D plane object and return error if not
    retval += ito::dObjHelper::verify2DDataObject(dObjIn, "dObjIn", 1, std::numeric_limits<ito::uint16>::max(), 1, std::numeric_limits<ito::uint16>::max(),
                                                                    3, ito::tUInt8, ito::tUInt16, ito::tRGBA32);
    if (retval.containsError())
    {
        return retval;
    }

    // get the x/y size of the object
    int xsize = dObjIn->getSize(1);
    int ysize = dObjIn->getSize(0);

    // check wether output object is of right type and size
    ito::RetVal rettemp = ito::dObjHelper::verify2DDataObject(dObjDst, "dObjDst", ysize, ysize, xsize, xsize, 1, dObjIn->getType());
    if (rettemp.containsError() || dObjIn == dObjDst) // or if both dataObjects are the same
    {
        // In this case, the output dataObject is not of correct size or type or it is identical (the same) object than the input object.
        // or both are null pointer, so we first check for NULL and in case of a null-pointer we will exit with an error
        if (dObjDst == NULL) return rettemp;

        // The filter does not work in-place, hence we need a temporary dataObject for the output-data
        createdNewObject = true;

        // To avoid to overwrite the input dataObject if the input and output-objects are the same, the create the new and correct
        // output temporarily dObjOut
        dObjOut = ito::DataObject(ysize, xsize, dObjIn->getType());
        dObjIn->copyAxisTagsTo(dObjOut);
        dObjIn->copyTagMapTo(dObjOut);
    }
    else
    {
        // Input and output object are of same size and type but not identical, so we can link the output object to the temporary dObjOut-object
        // via shallow-copy
        createdNewObject = false;
        dObjOut = *dObjDst;
        dObjIn->copyAxisTagsTo(dObjOut);
        dObjIn->copyTagMapTo(dObjOut);
    }

    if ( (*paramsOpt)[0].getVal<double>() > (*paramsOpt)[1].getVal<double>())
    {
        return ito::RetVal(ito::retError, 0, tr("Error: minValue must be smaller than maxValue").toLatin1().data());
    }

    DespeckleSettings filterSettings;
    memset(&filterSettings, 0, sizeof(DespeckleSettings));

    // The additional filter params are copied to the settings struct
    filterSettings.radiusMax    = (*paramsOpt)[2].getVal<int>();
    filterSettings.adaptRadius  = (*paramsOpt)[3].getVal<int>() == 1 ? true : false;
    filterSettings.blackLevel   = (*paramsOpt)[0].getVal<double>();
    filterSettings.whiteLevel   = (*paramsOpt)[1].getVal<double>();

    // Get a pointer to the internal cv::mats of the both objects
    const cv::Mat *cvMatIn  = (cv::Mat *)(dObjIn->get_mdata()[dObjIn->seekMat(0)]);
    cv::Mat *cvMatObj = (cv::Mat *)(dObjOut.get_mdata()[dObjOut.seekMat(0)]);

    // The filter block is templated for 8bit, 16bit and colored objects.
    switch(dObjIn->getType())
    {
        case ito::tUInt8:
            // start the filter for 8bit images
            BasicGPLFilters::despeckleAdaptedFilterBlock<ito::uint8>(filterSettings, cvMatIn, cvMatObj);
            break;
        case ito::tUInt16:
            // start the filter for 16bit images
            BasicGPLFilters::despeckleAdaptedFilterBlock<ito::uint16>(filterSettings, cvMatIn, cvMatObj);
            break;
        case ito::tRGBA32:
            // start the filter for colored images
            BasicGPLFilters::despeckleAdaptedFilterBlock<ito::Rgba32>(filterSettings, cvMatIn, cvMatObj);
            break;
        default:
            return ito::RetVal(ito::retError, 0, tr("DataType not supported for this filter").toLatin1().data());
    }

    // The filter does not work in-place but objects were identically or we needed a need output array. Hence we have
    // to reverse the shallow copy
    if (createdNewObject)
    {
        *dObjDst = dObjOut;
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if (!retval.containsError())
    {
        // Add Protokoll to output object
        QString msg = tr("despeckle median with ]%1, %2[, r = %3, adaption %4").arg(QString::number(filterSettings.blackLevel), QString::number(filterSettings.whiteLevel), filterSettings.adaptRadius ? tr("yes") : tr("no"));
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
