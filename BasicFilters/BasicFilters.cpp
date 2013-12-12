/*! \file BasicFilters.cpp
   \brief   This file contains the itomflters class and interface definitions.
   
   \author ITO 
   \date 12.2011
*/

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
    setObjectName("ITOM-Filter");

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
Q_EXPORT_PLUGIN2(BasicFiltersInterface, BasicFiltersInterface)

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

    filter = new FilterDef(BasicFilters::genericLowValueFilter, BasicFilters::genericStdParams, tr("Set each pixel to the lowest value within the kernel (x ,y) using the generic mcpp filter engine"));
    m_filterList.insert("lowValueFilter", filter);
    filter = new FilterDef(BasicFilters::genericHighValueFilter, BasicFilters::genericStdParams, tr("Set each pixel to the highest value within the kernel (x ,y) using the generic mcpp filter engine"));
    m_filterList.insert("lowValueFilter", filter);
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
        ito::Param param = ito::Param("scrImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input image").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output image").toAscii().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------