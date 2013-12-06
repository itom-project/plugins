/*! \file BasicFilters.h
   \brief   This is the main file for the M++Filter libary, which contains the interface definition. 
   
   The algorithms in this dll are mostly copied from the filter.h and filter.cpp. The filters are grouped in different sub .cpp-files.

   \author ITO 
   \date 12.2011
   */

#ifndef BASICFILTERS_H
#define BASICFILTERS_H

#include "common/addInInterface.h"

#include "DataObject/dataobj.h"

#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class BasicFiltersInterface
*   @brief ITO developed filter functions for the itom
*
*   AddIn Interface for the BasicFilters class s. also \ref BasicFilters
*/
class BasicFiltersInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
        Q_INTERFACES(ito::AddInInterfaceBase)

    protected:

    public:
        BasicFiltersInterface();
        ~BasicFiltersInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class BasicFilters
*   @brief Algorithms used to process images and dataobjects with filters develped at ITO
*
*   In this class the algorithms used for the processing of measurement data are implemented.
*
*/
class BasicFilters : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        BasicFilters();
        ~BasicFilters();

    public:
        friend class BasicFiltersInterface;
		 
        // Defined in BasicFilters.cpp
		static ito::RetVal stdParams2Objects(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);                     /**< Get the standard IO-Parameters for filters with two objects */
                  
		// Defined in itomspecialfilters.cpp
		static const char* replaceInfAndNaNDoc;
        static const char* flaten3Dto2DDoc;
        static const char* swapByteOrderDoc;
        static const char* mergeColorPlaneDoc;
        static const char* calcMeanOverZDoc;
        static const char* calcObjSliceDoc;

        static ito::RetVal replaceInfAndNaN(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal replaceInfAndNaNParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
		
        static ito::RetVal flaten3Dto2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< Combine 3 dataObjects by highest contrast of modulation maps */
        static ito::RetVal swapByteOrder(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);    /** Simply swap current byte order */

        // Defined in itomspecialfilter.cpp
        static ito::RetVal mergeColorPlanesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal mergeColorPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);        /**< Combine 3 object planes to a single uint32 object*/
        
        static ito::RetVal calcMeanOverZParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal calcMeanOverZ(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);        /**< Combine 3 object planes to a single uint32 object*/        
        
        static ito::RetVal calcObjSlice(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);       /**< Interpolate 1D-slice from 2D-Object */
        static ito::RetVal calcObjSliceParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);				/**< Get the IO-parameters to interpolate 1D-slice from 2D-Object */

    private:

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // BasicFilters_H
