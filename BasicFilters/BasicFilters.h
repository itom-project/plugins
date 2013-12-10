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
                  
		// Defined in BasicSpecialFilters.cpp
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


        static ito::RetVal mergeColorPlanesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal mergeColorPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);        /**< Combine 3 object planes to a single uint32 object*/
        
        static ito::RetVal calcMeanOverZParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal calcMeanOverZ(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);        /**< Combine 3 object planes to a single uint32 object*/        
        
        static ito::RetVal calcObjSlice(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);       /**< Interpolate 1D-slice from 2D-Object */
        static ito::RetVal calcObjSliceParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);				/**< Get the IO-parameters to interpolate 1D-slice from 2D-Object */

        // Defined in BasicGenericFilters.cpp
        static ito::RetVal genericStdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal BasicFilters::genericHighValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal BasicFilters::genericLowValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);
        static ito::RetVal BasicFilters::genericLowHighValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut, bool lowHigh);

    private:

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------
/*! \class GenericFilterEngine
\brief This class contains the new generic filter engine

\detail This generic filter engine class can be used to perform different linear and nonlinear filters.
E.g. LowPassFilter or SobelEdge-Detection. The code was inspired from former M++ generic filter engine.
The GenericFilterEngine-Class is inherited by each filter-Class, which contains filterspecific aditional components and a filterFunc-Function, which is executed by the runFilter-function.
To test the different filters, a python based testsuit will be implemented.
\author Christian Kohler, Wolfram Lyda
\date 12.2013
*/
template<typename _Tp> class GenericFilterEngine
{
    public:
        GenericFilterEngine() : m_pInpObj(NULL), m_pOutObj(NULL), m_pInLines(NULL), m_pOutLine(NULL), m_pInvalidMap(NULL),
            m_bufsize(0), m_kernelSizeX(0), m_kernelSizeY(0), m_x0(0), m_y0(0), m_dx(0), m_dy(0), m_AnchorX(0), m_AnchorY(0), m_initilized(false) {};
//        explicit GenericFilterEngine(ito::DataObject *in, ito::DataObject *out) : inp(in), outp(out), buf(NULL), bufsize(0) {};
        ~GenericFilterEngine()
        {

        }
        ito::RetVal runFilter(bool replaceNaN);

    protected:
        bool m_initilized;
        ito::DataObject *m_pInpObj;
        ito::DataObject *m_pOutObj;
        _Tp ** m_pInLines;              //< input buffer
        _Tp *  m_pOutLine;               //< output buffer
        ito::int8 ** m_pInvalidMap;        //< invalid map
        ito::int32 m_bufsize;
        ito::int16 m_kernelSizeX, m_kernelSizeY;   //< Horizontal / vertical Size of kernel (x) 
        ito::int32 m_x0, m_y0;     //< first point in x / y of filter region
        ito::int32 m_dx, m_dy;     //< Size of filter region
        ito::int32 m_AnchorX, m_AnchorY; //< Position of the data output in respect to the kernel (anchor)
        virtual ito::RetVal filterFunc() = 0;
};

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> class LowValueFilter : public GenericFilterEngine<_Tp>
{
    private:
        _Tp *kbuf;

    public:
        explicit LowValueFilter(ito::DataObject *in, 
            ito::DataObject *out, 
            ito::int32 roiX0, 
            ito::int32 roiY0, 
            ito::int32 roiXSize, 
            ito::int32 roiYSize, 
            ito::int16 kernelSizeX, 
            ito::int16 kernelSizeY,
            ito::int32 anchorPosX,
            ito::int32 anchorPosY
            ) : GenericFilterEngine()
        { 
            m_pInpObj = in;
            m_pOutObj = out;

            m_x0 = roiX0;
            m_y0 = roiY0;

            m_dx = roiXSize;
            m_dy = roiYSize;

            m_kernelSizeX = kernelSizeX;
            m_kernelSizeY = kernelSizeY;

            m_AnchorX = anchorPosX;
            m_AnchorY = anchorPosY;

            m_bufsize = m_kernelSizeX * m_kernelSizeY;

            kbuf = new _Tp[m_bufsize];

            if(kbuf != NULL)
            {
                m_initilized = true;
            }
        };
        ~LowValueFilter()
        {
            if(kbuf != NULL)
            {
                delete kbuf;
            }
        }

        ito::RetVal filterFunc();
};
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> class HighValueFilter : public GenericFilterEngine<_Tp>
{
    private:
        _Tp *kbuf;

    public:
        explicit HighValueFilter(ito::DataObject *in, 
            ito::DataObject *out, 
            ito::int32 roiX0, 
            ito::int32 roiY0, 
            ito::int32 roiXSize, 
            ito::int32 roiYSize, 
            ito::int16 kernelSizeX, 
            ito::int16 kernelSizeY,
            ito::int32 anchorPosX,
            ito::int32 anchorPosY
            ) : GenericFilterEngine()
        { 
            m_pInpObj = in;
            m_pOutObj = out;

            m_x0 = roiX0;
            m_y0 = roiY0;

            m_dx = roiXSize;
            m_dy = roiYSize;

            m_kernelSizeX = kernelSizeX;
            m_kernelSizeY = kernelSizeY;

            m_AnchorX = anchorPosX;
            m_AnchorY = anchorPosY;

            m_bufsize = m_kernelSizeX * m_kernelSizeY;

            kbuf = new _Tp[m_bufsize];

            if(kbuf != NULL)
            {
                m_initilized = true;
            }
        };
        ~HighValueFilter()
        {
            if(kbuf != NULL)
            {
                delete kbuf;
            }
        }

        ito::RetVal filterFunc();
};
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> class highPassFilter : public GenericFilterEngine<_Tp>
{
    ito::RetVal filterFunc();
};


//----------------------------------------------------------------------------------------------------------------------------------

#endif // BasicFilters_H
