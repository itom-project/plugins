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

/*! \file BasicFilters.h
   \brief   This is the main file for the M++Filter libary, which contains the interface definition.

   The algorithms in this dll are mostly copied from the filter.h and filter.cpp. The filters are grouped in different sub .cpp-files.

   \author ITO
   \date 12.2011
   */

#ifndef BASICFILTERS_H
#define BASICFILTERS_H

#ifdef USEOPENMP
    #define USEOMP 1
#else
    #define USEOMP 0
#endif

#include "common/addInInterface.h"
#include "DataObject/dataobj.h"

#include <qsharedpointer.h>

template<typename _Type> inline _Type myAbs(_Type val) {return val;}
template<> inline ito::int32 myAbs<ito::int32>(ito::int32 val) {return labs(val);}
template<> inline ito::float32 myAbs<ito::float32>(ito::float32 val) {return fabs(val);}
template<> inline ito::float64 myAbs<ito::float64>(ito::float64 val) {return fabs(val);}

//----------------------------------------------------------------------------------------------------------------------------------
/** @class BasicFiltersInterface
*   @brief ITO developed filter functions for the itom
*
*   AddIn Interface for the BasicFilters class s. also \ref BasicFilters
*/
class BasicFiltersInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        BasicFiltersInterface();
        ~BasicFiltersInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
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

        enum tFilterType
        {
            tGenericLowPass,
            tGenericMedian,
            tGenericGauss
        };

        // Defined in BasicFilters.cpp
        static ito::RetVal stdParams2Objects(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);                     /**< Get the standard IO-Parameters for filters with two objects */

        // Defined in BasicSpecialFilters.cpp
        static const QString replaceInfAndNaNDoc;
        static const QString flaten3Dto2DDoc;
        static const QString swapByteOrderDoc;
        static const QString mergeColorPlaneDoc;
        static const QString calcMeanOverZDoc;
        static const QString calcObjSliceDoc;
        static const QString clipValueDoc;
        static const QString calcHistDoc;
        static const QString clipAbyBDoc;
        static const QString fillGeometricDoc;

        static ito::RetVal replaceInfAndNaN(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal replaceInfAndNaNParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal flaten3Dto2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< Combine 3 dataObjects by highest contrast of modulation maps */
        static ito::RetVal swapByteOrder(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);    /** Simply swap current byte order */

        static ito::RetVal calcHistParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);                      /**< Get the standard IO-Parameters histogramm-filter */
        static ito::RetVal calcHistFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);/**< This function calucaltes a histogramm for every cvMat int the input dataObject  */

        static ito::RetVal clipValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/);
        static ito::RetVal clipValueFilterParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut);

        static ito::RetVal clipAbyBFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/);
        static ito::RetVal clipAbyBFilterParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut);

        static ito::RetVal mergeColorPlanesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal mergeColorPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);        /**< Combine 3 object planes to a single uint32 object*/

        static ito::RetVal calcMeanOverZParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal calcMeanOverZ(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);        /**< Combine 3 object planes to a single uint32 object*/

        static ito::RetVal calcObjSlice(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);       /**< Interpolate 1D-slice from 2D-Object */
        static ito::RetVal calcObjSliceParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);                /**< Get the IO-parameters to interpolate 1D-slice from 2D-Object */

        // Defined in BasicGenericFilters.cpp
        static ito::RetVal genericStdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString genericLowValueFilterDoc;
        static const QString genericHighValueFilterDoc;
        static const QString genericMedianFilterDoc;
        static const QString genericLowPassFilterDoc;
        static const QString genericGaussianEpsilonFilterDoc;
        static const QString genericGaussianFilterDoc;
        static ito::RetVal genericHighValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal genericLowValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);
        static ito::RetVal genericLowHighValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut, bool lowHigh);
        static ito::RetVal genericMedianFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);
        static ito::RetVal genericLowPassFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);

        static ito::RetVal genericSobelOptParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal genericSobelOptFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);
        static ito::RetVal genericKernelFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);

        static ito::RetVal genericGaussianEpsilonParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal genericGaussianEpsilonFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);

        static ito::RetVal genericGaussianParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal genericGaussianFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);

        // Further filters using the Generic Engine
        static const QString spikeMeanFilterDoc;
        static const QString spikeMedianFilterDoc;
        static ito::RetVal spikeMeanFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);
        static ito::RetVal spikeMedianFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);
        static ito::RetVal spikeCompFilterStdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        // Filter
        static ito::RetVal fillGeometricPrimitiv(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);
        static ito::RetVal fillGeometricParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString calcRadialMeanFilterDoc;
        static ito::RetVal calcRadialMeanFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/);
        static ito::RetVal calcRadialMeanFilterParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut);

        // Defined in AdditionalFilters
        static const QString labelingFilterDoc;
        static const QString findEllipsesFilterDoc;
        static ito::RetVal labeling(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal labelingParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal findLabel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal findLabelParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal findEllipses(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal findEllipsesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

    private:
        static ito::RetVal spikeGenericFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut, const tFilterType filter);

        template<typename _Tp> static void fillGeoCircle(cv::Mat *dst, const ito::float64 x0, const ito::float64 y0, const ito::float64 radius, const bool inside, const bool outside, const _Tp insideVal, const _Tp outsideVal);
        template<typename _Tp> static void fillGeoEllipse(cv::Mat *dst, const ito::float64 x0, const ito::float64 y0, const ito::float64 radiusX, const ito::float64 radiusY, const bool inside, const bool outside, const _Tp insideVal, const _Tp outsideVal);
        template<typename _Tp> static void fillGeoRectangle(cv::Mat *dst, const ito::float64 x0, const ito::float64 y0, const ito::float64 x1, const ito::float64 y1, const bool inside, const bool outside, const _Tp insideVal, const _Tp outsideVal);

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
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax
    public:
        GenericFilterEngine() : m_pInpObj(NULL), m_pOutObj(NULL), m_pInLines(NULL), m_pOutLine(NULL), m_pInvalidMap(NULL),
            m_bufsize(0), m_kernelSizeX(0), m_kernelSizeY(0), m_x0(0), m_y0(0), m_dx(0), m_dy(0), m_AnchorX(0), m_AnchorY(0), m_initialized(false) {}
//        explicit GenericFilterEngine(ito::DataObject *in, ito::DataObject *out) : inp(in), outp(out), buf(NULL), bufsize(0) {};
        ~GenericFilterEngine() {}
        ito::RetVal runFilter(bool replaceNaN);

    protected:
        bool m_initialized;
        const ito::DataObject *m_pInpObj;
        ito::DataObject *m_pOutObj;
        _Tp ** m_pInLines;               //< input buffer
        _Tp *  m_pOutLine;               //< output buffer
        ito::int8 **m_pInvalidMap;       //< invalid map
        ito::int32 m_bufsize;
        ito::int16 m_kernelSizeX, m_kernelSizeY;   //< Horizontal / vertical Size of kernel (x)
        ito::int32 m_x0, m_y0;           //< first point in x / y of filter region
        ito::int32 m_dx, m_dy;           //< Size of filter region
        ito::int32 m_AnchorX, m_AnchorY; //< Position of the data output in respect to the kernel (anchor)
        virtual void filterFunc() = 0;
        virtual void clearFunc() = 0;
};

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> class LowValueFilter : public GenericFilterEngine<_Tp>
{
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax
    private:
        #if (USEOMP)
        _Tp **kbuf;
        #else
        _Tp *kbuf;
        #endif

    public:
        explicit LowValueFilter(const ito::DataObject *in,
            ito::DataObject *out,
            ito::int32 roiX0,
            ito::int32 roiY0,
            ito::int32 roiXSize,
            ito::int32 roiYSize,
            ito::int16 kernelSizeX,
            ito::int16 kernelSizeY,
            ito::int32 anchorPosX,
            ito::int32 anchorPosY);

        ~LowValueFilter();

        void filterFunc();
        void clearFunc();
};

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> class HighValueFilter : public GenericFilterEngine<_Tp>
{
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax
    private:
        #if (USEOMP)
        _Tp **kbuf;
        #else
        _Tp *kbuf;
        #endif

    public:
        explicit HighValueFilter(const ito::DataObject *in,
            ito::DataObject *out,
            ito::int32 roiX0,
            ito::int32 roiY0,
            ito::int32 roiXSize,
            ito::int32 roiYSize,
            ito::int16 kernelSizeX,
            ito::int16 kernelSizeY,
            ito::int32 anchorPosX,
            ito::int32 anchorPosY);

        ~HighValueFilter();

        void filterFunc();
        void clearFunc();
};

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> class MedianFilter : public GenericFilterEngine<_Tp>
{
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax
    private:
        #if (USEOMP)
        _Tp **kbuf;
        _Tp ***kbufPtr;
        #else
        _Tp *kbuf;
        _Tp **kbufPtr;
        #endif

    public:
        explicit MedianFilter(const ito::DataObject *in,
            ito::DataObject *out,
            ito::int32 roiX0,
            ito::int32 roiY0,
            ito::int32 roiXSize,
            ito::int32 roiYSize,
            ito::int16 kernelSizeX,
            ito::int16 kernelSizeY,
            ito::int32 anchorPosX,
            ito::int32 anchorPosY);

        ~MedianFilter();

        void filterFunc();
        void clearFunc();
};

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> class SobelOptFilter : public GenericFilterEngine<_Tp>
{
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax
private:
    ito::uint8 m_gradDir;

public:
    explicit SobelOptFilter(ito::DataObject *in,
        ito::DataObject *out,
        ito::int32 roiX0,
        ito::int32 roiY0,
        ito::int32 roiXSize,
        ito::int32 roiYSize,
        ito::uint8 gradDir);

    ~SobelOptFilter();

    void filterFunc();
    void clearFunc();
};

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> class LowPassFilter : public GenericFilterEngine<_Tp>
{
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax
    private:
        ito::float64 *m_colwiseSumBuffer;
        bool m_isFilled;
        ito::float64 m_divisor;

    public:
        explicit LowPassFilter(const ito::DataObject *in,
            ito::DataObject *out,
            ito::int32 roiX0,
            ito::int32 roiY0,
            ito::int32 roiXSize,
            ito::int32 roiYSize,
            ito::int16 kernelSizeX,
            ito::int16 kernelSizeY,
            ito::int32 anchorPosX,
            ito::int32 anchorPosY);

        ~LowPassFilter();

        void filterFunc();
        void clearFunc();
};

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> class GaussianFilter : public GenericFilterEngine<_Tp>
{
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax
    private:
        ito::float64 *m_pRowKernel;
        ito::float64 *m_pColKernel;
        ito::float64 **m_pInLinesFiltered;
        bool m_isFilled;

    public:
        explicit GaussianFilter(const ito::DataObject *in,
                                ito::DataObject *out,
                                ito::int32 roiX0,
                                ito::int32 roiY0,
                                ito::int32 roiXSize,
                                ito::int32 roiYSize,
                                ito::float64 sigmaSizeX,
                                ito::float64 epsilonSizeX,
                                ito::float64 sigmaSizeY,
                                ito::float64 epsilonSizeY);

        explicit GaussianFilter(const ito::DataObject *in,
                                ito::DataObject *out,
                                ito::int32 roiX0,
                                ito::int32 roiY0,
                                ito::int32 roiXSize,
                                ito::int32 roiYSize,
                                ito::int32 kernelSizeX,
                                ito::int32 kernelSizeY,
                                ito::float64 sigmaSizeX,
                                ito::float64 sigmaSizeY);

        ~GaussianFilter();

        void filterFunc();
        void clearFunc();
};

//----------------------------------------------------------------------------------------------------------------------------------
#endif // BasicFilters_H
