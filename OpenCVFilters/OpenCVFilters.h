#ifndef OPENCVFILTERS_H
#define OPENCVFILTERS_H

#include "common/addInInterface.h"

#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class OpenCVFiltersInterface
*   @brief Algorithms used for the processing of images using OpenCV
*
*   AddIn Interface for the OpenCVFilters class s. also \ref OpenCVFilters
*/
class OpenCVFiltersInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
        Q_INTERFACES(ito::AddInInterfaceBase)

    protected:

    public:
        OpenCVFiltersInterface();       /*! <Class constructor */
        ~OpenCVFiltersInterface();      /*! <Class destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*! <Create a new instance of OpenCVFilter-Class */

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*! <Destroy the loaded instance of OpenCVFilter-Class */


    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class OpenCVFilters
*   @brief Algorithms used to process images and dataobjects with filters provided by openCV
*
*   In this class the algorithms used for the processing of images are implemented.
*   The filters wrapp openCV-Filters to python interface. Handling of 3D-Objects differs depending on the filter.
*
*/
class OpenCVFilters : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        OpenCVFilters();    /*! <Class constructor */
        ~OpenCVFilters();               /*! <Class destructor */

    public:
        friend class OpenCVFiltersInterface;

        static const char * cvDilateDoc;
        static const char * cvErodeDoc;
        static ito::RetVal cvDilateErodeParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);           /*! This function returns the parameters for the dilate or erode function.*/
        static ito::RetVal cvDilateErode(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, bool erodeNotDilate);    /*! This function executes either the dilate or erodate filter of OpenCV.*/
        static ito::RetVal cvDilate(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut); /*! Function performs a "Dilate-Filter" on the input object*/
        static ito::RetVal cvErode(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);  /*! Function performs a "Erode-Filter" on the input object*/

        static ito::RetVal cvBlur(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs a "Blur-Filter" on the input object*/
        static ito::RetVal cvBlurParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "Blur-Filter"*/

        static ito::RetVal cvMedianBlur(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs a "median Blur-Filter" on the input object*/
        static ito::RetVal cvMedianBlurParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "MedianBlur-Filter"*/

        static ito::RetVal cvIFFT2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the 2D inverse DFT from the openCV-library on the input object*/
        static ito::RetVal cvIFFT1D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the 2D DFT from the openCV-library on the input object*/
        static ito::RetVal cvFFT2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the linewise inverse DFT from the openCV-library on the input object*/
        static ito::RetVal cvFFT1D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs the linewise DFT from the openCV-library on the input object*/
        static ito::RetVal cvFFTParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "FFT-Filter"*/

        static ito::RetVal cvCalcHist(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function calculates the histogramm for the input object*/
        static ito::RetVal cvCalcHistParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the parameter for the histogramm function*/

		static ito::RetVal cvFindCircles(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function performs a "Blur-Filter" on the input object*/
        static ito::RetVal cvFindCirclesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);   /*! Function gives back the optional and mandatory parameters for "Blur-Filter"*/

        static ito::RetVal cvFindChessboardCorners(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut); 
        static ito::RetVal cvFindChessboardCornersParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut); 

        static ito::RetVal cvCornerSubPix(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut); 
        static ito::RetVal cvCornerSubPixParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal cvFlipUpDown(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);     /*! Function flips openCV-Mats upside down by executing cvFlip(..., false)*/
        static ito::RetVal cvFlipLeftRight(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);   /*! Function flips openCV-Mats left right by executing cvFlip(..., true)*/
        
        static ito::RetVal stdParams2Objects(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut); 
    private:

        static ito::RetVal cvFlip(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, bool colsIfTrue);        /*! Flip upside/down (colsIfTrue == false) or left/right (colsIfTrue == true)*/

		 

		static ito::RetVal checkInputOutputEqual(ito::DataObject * p_input, ito::DataObject * p_output, bool * unequal);    /*! <Checks if input and output objects are equal and if the object pointers are valid*/
		static ito::RetVal makeInputOutputEqual(ito::DataObject * p_input, ito::DataObject * p_output);                     /*! <Realloc the output object to size and type of input object */

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> /*val*/, ItomSharedSemaphore * /*waitCond*/) { return ito::retOk; }
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> /*val*/, ItomSharedSemaphore * /*waitCond*/) { return ito::retOk; }
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // OPENCVFILTERS_H
