/* ********************************************************************
    Plugin "dataobjectarithmetic" for itom software
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

#ifndef DATAOBJECTARITHMETIC_H
#define DATAOBJECTARITHMETIC_H

#include "common/addInInterface.h"

#include "DataObject/dataobj.h"

#include <qsharedpointer.h>
#include <qnumeric.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DataObjectArithmeticInterface
*   @brief short description of this class
*
*   AddIn Interface for the DataObjectArithmetic class s. also \ref DataObjectArithmetic
*/
class DataObjectArithmeticInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        DataObjectArithmeticInterface();       /*! <Class constructor */
        ~DataObjectArithmeticInterface();      /*! <Class destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*! <Create a new instance of FittingFilters-Class */

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*! <Destroy the loaded instance of FittingFilters-Class */

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class FittingFilters
*   @brief short description of this filter class
*
*   long description of this filter class
*
*/
class DataObjectArithmetic : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        DataObjectArithmetic();    /*! <Class constructor */
        ~DataObjectArithmetic();               /*! <Class destructor */

    public:
        friend class DataObjectArithmeticInterface;

        static const char* minValueDoc;
        static ito::RetVal minValue(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);       //*< Static filter function to calcuate the minimum Value of a dataObject */
        
        static const char* maxValueDoc;
        static ito::RetVal maxValue(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);       //*< Static filter function to calcuate the maximum Value of a dataObject */
        
        static const char* minMaxValueDoc;
        static ito::RetVal minMaxValue(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);       //*< Static filter function to calcuate the maximum and minimum Value of a dataObject */
        
        static const char* meanValueDoc;
        static ito::RetVal meanValue(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);      //*< Static filter function to calcuate the mean Value of a dataObject */
        
        static const char* devValueDoc;
        static ito::RetVal devValue(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);       //*< Static filter function to calcuate the mean Value and the standard deviation of a dataObject */
        
        static const char* areEqualDoc;
        static ito::RetVal areEqual(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);       //*< Static filter function to compare to dataObjects element-wise and by type */
        static ito::RetVal devValueParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);             //*< Static parameter function for the deviation filter function */
        static ito::RetVal minMaxValueParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);             //*< Static parameter function for the deviation filter function */
        static ito::RetVal singleDObjInputParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);      //*< Static parameter function for dataObjects with a single input DataObject */
        static ito::RetVal singleDObjInputInfParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);      //*< Static parameter function for dataObjects with a single input DataObject */
        static ito::RetVal singleDObjInputValueAndPositionOutParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);      //*< Static parameter function for dataObjects with a single input DataObject and value and position output */
        static ito::RetVal doubleDObjInputParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);      //*< Static parameter function for dataObjects with two DataObjects */
        
        static const char* centerOfGravityDoc;
        static ito::RetVal centerOfGravity(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut); //*< Static filter function to calcuate the center of gravity of a dataObject in x and y */
        static ito::RetVal centerOfGravityParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);              //*< Static parameter function for the centerOfGravity-Filter */
        
        static const char* centerOfGravity1DimDoc;
        static ito::RetVal centerOfGravity1Dim(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);    //*< Static filter function to calcuate the center of gravity of a dataObject along the x or y axis*/ 
        static ito::RetVal centerOfGravity1DimParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);          //*< Static parameter function for the centerOfGravity1Dim-Filter */
        //static ito::RetVal meanValueAxisDir(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);       //*< Static parameter function to calculate the mean-value along one axis direction*/

        static const char *getPercentageThresholdDoc;
        static ito::RetVal getPercentageThresholdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);                 
        static ito::RetVal getPercentageThreshold(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);

    private:
        template<typename _Tp> static ito::RetVal centroidHelper(const cv::Mat *mat, const ito::float64 &lowTreshold, const ito::float64 &highTreshold, ito::float64 &xCOG, ito::float64 &yCOG);             
        template<typename _Tp> static ito::RetVal centroidHelperFor1D(const cv::Mat *inMat, ito::float64 *outCOG, _Tp *outINT, const _Tp &pvThreshold, const _Tp &lowerThreshold, const ito::float64 &dynamicTreshold, const ito::float64 &scale, const ito::float64 &offset, bool alongCols);

        template<typename _Tp> static ito::RetVal getPercentageThresholdHelper(const ito::DataObject *dObj, double percentage, double &value);
        template<typename _Tp> static bool cmpLT(_Tp i, _Tp j) { return (i<j); }
        template<typename _Tp> static bool cmpGT(_Tp i, _Tp j) { return (i>j); }

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // DATAOBJECTARITHMETIC_H
