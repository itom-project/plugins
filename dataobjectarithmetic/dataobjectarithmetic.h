/* ********************************************************************
    Plugin "dataobjectarithmetic" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2021, Institut für Technische Optik (ITO),
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

#include <qnumeric.h>
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DataObjectArithmeticInterface
 *   @brief short description of this class
 *
 *   AddIn Interface for the DataObjectArithmetic class s. also \ref DataObjectArithmetic
 */
class DataObjectArithmeticInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase")
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

protected:
public:
    DataObjectArithmeticInterface(); /*! <Class constructor */
    ~DataObjectArithmeticInterface(); /*! <Class destructor */
    ito::RetVal getAddInInst(
        ito::AddInBase** addInInst); /*! <Create a new instance of FittingFilters-Class */
private:
    ito::RetVal closeThisInst(
        ito::AddInBase** addInInst); /*! <Destroy the loaded instance of FittingFilters-Class */
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
    DataObjectArithmetic(); /*! <Class constructor */
    ~DataObjectArithmetic(); /*! <Class destructor */

public:
    friend class DataObjectArithmeticInterface;

    //*< filter function to calculate the minimum Value of a dataObject */
    static const QString minValueDoc;
    static ito::RetVal minValue(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);

    //*< filter function to calculate the maximum Value of a dataObject */
    static const QString maxValueDoc;
    static ito::RetVal maxValue(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);

    //*< filter function to calculate the maximum and minimum Value of a dataObject */
    static const QString minMaxValueDoc;
    static ito::RetVal minMaxValue(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);

    //*< filter function to calculate the mean Value of a dataObject */
    static const QString meanValueDoc;
    static ito::RetVal meanValue(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);

    //*< filter function to calculate the median Value of a dataObject */
    static const QString medianValueDoc;
    static ito::RetVal medianValue(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);

    //*< filter function to calculate the standard deviation Value and the standard deviation of a dataObject */
    static const QString devValueDoc;
    static ito::RetVal devValue(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);

    //*< filter function to compare to dataObjects element-wise and by type */
    static const QString areEqualDoc;
    static ito::RetVal areEqual(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);

    //*< parameter function for the deviation filter function */
    static ito::RetVal devValueParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    //*< parameter function for the deviation filter function */
    static ito::RetVal minMaxValueParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    //*< parameter function for dataObjects with a single input DataObject */
    static ito::RetVal singleDObjInputParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    //*< parameter function for dataObjects with a single input DataObject */
    static ito::RetVal singleDObjInputInfParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    //*< parameter function for dataObjects with a single input DataObject and value and position output */
    static ito::RetVal singleDObjInputValueAndPositionOutParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    //*< parameter function for dataObjects with two DataObjects */
    static ito::RetVal doubleDObjInputParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    static const QString centerOfGravityDoc;
    //*< filter function to calculate the center of gravity of a dataObject in x and y */
    static ito::RetVal centerOfGravity(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);
    //*< parameter function for the centerOfGravity-Filter */
    static ito::RetVal centerOfGravityParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    static const QString localCenterOfGravityDoc;
    //*< filter function to calculate the center of gravity of a dataObject in x and y */
    static ito::RetVal localCenterOfGravity(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);
    //*< parameter function for the centerOfGravity-Filter */
    static ito::RetVal localCenterOfGravityParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    static const QString boundingBoxDoc;
    //*< filter function to calculate the center of gravity of a dataObject in x and y */
    static ito::RetVal boundingBox(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);
    //*< parameter function for the centerOfGravity-Filter */
    static ito::RetVal boundingBoxParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    static const QString centerOfGravity1DimDoc;
    //*< filter function to calculate the center of gravity of a dataObject along the x or y axis*/
    static ito::RetVal centerOfGravity1Dim(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);
    //*< parameter function for the centerOfGravity1Dim-Filter */
    static ito::RetVal centerOfGravity1DimParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    static const QString gaussianFit1DimDoc;
    //*< filter function to calculate a Gaussian fit of a dataObject along the x or y axis*/
    static ito::RetVal gaussianFit1Dim(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);
    //*< parameter function for the gaussianFit1Dim-Filter */
    static ito::RetVal gaussianFit1DimParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

    static const QString getPercentageThresholdDoc;
    static ito::RetVal getPercentageThresholdParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);
    static ito::RetVal getPercentageThreshold(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);

    static const QString autoFocusEstimateDoc;
    static ito::RetVal autoFocusEstimateParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);
    static ito::RetVal autoFocusEstimate(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);

    static const QString findMultiSpotsDoc;
    static ito::RetVal findMultiSpots(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        QVector<ito::ParamBase>* paramsOut);
    static ito::RetVal findMultiSpotsParams(
        QVector<ito::Param>* paramsMand,
        QVector<ito::Param>* paramsOpt,
        QVector<ito::Param>* paramsOut);

private:
    template <typename _Tp>
    static ito::RetVal centroidHelper(
        const cv::Mat* mat,
        const ito::float64& lowTreshold,
        const ito::float64& highTreshold,
        ito::float64& xCOG,
        ito::float64& yCOG);

    template <typename _Tp>
    static ito::RetVal boundingBoxHelper(
        const cv::Mat* mat,
        const ito::float64& lowThreshold,
        const ito::float64& highThreshold,
        int* roi);

    template <typename _Tp>
    static ito::RetVal centroidHelperFor1D(
        const cv::Mat* inMat,
        ito::float64* outCOG,
        _Tp* outINT,
        const _Tp& pvThreshold,
        const _Tp& lowerThreshold,
        const ito::float64& dynamicTreshold,
        const ito::float64& scale,
        const ito::float64& offset,
        bool alongCols);

    template <typename _Tp>
    static ito::RetVal getPercentageThresholdHelper(
        const ito::DataObject* dObj, double percentage, double& value);
    template <typename _Tp> static bool cmpLT(_Tp i, _Tp j)
    {
        return (i < j);
    }
    template <typename _Tp> static bool cmpGT(_Tp i, _Tp j)
    {
        return (i > j);
    }

    struct MultiSpotParameters
    {
        int backgroundNoise;
        int minPeakHeight;
        int maxPeakDiameter;
        int searchStepSizeHeight;
        int searchStepSizeWidth;
        int maxBackgroundLevel;
        int mode;
    };

    template <typename _Tp> struct Spot
    {
        Spot() : row(-1), col(0), value(0), next(NULL){};
        int row;
        int col;
        _Tp value;
        Spot* next;
    };

    template <typename _Tp>
    static void findMultiSpots1D(
        const _Tp* lineData,
        const int& row,
        const int& cols,
        Spot<_Tp>* spots,
        const MultiSpotParameters& params);
    template <typename _Tp>
    static void clusterSpots(
        Spot<_Tp>* spots,
        Spot<_Tp>** finalSpotsLast,
        Spot<_Tp>* tempSpotsRoot,
        const int rowIdx,
        const int& spotsSlice,
        const MultiSpotParameters& params);
    template <typename _Tp>
    static void fastCOG(
        const cv::Mat* img,
        const int row,
        const int col,
        const int halfSize,
        const _Tp lowThreshold,
        ito::float32& rowSubPix,
        ito::float32& colSubPix,
        ito::uint16& area);

public slots:
    ito::RetVal init(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        ItomSharedSemaphore* waitCond = NULL);
    ito::RetVal close(ItomSharedSemaphore* waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // DATAOBJECTARITHMETIC_H
