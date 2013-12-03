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
        Q_INTERFACES(ito::AddInInterfaceBase)

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

        //template<typename _Tp> static ito::RetVal minValueHelper(ito::DataObject *dObj, QVector<ito::ParamBase> *paramsOut);
        //template<typename _Tp> static ito::RetVal maxValueHelper(ito::DataObject *dObj, QVector<ito::ParamBase> *paramsOut);
        //template<typename _Tp, typename _BufTp> static ito::RetVal meanValueHelper(ito::DataObject *dObj, QVector<ito::ParamBase> *paramsOut);
        //template<typename _Tp, typename _BufTp> static ito::RetVal meanValueHelperFloating(ito::DataObject *dObj, QVector<ito::ParamBase> *paramsOut);
        //template<typename _Tp, typename _BufTp> static ito::RetVal devValueHelper(ito::DataObject *dObj, int flag, QVector<ito::ParamBase> *paramsOut);
        //template<typename _Tp, typename _BufTp> static ito::RetVal devValueHelperFloating(ito::DataObject *dObj, int flag, QVector<ito::ParamBase> *paramsOut);
        template<typename _Tp> static ito::RetVal centroidHelper(cv::Mat *mat, const _Tp lowTreshold, const _Tp highTreshold, ito::float64 &xCOG, ito::float64 &yCOG);             
        template<typename _Tp> static ito::RetVal centroidHelperFor1D(cv::Mat *inMat, cv::Mat *outCOG, cv::Mat *outINT, const _Tp lowTreshold, const ito::float64 dynamicTreshold, const ito::float64 scale, bool alongCols);

        template<typename _Tp> static ito::RetVal getPercentageThresholdHelper(const ito::DataObject *dObj, double percentage, double &value);
        template<typename _Tp> static bool cmpLT(_Tp i, _Tp j) { return (i<j); }
        template<typename _Tp> static bool cmpGT(_Tp i, _Tp j) { return (i>j); }

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // DATAOBJECTARITHMETIC_H
