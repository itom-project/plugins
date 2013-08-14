#ifndef FITTINGFILTERS_H
#define FITTINGFILTERS_H

#include "common/addInInterface.h"
#include "common/sharedStructures.h"

#include "DataObject/dataobj.h"

#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class FittingFiltersInterface
*   @brief short description of this class
*
*   AddIn Interface for the FittingFilters class s. also \ref FittingFilters
*/
class FittingFiltersInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
        Q_INTERFACES(ito::AddInInterfaceBase)

    protected:

    public:
        FittingFiltersInterface();       /*! <Class constructor */
        ~FittingFiltersInterface();      /*! <Class destructor */
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
class FittingFilters : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        FittingFilters();    /*! <Class constructor */
        ~FittingFilters();               /*! <Class destructor */

    public:
        friend class FittingFiltersInterface;

        static ito::RetVal fitPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal fitPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal subtractPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal subtractPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal subtractRegressionPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal subtractRegressionPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal fitPolynom2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal fitPolynom2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal polyfitWeighted2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal polyfitWeighted2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal polyval2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal polyval2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

    private:
        struct Koeffizienten {
		     int    gradX;
		     int    gradY;
		     int    sizeX;
		     int    sizeY;
		     double *b;
		     double *alphaX;
		     double *betaX;
		     double *gammaX;
		     double *alphaY;
		     double *betaY;
		     double *gammaY;
		};

        static ito::RetVal polyfit(int *x, int *y, cv::Mat *dblData, cv::Mat *dblFittedData, int gradX, int gradY, int sizeX, int sizeY, double *sigma, struct Koeffizienten *koeff, bool fillNaNValues = false);
        static ito::RetVal calcKoeff(int anzahl,int PolyGrad, double *Alpha, double *Beta, double *Gamma);
        static void OrthPolAuswerten(int anzahl, int PolyGrad, double t, double *W, double *alpha,double *beta, double *gamma);
        static double Fitwerte(double tx, double ty, struct Koeffizienten *koeff);

        static ito::RetVal fitLeastSquarePlaneSVD(cv::Mat *inputMatrix, double &A, double &B, double &C);
        static ito::RetVal fitLeastSquarePlane(cv::Mat *inputMatrix, double &A, double &B, double &C);
        template<typename _Tp> static ito::RetVal subtractPlaneTemplate(cv::Mat *inputMatrix, cv::Mat *destMatrix, double A, double B, double C);

        /*! < Fit a n-Grade polynome to either inputX or if inputX == NULL to offset and scale */
        template<typename _Tp> static ito::RetVal polyFit1DMatLab(double &offset, double &scale, cv::Mat *inputY, int order, QVector<QVariant> *outVals);

        static ito::RetVal calcPolyfitWeighted2D(const ito::DataObject *dataZ, int orderX, int orderY, std::vector<double> &coefficients, double margin = 0.2, const ito::DataObject *weights = NULL);
        static ito::RetVal calcPolyval2D(ito::DataObject *dataZ, int orderX, int orderY, const std::vector<double> coefficients);

        
    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);      
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // FITTINGFILTERS_H
