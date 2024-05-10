/* ********************************************************************
    Plugin "FittingFilters" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#ifndef FITTINGFILTERS_H
#define FITTINGFILTERS_H

#include "common/addInInterface.h"
#include "common/sharedStructures.h"

#include "DataObject/dataobj.h"
#include "numericalRecipes.h"

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
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

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

        static const QString fitPlaneDoc;
        static ito::RetVal fitPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal fitPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString subtractPlaneDoc;
        static ito::RetVal subtractPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal subtractPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString subtractRegressionPlaneDoc;
        static ito::RetVal subtractRegressionPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal subtractRegressionPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString fitPolynom2DDoc;
        static ito::RetVal fitPolynom2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal fitPolynom2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString polyfitWeighted2DDoc;
        static ito::RetVal polyfitWeighted2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal polyfitWeighted2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString polyfitWeighted2DSinglePointsDoc;
        static ito::RetVal polyfitWeighted2DSinglePoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal polyfitWeighted2DSinglePointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString polyval2DDoc;
        static ito::RetVal polyval2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal polyval2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString polyval2DSinglePointsDoc;
        static ito::RetVal polyval2DSinglePoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal polyval2DSinglePointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString fitPolynom1D_ZDoc;
        static ito::RetVal fitPolynom1D_Z(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal fitPolynom1D_ZParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString subtract1DRegressionPolynomDoc;
        static ito::RetVal subtract1DRegressionPolynom(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal subtract1DRegressionPolynomParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString getInterpolatedValuesDoc;
        static ito::RetVal getInterpolatedValues(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal getInterpolatedValuesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);



#if (CV_MAJOR_VERSION >= 3)
        static const QString fillInvalidAreasDoc;
        static ito::RetVal fillInvalidAreas(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal fillInvalidAreasParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        template<typename _Tp> static cv::Mat getNaNMask(const cv::Mat* mat);
#endif

    private:
        struct Koeffizienten {
            Koeffizienten() : gradX(0), gradY(0), sizeX(0), sizeY(0), b(NULL), alphaX(NULL), gammaX(NULL),
                betaX(NULL), alphaY(NULL), betaY(NULL), gammaY(NULL) {}
            ~Koeffizienten() {}
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

        static ito::RetVal fitLeastSquarePlaneSVD(const cv::Mat *inputMatrix, double &A, double &B, double &C);
        static ito::RetVal fitLeastSquarePlane(cv::Mat *inputMatrix, double &A, double &B, double &C);
        template<typename _Tp> static ito::RetVal subtractPlaneTemplate(cv::Mat *inputMatrix, cv::Mat *destMatrix, double A, double B, double C);
        template<typename _Tp> static ito::RetVal lsqFitPlane(const cv::Mat *mat, double &A, double &B, double &C);
        template<typename _Tp> static ito::RetVal lmedsFitPlane(const cv::Mat *mat, double &A, double &B, double &C, const double &valid_probability, const double &alarm_rate);
        template<typename _Tp> static ito::RetVal getRandomValidMinimalSampleSet(const cv::Mat *mat, cv::RNG &rng, int numSamples, int *rows, int *cols, int maxIter = 1000);

        static ito::RetVal calcPolyfitWeighted2DRegularGrid(const ito::DataObject *dataZ, int orderX, int orderY, std::vector<double> &coefficients, double reduceFactor = -1.0, const ito::DataObject *weights = NULL);
        static ito::RetVal calcPolyfitWeighted2DSinglePoints(const ito::DataObject *dataX, const ito::DataObject *dataY, const ito::DataObject *dataZ, int orderX, int orderY, std::vector<double> &coefficients, const ito::DataObject *weights = NULL);
        template<typename _Tp> static ito::RetVal calcPolyfitWeighted2D(const _Tp *X, const _Tp *Y, const _Tp *Z, const _Tp *W, int orderX, int orderY, int nrOfPoints, std::vector<double> &coefficients);

        static ito::RetVal calcPolyval2DRegularGrid(ito::DataObject *dataZ, int orderX, int orderY, const std::vector<double> &coefficients);
        static ito::RetVal calcPolyval2DSinglePoints(const ito::DataObject *dataX, const ito::DataObject *dataY, ito::DataObject *dataZ, int orderX, int orderY, const std::vector<double> &coefficients);

        static void linearRegression(VecDoub_I &x, VecDoub_I &y, VecDoub_I &w, VecDoub_O &p, Doub& residual);

        template<typename _Tp> static void polyval1d_subtract_basic(_Tp *output, size_t length, const VecDoub &coeffs, size_t output_step = 1);
        template<typename _Tp> static void polyfit1d_basic(const _Tp *input, size_t length, VecDoub &indices_storage, VecDoub &value_storage, VecDoub &weight_storage, FitSVDSimple *fit_func, VecDoub &coeffs, size_t input_step = 1);

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // FITTINGFILTERS_H
