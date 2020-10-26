/* ********************************************************************
    Plugin "FittingFilters" for itom software
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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "fittingfilters.h"

#include "common/numeric.h"
#include "DataObject/dataObjectFuncs.h"
#include "opencv2/imgproc/imgproc.hpp"

#include <omp.h>
#include <QtCore/QtPlugin>
#include <qstringlist.h>
#include <qvariant.h>

#include "pluginVersion.h"
#include "gitVersion.h"
#include <qnumeric.h>

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FittingFiltersInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(FittingFilters)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FittingFiltersInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(FittingFilters);
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
FittingFiltersInterface::FittingFiltersInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("FittingFilters");
    
/*    char docstring[] = \
"This plugin contains algorithms for fitting planes and other two dimensional polynomials to dataObjects \
mainly using the method of least-squares. Some of the included algorithms can also be called with \
weighted values, such that more precise fitting results are achievable. \n\
\n\
Furthermore this plugin also contains methods to finally subtract or reconstruct the fitted surfaces.";
*/    
    m_description = QObject::tr("Plugin with fitting algorithms.");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"This plugin contains algorithms for fitting planes and other two dimensional polynomials to dataObjects \
mainly using the method of least-squares. Some of the included algorithms can also be called with \
weighted values, such that more precise fitting results are achievable. \n\
\n\
Furthermore this plugin also contains methods to finally subtract or reconstruct the fitted surfaces.");

    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LPGL");
    m_aboutThis = QObject::tr(GITVERSION);      
}

//----------------------------------------------------------------------------------------------------------------------------------
FittingFiltersInterface::~FittingFiltersInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
FittingFilters::FittingFilters() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
FittingFilters::~FittingFilters()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::fitPlaneDoc = QObject::tr(
"fits plane in 2D-dataObject and returns plane-parameters A,B,C (z=A+Bx+Cy) \n\
\n\
This fit can be executed by different fit strategies: \n\
- leastSquareFit minimizes the sum of  the squared distances of all valid points to the plane (direct solution)\n\
- leastSquareFitSVD does the same using a svd algorithm \n\
- leastMedianFit minimizes the median of the absolute distances of all valid points to the plane \n\
\n\
The probability values are only important for the least median fit and determine the number of iterations for the \n\
a random search using the equation \n\
\n\
iterations >= ceil(log(allowedErrorProbability)/log(1-validPointProbability)))");

RetVal FittingFilters::fitPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( Param("sourceImage", ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object").toLatin1().data()) );

    paramsOpt->append( Param("method", ParamBase::String | ParamBase::In, "leastSquareFit", tr("fitting method (leastSquareFit [default], leastSquareFitSVD, leastMedianFit)").toLatin1().data()) );
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "leastSquareFit");
    sm->addItem("leastSquareFitSVD");
    sm->addItem("leastMedianFit");
    (*paramsOpt)[0].setMeta(sm,true);

    paramsOpt->append( Param("validPointProbability", ParamBase::Double | ParamBase::In, 0.0, 0.999999, 0.2, tr("probability that 3 randomly selected point of all points only contain trustful (valid) points. (only important for leastMedianFit)").toLatin1().data()) );
    paramsOpt->append( Param("allowedErrorProbability", ParamBase::Double | ParamBase::In, 0.0000001, 1.0, 0.001, tr("allowed probability that the fit is based on a possible outlier (non correct fit). (only important for leastMedianFit)").toLatin1().data()) );

    *paramsOut << Param("A", ParamBase::Double | ParamBase::Out, 0.0, ito::DoubleMeta::all(), tr("Parameter A of regression plane z = A + Bx + Cy").toLatin1().data());
    *paramsOut << Param("B", ParamBase::Double | ParamBase::Out, 0.0, ito::DoubleMeta::all(), tr("Parameter B of regression plane z = A + Bx + Cy").toLatin1().data());
    *paramsOut << Param("C", ParamBase::Double | ParamBase::Out, 0.0, ito::DoubleMeta::all(), tr("Parameter C of regression plane z = A + Bx + Cy").toLatin1().data());

    /*Info: if you change any parameters here, also check the method subtractRegressionLine and adapt it.*/
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
RetVal FittingFilters::fitPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject dObjImages = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "sourceImage", ito::Range::all(), ito::Range::all(), \
                                                                                  retval, -1, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    char *temp = (*paramsOpt)[0].getVal<char*>();
    QString method = static_cast<char*>( temp ); //borrowed reference

    if (!retval.containsError())
    {
        const cv::Mat *plane = dObjImages.getCvPlaneMat(0);

        if (QString::compare(method, "leastSquareFit", Qt::CaseInsensitive) == 0)
        {
            double A,B,C;
            switch (dObjImages.getType())
            {
            case ito::tUInt8:
                retval += lsqFitPlane<ito::uint8>(plane, A, B, C);
                break;
            case ito::tUInt16:
                retval += lsqFitPlane<ito::uint16>(plane, A, B, C);
                break;
            case ito::tUInt32:
                retval += lsqFitPlane<ito::uint32>(plane, A, B, C);
                break;
            case ito::tInt8:
                retval += lsqFitPlane<ito::int8>(plane, A, B, C);
                break;
            case ito::tInt16:
                retval += lsqFitPlane<ito::int16>(plane, A, B, C);
                break;
            case ito::tInt32:
                retval += lsqFitPlane<ito::int32>(plane, A, B, C);
                break;
            case ito::tFloat32:
                retval += lsqFitPlane<ito::float32>(plane, A, B, C);
                break;
            case ito::tFloat64:
                retval += lsqFitPlane<ito::float64>(plane, A, B, C);
                break;
            default:
                retval += ito::RetVal(ito::retError, 0, "invalid data type");
            }

            (*paramsOut)[0].setVal<double>(A);
            (*paramsOut)[1].setVal<double>(B);
            (*paramsOut)[2].setVal<double>(C);
        }
        else if (QString::compare(method, "leastSquareFitSVD", Qt::CaseInsensitive) == 0)
        {
            double A,B,C;
            retval += fitLeastSquarePlaneSVD(plane,A,B,C);
            (*paramsOut)[0].setVal<double>(A);
            (*paramsOut)[1].setVal<double>(B);
            (*paramsOut)[2].setVal<double>(C);
        }
        else if (QString::compare(method, "leastMedianFit", Qt::CaseInsensitive) == 0)
        {
            double alarm_rate = paramsOpt->at(2).getVal<double>();
            double valid_probability = paramsOpt->at(1).getVal<double>();
            double A,B,C;
            switch (dObjImages.getType())
            {
            case ito::tUInt8:
                retval += lmedsFitPlane<ito::uint8>(plane, A, B, C, valid_probability, alarm_rate);
                break;
            case ito::tUInt16:
                retval += lmedsFitPlane<ito::uint16>(plane, A, B, C, valid_probability, alarm_rate);
                break;
            case ito::tUInt32:
                retval += lmedsFitPlane<ito::uint32>(plane, A, B, C, valid_probability, alarm_rate);
                break;
            case ito::tInt8:
                retval += lmedsFitPlane<ito::int8>(plane, A, B, C, valid_probability, alarm_rate);
                break;
            case ito::tInt16:
                retval += lmedsFitPlane<ito::int16>(plane, A, B, C, valid_probability, alarm_rate);
                break;
            case ito::tInt32:
                retval += lmedsFitPlane<ito::int32>(plane, A, B, C, valid_probability, alarm_rate);
                break;
            case ito::tFloat32:
                retval += lmedsFitPlane<ito::float32>(plane, A, B, C, valid_probability, alarm_rate);
                break;
            case ito::tFloat64:
                retval += lmedsFitPlane<ito::float64>(plane, A, B, C, valid_probability, alarm_rate);
                break;
            default:
                retval += ito::RetVal(ito::retError, 0, "invalid data type");
            }

            (*paramsOut)[0].setVal<double>(A);
            (*paramsOut)[1].setVal<double>(B);
            (*paramsOut)[2].setVal<double>(C);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::subtractPlaneDoc = QObject::tr("subtracts plane from 2D-dataObject given by plane-parameters A,B,C (z=A+Bx+Cy) \n\
\n\
If the destinationImage is not the same than the sourceImage, the destinationImage finally is a new data object with the same \
size and type than the sourceImage and contains the data of the sourceImage subtracted by the given plane. If both are the same, \
the subtraction is executed in-place. \n\
\n\
If the input dataObject contains more than one plane, the subtraction is executed separately for each plane.");

RetVal FittingFilters::subtractPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    *paramsMand << Param("sourceImage", ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object").toLatin1().data());
    *paramsMand << Param("destinationImage", ParamBase::DObjPtr | ParamBase::In | ParamBase::Out, NULL, tr("destination image data object").toLatin1().data());
    *paramsMand << Param("A", ParamBase::Double, 0.0, ito::DoubleMeta::all(), tr("Parameter A of regression plane z = A + Bx + Cy, which is subtracted").toLatin1().data());
    *paramsMand << Param("B", ParamBase::Double, 0.0, ito::DoubleMeta::all(), tr("Parameter B of regression plane z = A + Bx + Cy, which is subtracted").toLatin1().data());
    *paramsMand << Param("C", ParamBase::Double, 0.0, ito::DoubleMeta::all(), tr("Parameter C of regression plane z = A + Bx + Cy, which is subtracted").toLatin1().data());

    /*Info: if you change any parameters here, also check the method subtractRegressionLine and adapt it.*/

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
RetVal FittingFilters::subtractPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObjInput  = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );
    ito::DataObject *dObjOutput = static_cast<ito::DataObject*>( (*paramsMand)[1].getVal<void*>() );

    double A = static_cast<double>( (*paramsMand)[2].getVal<double>() );
    double B = static_cast<double>( (*paramsMand)[3].getVal<double>() );
    double C = static_cast<double>( (*paramsMand)[4].getVal<double>() );

    if (dObjInput->getDims() != 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image must be two-dimensional.").toLatin1().data());
    }

    if (dObjInput->getType() == tComplex64 || dObjInput->getType() == tComplex128)
    {
        return ito::RetVal(retError, 0, tr("source matrix must be of type (u)int8, (u)int16, (u)int32, float32 or float64").toLatin1().data());
    }

    if (dObjOutput == NULL)
    {
        return ito::RetVal(retError, 0, tr("destination matrix is NULL").toLatin1().data());
    }

    ito::DataObject dObjDst;
    if (dObjInput == dObjOutput)
    {
        dObjDst = *dObjInput;
    }
    else
    {
        int *sizes = new int[ dObjInput->getDims() ];
        for (int i = 0; i < dObjInput->getDims(); ++i)
        {
            sizes[i] = dObjInput->getSize(i);
        }
        dObjDst = ito::DataObject(dObjInput->getDims(), sizes, dObjInput->getType());
        dObjInput->copyAxisTagsTo(dObjDst);
        dObjInput->copyTagMapTo(dObjDst);
        *dObjOutput = dObjDst;
    }

    int numPlanes = dObjDst.calcNumMats();
    cv::Mat *inpPlane = NULL;
    cv::Mat *outputPlane = NULL;

    for (int i = 0; i < numPlanes; ++i)
    {
        inpPlane = (cv::Mat*)(dObjInput->get_mdata()[ dObjInput->seekMat(i) ]);
        outputPlane = (cv::Mat*)(dObjDst.get_mdata()[ dObjDst.seekMat(i) ]);

        switch( dObjInput->getType() )
        {
        case tUInt8:
            retval += subtractPlaneTemplate<uint8>(inpPlane, outputPlane, A, B, C);
            break;
        case tInt8:
            retval += subtractPlaneTemplate<int8>(inpPlane, outputPlane, A, B, C);
            break;
        case tUInt16:
            retval += subtractPlaneTemplate<uint16>(inpPlane, outputPlane, A, B, C);
            break;
        case tInt16:
            retval += subtractPlaneTemplate<int16>(inpPlane, outputPlane, A, B, C);
            break;
        case tUInt32:
            retval += subtractPlaneTemplate<uint32>(inpPlane, outputPlane, A, B, C);
            break;
        case tInt32:
            retval += subtractPlaneTemplate<int32>(inpPlane, outputPlane, A, B, C);
            break;
        case tFloat32:
            retval += subtractPlaneTemplate<ito::float32>(inpPlane, outputPlane, A, B, C);
            break;
        case tFloat64:
            retval += subtractPlaneTemplate<ito::float64>(inpPlane, outputPlane, A, B, C);
            break;
        }
    }

    QString msg;
    msg = tr("Substracted plane with A = %1, B = %2, C = %3").arg(A).arg(B).arg(C);
    dObjDst.addToProtocol(std::string(msg.toLatin1().data()));

    return retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::subtractRegressionPlaneDoc = QObject::tr("subtracts a fitted regression plane from the given 2D input dataObject . \n\
\n\
This method firstly executes the filter *fitPlane* followed by *subtractPlane*.");

RetVal FittingFilters::subtractRegressionPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    *paramsMand << Param("sourceImage", ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("source image data object").toLatin1().data());
    *paramsMand << Param("destinationImage", ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("destination image data object").toLatin1().data());

    paramsOpt->append(Param("method", ParamBase::String | ParamBase::In, "leastSquareFit", tr("fitting method (leastSquareFit [default], leastSquareFitSVD, leastMedianFit)").toLatin1().data()));
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "leastSquareFit");
    sm->addItem("leastSquareFitSVD");
    sm->addItem("leastMedianFit");
    (*paramsOpt)[0].setMeta(sm, true);

    paramsOpt->append(Param("validPointProbability", ParamBase::Double | ParamBase::In, 0.0, 0.999999, 0.2, tr("probability that 3 randomly selected point of all points only contain trustful (valid) points. (only important for leastMedianFit)").toLatin1().data()));
    paramsOpt->append(Param("allowedErrorProbability", ParamBase::Double | ParamBase::In, 0.0000001, 1.0, 0.001, tr("allowed probability that the fit is based on a possible outlier (non correct fit). (only important for leastMedianFit)").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
RetVal FittingFilters::subtractRegressionPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    QVector<ito::ParamBase> paramsMandTemp;
    QVector<ito::ParamBase> paramsOptTemp;
    QVector<ito::ParamBase> paramsOutTemp;

    paramsMandTemp << (*paramsMand)[0]; //source object
    paramsOptTemp << (*paramsOpt)[0]; //method
    paramsOptTemp << (*paramsOpt)[1]; //validPointProbability
    paramsOptTemp << (*paramsOpt)[2]; //allowedErrorProbability
    paramsOutTemp << ParamBase("A", ParamBase::Double, 0.0);
    paramsOutTemp << ParamBase("B", ParamBase::Double, 0.0);
    paramsOutTemp << ParamBase("C", ParamBase::Double, 0.0);


    retval += FittingFilters::fitPlane( &paramsMandTemp, &paramsOptTemp, &paramsOutTemp );

    if (retval == retOk)
    {
        paramsMandTemp.clear();
        paramsOptTemp.clear();
        paramsMandTemp << (*paramsMand)[0] << (*paramsMand)[1] << paramsOutTemp;
        paramsOutTemp.clear();
        retval += FittingFilters::subtractPlane( &paramsMandTemp, &paramsOptTemp, paramsOut);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::polyfitWeighted2DDoc = QObject::tr("This method fits a two-dimensional polynomial of given order in x- and y-direction to the \
data 'inputData'. \n\
\n\
For the fit, the optional scale and offset values of the input data object are considered. The fit is executed in double precision, \
such that the input is converted to float64 (if not yet done). NaN values in the input data object are ignored. Optionally, you \
can give a weighting data object (needs to have the same dimension and size than inputData) such that the values are weighted with \
the values of the data object 'weights'. Values with corresponding weights <= 0 are ignored as well. \n\
\n\
Depending on the orders, the fitted polynomial, whose coefficients are returned by this filter, has the following form: \n\
\n\
    if (orderX <= orderY): \n\
    \n\
        f(x,y) = \\sum_{i=0}^orderX \\sum_{j=0}^{orderY-i} p_{ij} x^i y^j \n\
    else: \n\
    \n\
        f(x,y) = \\sum_{j=0}^orderY \\sum_{i=0}^{orderX-i} p_{ij} x^i y^j \n\
\n\
The coefficients p_ij are stored in the coefficients vector in the order they appear in the equation above. \n\
\n\
The solver uses a Vandermonde matrix V as solving strategy and tries to solve V*p=Z, where Z are the valid values of the input data object. \
The overdetermined system of linear equations is finally solved using a QR factorization of V. If this module is compiled with LAPACK, its solvers \
are used, else the solve-command of OpenCV (slower) is called. In order to speed up the calculation you can use the parameter 'reduceFactor'. If \
it is set to any value >= 1, The input plane is divided into a grid of (orderY+1)*reduceFactor x (orderX+1)*reduceFactor rectangles. In every rectangle \
an arbitrary valid value is selected and used for the determination only. If no valid value could be found after a certain number of new random values, \
no value is taken from this rectangle. The algorithm returns an error if less values could have been selected than are needed for the fit of given orders. \n\
\n\
The definition of the polynomial function is slightly different than the one used in the similar fitting function 'fitPolynom2D'.");

/*static*/ ito::RetVal FittingFilters::polyfitWeighted2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("inputData", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input data object, the fit is based on the regular grid (x,y) defined by the scaling and offset of this data object") );
    paramsMand->append( ito::Param("orderX", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in x-direction"));
    paramsMand->append( ito::Param("orderY", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in y-direction"));

    paramsOpt->append( ito::Param("weights", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "weights, if not given all values are equally weighted") );
    paramsOpt->append( ito::Param("reduceFactor", ito::ParamBase::Double | ito::ParamBase::In, -1.0, 100000.0, -1.0, "If this factor is >= 1.0, every plane of the data object is divided into fields (reduceFactor * (orderXorY+1)) in x-direction and y-direction respectively, where one random, valid value is picked. If < 1.0 (e.g. -1.0 [default]) all valid points are picked"));

    paramsOut->append( ito::Param("coefficients", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, "fitted polynomial coefficients"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::polyfitWeighted2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject *input = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *weights = paramsOpt->at(0).getVal<ito::DataObject*>();
    double reduceFactor = paramsOpt->at(1).getVal<double>();
    if (reduceFactor < 1.0) reduceFactor = -1.0;

    int orderX = paramsMand->at(1).getVal<int>();
    int orderY = paramsMand->at(2).getVal<int>();

    std::vector<double> coefficients;

    retval += calcPolyfitWeighted2DRegularGrid(input, orderX, orderY, coefficients, reduceFactor, weights);

    (*paramsOut)[0].setVal<double*>( coefficients.data(),  (int)coefficients.size() );

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::polyfitWeighted2DSinglePointsDoc = QObject::tr("This method fits a two-dimensional polynomial of given order in x- and y-direction to the \
points whose x, y and z coordinates are given in 'xData', 'yData' and 'zData'. \n\
\n\
The fit is executed in double precision, \
such that the input is converted to float64 (if not yet done). NaN values in the x, y or z data objects are ignored. Optionally, you \
can give a weighting data object (needs to have the same dimension and size than inputData) such that the values are weighted with \
the values of the data object 'weights'. Values with corresponding weights <= 0 are ignored as well. \n\
\n\
All input data objects must have the same size. \n\
\n\
Depending on the orders, the fitted polynomial, whose coefficients are returned by this filter, has the following form: \n\
\n\
    if (orderX <= orderY): \n\
    \n\
        z = f(x,y) = \\sum_{i=0}^orderX \\sum_{j=0}^{orderY-i} p_{ij} x^i y^j \n\
    else: \n\
    \n\
        z = f(x,y) = \\sum_{j=0}^orderY \\sum_{i=0}^{orderX-i} p_{ij} x^i y^j \n\
\n\
The coefficients p_ij are stored in the coefficients vector in the order they appear in the equation above. \n\
\n\
The solver uses a Vandermonde matrix V as solving strategy and tries to solve V*p=Z, where Z are the valid values of the input data object. \
The overdetermined system of linear equations is finally solved using a QR factorization of V. If this module is compiled with LAPACK, its solvers \
are used, else the solve-command of OpenCV (slower) is called. \n\
\n\
The definition of the polynomial function is slightly different than the one used in the similar fitting function 'fitPolynom2D'.");


/*static*/ ito::RetVal FittingFilters::polyfitWeighted2DSinglePointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("xData", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input data object with x-coordinates of points") );
    paramsMand->append( ito::Param("yData", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input data object with y-coordinates of points") );
    paramsMand->append( ito::Param("zData", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input data object with z-coordinates of points") );
    paramsMand->append( ito::Param("orderX", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in x-direction"));
    paramsMand->append( ito::Param("orderY", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in y-direction"));

    paramsOpt->append( ito::Param("weights", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "weights, if not given all values are equally weighted") );

    paramsOut->append( ito::Param("coefficients", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, "fitted polynomial coefficients"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::polyfitWeighted2DSinglePoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    const ito::DataObject *xData = paramsMand->at(0).getVal<ito::DataObject*>();
    const ito::DataObject *yData = paramsMand->at(1).getVal<ito::DataObject*>();
    const ito::DataObject *zData = paramsMand->at(2).getVal<ito::DataObject*>();
    ito::DataObject *weights = paramsOpt->at(0).getVal<ito::DataObject*>();

    int orderX = paramsMand->at(3).getVal<int>();
    int orderY = paramsMand->at(4).getVal<int>();

    std::vector<double> coefficients;

    retval += calcPolyfitWeighted2DSinglePoints(xData, yData, zData, orderX, orderY, coefficients, weights);

    (*paramsOut)[0].setVal<double*>( coefficients.data(),  (int)coefficients.size() );

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::polyval2DDoc = QObject::tr("This method evaluates a two-dimensional polynom for every point in a given data object\n\
\n\
For every single pixel in the input data object 'dataZ', its physical coordinate (using scale and offset of the data object) \
is taken and the polynomial (given by its coefficients) is evaluated and stored in the pixel. \
The data object is hereby converted to float64. \n\
\n\
The polynomial coefficients (p0, p1, ...) are those returned by the filter 'fitPolynom2D' and depend on the polynomial order in X and Y \
direction: \n\
\n\
    if (orderX <= orderY): \n\
    \n\
        f(x,y) = \\sum_{i=0}^orderX \\sum_{j=0}^{orderY-i} p_{ij} x^i y^j \n\
    else: \n\
    \n\
        f(x,y) = \\sum_{j=0}^orderY \\sum_{i=0}^{orderX-i} p_{ij} x^i y^j \n\
\n\
The coefficients p_ij are stored in the coefficients vector in the order they appear in the equation above.");

/*static*/ ito::RetVal FittingFilters::polyval2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("dataZ", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "data object with given size and data type including scale and offset value. Depending on these values, this data object is finally filled with the evaluated polynomial function") );

    paramsMand->append( ito::Param("coefficients", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, "polynom coefficients as they come from polyfitWeighted2D"));

    paramsMand->append( ito::Param("orderX", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in x-direction"));
    paramsMand->append( ito::Param("orderY", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in y-direction"));
    
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::polyval2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject *dataZ = paramsMand->at(0).getVal<ito::DataObject*>();

    int lp = paramsMand->at(1).getLen();
    double *c = paramsMand->at(1).getVal<double*>();

    std::vector<double> coefficients;

    if (lp > 0)
    {
        coefficients.resize(lp);
        memcpy( coefficients.data(), c, sizeof(double)*lp);
    }

    retval += calcPolyval2DRegularGrid(dataZ, paramsMand->at(2).getVal<int>(), paramsMand->at(3).getVal<int>(), coefficients);

    QString msg;
    msg = tr("Generated object via polyval2D with order X = %1, Y = %2").arg(paramsMand->at(2).getVal<int>()).arg(paramsMand->at(3).getVal<int>());
    dataZ->addToProtocol(std::string(msg.toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::polyval2DSinglePointsDoc = QObject::tr("This method evaluates a two-dimensional polynom for every x- and y- coordinate given in xData in yData\n\
\n\
For every single pixel whose x- and y-coordinate is given by corresponding values in xData and yData \
the polynomial (given by its coefficients) is evaluated and stored in zData (float64, same size than xData and yData). \
\n\
The polynomial coefficients (p0, p1, ...) are those returned by the filter 'fitPolynom2D' and depend on the polynomial order in X and Y \
direction: \n\
\n\
    if (orderX <= orderY): \n\
    \n\
        f(x,y) = \\sum_{i=0}^orderX \\sum_{j=0}^{orderY-i} p_{ij} x^i y^j \n\
    else: \n\
    \n\
        f(x,y) = \\sum_{j=0}^orderY \\sum_{i=0}^{orderX-i} p_{ij} x^i y^j \n\
\n\
The coefficients p_ij are stored in the coefficients vector in the order they appear in the equation above.");

/*static*/ ito::RetVal FittingFilters::polyval2DSinglePointsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("dataX", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "data object with x-coordinates (interpreted as float64).") );

    paramsMand->append( ito::Param("dataY", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "data object with y-coordinates (interpreted as float64).") );

    paramsMand->append( ito::Param("dataZ", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "This data object is finally filled with the evaluated polynomial function (float64)") );

    paramsMand->append( ito::Param("coefficients", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, "polynom coefficients as they come from polyfitWeighted2D"));

    paramsMand->append( ito::Param("orderX", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in x-direction"));
    paramsMand->append( ito::Param("orderY", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in y-direction"));
    
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::polyval2DSinglePoints(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    const ito::DataObject *dataX = paramsMand->at(0).getVal<ito::DataObject*>();
    const ito::DataObject *dataY = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::DataObject *dataZ = paramsMand->at(2).getVal<ito::DataObject*>();

    int lp = paramsMand->at(3).getLen();
    double *c = paramsMand->at(3).getVal<double*>();

    std::vector<double> coefficients;

    if (lp > 0)
    {
        coefficients.resize(lp);
        memcpy( coefficients.data(), c, sizeof(double)*lp);
    }

    retval += calcPolyval2DSinglePoints(dataX, dataY, dataZ, paramsMand->at(4).getVal<int>(), paramsMand->at(5).getVal<int>(), coefficients);

    QString msg;
    msg = tr("Generated object via polyval2DSinglePoints with order X = %1, Y = %2").arg(paramsMand->at(4).getVal<int>()).arg(paramsMand->at(5).getVal<int>());
    dataZ->addToProtocol(std::string(msg.toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::fitPolynom1D_ZDoc = QObject::tr("One-dimensional polynomial fit in z-direction for a 3D - data object. \n\
\n\
The input data object must be three-dimensional and is internally casted to float64 (if not yet done). The resulting polynomial \
parameters per pixel are stored in the output data object 'polynoms' whose z-dimension is equal to (order+2). The first (order+1) \
planes contain the coefficients p0...pn and the last plane contain the pixel wise residual error. \n\
\n\
The polynomial is y(x) = p0 + x*p1 ... + x^n*pn \n\
The residual is the sum of the quadratical errors from each valid pixel to the fitted polynomial. \n\
\n\
If no 'xVals' are assigned, the x-values for each plane are calculated using the offset and scale of the data-object in z-direction, \
such that an equally spaced vector of (0,1,2,3...) is the default. \n\
\n\
You can additionally give a weight data object (same dimension than 'data') for weighting the values. NaN values in 'data' and \
weights <= 0 are ignored. If a fit cannot be done due to too less or degenerated values, NaN is returned in 'polynoms' at this pixel. \n\
\n\
For a first order fit, a direct least squares solution is used which is very fast, for the other orders a system of linear equations \
is solved (using a SVD decomposition) which can be slower. On a multi-core processor you can assign a number of threads that are used \
to parallely compute the approximations for each pixel.");

/*static*/ ito::RetVal FittingFilters::fitPolynom1D_ZParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("data", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input data object (3 dimensions, [z,y,x]).") );
    paramsMand->append( ito::Param("polynoms", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "float64, size: [(order+2),y,x]. The first (order+1) planes contain the coefficients, the last plane contains the residual error for each polynomial fit.") );
    paramsMand->append( ito::Param("order", ito::ParamBase::Int | ito::ParamBase::In, 1, 7, 1, "polynomial order"));

    paramsOpt->append( ito::Param("weights", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "weights (same dimensions than data)") );
    paramsOpt->append( ito::Param("xVals", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, "x-value vector (length must be the same than z-size of data) [default: equally spaced values are assumed using scale and offset of the data object in z-direction]") );
    paramsOpt->append( ito::Param("numThreads", ito::ParamBase::Int | ito::ParamBase::In, 1, omp_get_max_threads(), 1,  "number of threads used for the parallel determination") );

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::fitPolynom1D_Z(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject *input = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *inputWeights = paramsOpt->at(0).getVal<ito::DataObject*>();
    ito::DataObject *output = paramsMand->at(1).getVal<ito::DataObject*>();
    double *xValsArray = paramsOpt->at(1).getVal<double*>();
    int xValsArrayLen = paramsOpt->at(1).getLen();
    int numThreads = paramsOpt->at(2).getVal<int>();

    int order = 0;

    if (input == NULL || output == NULL)
    {
        retval += ito::RetVal(ito::retError,0,"parameters 'data' or 'polynoms' are NULL");
    }
    else if (input == output)
    {
        retval += ito::RetVal(ito::retError,0,"inplace determination not possible. Different 'data' and 'polynoms' parameters necessary.");
    }
    else
    {
        order = paramsMand->at(2).getVal<int>();

        ito::DataObject *data = apiCreateFromDataObject(input, 3, ito::tFloat64, NULL, &retval);

        if (retval.containsError()) { if (data) delete data; return retval; }

        int planes = data->getSize(0);
        int m = data->getSize(1);
        int n = data->getSize(2);
        int sizes[] = {planes, planes, m, m, n, n};
        ito::DataObject *weights = inputWeights ? apiCreateFromDataObject(inputWeights,3,ito::tFloat64,sizes,&retval) : NULL;

        if (xValsArrayLen > 0 && xValsArrayLen != planes)
        {
            retval += ito::RetVal(ito::retError,0,"xVals vector must have the same size than the size of 'data' in z-direction");
        }

        if (!retval.containsError())
        {
            *output = ito::DataObject(order+2, data->getSize(1), data->getSize(2), ito::tFloat64); //last plane contains the quadratical error

            FitSVDSimple *fit = NULL;

            switch(order)
            {
            case 1:
                fit = NULL; //new FitSVDSimple( FitSVDSimple::poly1d_1 );
                break;
            case 2:
                fit = new FitSVDSimple( FitSVDSimple::poly1d_2 );
                break;
            case 3:
                fit = new FitSVDSimple( FitSVDSimple::poly1d_3 );
                break;
            case 4:
                fit = new FitSVDSimple( FitSVDSimple::poly1d_4 );
                break;
            case 5:
                fit = new FitSVDSimple( FitSVDSimple::poly1d_5 );
                break;
            case 6:
                fit = new FitSVDSimple( FitSVDSimple::poly1d_6 );
                break;
            case 7:
                fit = new FitSVDSimple( FitSVDSimple::poly1d_7 );
                break;
            }

            VecDoub xVals(planes);

            bool isInsideImage;

            cv::Mat** dataMats = new cv::Mat*[planes];
            cv::Mat** weightMats = new cv::Mat*[planes];
            cv::Mat** coeffMats = new cv::Mat*[order+2];

            for (int i = 0; i < planes; ++i)
            {
                if (xValsArrayLen > 0)
                {
                    xVals[i] = xValsArray[i];
                }
                else
                {
                    xVals[i] = data->getPixToPhys(0, (double)i, isInsideImage);
                }

                dataMats[i] = (cv::Mat*)(data->get_mdata()[ data->seekMat(i) ]);

                if (weights)
                {
                    weightMats[i] = (cv::Mat*)(weights->get_mdata()[ weights->seekMat(i) ]);
                } 
            }

            for (int i = 0; i < (order+2); ++i)
            {
                coeffMats[i] = (cv::Mat*)(output->get_mdata()[ output->seekMat(i) ]);
            }

            int dataSteps[] = { (int)dataMats[0]->step[0], (int)dataMats[0]->step[1] };
            int weightSteps[] = { 0, 0 };
            int coeffSteps[] = { (int)coeffMats[0]->step[0], (int)coeffMats[0]->step[1] };

            if (weights)
            {
                weightSteps[0] = (int)weightMats[0]->step[0];
                weightSteps[1] = (int)weightMats[0]->step[1];
            }

            omp_set_num_threads( std::min(omp_get_max_threads(),numThreads));

            /*#pragma omp parallel default(shared)
            {*/
                VecDoub yVals(planes);
                VecDoub wVals(planes, 1);
                VecDoub coefficients(order+1);
                Doub chisq; // = 0.0;

                #pragma omp parallel for firstprivate(yVals, wVals, coefficients, chisq)
                for (int mi = 0; mi < m; ++mi)
                {                  
                    for (int ni = 0; ni < n; ++ni)
                    {
                        //fill yVals
                        for (int i = 0; i < planes; ++i)
                        {
                            yVals[i] = *( (ito::float64*)(dataMats[i]->data + dataSteps[0] * mi + dataSteps[1] * ni) );
                        }

                        if (weights)
                        {
                            //fill wVals
                            for (int i = 0; i < planes; ++i)
                            {
                                wVals[i] = *( (ito::float64*)(weightMats[i]->data + weightSteps[0] * mi + weightSteps[1] * ni) );
                            }
                        }


                        if (fit)
                        {
                            fit->fit( xVals, yVals, wVals, coefficients, chisq );
                        }
                        else
                        {
                            linearRegression( xVals, yVals, wVals, coefficients, chisq );
                        }

                        //write results
                        for (int i = 0; i <= order; ++i)
                        {
                            *( (ito::float64*)(coeffMats[i]->data + coeffSteps[0] * mi + coeffSteps[1] * ni) ) = coefficients[i];
                        }

                        *( (ito::float64*)(coeffMats[order+1]->data + coeffSteps[0] * mi + coeffSteps[1] * ni) ) = chisq;
                    }
                }
            //} //end pragma parallel

            delete[] dataMats;
            delete[] weightMats;
            delete[] coeffMats;
            delete fit;
        }

        delete weights;
        delete data;
    }

    if (!retval.containsError())
    {
        QString msg;
        msg = tr("Calculated polynomical coeffs along z-direction with order Z = %1").arg(order);
        output->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::subtract1DRegressionPolynomDoc = QObject::tr(
"subtracts a one-dimensional fitted polynom (along a given axis) from the values in the given 1D or 2D data object. \n\
\n\
This filter operates inplace and subtracts from fitted one-dimensional polynoms from the values in the given 2D data object 'data'. The polynoms are either fitted along the \n\
vertical or horizontal axis. You can choose a polynomial order between 1 (line) and 7. The values are uniformly weighted for the fit. The algorithm uses a fast, direct solution \n\
for the line regression fits and a singular value decomposition for all other cases (see fitPolynom1D_Z). The fit is done in double precision while the type of 'data' is not changed.");

/*static*/ ito::RetVal FittingFilters::subtract1DRegressionPolynomParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("input", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "real valued input object (1D or 2D)") );
    paramsMand->append( ito::Param("output", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "resulting output object (same than input is allowed, else output has the same type and size than input)") );
    paramsMand->append( ito::Param("order", ito::ParamBase::Int | ito::ParamBase::In, 1, 7, 1, "polynomial order"));
    
    paramsOpt->append( ito::Param("axis", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, "axis index along which the 1d regressions are independently determined and subtracted (0: vertical, 1: horizontal)."));
    paramsOpt->append( ito::Param("numThreads", ito::ParamBase::Int | ito::ParamBase::In, 1, omp_get_max_threads(), 1,  "number of threads used for the parallel determination") );

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::subtract1DRegressionPolynom(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    const ito::DataObject *input = paramsMand->at(0).getVal<const ito::DataObject*>();
    ito::DataObject *output = paramsMand->at(1).getVal<ito::DataObject*>();
    int m = 0;
    int n = 0;
    
    int numThreads = paramsOpt->at(1).getVal<int>();
    int axis = paramsOpt->at(0).getVal<int>();

    int order = paramsMand->at(2).getVal<int>();

    if (input == NULL)
    {
        retval += ito::RetVal(ito::retError,0,"parameter 'input' is NULL");
    }
    else
    {
        retval += ito::dObjHelper::verify2DDataObject(input, "input", 1, std::numeric_limits<int>::max(), 1, std::numeric_limits<int>::max(), 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    }

    if (!retval.containsError())
    {
        if (input != output) //not inplace
        {
            retval += input->copyTo(*output);
        }
    }

    if (!retval.containsError())
    {
        m = input->getSize(0);
        n = input->getSize(1);

        if (axis == 0 && m <= order)
        {
            retval += ito::RetVal::format(ito::retError, 0, "input must have at least %i rows", order+1);
        }
        else if (axis == 1 && n <= order)
        {
            retval += ito::RetVal::format(ito::retError, 0, "input must have at least %i columns", order+1);
        }
    }

    if (!retval.containsError())
    {
        const cv::Mat *input_ = input->getCvPlaneMat(0);
        cv::Mat *output_ = output->getCvPlaneMat(0);
        size_t step = input_->step1(axis);
        FitSVDSimple *fit = NULL;

        switch(order)
        {
        case 1:
            fit = NULL; //new FitSVDSimple( FitSVDSimple::poly1d_1 );
            break;
        case 2:
            fit = new FitSVDSimple( FitSVDSimple::poly1d_2 );
            break;
        case 3:
            fit = new FitSVDSimple( FitSVDSimple::poly1d_3 );
            break;
        case 4:
            fit = new FitSVDSimple( FitSVDSimple::poly1d_4 );
            break;
        case 5:
            fit = new FitSVDSimple( FitSVDSimple::poly1d_5 );
            break;
        case 6:
            fit = new FitSVDSimple( FitSVDSimple::poly1d_6 );
            break;
        case 7:
            fit = new FitSVDSimple( FitSVDSimple::poly1d_7 );
            break;
        }

        VecDoub values(axis == 0 ? m : n);
        VecDoub indices(axis == 0 ? m : n);
        for (int i = 0; i < indices.size(); ++i)
        {
            indices[i] = i;
        }
        VecDoub weights(axis == 0 ? m : n, 1.0);
        VecDoub coefficients(order+1);

        omp_set_num_threads( std::min(omp_get_max_threads(),numThreads));

        if (axis == 0) //vertical
        {
            switch (input->getType())
            {
            case ito::tUInt8:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < n; ++i)
                {   
                    polyfit1d_basic(&((const ito::uint8*)(input_->data))[i], m, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(&((ito::uint8*)(output_->data))[i], m, coefficients, step);
                }
                break;
            case ito::tInt8:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < n; ++i)
                {   
                    polyfit1d_basic(&((const ito::int8*)(input_->data))[i], m, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(&((ito::int8*)(output_->data))[i], m, coefficients, step);
                }
                break;
            case ito::tUInt16:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < n; ++i)
                {   
                    polyfit1d_basic(&((const ito::uint16*)(input_->data))[i], m, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(&((ito::uint16*)(output_->data))[i], m, coefficients, step);
                }
                break;
            case ito::tInt16:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < n; ++i)
                {   
                    polyfit1d_basic(&((const ito::int16*)(input_->data))[i], m, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(&((ito::int16*)(output_->data))[i], m, coefficients, step);
                }
                break;
            case ito::tUInt32:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < n; ++i)
                {   
                    polyfit1d_basic(&((const ito::uint32*)(input_->data))[i], m, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(&((ito::uint32*)(output_->data))[i], m, coefficients, step);
                }
                break;
            case ito::tInt32:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < n; ++i)
                {   
                    polyfit1d_basic(&((const ito::int32*)(input_->data))[i], m, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(&((ito::int32*)(output_->data))[i], m, coefficients, step);
                }
                break;
            case ito::tFloat32:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < n; ++i)
                {   
                    polyfit1d_basic(&((const ito::float32*)(input_->data))[i], m, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(&((ito::float32*)(output_->data))[i], m, coefficients, step);
                }
                break;
            case ito::tFloat64:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < n; ++i)
                {   
                    polyfit1d_basic(&((const ito::float64*)(input_->data))[i], m, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(&((ito::float64*)(output_->data))[i], m, coefficients, step);
                }
                break;
            }
        }
        else //horizontal
        {
            switch (input->getType())
            {
            case ito::tUInt8:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < m; ++i)
                {   
                    polyfit1d_basic((const ito::uint8*)(input_->ptr(i)), n, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(((ito::uint8*)(output_->ptr(i))), n, coefficients);
                }
                break;
            case ito::tInt8:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < n; ++i)
                {   
                    polyfit1d_basic(((const ito::int8*)(input_->ptr(i))), n, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(((ito::int8*)(output_->ptr(i))), n, coefficients);
                }
                break;
            case ito::tUInt16:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < m; ++i)
                {   
                    polyfit1d_basic(((const ito::uint16*)(input_->ptr(i))), n, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(((ito::uint16*)(output_->ptr(i))), n, coefficients);
                }
                break;
            case ito::tInt16:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < m; ++i)
                {   
                    polyfit1d_basic(((const ito::int16*)(input_->ptr(i))), n, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(((ito::int16*)(output_->ptr(i))), n, coefficients);
                }
                break;
            case ito::tUInt32:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < m; ++i)
                {   
                    polyfit1d_basic(((const ito::uint32*)(input_->ptr(i))), n, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(((ito::uint32*)(output_->ptr(i))), n, coefficients);
                }
                break;
            case ito::tInt32:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < m; ++i)
                {   
                    polyfit1d_basic(((const ito::int32*)(input_->ptr(i))), n, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(((ito::int32*)(output_->ptr(i))), n, coefficients);
                }
                break;
            case ito::tFloat32:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < m; ++i)
                {   
                    polyfit1d_basic(((const ito::float32*)(input_->ptr(i))), n, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(((ito::float32*)(output_->ptr(i))), n, coefficients);
                }
                break;
            case ito::tFloat64:
                #pragma omp parallel for firstprivate(coefficients)
                for (int i = 0; i < m; ++i)
                {   
                    polyfit1d_basic(((const ito::float64*)(input_->ptr(i))), n, indices, values, weights, fit, coefficients, step);
                    polyval1d_subtract_basic(((ito::float64*)(output_->ptr(i))), n, coefficients);
                }
                break;
            }
        }

        delete fit;
    }

    if (!retval.containsError())
    {
        QString msg;
        msg = tr("Subtracted polynomial of %1th order along axis %2").arg(order).arg(axis);
        output->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FittingFilters::getInterpolatedValuesDoc = QObject::tr("returns the linearly interpolated values of a given input dataObject at specific 2D point coordinates. \n\
\n\
The given input data object must be a real valued object with two dimensions or a region of interest that only contains one plane (e.g. 1xMxN). \n\
The point coordinates (coordsSubPix) is a Nx2 floating point data object where each row is the row and column coordinate (sub-pixel) of the desired value. The values must be given \n\
in the coordinates of the data object (scale values).\n\
The resulting interpolated values are returned as 'values' list. The input data object is allowed to contain non-finite values. \n\
\n\
For the interpolation a search rectangle whose height and width is given by 'searchRect' is centered at the rounded coordinate and a plane is robustly fitted into the valid \n\
values that lie within the rectangle. The value is then determined using the coefficients of the fitted plane. Infinite values are ignored for the determination of the plane. \n\
The plane is calculated by least-squares fit. If the rectangle exceeds the boundaries of the given matrix, it moved inside of the matrix such that the searched coordinate still lies within \n\
the rectangle. If this is not possible, NaN is returned as value.");

/*static*/ ito::RetVal FittingFilters::getInterpolatedValuesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("dataObj", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input real-valued data object (2D or ROI containing only one plane).") );
    paramsMand->append( ito::Param("coordsSubPix", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input Nx2 data object containing the sub-pixel (column,row) coordinates of each point (given in scale value of dataObj).") );

    int rect[] = {2,2};
    paramsOpt->append( ito::Param("searchRect", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, "[height, width] of the search rectangle for the linear interpolation. A plane fit is executed for all finite values within the rectangle and the output value is determined based on the plane coefficients. If the size if even, its size drifts towards the trend direction given by the coordinate value.") );
    (*paramsOpt)[0].setVal<int*>(rect, 2);

    paramsOpt->append( Param("method", ito::ParamBase::String | ParamBase::In, "LeastSquares", tr("LeastSquares (default), LMedS (Least median of squares)").toLatin1().data()));
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "LeastSquares");
    sm->addItem("LMedS");
    (*paramsOpt)[1].setMeta(sm,true);

    paramsOpt->append( Param("validPointProbability", ParamBase::Double | ParamBase::In, 0.0, 0.999999, 0.2, tr("probability that 3 randomly selected point of all points only contain trustful (valid) points. (only important for leastMedianFit)").toLatin1().data()) );
    paramsOpt->append( Param("allowedErrorProbability", ParamBase::Double | ParamBase::In, 0.0000001, 1.0, 0.001, tr("allowed probability that the fit is based on a possible outlier (non correct fit). (only important for leastMedianFit)").toLatin1().data()) );

    paramsOut->append( ito::Param("values", ito::ParamBase::DoubleArray | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output vector of type ito::float64 containing the interpolated values (NaN if no value could be found)"));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::getInterpolatedValues(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    int rect[] = {2,2};
    if (paramsOpt->at(0).getLen() != 2)
    {
        retval += ito::RetVal(ito::retError, 0, "searchRect must have two values");
    }
    else
    {
        int *t = paramsOpt->at(0).getVal<int*>();
        rect[0] = t[0];
        rect[1] = t[1];
    }

    if (rect[0] < 2 || rect[1] < 2)
    {
        retval += ito::RetVal(ito::retError, 0, "values of searchRect must be >= 2");
    }

    bool lmeds = (QString::compare("LMedS", paramsOpt->at(1).getVal<char*>(), Qt::CaseInsensitive) == 0); //least median of squares or least squares fit
    double validPointProbability = paramsOpt->at(2).getVal<double>();
    double allowedErrorProbability = paramsOpt->at(3).getVal<double>();

    ito::DataObject dataObj = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "dataObj", ito::Range::all(), ito::Range::all(), retval, -1, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    ito::DataObject coords = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(1).getVal<ito::DataObject*>(), "coordsSubPix", ito::Range::all(), ito::Range(2,2), retval, ito::tFloat64, 0);

    if (!retval.containsError())
    {
        const cv::Mat *mat = dataObj.getCvPlaneMat(0); //shallow copy, will be used to change roi's without impact to the original object
        cv::Mat matRoi;
        ito::float64 *xyCoords;
        int numCoords = coords.getSize(0);

        ito::float64 *val = new ito::float64[numCoords];

        double yPx, xPx;
        double xPxRounded, yPxRounded;
        bool xInside, yInside;
        cv::Rect roi;
        bool valid;
        double A,B,C;

        for (int i = 0; i < numCoords; ++i)
        {
            valid = true;
            xyCoords = (ito::float64*)coords.rowPtr(0,i);

            dataObj.getPhysToPix2D(xyCoords[1], yPx, yInside, xyCoords[0], xPx, xInside);
            
            xPxRounded = qRound(xPx);
            yPxRounded = qRound(yPx);

            //at first try to find the integer based pixel coordinate of the real rectangle, then check if the rectangle is inside of the valid image area
            roi.height = rect[0];
            if (rect[0] % 2 == 0) //height of rect is even ('gerade')
            {
                roi.y = yPxRounded - (rect[0] / 2); //floor
                if (yPx > yPxRounded) //drift versus bottom
                {
                    roi.y ++;
                }
            }
            else //odd
            {
                roi.y = yPxRounded - (rect[0] / 2);
            }

            roi.width = rect[0];
            if (rect[0] % 2 == 0) //width of rect is even ('gerade')
            {
                roi.x = xPxRounded - (rect[0] / 2); //floor
                if (xPx > xPxRounded) //drift versus right side
                {
                    roi.x ++;
                }
            }
            else //odd
            {
                roi.x = xPxRounded - (rect[0] / 2);
            }

            //now check for validity (a correction of half the rect size is allowed)
            if (roi.x < 0)
            {
                if (roi.x < (-rect[1] / 2))
                {
                    valid = false;
                }
                else //move
                {
                    roi.x = 0;
                }
            }

            if (roi.y < 0)
            {
                if (roi.y < (-rect[0] / 2))
                {
                    valid = false;
                }
                else //move
                {
                    roi.y = 0;
                }
            }

            if (roi.x + roi.width > mat->cols)
            {
                int diff = std::min(rect[1] / 2, (roi.x + roi.width - mat->cols)); //mat->cols - roi.x - roi.width - 1);
                roi.x -= diff;

                if (roi.x + roi.width > mat->cols)
                {
                    valid = false;
                }
            }

            if (roi.y + roi.height > mat->rows)
            {
                int diff = std::min(rect[0] / 2, (roi.y + roi.height - mat->rows)); //mat->rows - roi.y - roi.height - 1);
                roi.y -= diff;

                if (roi.y + roi.height > mat->rows)
                {
                    valid = false;
                }
            }

            if (valid)
            {
                matRoi = mat->operator()(roi);

                if (!lmeds)
                {
                    switch (dataObj.getType())
                    {
                    case ito::tUInt8:
                        retval += lsqFitPlane<ito::uint8>(&matRoi, A, B, C);
                        break;
                    case ito::tUInt16:
                        retval += lsqFitPlane<ito::uint16>(&matRoi, A, B, C);
                        break;
                    case ito::tUInt32:
                        retval += lsqFitPlane<ito::uint32>(&matRoi, A, B, C);
                        break;
                    case ito::tInt8:
                        retval += lsqFitPlane<ito::int8>(&matRoi, A, B, C);
                        break;
                    case ito::tInt16:
                        retval += lsqFitPlane<ito::int16>(&matRoi, A, B, C);
                        break;
                    case ito::tInt32:
                        retval += lsqFitPlane<ito::int32>(&matRoi, A, B, C);
                        break;
                    case ito::tFloat32:
                        retval += lsqFitPlane<ito::float32>(&matRoi, A, B, C);
                        break;
                    case ito::tFloat64:
                        retval += lsqFitPlane<ito::float64>(&matRoi, A, B, C);
                        break;
                    default:
                        valid = false;
                    }
                }
                else
                {
                    switch (dataObj.getType())
                    {
                    case ito::tUInt8:
                        retval += lmedsFitPlane<ito::uint8>(&matRoi, A, B, C, validPointProbability, allowedErrorProbability);
                        break;
                    case ito::tUInt16:
                        retval += lmedsFitPlane<ito::uint16>(&matRoi, A, B, C, validPointProbability, allowedErrorProbability);
                        break;
                    case ito::tUInt32:
                        retval += lmedsFitPlane<ito::uint32>(&matRoi, A, B, C, validPointProbability, allowedErrorProbability);
                        break;
                    case ito::tInt8:
                        retval += lmedsFitPlane<ito::int8>(&matRoi, A, B, C, validPointProbability, allowedErrorProbability);
                        break;
                    case ito::tInt16:
                        retval += lmedsFitPlane<ito::int16>(&matRoi, A, B, C, validPointProbability, allowedErrorProbability);
                        break;
                    case ito::tInt32:
                        retval += lmedsFitPlane<ito::int32>(&matRoi, A, B, C, validPointProbability, allowedErrorProbability);
                        break;
                    case ito::tFloat32:
                        retval += lmedsFitPlane<ito::float32>(&matRoi, A, B, C, validPointProbability, allowedErrorProbability);
                        break;
                    case ito::tFloat64:
                        retval += lmedsFitPlane<ito::float64>(&matRoi, A, B, C, validPointProbability, allowedErrorProbability);
                        break;
                    default:
                        valid = false;
                    }
                }

                if (valid && retval == ito::retOk)
                {
                    val[i] = A + B * (xPx - roi.x) + C * (yPx - roi.y);
                }
                else
                {
                    val[i] = std::numeric_limits<ito::float64>::quiet_NaN();
                }
            }
            else
            {
                val[i] = std::numeric_limits<ito::float64>::quiet_NaN();
            }
        }

        (*paramsOut)[0].setVal<ito::float64*>(val, numCoords);

        delete[] val;
        val = NULL;

    }

    return retval;
}

//--------------------------------------------------------------------------------------
//careful: values will be changed within this function!
template <typename _Tp> _Tp nanmedian(_Tp *values, uint32 num)
{
    //this algorithms seems to be like the following: http://www.i-programmer.info/babbages-bag/505-quick-median.html?start=1
    ito::uint32 halfKernSize = num / 2;
    ito::uint32 leftElement = 0;
    ito::uint32 rightElement = num - 1;
    ito::uint32 leftPos, rightPos;
    _Tp a;
    _Tp tempValue;
    while (leftElement < rightElement)
    {
        a = values[halfKernSize];
        leftPos = leftElement;
        rightPos = rightElement;
        do
        {
            while (values[leftPos] < a)
            {
                leftPos++;
            }
            while (values[rightPos] > a)
            {
                rightPos--;
            }
            if (leftPos <= rightPos)
            {
                tempValue = values[leftPos];
                values[leftPos] = values[rightPos];
                values[rightPos] = tempValue;
                leftPos++;
                rightPos--;
            }
        } while (leftPos <= rightPos);

        if (rightPos < halfKernSize)
        {
            leftElement = leftPos;
        }
        if (halfKernSize < leftPos)
        {
            rightElement = rightPos;
        }
    }
    return values[halfKernSize];
}

#if (CV_MAJOR_VERSION >= 3)
//----------------------------------------------------------------------------------------------------------------------------------
const QString FittingFilters::fillInvalidAreasDoc = QObject::tr("This filter can be used to fill NaN areas in a float32 or float64 input object with interpolated values. \n\
\n\
This filter processes independently every plane of the input object. For every plane the following steps are done: \n\
\n\
1. Locate connected areas with NaN values (using OpenCVs method 'connectedComponentsWithStats'). \n\
2. For every area whose size does not exceed 'maxAreaSize': \n\
   \n\
   - depending on 'regionType' and 'regionExtend' an enlarged area around the NaN area is taken \n\
   - depending on 'method' an interpolation plane or single value is calculated using all valid values within the enlarged area \n\
   - The NaN values within the area are filled with the interpolated plane value of the interpolated scalar \n\
\n\
**RegionType** \n\
\n\
* Per default, the region type is 'BoundingBox'. Then, a rectangular bounding box is set around each area of invalid values and the size of the bounding box. \n\
  is increased in all directions by the parameter 'regionExtend' (different value possible for horizontal and vertical extend). 'BoundingBox' is the fastest\n\
  method, however may produce very varying results, if the shape of the areas is different from simple rectangles. \n\
* ErodeRect: Around the invalid area, an erosion is calculated with a rectangle element. The interpolation is only done with values within the eroded ribbon. \n\
  The extend is the maximum horizontal and vertical distance from a edge element of the invalid area, e.g. (1,1) is a 1-pixel ribbon whose valid values are used for interpolation. \n\
* ErodeEllipse: This is the same than 'ErodeRect', however the element has an ellipse form and not a rectangle making the ribbon more smooth. \n\
\n\
**Method** \n\
\n\
* LeastSquaresPlane: A plane is fitted into the valid values of the region using the least squares fit approach. \n\
* LMedSPlane: A plane is fitted into the valid values of the region using the approach to minimize the median of the squared distances. \n\
* Mean: A mean value of a valid values of the region replaces the invalid values within the region. \n\
* Median: A median value of a valid values of the region replaces the invalid values within the region. \n\
\n\
Currently, the filter does not work inplace such that the output object is always a newly allocated dataObject of the same type and size than the input object.");
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FittingFilters::fillInvalidAreasParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("inputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input real-valued data object with possible NaN values (float32, float64)."));
    paramsMand->append(ito::Param("outputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output real-valued data object that equals 'inputObject' but NaN values are filled with interpolated values (float32, float64)."));  

    paramsOpt->append(Param("method", ito::ParamBase::String | ParamBase::In, "LeastSquaresPlane", tr("LeastSquaresPlane (default), LMedSPlane (Least median of squares), Median, Mean").toLatin1().data()));
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "LeastSquaresPlane");
    sm->addItem("LMedSPlane");
    sm->addItem("Median");
    sm->addItem("Mean");
    (*paramsOpt)[0].setMeta(sm, true);

    paramsOpt->append(Param("regionType", ito::ParamBase::String | ParamBase::In, "BoundingBox", tr("BoundingBox (default): a bounding box is set around each invalid area and the interpolation is done over all valid values within the bounding box. For other region types see the description of this filter.").toLatin1().data()));
    sm = new ito::StringMeta(ito::StringMeta::String, "BoundingBox");
    sm->addItem("ErodeRect");
    sm->addItem("ErodeEllipse");
    (*paramsOpt)[1].setMeta(sm, true);

    int32 extend[] = { 3, 3 };
    paramsOpt->append(Param("regionExtend", ito::ParamBase::IntArray | ParamBase::In, 2, extend, tr("The invalid region is extend by the 'regionType'. The size of the extend in x- and y- direction is given by this parameter as tuple (dx,dy).").toLatin1().data()));
    (*paramsOpt)[2].setMeta(new ito::IntArrayMeta(1, 2001, 1, 2, 2, 1), true);

    paramsOpt->append(Param("maxAreaSize", ito::ParamBase::Int | ParamBase::In, 1, std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), tr("Only invalid area whose number of pixels is below this value are filled by interpolated values.").toLatin1().data()));

    paramsOpt->append(Param("validPointProbability", ParamBase::Double | ParamBase::In, 0.0, 0.999999, 0.2, tr("probability that 3 randomly selected point of all points only contain trustful (valid) points. (only important for leastMedianFitPlane)").toLatin1().data()));
    paramsOpt->append(Param("allowedErrorProbability", ParamBase::Double | ParamBase::In, 0.0000001, 1.0, 0.001, tr("allowed probability that the fit is based on a possible outlier (non correct fit). (only important for leastMedianFitPlane)").toLatin1().data()));

    paramsOpt->append(Param("statistics", ParamBase::DObjPtr | ParamBase::In | ParamBase::Out, NULL, tr("optional dataObject (only possible if 'inputObject' only contains one plane). If given, the dataObject will be a float32 Mx8 dataObject where each row corresponds to one detected area. Its content is [inclusive leftmost pixel-coordinate of the area, inclusive topmost pixel-coordinate of the area, width of bounding box of area in pixel, height of bound box of area in pixel, total number of pixels in area, x-coordinate of centroid of area in physical units, y-coordinate of centroid of area in physical units, 1: area has been filled, else 0]").toLatin1().data()));
    
    paramsOut->append(Param("numTotalInvalidAreas", ParamBase::Int | ParamBase::Out, 0, std::numeric_limits<int>::max(), 0, tr("total number of detected invalid areas").toLatin1().data()));
    paramsOut->append(Param("numFilledAreas", ParamBase::Int | ParamBase::Out, 0, std::numeric_limits<int>::max(), 0, tr("number of areas that where filled (their area size was <= maxAreaSize and the interpolation values based on surrounding pixels are valid).").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> cv::Mat FittingFilters::getNaNMask(const cv::Mat* mat)
{
    cv::Mat mask = cv::Mat::zeros(mat->rows, mat->cols, CV_8U);  // binary mask
    for (int y = 0; y < mat->rows; y++)
    {
        const _Tp* rowPtr = mat->ptr<_Tp>(y);
        int8* rowNaNPtr = mask.ptr<int8>(y);
        for (int x = 0; x < mat->cols; x++)
        {
            rowNaNPtr[x] = cvIsNaN(rowPtr[x]) == true ? 255 : 0;
        }
    }

    return mask;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FittingFilters::fillInvalidAreas(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    const ito::DataObject *inputObject = paramsMand->at(0).getVal<const ito::DataObject*>();
    retval += ito::dObjHelper::verifyDataObjectType(inputObject, "inputObject", 2, ito::tFloat32, ito::tFloat64);
    QByteArray method = paramsOpt->at(0).getVal<const char*>();
    int meth;
    QByteArray regionType = paramsOpt->at(1).getVal<const char*>();
    const int *extend = paramsOpt->at(2).getVal<const int32*>();
    int maxAreaSize = paramsOpt->at(3).getVal<int>();
    double valid_probability = paramsOpt->at(4).getVal<double>();
    double alarm_rate = paramsOpt->at(5).getVal<double>();
    ito::DataObject *statsObj = paramsOpt->at(6).getVal<ito::DataObject*>();

    const cv::Mat *input = NULL;
    cv::Mat *output = NULL;
    cv::Mat nanMask;
    cv::Mat labels, stats, centroids;
    int numTotalAreas = 0;
    int numTotalFilledAreas = 0;
    ito::int32 *statsRow;
    cv::Mat inputPatch;
    cv::Mat outputPatch;
    cv::Mat labelPatch;
    cv::Mat inputPatchValidMask;
    cv::Scalar scalar;
    std::vector<ito::float32> floatVec;
    std::vector<ito::float64> doubleVec;
    ito::float32 *floatPtr;
    ito::float64 *doublePtr;
    ito::int32 *labelPtr;
    int x, y, w, h, x2, y2;
    double A, B, C;
    cv::Mat erodeElement;
    ito::DataObject statisticObject;
    ito::float32 *statisticObjectRowPtr = NULL;
    int dims = inputObject->getDims();

    if (method == "LeastSquaresPlane")
    {
        meth = inputObject->getType() | 0x01000000;
    }
    else if (method == "LMedSPlane")
    {
        meth = inputObject->getType() | 0x02000000;
    }
    else if (method == "Median")
    {
        meth = inputObject->getType() | 0x04000000;
    }
    else if (method == "Mean")
    {
        meth = inputObject->getType() | 0x08000000;
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "unsupported method.");
    }

    if (!retval.containsError())
    {
        int numPlanes = inputObject->getNumPlanes();

        //create a copy of the input object to output object
        ito::DataObject outputObject;
        inputObject->copyTo(outputObject, 1);
        inputObject->copyAxisTagsTo(outputObject);
        inputObject->copyTagMapTo(outputObject);

        if (regionType == "ErodeRect")
        {
            erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1 + 2 * extend[0], 1 + 2 * extend[1]));
        }
        else if (regionType == "ErodeEllipse")
        {
            erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1 + 2 * extend[0], 1 + 2 * extend[1]));
        }

        if (numPlanes > 1 && statsObj)
        {
            retval += ito::RetVal(ito::retWarning, 0, "The inputObject contains more than one plane. The statistic can then not be returned as 'statistic' dataObject.");
            statsObj = NULL;
        }

        for (int planeIdx = 0; planeIdx < numPlanes; ++planeIdx)
        {
            
            input = inputObject->getCvPlaneMat(0);
            output = outputObject.getCvPlaneMat(0);
            
            // operation "nanMask = cv::Mat(*input != *input);" is buggy
            // https://stackoverflow.com/questions/41759247/filter-opencv-mat-for-nan-values 

            switch (input->type())
            {
            case CV_32F:
                nanMask = getNaNMask<float>(input);
                break;
            case CV_64F:
                nanMask = getNaNMask<double>(input);
                break;
            }

            //label all NaN areas
            int numAreas = cv::connectedComponentsWithStats(nanMask, labels, stats, centroids, 4, CV_32S);
            numTotalAreas += (numAreas - 1); //the first area is the background


            if (statsObj && numAreas > 1)
            {
                statisticObject.zeros(numAreas - 1, 8, ito::tFloat32);
            }

            for (int areaIdx = 1; areaIdx < numAreas; ++areaIdx)
            {
                statsRow = stats.ptr<ito::int32>(areaIdx);

                if (statsObj)
                {
                    statisticObjectRowPtr = statisticObject.rowPtr<ito::float32>(0, areaIdx - 1);
                    statisticObjectRowPtr[0] = statsRow[cv::CC_STAT_LEFT];
                    statisticObjectRowPtr[1] = statsRow[cv::CC_STAT_TOP];
                    statisticObjectRowPtr[2] = statsRow[cv::CC_STAT_WIDTH];
                    statisticObjectRowPtr[3] = statsRow[cv::CC_STAT_HEIGHT];
                    statisticObjectRowPtr[4] = statsRow[cv::CC_STAT_AREA];
                    statisticObjectRowPtr[5] = inputObject->getPixToPhys(dims - 1, centroids.at<double>(areaIdx, 0));
                    statisticObjectRowPtr[6] = inputObject->getPixToPhys(dims - 2, centroids.at<double>(areaIdx, 1));
                    statisticObjectRowPtr[7] = statsRow[cv::CC_STAT_AREA] <= maxAreaSize;
                }
                
                if (statsRow[cv::CC_STAT_AREA] <= maxAreaSize)
                {
                    x = qBound(0, statsRow[cv::CC_STAT_LEFT] - extend[0], input->cols - 1);
                    y = qBound(0, statsRow[cv::CC_STAT_TOP] - extend[1], input->rows - 1);
                    x2 = qBound(0, statsRow[cv::CC_STAT_LEFT] + statsRow[cv::CC_STAT_WIDTH] + extend[0], input->cols); //excluded
                    y2 = qBound(0, statsRow[cv::CC_STAT_TOP] + statsRow[cv::CC_STAT_HEIGHT] + extend[1], input->rows);//excluded
                    w = x2 - x;
                    h = y2 - y;

                    if (w > 0 && h > 0)
                    {
                        outputPatch = (*output)(cv::Rect(x, y, w, h));
                        labelPatch = labels(cv::Rect(x, y, w, h));

                        if (regionType == "BoundingBox")
                        {
                            inputPatch = (*input)(cv::Rect(x, y, w, h));
                        }
                        else //eroding
                        {
                            //the inputPatch is manipulated (therefore a copy is necessary), such
                            //that all values that are in the boundingBox but outside of a ribbon generated by eroding the
                            //invalid area are also set to invalid. Then, all the outer values are not considered for calculating
                            //the interpolated plane, mean or median value.
                            cv::Mat labelPatchMorph = (labelPatch != areaIdx); //0 is within invalid area, 1 is outside
                            cv::Mat labelPatchMorphEroded;
                            cv::erode(labelPatchMorph, labelPatchMorphEroded, erodeElement);
                            (*input)(cv::Rect(x, y, w, h)).copyTo(inputPatch);
                            if (inputObject->getType() == ito::tFloat32)
                            {
                                inputPatch.setTo(std::numeric_limits<ito::float32>::quiet_NaN(), labelPatchMorphEroded);
                            }
                            else
                            {
                                inputPatch.setTo(std::numeric_limits<ito::float64>::quiet_NaN(), labelPatchMorphEroded);
                            }
                        }

                        outputPatch = (*output)(cv::Rect(x, y, w, h));
                        labelPatch = labels(cv::Rect(x, y, w, h));

                        switch (meth)
                        {
                        case ito::tFloat32 | 0x01000000: //Least Squares Plane with float32 input
                            retval += lsqFitPlane<ito::float32>(&inputPatch, A, B, C);

                            if (ito::isFinite(A))
                            {
                                for (int r = 0; r < inputPatch.rows; ++r)
                                {
                                    floatPtr = outputPatch.ptr<ito::float32>(r);
                                    labelPtr = labelPatch.ptr<ito::int32>(r);
                                    for (int c = 0; c < inputPatch.cols; ++c)
                                    {
                                        if (labelPtr[c] == areaIdx)
                                        {
                                            floatPtr[c] = A + B * c + C * r;
                                        }
                                    }
                                }
                                numTotalFilledAreas++;
                            }
                            break;
                        case ito::tFloat64 | 0x01000000: //Least Squares Plane with float64 input
                            retval += lsqFitPlane<ito::float64>(&inputPatch, A, B, C);

                            if (ito::isFinite(A))
                            {
                                for (int r = 0; r < inputPatch.rows; ++r)
                                {
                                    doublePtr = outputPatch.ptr<ito::float64>(r);
                                    labelPtr = labelPatch.ptr<ito::int32>(r);
                                    for (int c = 0; c < inputPatch.cols; ++c)
                                    {
                                        if (labelPtr[c] == areaIdx)
                                        {
                                            doublePtr[c] = A + B * c + C * r;
                                        }
                                    }
                                }
                                numTotalFilledAreas++;
                            }
                            break;
                        case ito::tFloat32 | 0x02000000: //Least Median Plane with float32 input
                            retval += lmedsFitPlane<ito::float32>(&inputPatch, A, B, C, valid_probability, alarm_rate);

                            if (ito::isFinite(A))
                            {
                                for (int r = 0; r < inputPatch.rows; ++r)
                                {
                                    floatPtr = outputPatch.ptr<ito::float32>(r);
                                    labelPtr = labelPatch.ptr<ito::int32>(r);
                                    for (int c = 0; c < inputPatch.cols; ++c)
                                    {
                                        if (labelPtr[c] == areaIdx)
                                        {
                                            floatPtr[c] = A + B * c + C * r;
                                        }
                                    }
                                }
                                numTotalFilledAreas++;
                            }
                            break;
                        case ito::tFloat64 | 0x02000000: //Least Median Plane with float64 input
                            retval += lmedsFitPlane<ito::float64>(&inputPatch, A, B, C, valid_probability, alarm_rate);

                            if (ito::isFinite(A))
                            {
                                for (int r = 0; r < inputPatch.rows; ++r)
                                {
                                    doublePtr = outputPatch.ptr<ito::float64>(r);
                                    labelPtr = labelPatch.ptr<ito::int32>(r);
                                    for (int c = 0; c < inputPatch.cols; ++c)
                                    {
                                        if (labelPtr[c] == areaIdx)
                                        {
                                            doublePtr[c] = A + B * c + C * r;
                                        }
                                    }
                                }
                                numTotalFilledAreas++;
                            }
                            break;
                        case ito::tFloat32 | 0x04000000: //Median with float32 input
                        {
                            //calculate median
                            floatVec.clear();
                            floatVec.reserve(inputPatch.rows * inputPatch.cols);
                            for (int r = 0; r < inputPatch.rows; ++r)
                            {
                                floatPtr = inputPatch.ptr<ito::float32>(r);
                                for (int c = 0; c < inputPatch.cols; ++c)
                                {
                                    if (ito::isFinite(floatPtr[c]))
                                    {
                                        floatVec.push_back(floatPtr[c]);
                                    }
                                }
                            }
                                
                            scalar = nanmedian<ito::float32>(floatVec.data(), floatVec.size());
                            if (ito::isFinite(scalar[0]))
                            {
                                outputPatch.setTo(scalar, labelPatch == areaIdx);
                                numTotalFilledAreas++;
                            }
                        }
                        break;
                        case ito::tFloat64 | 0x04000000: //Median Plane with float64 input
                        {
                            //calculate median
                            doubleVec.clear();
                            doubleVec.reserve(inputPatch.rows * inputPatch.cols);
                            for (int r = 0; r < inputPatch.rows; ++r)
                            {
                                doublePtr = inputPatch.ptr<ito::float64>(r);
                                for (int c = 0; c < inputPatch.cols; ++c)
                                {
                                    if (ito::isFinite(doublePtr[c]))
                                    {
                                        doubleVec.push_back(doublePtr[c]);
                                    }
                                }
                            }

                            scalar = nanmedian<ito::float64>(doubleVec.data(), doubleVec.size());
                            if (ito::isFinite(scalar[0]))
                            {
                                outputPatch.setTo(scalar, labelPatch == areaIdx);
                                numTotalFilledAreas++;
                            }
                        }
                        break;
                        case ito::tFloat32 | 0x08000000: //Mean with float32 input
                            inputPatchValidMask = (inputPatch == inputPatch);
                            scalar = cv::mean(inputPatch, inputPatchValidMask);
                            if (ito::isFinite(scalar[0]))
                            {
                                outputPatch.setTo(scalar, labelPatch == areaIdx);
                                numTotalFilledAreas++;
                            }
                            break;
                        case ito::tFloat64 | 0x08000000: //Mean with float64 input
                            inputPatchValidMask = (inputPatch == inputPatch);
                            scalar = cv::mean(inputPatch, inputPatchValidMask);
                            if (ito::isFinite(scalar[0]))
                            {
                                outputPatch.setTo(scalar, labelPatch == areaIdx);
                                numTotalFilledAreas++;
                            }
                            break;
                        }
                    }
                    else
                    {
                        retval += ito::RetVal(ito::retError, 0, "unsupported regionType. Only 'BoundingBox' is currently implemented.");
                        break;
                    }

                    
                }
            }

            if (statsObj)
            {
                *((*paramsOpt)[6].getVal<ito::DataObject*>()) = statisticObject;
            }

			QString msg;
			if (method == "LMedSPlane")
			{
				msg = tr("%1 of %2 found invalid areas filled with method = %3, maxAreaSize = %4, regionType = %5, regionExtend = %6, validPointProbability = %7, allowedErrorProbability = %8").arg(numTotalFilledAreas).arg(numTotalAreas).arg(QString(method)).arg(maxAreaSize).arg(QString(regionType)).arg(*extend).arg(valid_probability).arg(alarm_rate);
				outputObject.addToProtocol(std::string(msg.toLatin1().data()));
			}
			else
			{
				msg = tr("%1 of %2 found invalid areas filled with method = %3, maxAreaSize = %4, regionType = %5, regionExtend = %6").arg(numTotalFilledAreas).arg(numTotalAreas).arg(QString(method)).arg(maxAreaSize).arg(QString(regionType)).arg(*extend);
				outputObject.addToProtocol(std::string(msg.toLatin1().data()));
			}
        }

        *((*paramsMand)[1].getVal<ito::DataObject*>()) = outputObject;
    }

    (*paramsOut)[0].setVal<int>(numTotalAreas);
    (*paramsOut)[1].setVal<int>(numTotalFilledAreas);

	

    return retval;
}
#endif


//----------------------------------------------------------------------------------------------------------------------------------
//----- HELPER METHODS ------------------------------

//! brief description
/*!
    for each point (x,y,z) where (x,y) are lying on the image grid, a regression plane is calculated
    with the result vector (A,B,C) such that

    z = A + B*x + C*y

    \param name description
    \return description
    \sa (see also) keywords (comma-separated)
*/
RetVal FittingFilters::fitLeastSquarePlaneSVD(const cv::Mat *inputMatrix, double &A, double &B, double &C)
{
    int nrOfElements = inputMatrix->cols * inputMatrix->rows;
    cv::Mat mat1 = cv::Mat(3, nrOfElements, CV_64FC1 );
    cv::Mat mat2 = cv::Mat(1, nrOfElements, CV_64FC1 );
    cv::Mat result = cv::Mat(3, 1, CV_32FC1 );
    cv::Mat input;

    if (inputMatrix->type() != CV_64FC1)
    {
        inputMatrix->convertTo(input, CV_64FC1);
    }
    else
    {
        input = *inputMatrix;
    }

    double *zValue_row = mat2.ptr<double>(0);
    double *oneValue_row = mat1.ptr<double>(0);
    double *xValue_row = mat1.ptr<double>(1);
    double *yValue_row = mat1.ptr<double>(2);

    int counter = 0;
    const double *row;
    int i,j;

    for (i = 0; i<input.rows; i++)
    {
        row = input.ptr<double>(i);
        for (j = 0; j<input.cols; j++)
        {
            if (! cvIsNaN( row[j] ))
            {
                zValue_row[counter] = row[j];
                oneValue_row[counter] = 1;
                xValue_row[counter] = j;
                yValue_row[counter] = i;
                counter++;
            }
        }
    }

    mat1 = mat1( cv::Range(0,3), cv::Range(0,counter));
    mat2 = mat2( cv::Range(0,1), cv::Range(0,counter));

    cv::solve( mat1.t(), mat2.t(), result, cv::DECOMP_SVD );

    A = result.at<double>(0,0);
    B = result.at<double>(1,0);
    C = result.at<double>(2,0);

    return retOk;
}

//---------------------------------------------------------------------------------------------------------------
//this code is copied from 'm': param3d.cpp, line 168, method 'LeastSquaresPlaneWindow'
/*
The basics of this function is the plane equation

z = A + B*x + C*y, where A,B,C must be determined

For each pixel i, the error is
e_i = |z_i - A - B*x_i - C*y_i|

The total error is then

e = sum_{i=0}^{n-1} (e_i) for n valid pixels

This error should be minimized, hence 
de/dA != 0
de/dB != 0
de/dC != 0 (!= means "should be equal than")

This leads to:
sz = sum_{i=0}^{n-1}z_i for all valid pixels
sxy = sum_{i=0}^{n-1}x_i*y_i for all valid pixels
...

And finally

n*A - sz + sx*B + sy*C != 0
sx*A - sxz + sxx*B + sxy*C != 0
sy*A - syz + sxy*B + syy*C != 0

Solve this system of linear equations and obtain A,B,C!
*/
template<typename _Tp> ito::RetVal FittingFilters::lsqFitPlane(const cv::Mat *mat, double &A, double &B, double &C)
{
    ito::RetVal retVal(ito::retOk);
    int   i, j;
    long   n = 0;
    double sx, sy, sz, sxz, syz, sxx, syy, sxy;
    const _Tp *row;
    double temp;

    if (mat->channels() > 1)
    {
        return ito::RetVal(ito::retError, 0, "only integer and floating point data types are allowed for lsqFitPlane");
    }

    sx=sy=sz=sxx=syy=sxz=syz=sxy=0.0;

    if (std::numeric_limits<_Tp>::is_exact)
    {
        for (i = 0; i < mat->rows; ++i)
        {
            row = mat->ptr<_Tp>(i);

            for (j = 0; j < mat->cols; j++)
            {
                temp = (double)(row[j]);
                sxz += j*temp;
                syz += i*temp;
                sz  += temp;
                sxx += j*j;
                syy += i*i;
                sx  += j;
                sy  += i;
                sxy += i*j;
            }
        }

        n = mat->rows * mat->cols;
    }
    else
    {
        for (i = 0; i < mat->rows; ++i)
        {
            row = mat->ptr<_Tp>(i);

            for (j = 0; j < mat->cols; j++)
            {
                temp = (double)(row[j]);
                if (!cvIsNaN(temp))
                {
                   sxz += j*temp;
                   syz += i*temp;
                   sz  += temp;
                   sxx += j*j;
                   syy += i*i;
                   sx  += j;
                   sy  += i;
                   sxy += i*j;
                   n++;
                }
            }
        }
    }

    double denom = 0.0;

    if (mat->cols != 1 && mat->rows != 1)
    {
        denom = syy*sx*sx - 2*sx*sxy*sy + n*sxy*sxy + sxx*sy*sy - n*sxx*syy;
        
    }
    else if (mat->cols == 1 && mat->rows != 1)
    {
        syy = 1.0;
        denom = sx*sx - n*sxx;
        retVal += ito::RetVal(ito::retWarning, 0, "Please use polynomical fit or 1D-Linefitting for 1xN objects.");
    }
    else if (mat->rows == 1 && mat->cols != 1) 
    {
        sxx = 1.0;
        denom = sy*sy - n*syy;
        retVal += ito::RetVal(ito::retWarning, 0, "Please use polynomical fit or 1D-Linefitting for Mx1 objects.");
    }

    if (std::abs(denom) < std::numeric_limits<double>::epsilon())
    {
        A = std::numeric_limits<double>::quiet_NaN();
        B = A;
        C = A;
    }
    else
    {
        if (mat->dims == 2 && mat->cols != 1)
        {
            B = (sxz*sy*sy + n*sxy*syz - n*syy*sxz - sx*syz*sy - sy*sxy*sz + syy*sx*sz) / denom;
        }
        else
        {
            B = 0.0;
        }

        if (mat->dims == 2 && mat->rows != 1)
        {
            C = (syz*sx*sx + n*sxy*sxz - n*sxx*syz - sx*sxz*sy - sx*sxy*sz + sxx*sy*sz) / denom;
        }
        else
        {
            C = 0.0;
        }

        A = (sz - B*sx - C*sy) / n;
    }

    return retVal;
}

//---------------------------------------------------------------------------------------------------------------
//The probability values valid_probability and alarm_rate are used to determine the number of iterations.
//They are defined as follows (see Marco Zuliani, RANSAC for Dummies, vision.ece.ucsb.edu/~zuliani):
//
// For a ransac plane fit, many randomly chosen minimum sample sets MSS (here with 3 points each) needs to
// be selected from the entire data set. valid_probability is the probability that one of those sets contains
// no outlier points. The probability to have an invalid MSS is then (1 - valid_probability).
// If we have iter iterations, the probability to only have MSSs with at least one outlier each, tends to zero and is
//                                   (1 - valid_probability)^iter
// The alarm_rate indicates the maximum probability level to finally obtain a wrong result. Hence,
//                                  alarm_rate >= (1 - valid_probability)^iter
// The number of iterations iter is then
//                     iter >= ceil(log(alarm_rate) / (log(1- valid_probability))
//
// this version of the least median fits randomly selects three points from the plane, determines the plane out of
// these three points and determines the distance from every point to the plane. Finally the plane is chosen
// whose median of all distances is minimum.
template<typename _Tp> ito::RetVal FittingFilters::lmedsFitPlane(const cv::Mat *mat, double &A, double &B, double &C, const double &valid_probability, const double &alarm_rate)
{
    if (mat->channels() > 1)
    {
        return ito::RetVal(ito::retError, 0, "only integer and floating point data types are allowed for lsqFitPlane");
    }
    if (mat->rows * mat->cols < 4)
    {
        return ito::RetVal(ito::retError, 0, "a plane fit can only be done with at least 3 points");
    }

    ito::RetVal retval;
    int iter = std::ceil( std::log(alarm_rate) / std::log(1. - valid_probability) );
    iter = std::max(3, iter);
    double minimum_distance = std::numeric_limits<double>::max();
    cv::Vec3d best_normal;
    double best_distance;

    cv::RNG random((uint64)cv::getTickCount() ^ 0xa8e5f936);

    int rows[3];
    int cols[3];
    cv::Vec3d p1, p2, p3; //three points of plane
    cv::Vec3d normal;
    double d;
    std::vector<double> distanceBucket;
    distanceBucket.resize(mat->rows * mat->cols);
    int bucketIdx = 0;
    _Tp *rowPtr;
    double medianDistance;

    for (int i = 0; i < iter; ++i)
    {
        retval += getRandomValidMinimalSampleSet<_Tp>(mat, random, 3, rows, cols);

        if (retval.containsError())
        {
            break;
        }

        //calculate plane from three randomly chosen values (Hesse: normal * point = d)
        p1[0] = cols[0]; p1[1] = rows[0]; p1[2] = mat->at<_Tp>(rows[0],cols[0]);
        p2[0] = cols[1]; p2[1] = rows[1]; p2[2] = mat->at<_Tp>(rows[1],cols[1]);
        p3[0] = cols[2]; p3[1] = rows[2]; p3[2] = mat->at<_Tp>(rows[2],cols[2]);
        normal = (p2-p1).cross(p3-p1);
        normal = normal / cv::norm(normal);
        if (normal[2] < 0) normal *= -1;
        d = p1.dot(normal);
        //

        //determine all distances to the plane and save it in distanceBucket
        bucketIdx = 0;

        for (int r = 0; r < mat->rows; ++r)
        {
            rowPtr = (_Tp*)(mat->ptr(r));
            p1[1] = (double)r;

            if (std::numeric_limits<_Tp>::is_exact)
            {
                for (int c = 0; c < mat->cols; ++c)
                {
                    p1[0] = (double)c;
                    p1[2] = (double)rowPtr[c];
                    distanceBucket[bucketIdx++] = std::abs((normal.dot(p1)) - d);
                }
            }
            else
            {
                for (int c = 0; c < mat->cols; ++c)
                {
                    if (!cvIsNaN((double)rowPtr[c]))
                    {
                        p1[0] = (double)c;
                        p1[2] = (double)rowPtr[c];
                        distanceBucket[bucketIdx++] = std::abs((normal.dot(p1)) - d);
                    }
                }
            }
        }

        std::sort(distanceBucket.begin(), distanceBucket.begin() + bucketIdx);
        medianDistance = distanceBucket[bucketIdx / 2];

        if (medianDistance < minimum_distance)
        {
            minimum_distance = medianDistance;
            best_normal = normal;
            best_distance = d;
        }
    }

    if (!retval.containsError())
    {
        if (std::numeric_limits<double>::max() - minimum_distance < 1.0)
        {
            return ito::RetVal(ito::retError, 0, "plane fit failed.");
        }

        A = best_distance / best_normal[2];
        B = -best_normal[0] / best_normal[2];
        C = -best_normal[1] / best_normal[2];
    }

    return ito::retOk;
}

//---------------------------------------------------------------------------------------------------------------
//returns numSamples pairs of unique (rows,cols) within mat (only valid values) that are randomly chosen.
//If the algorithm does not find the required number of unique and valid points after maxIter, an error is returned.
//rows and cols must be allocated with at least numSamples values. rng must be initialized with seed value.
template<typename _Tp> ito::RetVal FittingFilters::getRandomValidMinimalSampleSet(const cv::Mat *mat, cv::RNG &rng, int numSamples, int *rows, int *cols, int maxIter /*= 1000*/)
{
    int nrOfValues = mat->rows * mat->cols;
    int iter = 0;
    int nd = 0; //next digit
    int uni;

    //first use rows to save continuous index. this is finally split into rows and cols.

    if (std::numeric_limits<_Tp>::is_exact)
    {
        while (nd < numSamples && iter < maxIter)
        {
            uni = rows[nd++] = rng.uniform(0, nrOfValues);

            //check that the new index is not already available in previous indexes
            for (int i = 0; i < (nd - 1); ++i)
            {
                if (uni == rows[i]) //duplicate
                {
                    nd--;
                    break;
                }
            }

            iter++;
        }
    }
    else
    {
        int r,c;

        while (nd < numSamples && iter < maxIter)
        {
            uni = rng.uniform(0, nrOfValues);
            c = uni % mat->cols;
            r = (uni - c) / mat->cols;
            
            if (!cvIsNaN((double)mat->at<_Tp>(r,c)))
            {
                rows[nd++] = uni;

                //check that the new index is not already available in previous indexes
                for (int i = 0; i < (nd - 1); ++i)
                {
                    if (uni == rows[i]) //duplicate
                    {
                        nd--;
                        break;
                    }
                }
            }

            iter++;
        }
    }

    if (nd == numSamples)
    {
        //get rows,cols from continuous index
        for (int i = 0; i < numSamples; ++i)
        {
            cols[i] = rows[i] % mat->cols;
            rows[i] = (rows[i] - cols[i]) / mat->cols;
        }

        return ito::retOk;
    }

    return ito::RetVal(ito::retError, 0, "no valid minimal sample set could be found");
}

//---------------------------------------------------------------------------------------------------------------
template<typename _Tp> RetVal FittingFilters::subtractPlaneTemplate(cv::Mat *inputMatrix, cv::Mat *destMatrix, double A, double B, double C)
{
    CV_DbgAssert( inputMatrix->dims == destMatrix->dims && inputMatrix->size == destMatrix->size && inputMatrix->type() == destMatrix->type() );
    _Tp *row;
    _Tp *rowDest;
    int i,j;

    //create lookups
    _Tp *Bx = new _Tp[inputMatrix->cols];
    _Tp *Cy = new _Tp[inputMatrix->rows];
    _Tp Anew = static_cast<_Tp>(A);

    for(j=0; j<inputMatrix->cols; j++)
    {
        Bx[j] = static_cast<_Tp>(B*j);
    }

    for (i=0; i<inputMatrix->rows; i++)
    {
        Cy[i] = static_cast<_Tp>(C*i);

        row = inputMatrix->ptr<_Tp>(i);
        rowDest = destMatrix->ptr<_Tp>(i);

        for (j=0; j<inputMatrix->cols; j++)
        {
            if (!cvIsNaN((double)row[j]))
            {
                rowDest[j] = row[j] - Anew - Bx[j] - Cy[i];
            }
            else
            {
                rowDest[j] = std::numeric_limits<_Tp>::quiet_NaN();
            }
        }
    }

    delete[] Bx;
    delete[] Cy;

    return retOk;
}

//---------------------------------------------------------------------------------------------------------------
/*static*/ void FittingFilters::linearRegression(VecDoub_I &x, VecDoub_I &y, VecDoub_I &w, VecDoub_O &p, Doub& residual)
{
    /*
    This method is motivated by see polynomial regression section (motivational example of wikipedia: http://en.wikipedia.org/wiki/Linear_least_squares_%28mathematics%29)

    Given the line equation y = p0 + p1 * x
    as well as n tuples (x_i, y_i)

    Then we have n-equations y_i = p0 + p1 * x_i

    The sum-of-squares of the error is then
    S(p0,p1) = \sum_{i=1}^{n} (y_i - p_0 - x_i*p_1)^2

    This equation must be minimal, hence
    dS/dp0 -> 0 and dS/dp1 -> 0

    finally this leads to
    
    1. -sum(y_i) + n * p0 + sum(x_i) * p1 -> 0

    2. -sum(x_i * y_i) + sum(x_i) * p0 + sum(x_i^2) * p1 -> 0

    If weights come into this "game" ;), every pair is weighted such that the error
    is

    w_i*y_i = w_i*p0 + w_i * x_i * p1

    This leads to the final equations

    1. -sum(w_i^2*y_i) + sum(w_i^2) * p0 + sum(w_i^2*x_i) * p1 -> 0

    2. -sum(x_i * y_i * w_i^2) + sum(x_i * w_i^2) * p0 + sum(x_i^2 * w_i^2) * p1 -> 0
    */

    Int ndat = x.size();
    Int n = 0;
    Int i;
    Doub w2;
    Doub sw2y = 0; //sum(w_i^2*y_i)
    Doub sw2 = 0;  //sum(w_i^2)
    Doub sw2x = 0; //sum(w_i^2*x_i)
    Doub sw2xy = 0;//sum(x_i * y_i * w_i^2)
    Doub sw2x2 = 0;//sum(x_i^2 * w_i^2)
    p.resize(2);

    for (i=0;i<ndat;i++)
    {
        if (qIsFinite(y[i]) && qIsFinite(x[i]) && w[i] > 0)
        {
            w2 = w[i] * w[i];
            sw2 += w2;
            sw2y += w2*y[i];
            sw2x += w2*x[i];
            sw2xy += w2*x[i]*y[i];
            sw2x2 += w2*x[i]*x[i];
            n++;
        }
    }

    if (n<2 || std::abs(sw2) < std::numeric_limits<double>::epsilon())
    {
        p[0] = std::numeric_limits<double>::quiet_NaN();
        p[1] = std::numeric_limits<double>::quiet_NaN();
        residual = std::numeric_limits<double>::quiet_NaN();
    }
    else
    {
        //direct solution for
        // [a b ; c d] * [p0 ; p1] = [e ; f] with [sw2 sw2x ; sw2x sw2x2] * [p0 ; p1] = [sw2y ; sw2xy]
        // is p1 = (ce-af)/(bc-ad)
        // and p0 = e-b*p1/a

        Doub denominator = sw2x * sw2x - sw2 * sw2x2;

        if (std::abs(denominator) >= std::numeric_limits<double>::epsilon())
        {
            p[1] = (sw2x * sw2y - sw2 * sw2xy) / denominator;
            p[0] = (sw2y - sw2x * p[1]) / sw2;

            residual = 0.0;
            Doub t;

            for (i=0;i<ndat;i++)
            {
                if (qIsFinite(y[i]) && qIsFinite(x[i]))
                {
                    t = (y[i]-p[0]-p[1]*x[i]);
                    residual += (t*t);
                }
            }
        }
        else
        {
            p[0] = std::numeric_limits<double>::quiet_NaN();
            p[1] = std::numeric_limits<double>::quiet_NaN();
            residual = std::numeric_limits<double>::quiet_NaN();
        }

        /*if (std::abs(sw2) <
        MatDoub aa(2,2);
        VecDoub b(2);
        aa[0][0] = sw2;
        aa[0][1] = sw2x;
        aa[1][0] = sw2x;
        aa[1][1] = sw2x2;
        b[0] = sw2y;
        b[1] = sw2xy;*/
        /*SVD svd(aa);
        svd.solve(b,p,-1.);*/
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> /*static*/ void FittingFilters::polyval1d_subtract_basic(_Tp *output, size_t length, const VecDoub &coeffs, size_t output_step /*= 1*/)
{
    _Tp *o_ = output;
    int degree = coeffs.size() - 1;

    switch (degree)
    {
    case 0:
        for (size_t c = 0;  c < length; ++c)
        {
            *o_ -= cv::saturate_cast<_Tp>(coeffs[0]);
            o_ += output_step;
        }
        break;
    case 1:
        for (size_t c = 0;  c < length; ++c)
        {
            *o_ -= cv::saturate_cast<_Tp>((coeffs[1] * c) + coeffs[0]);
            o_ += output_step;
        }
        break;
    case 2:
        for (size_t c = 0;  c < length; ++c)
        {
            *o_ -= cv::saturate_cast<_Tp>((coeffs[2] * c * c) +(coeffs[1] * c) + coeffs[0]);
            o_ += output_step;
        }
        break;
    case 3:
        for (size_t c = 0;  c < length; ++c)
        {
            *o_ -= cv::saturate_cast<_Tp>((coeffs[3] * c * c * c) + (coeffs[2] * c * c) + (coeffs[1] * c) + coeffs[0]);
            o_ += output_step;
        }
        break;
    default:
        {
            ito::float64 v_temp = 1;
            for (size_t c = 0;  c < length; ++c)
            {
                *o_ -= cv::saturate_cast<_Tp>(coeffs[0]);
                v_temp = 1;
                for (int d = 1; d <= degree; ++d)
                {
                    v_temp *= c;
                    *o_ -= cv::saturate_cast<_Tp>(v_temp * coeffs[d]);
                }
                o_ += output_step;
            }
        }
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> /*static*/ void FittingFilters::polyfit1d_basic(const _Tp *input, size_t length, VecDoub &indices_storage, VecDoub &value_storage, VecDoub &weight_storage, FitSVDSimple *fit_func, VecDoub &coeffs, size_t input_step /*= 1*/)
{
    const _Tp *input_ = input;
    Doub chisq;

    if (std::numeric_limits<_Tp>::is_exact)
    {
        for (size_t i = 0; i < length; ++i)
        {
            value_storage[i] = cv::saturate_cast<double>(*input_);
            input_ += input_step;
        }

        if (fit_func)
        {
            fit_func->fit( indices_storage, value_storage, weight_storage, coeffs, chisq );
        }
        else
        {
            linearRegression( indices_storage, value_storage, weight_storage, coeffs, chisq );
        }
    }
    else
    {
        for (size_t i = 0; i < length; ++i)
        {
            if (ito::isFinite(*input_))
            {
                value_storage[i] = cv::saturate_cast<double>(*input_);
            }
            else
            {
                value_storage[i] = std::numeric_limits<double>::quiet_NaN();
            }
            input_ += input_step;
        }

        if (fit_func)
        {
            fit_func->fit( indices_storage, value_storage, weight_storage, coeffs, chisq );
        }
        else
        {
            linearRegression( indices_storage, value_storage, weight_storage, coeffs, chisq );
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
RetVal FittingFilters::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(FittingFilters::fitPlane, FittingFilters::fitPlaneParams, fitPlaneDoc);
    m_filterList.insert("fitPlane", filter);

    filter = new FilterDef(FittingFilters::subtractPlane, FittingFilters::subtractPlaneParams, subtractPlaneDoc);
    m_filterList.insert("subtractPlane", filter);

    filter = new FilterDef(FittingFilters::subtractRegressionPlane, FittingFilters::subtractRegressionPlaneParams, subtractRegressionPlaneDoc);
    m_filterList.insert("subtractRegressionPlane", filter);

//    TODO: undefined reference see polyfit2d.cpp
    filter = new FilterDef(FittingFilters::fitPolynom2D, FittingFilters::fitPolynom2DParams, fitPolynom2DDoc); 
    m_filterList.insert("fitPolynom2D", filter);

    filter = new FilterDef(FittingFilters::polyfitWeighted2D, FittingFilters::polyfitWeighted2DParams, polyfitWeighted2DDoc);
    m_filterList.insert("polyfitWeighted2D", filter);

    filter = new FilterDef(FittingFilters::polyfitWeighted2DSinglePoints, FittingFilters::polyfitWeighted2DSinglePointsParams, polyfitWeighted2DSinglePointsDoc);
    m_filterList.insert("polyfitWeighted2DSinglePoints", filter);

    filter = new FilterDef(FittingFilters::polyval2D, FittingFilters::polyval2DParams, polyval2DDoc);
    m_filterList.insert("polyval2D", filter);

    filter = new FilterDef(FittingFilters::polyval2DSinglePoints, FittingFilters::polyval2DSinglePointsParams, polyval2DSinglePointsDoc);
    m_filterList.insert("polyval2DSinglePoints", filter);

    filter = new FilterDef(FittingFilters::fitPolynom1D_Z, FittingFilters::fitPolynom1D_ZParams, fitPolynom1D_ZDoc);
    m_filterList.insert("fitPolynom1D_Z", filter);

    filter = new FilterDef(FittingFilters::getInterpolatedValues, FittingFilters::getInterpolatedValuesParams, getInterpolatedValuesDoc);
    m_filterList.insert("getInterpolatedValues", filter);

    filter = new FilterDef(FittingFilters::subtract1DRegressionPolynom, FittingFilters::subtract1DRegressionPolynomParams, subtract1DRegressionPolynomDoc);
    m_filterList.insert("subtract1DRegression", filter);

#if (CV_MAJOR_VERSION >= 3)
    filter = new FilterDef(FittingFilters::fillInvalidAreas, FittingFilters::fillInvalidAreasParams, fillInvalidAreasDoc);
    m_filterList.insert("fillInvalidAreas", filter);
#endif
    
    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
RetVal FittingFilters::close(ItomSharedSemaphore * /*waitCond*/)
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();

    ito::RetVal retval = ito::retOk;

    return retval;
}
