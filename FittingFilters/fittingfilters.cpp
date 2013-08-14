#include "fittingfilters.h"

#include "common/helperCommon.h"

#include <QtCore/QtPlugin>
#include <qstringlist.h>
#include <qvariant.h>

#include "pluginVersion.h"


using namespace ito;


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FittingFiltersInterface::getAddInInst(ito::AddInBase **addInInst)
{
    FittingFilters* newInst = new FittingFilters();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);
    QList<QString> keyList = newInst->m_filterList.keys();
    for (int i = 0; i < newInst->m_filterList.size(); i++)
    {
        newInst->m_filterList[keyList[i]]->m_pBasePlugin = this;
    }

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FittingFiltersInterface::closeThisInst(ito::AddInBase **addInInst)
{
       if (*addInInst)
    {
        FittingFilters * thisInst = qobject_cast<FittingFilters*>(*addInInst);
        if(thisInst)
        {
            delete thisInst;
            int idx = m_InstList.indexOf(*addInInst);
            m_InstList.removeAt(idx);
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("plugin-instance cannot be converted to class FittingFilters. Close operation failed").toAscii().data());
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
FittingFiltersInterface::FittingFiltersInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("FittingFilters");
    
    m_description = QObject::tr("Filter-Plugin for fitting-methods.");
    m_detaildescription = QObject::tr("Please fill in detailed description");
    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LPGL");
    m_aboutThis = QObject::tr("N.A.");      
    
}

//----------------------------------------------------------------------------------------------------------------------------------
FittingFiltersInterface::~FittingFiltersInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(FittingFiltersInterface, FittingFiltersInterface)

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
static char fitPlaneDoc[] = "fits plane in 2D-dataObject and returns plane-parameters A,B,C (z=A+Bx+Cy)";

RetVal FittingFilters::fitPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    paramsMand->append( Param("sourceImage", ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object").toAscii().data()) );

    paramsOpt->append( Param("method", ParamBase::String | ParamBase::In, "leastSquareFit", tr("fitting method (leastSquareFit [default], leastSquareFitSVD)").toAscii().data()) );

    *paramsOut << Param("A", ParamBase::Double | ParamBase::Out, 0.0, ito::DoubleMeta::all(), tr("Parameter A of regression plane z = A + Bx + Cy").toAscii().data());
    *paramsOut << Param("B", ParamBase::Double | ParamBase::Out, 0.0, ito::DoubleMeta::all(), tr("Parameter B of regression plane z = A + Bx + Cy").toAscii().data());
    *paramsOut << Param("C", ParamBase::Double | ParamBase::Out, 0.0, ito::DoubleMeta::all(), tr("Parameter C of regression plane z = A + Bx + Cy").toAscii().data());

    return retval;
}

RetVal FittingFilters::fitPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObjImages = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );
    char *temp = (*paramsOpt)[0].getVal<char*>();
    QString method = static_cast<char*>( temp ); //borrowed reference

    QStringList availableMethods = QStringList() << "leastSquareFit" << "leastSquareFitSVD";

    if (dObjImages->getDims() != 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image must be two-dimensional.").toAscii().data());
    }

    if(dObjImages->getType() == tComplex64 || dObjImages->getType() == tComplex128)
    {
        return ito::RetVal(retError, 0, tr("source matrix must be of type (u)int8, (u)int16, (u)int32, float32 or float64").toAscii().data());
    }

    if(!availableMethods.contains(method, Qt::CaseInsensitive))
    {
        return ito::RetVal(retError, 0, tr("the chosen method is unknown").toAscii().data());
    }

    int index = dObjImages->seekMat(0);
    cv::Mat *plane = reinterpret_cast<cv::Mat*>(dObjImages->get_mdata()[index]);


    if(QString::compare(method, "leastSquareFit", Qt::CaseInsensitive) == 0)
    {
        double A,B,C;
        retval += fitLeastSquarePlane(plane,A,B,C);
        (*paramsOut)[0].setVal<double>(A);
        (*paramsOut)[1].setVal<double>(B);
        (*paramsOut)[2].setVal<double>(C);
    }
    else if(QString::compare(method, "leastSquareFitSVD", Qt::CaseInsensitive) == 0)
    {
        double A,B,C;
        retval += fitLeastSquarePlaneSVD(plane,A,B,C);
        (*paramsOut)[0].setVal<double>(A);
        (*paramsOut)[1].setVal<double>(B);
        (*paramsOut)[2].setVal<double>(C);
    }

    return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------
static char subtractPlaneDoc[] = "subtracts plane from 2D-dataObject given by plane-parameters A,B,C (z=A+Bx+Cy) \n\
\n\
If the destinationImage is not the same than the sourceImage, the destinationImage finally is a new data object with the same \
size and type than the sourceImage and contains the data of the sourceImage subtracted by the given plane. If both are the same, \
the subtraction is executed in-place. \n\
\n\
If the input dataObject contains more than one plane, the subtraction is executed separately for each plane.";

RetVal FittingFilters::subtractPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    *paramsMand << Param("sourceImage", ParamBase::DObjPtr, NULL, tr("source image data object").toAscii().data());
    *paramsMand << Param("destinationImage", ParamBase::DObjPtr, NULL, tr("destination image data object").toAscii().data());
    *paramsMand << Param("A", ParamBase::Double, 0.0, ito::DoubleMeta::all(), tr("Parameter A of regression plane z = A + Bx + Cy, which is subtracted").toAscii().data());
    *paramsMand << Param("B", ParamBase::Double, 0.0, ito::DoubleMeta::all(), tr("Parameter B of regression plane z = A + Bx + Cy, which is subtracted").toAscii().data());
    *paramsMand << Param("C", ParamBase::Double, 0.0, ito::DoubleMeta::all(), tr("Parameter C of regression plane z = A + Bx + Cy, which is subtracted").toAscii().data());

    return retval;
}

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
        return ito::RetVal(ito::retError, 0, tr("Error: source image must be two-dimensional.").toAscii().data());
    }

    if(dObjInput->getType() == tComplex64 || dObjInput->getType() == tComplex128)
    {
        return ito::RetVal(retError, 0, tr("source matrix must be of type (u)int8, (u)int16, (u)int32, float32 or float64").toAscii().data());
    }

    if(dObjOutput == NULL)
    {
        return ito::RetVal(retError, 0, tr("destination matrix is NULL").toAscii().data());
    }

    ito::DataObject dObjDst;
    if (dObjInput == dObjOutput)
    {
        dObjDst = *dObjInput;
    }
    else
    {
        size_t *sizes = new size_t[ dObjInput->getDims() ];
        for (int i = 0; i < dObjInput->getDims(); ++i)
        {
            sizes[i] = dObjInput->getSize(i);
        }
        dObjDst = ito::DataObject(dObjInput->getDims(), sizes, dObjInput->getType());
        dObjInput->copyAxisTagsTo(dObjDst);
        dObjInput->copyTagMapTo(dObjDst);
        *dObjOutput = dObjDst;
    }

    size_t numPlanes = dObjDst.calcNumMats();
    cv::Mat *inpPlane = NULL;
    cv::Mat *outputPlane = NULL;

    for (size_t i = 0; i < numPlanes; ++i)
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
    dObjDst.addToProtocol(std::string(msg.toAscii().data()));

    return retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
static char subtractRegressionPlaneDoc[] = "subtracts a fitted regression plane from the given 2D input dataObject . \n\
\n\
This method firstly executes the filter *fitPlane* followed by *subtractPlane*.";

RetVal FittingFilters::subtractRegressionPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    *paramsMand << Param("sourceImage", ParamBase::DObjPtr, NULL, tr("source image data object").toAscii().data());
    *paramsMand << Param("destinationImage", ParamBase::DObjPtr, NULL, tr("destination image data object").toAscii().data());

    paramsOpt->append( Param("method", ParamBase::String | ParamBase::In, "leastSquareFit", tr("fitting method (leastSquareFit [default], leastSquareFitSVD)").toAscii().data()) );
    return retval;
}

RetVal FittingFilters::subtractRegressionPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    QVector<ito::ParamBase> paramsMandTemp;
    QVector<ito::ParamBase> paramsOptTemp;
    QVector<ito::ParamBase> paramsOutTemp;

    paramsMandTemp << (*paramsMand)[0]; //source object
    paramsOptTemp << (*paramsOpt)[0]; //method
    paramsOutTemp << ParamBase("A", ParamBase::Double, 0.0);
    paramsOutTemp << ParamBase("B", ParamBase::Double, 0.0);
    paramsOutTemp << ParamBase("C", ParamBase::Double, 0.0);


    retval += FittingFilters::fitPlane( &paramsMandTemp, &paramsOptTemp, &paramsOutTemp );

    if(retval == retOk)
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
static char polyfitWeighted2DDoc[] = "This method fits a two-dimensional polynomial of given order in x- and y-direction to the \
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
        f(x,y) = \\sum_{i=0}^orderX \\sum_{j=0}^{orderY-i} p_{ij} x^i y^j \n\
    else: \n\
        f(x,y) = \\sum_{j=0}^orderY \\sum_{i=0}^{orderX-i} p_{ij} x^i y^j \n\
\n\
The coefficients p_ij are stored in the coefficients vector in the order they appear in the equation above. \n\
\n\
The solver uses a Vandermonde matrix V as solving strategy and tries to solve V*p=Z, where Z are the valid values of the input data object. \
The overdetermined system of linear equations is finally solved using a QR factorization of V. If this module is compiled with LAPACK, its solvers \
are used, else the solve-command of OpenCV (slower) is called.";


/*static*/ ito::RetVal FittingFilters::polyfitWeighted2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    paramsMand->append( ito::Param("inputData", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input data object") );
    paramsMand->append( ito::Param("orderX", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in x-direction"));
    paramsMand->append( ito::Param("orderY", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in y-direction"));

    paramsOpt->append( ito::Param("weights", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "weights") );

    paramsOut->append( ito::Param("coefficients", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, "fitted polynom coefficients"));

    return retval;
}

/*static*/ ito::RetVal FittingFilters::polyfitWeighted2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject *input = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *weights = paramsOpt->at(0).getVal<ito::DataObject*>();

    int orderX = paramsMand->at(1).getVal<int>();
    int orderY = paramsMand->at(2).getVal<int>();

    std::vector<double> coefficients;

    retval += calcPolyfitWeighted2D(input, orderX, orderY, coefficients, 0.2, weights);

    (*paramsOut)[0].setVal<double*>( coefficients.data(),  coefficients.size() );

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
static char polyval2DDoc[] = "This method evaluates a two-dimensional polynom for every point in a given data object\n\
\n\
For every single pixel in the input data object 'dataZ', its physical coordinate (using scale and offset of the data object) \
is taken and the polynomial (given by its coefficients) is evaluated and stored in the pixel. \
The data object is hereby converted to float64. \n\
\n\
The polynomial coefficients (p0, p1, ...) are those returned by the filter 'fitPolynom2D' and depend on the polynomial order in X and Y \
direction: \n\
    if (orderX <= orderY): \n\
        f(x,y) = \\sum_{i=0}^orderX \\sum_{j=0}^{orderY-i} p_{ij} x^i y^j \n\
    else: \n\
        f(x,y) = \\sum_{j=0}^orderY \\sum_{i=0}^{orderX-i} p_{ij} x^i y^j \n\
\n\
The coefficients p_ij are stored in the coefficients vector in the order they appear in the equation above.";

/*static*/ ito::RetVal FittingFilters::polyval2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    paramsMand->append( ito::Param("dataZ", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "data object with given size and data type including scale and offset value. Depending on these values, this data object is finally filled with the evaluated polynomial function") );

    paramsMand->append( ito::Param("coefficients", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, "polynom coefficients as they come from polyfitWeighted2D"));

    paramsMand->append( ito::Param("orderX", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in x-direction"));
    paramsMand->append( ito::Param("orderY", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 2, "polynomial order in y-direction"));
    
    return retval;
}

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

    retval += calcPolyval2D(dataZ, paramsMand->at(2).getVal<int>(), paramsMand->at(3).getVal<int>(), coefficients);

    return retval;
}
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
RetVal FittingFilters::fitLeastSquarePlaneSVD(cv::Mat *inputMatrix, double &A, double &B, double &C)
{
    int nrOfElements = inputMatrix->cols * inputMatrix->rows;
    cv::Mat mat1 = cv::Mat(3, nrOfElements, CV_64FC1 );
    cv::Mat mat2 = cv::Mat(1, nrOfElements, CV_64FC1 );
    cv::Mat result = cv::Mat(3, 1, CV_32FC1 );
    cv::Mat input;

    if(inputMatrix->type() != CV_64FC1)
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


//this code is copied from 'm': param3d.cpp, line 168, method 'LeastSquaresPlaneWindow'
RetVal FittingFilters::fitLeastSquarePlane(cv::Mat *inputMatrix, double &A, double &B, double &C)
{
    cv::Mat input;
    int   i, j;
    long   n = 0;
    double sx, sy, sz, sxz, syz, sxx, syy;
    const double *row;
    double temp;

    if(inputMatrix->type() != CV_64FC1)
    {
        inputMatrix->convertTo(input, CV_64FC1);
    }
    else
    {
        input = *inputMatrix;
    }

    sx=sy=sz=sxx=syy=sxz=syz=0.0;
    for (i=0; i<input.rows; i++)
    {
        row = input.ptr<double>(i);

        for (j=0; j<input.cols; j++)
        {
            temp = row[j];
            if(!cvIsNaN(temp))
            {
               sxz += j*temp;
               syz += i*temp;
               sz  += temp;
               sxx += j*j;
               syy += i*i;
               sx  += j;
               sy  += i;
               n++;
            }
        }
    }

    if (input.dims == 2 && input.cols != 1 )
        B = (sxz-sx*sz/n)/(sxx-sx*sx/n);
    else
        B = 0;

    if (input.dims == 2 && input.rows != 1 )
        C=(syz-sy*sz/n)/(syy-sy*sy/n);
    else
        C=0;

    A = sz/n-B*sx/n-C*sy/n;

    return retOk;
}


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
            if(!cvIsNaN(row[j]))
            {
                rowDest[j] = row[j] - Anew - Bx[j] - Cy[i];
            }
            else
            {
                rowDest[j] = std::numeric_limits<_Tp>::signaling_NaN();
            }
        }
    }

    delete[] Bx;
    delete[] Cy;


    return retOk;
}

/*!
   \detail
   \param[in]   topoIn1     Image data of 1. channel
   \author ITO
   \sa
   \date
*/
template<typename _Tp> ito::RetVal FittingFilters::polyFit1DMatLab(double &offset, double &scale, cv::Mat *inputY, int order, QVector<QVariant> *outVals)
{
    double mu1, mu2;

    if(inputY->rows > 1 && inputY->cols > 1)
        return ito::RetVal(ito::retError, 0, L"Error in Polyfit 1D, matrix must be 1D-Line or Column");
    /*
    long totalPix = inputY->rows * inputY->cols;

    if(totalPix - 1 < order )
        return ito::RetVal(ito::retError, 0, L"Error in Polyfit 1D: less elements than polynomical order specified");

    mu1 = (((totalPix - 1 - offset) * scale) + (0 - offset * scale)) / 2;
    mu2 = 1 / sqrt(3.0) * (((totalPix - 1 - offset) * scale) - (0 - offset * scale)) / 2;

    cv::Mat_<double> x(1, totalPix);
    _Tp* xptr = x.ptr(0);

    cv::Mat_<double> y(1, totalPix);
    _Tp* yptr = y.ptr(0);

    for(int i; i < totalPix; i++)
    {
        xptr[i] = ((i - offset) * scale - mu1)/mu2;
        yptr[i] = (double)(inputY->at<_Tp>(i));
    }

    cv::Mat<double> V(totalPixel, order + 1);
    memset(V->ptr(0), 1, totalPixel* (order + 1)* sizeof(double));

    for(int j = n; j > 0; j--)
    {
        for(int i; i < totalPix; i++)
        {
            V->at<double>(i, j-1) = x[i] * V->at<double>(i, j);
        }
    }

*/
    return ito::retError;
}


//----------------------------------------------------------------------------------------------------------------------------------
RetVal FittingFilters::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(FittingFilters::fitPlane, FittingFilters::fitPlaneParams, tr(fitPlaneDoc));
    m_filterList.insert("fitPlane", filter);

    filter = new FilterDef(FittingFilters::subtractPlane, FittingFilters::subtractPlaneParams, tr(subtractPlaneDoc));
    m_filterList.insert("subtractPlane", filter);

    filter = new FilterDef(FittingFilters::subtractRegressionPlane, FittingFilters::subtractRegressionPlaneParams, tr(subtractRegressionPlaneDoc));
    m_filterList.insert("subtractRegressionPlane", filter);

//    TODO: undefined reference see polyfit2d.cpp
    filter = new FilterDef(FittingFilters::fitPolynom2D, FittingFilters::fitPolynom2DParams, tr("fits 2D-polynomial in 2D-dataObject and returns a double-DataObject with the fitted surface as well as an error value sigma"));
    m_filterList.insert("fitPolynom2D", filter);

    filter = new FilterDef(FittingFilters::polyfitWeighted2D, FittingFilters::polyfitWeighted2DParams, tr(polyfitWeighted2DDoc));
    m_filterList.insert("polyfitWeighted2D", filter);

    filter = new FilterDef(FittingFilters::polyval2D, FittingFilters::polyval2DParams, tr(polyval2DDoc));
    m_filterList.insert("polyval2D", filter);

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