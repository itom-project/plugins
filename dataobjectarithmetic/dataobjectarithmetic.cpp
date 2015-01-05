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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "dataobjectarithmetic.h"

#include <QtCore/QtPlugin>
#include <qstringlist.h>
#include <qvariant.h>
#include <qnumeric.h>

#include "common/helperCommon.h"

#include "DataObject/dataObjectFuncs.h"

#include "pluginVersion.h"

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmeticInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(DataObjectArithmetic)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmeticInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(DataObjectArithmetic)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectArithmeticInterface::DataObjectArithmeticInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("DataObjectArithmetic");
    
    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This plugin provides several arithmetic calculations for dataObject. These are for instance: \n\
- min- or maximum value\n\
- centroid along dimensions or inplane \n\
\n\
This plugin does not have any unusual dependencies.";

    m_description = QObject::tr("Operations and arithmetic calculations of dataObject.");
    m_detaildescription = QObject::tr(docstring);

    m_author            = "W. Lyda, M. Gronle, ITO, University Stuttgart";
    m_license           = QObject::tr("LGPL");
    m_version           = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer        = MINVERSION;
    m_maxItomVer        = MAXVERSION;
    m_aboutThis         = tr("Arithmetic algorithms filters.");     
    
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectArithmeticInterface::~DataObjectArithmeticInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(DataObjectArithmeticInterface, DataObjectArithmeticInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
DataObjectArithmetic::DataObjectArithmetic() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::singleDObjInputParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = Param("sourceImage", ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object for operation").toLatin1().data());
        paramsMand->append(param);
        paramsOut->append( Param("result", ParamBase::Double | ParamBase::Out, 0.0, NULL, tr("result of calculation. This param can be int or double").toLatin1().data()));
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::singleDObjInputInfParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = Param("sourceImage", ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object for operation").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("ignoreInf", ParamBase::Int | ParamBase::In, 0, 1, 0, tr("source image data object for operation").toLatin1().data());
        paramsOpt->append(param);

        paramsOut->append( Param("result", ParamBase::Double | ParamBase::Out, 0.0, NULL, tr("result of calculation. This param can be int or double").toLatin1().data()));
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::singleDObjInputValueAndPositionOutParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = Param("sourceImage", ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object for operation").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("ignoreInf", ParamBase::Int | ParamBase::In, 0, 1, 0, tr("Ignore invalid-Values for floating point").toLatin1().data());
        paramsOpt->append(param);
        paramsOut->append( Param("result", ParamBase::Double | ParamBase::Out, 0.0, NULL, tr("result of calculation. This param can be int or double").toLatin1().data()));
        paramsOut->append( Param("plane", ParamBase::Int | ParamBase::Out, 0, NULL, tr("Index of the plane, which contains the result.").toLatin1().data()));
        paramsOut->append( Param("y", ParamBase::Int | ParamBase::Out, 0, NULL, tr("Pixelindex in y-direction.").toLatin1().data()));
        paramsOut->append( Param("x", ParamBase::Int | ParamBase::Out, 0, NULL, tr("Pixelindex in x-direction.").toLatin1().data()));
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::doubleDObjInputParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = Param("sourceImage1", ParamBase::DObjPtr | ParamBase::In, NULL, tr("1. source image data object for operation").toLatin1().data());
        paramsMand->append(param);
        param = Param("sourceImage2", ParamBase::DObjPtr | ParamBase::In, NULL, tr("2. source image data object for operation").toLatin1().data());
        paramsMand->append(param);

        paramsOut->append( Param("result", ParamBase::Int | ParamBase::Out, 0, tr("0 if both data objects are not equal, else 1").toLatin1().data()));
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
const char* DataObjectArithmetic::minValueDoc = "This filter calculated the minimal value and its first location within the dataObject. \n\
\n\
The result value will be Integer vor all integer types or Double for all floating point types\n\
\n\
The filter do not work with RGBA32, Complex64 and Complex128, but with all other data-types\n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::minValue(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObj = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );

    if(dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NUL").toLatin1().data());
    }

    if(dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

// new Version using the SDK-minValueHelper
    ito::float64 result = 0.0;
    ito::uint32 location[3] = {0,0,0};
    bool ignoreInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;

    retval += ito::dObjHelper::minValue(dObj, result, location, ignoreInf);

    switch( dObj->getType() )
    {
    case tUInt8:
    case tInt8:
    case tUInt16:
    case tInt16:
    case tUInt32:
    case tInt32:
        ignoreInf = false;
        (*paramsOut)[0] = ParamBase("result",ParamBase::Int | ParamBase::Out, static_cast<int>(result));
        break;
    case tFloat32:
    case tFloat64:
    case tComplex64:
    case tComplex128:
        (*paramsOut)[0] = ParamBase("result",ParamBase::Double | ParamBase::Out, static_cast<double>(result));
        break;
    default:
        retval += ito::RetVal(retError, 0, tr("data type not supported").toLatin1().data());
        //outVals->clear();
    }

    (*paramsOut)[1] = ParamBase("plane",ParamBase::Int | ParamBase::Out, static_cast<int>(location[0]));
    (*paramsOut)[2] = ParamBase("y",ParamBase::Int | ParamBase::Out, static_cast<int>(location[1]));
    (*paramsOut)[3] = ParamBase("x",ParamBase::Int | ParamBase::Out, static_cast<int>(location[2]));


    return retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
const char* DataObjectArithmetic::maxValueDoc = "This filter calculated the maximal value and its first location within the dataObject. \n\
\n\
The result value will be Integer vor all integer types or Double for all floating point types\n\
\n\
The filter do not work with RGBA32, Complex64 and Complex128, but with all other data-types\n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::maxValue(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObj = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );

    if(dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NUL").toLatin1().data());
    }

    if(dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

// new Version using the SDK-minValueHelper
    ito::float64 result = 0.0;
    ito::uint32 location[3] = {0,0,0};

    bool ignoreInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;

    retval += ito::dObjHelper::maxValue(dObj, result, location, ignoreInf);

    switch( dObj->getType() )
    {
    case tUInt8:
    case tInt8:
    case tUInt16:
    case tInt16:
    case tUInt32:
    case tInt32:
        ignoreInf = false;
        (*paramsOut)[0] = ParamBase("result",ParamBase::Int | ParamBase::Out, static_cast<int>(result));
        break;
    case tFloat32:
    case tFloat64:
    case tComplex64:
    case tComplex128:
        (*paramsOut)[0] = ParamBase("result",ParamBase::Double | ParamBase::Out, static_cast<double>(result));
        break;
    default:
        retval += ito::RetVal(retError, 0, tr("data type not supported").toLatin1().data());
        //outVals->clear();
    }

    (*paramsOut)[1] = ParamBase("plane",ParamBase::Int | ParamBase::Out, static_cast<int>(location[0]));
    (*paramsOut)[2] = ParamBase("y",ParamBase::Int | ParamBase::Out, static_cast<int>(location[1]));
    (*paramsOut)[3] = ParamBase("x",ParamBase::Int | ParamBase::Out, static_cast<int>(location[2]));

    return retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
const char* DataObjectArithmetic::minMaxValueDoc = "This filter calculated the minimal and maximal value and its first location within the dataObject. \n\
\n\
The result value will be Integer vor all integer types or Double for all floating point types\n\
\n\
The filter do not work with RGBA32 but with all other data-types\n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::minMaxValueParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object for operation").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("ignoreInf", ParamBase::Int | ParamBase::In, 0, 1, 0, tr("Ignore invalid-Values for floating point").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("complexHandling", ParamBase::Int | ParamBase::In, 0, 3, 0, tr("Switch complex handling, 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value").toLatin1().data());
        paramsOpt->append(param);
        paramsOut->append( Param("minimum", ParamBase::Double | ParamBase::Out, 0.0, NULL, tr("Minimal value, this parameter be int or double").toLatin1().data()));
        paramsOut->append( Param("planeMin", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Index of the plane, which contains the result.").toLatin1().data()));
        paramsOut->append( Param("yMin", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Pixelindex in y-direction.").toLatin1().data()));
        paramsOut->append( Param("xMin", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Pixelindex in x-direction.").toLatin1().data()));
        paramsOut->append( Param("maximum", ParamBase::Double | ParamBase::Out, 0.0, NULL, tr("Maximum value, this parameter. This param can be int or double").toLatin1().data()));
        paramsOut->append( Param("planeMax", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Index of the plane, which contains the result.").toLatin1().data()));
        paramsOut->append( Param("yMax", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Pixelindex in y-direction.").toLatin1().data()));
        paramsOut->append( Param("xMax", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Pixelindex in x-direction.").toLatin1().data()));
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::minMaxValue(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObj = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );

    if(dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NUL").toLatin1().data());
    }

    if(dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

    ito::float64 minVal = 0.0;
    ito::float64 maxVal = 0.0;
    ito::uint32 locationMin[3] = {0,0,0};
    ito::uint32 locationMax[3] = {0,0,0};

    bool ignoreInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;
    int cmplxState = (*paramsOpt)[1].getVal<int>();

    retval += ito::dObjHelper::minMaxValue(dObj, minVal, locationMin, maxVal, locationMax, ignoreInf, cmplxState);

    switch( dObj->getType() )
    {
    case tUInt8:
    case tInt8:
    case tUInt16:
    case tInt16:
    case tUInt32:
    case tInt32:
        ignoreInf = false;
        (*paramsOut)[0] = ParamBase("minimum",ParamBase::Int | ParamBase::Out, static_cast<int>(minVal));
        (*paramsOut)[4] = ParamBase("maximum",ParamBase::Int | ParamBase::Out, static_cast<int>(maxVal));
        break;
    case tFloat32:
    case tFloat64:
    case tComplex64:
    case tComplex128:
        (*paramsOut)[0] = ParamBase("minimum",ParamBase::Double | ParamBase::Out, static_cast<double>(minVal));
        (*paramsOut)[4] = ParamBase("maximum",ParamBase::Double | ParamBase::Out, static_cast<double>(maxVal));
        break;
    default:
        retval += ito::RetVal(retError, 0, tr("data type not supported").toLatin1().data());
        //outVals->clear();
    }

    (*paramsOut)[1] = ParamBase("planeMin",ParamBase::Int | ParamBase::Out, static_cast<int>(locationMin[0]));
    (*paramsOut)[2] = ParamBase("yMin",ParamBase::Int | ParamBase::Out, static_cast<int>(locationMin[1]));
    (*paramsOut)[3] = ParamBase("xMin",ParamBase::Int | ParamBase::Out, static_cast<int>(locationMin[2]));

    (*paramsOut)[5] = ParamBase("planeMax",ParamBase::Int | ParamBase::Out, static_cast<int>(locationMax[0]));
    (*paramsOut)[6] = ParamBase("yMax",ParamBase::Int | ParamBase::Out, static_cast<int>(locationMax[1]));
    (*paramsOut)[7] = ParamBase("xMax",ParamBase::Int | ParamBase::Out, static_cast<int>(locationMax[2]));


    return retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
const char* DataObjectArithmetic::meanValueDoc = "This filter calculated the mean value within the dataObject. \n\
\n\
The return value containing the mean value of the dataObject.\n\
\n\
The filter do not work with RGBA32, Complex64 and Complex128, but with all other data-types\n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::meanValue(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObj = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );

    if(dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NUL").toLatin1().data());
    }

    if(dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

// new Version using the SDK-minValueHelper


    ito::float64 result = 0.0;
    bool toggleInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;
    

    retval += ito::dObjHelper::meanValue(dObj, result, toggleInf);

    (*paramsOut)[0] = ParamBase("result",ParamBase::Double | ParamBase::Out, static_cast<double>(result));

    return retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::devValueParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object for operation").toLatin1().data());
        paramsMand->append(param);
        param = Param("flag", ito::ParamBase::Int | ParamBase::In, 0, 1, 0, tr("Toggles the calculation mode of standard deviation over N or N-1 elements").toLatin1().data());
        paramsOpt->append(param);
        param = Param("ignoreInf", ParamBase::Int | ParamBase::In, 0, 1, 1, tr("Ignore invalid-Values for floating point").toLatin1().data());
        paramsOpt->append(param);

        paramsOut->append( ito::Param("mean", ito::ParamBase::Double | ito::ParamBase::Out, 0.0, NULL, tr("mean result").toLatin1().data()));
        paramsOut->append( ito::Param("dev", ito::ParamBase::Double | ito::ParamBase::Out, 0.0, NULL, tr("deviation result").toLatin1().data()));
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
const char* DataObjectArithmetic::devValueDoc = "The filter returns the arithmetic mean and the standard deviation of the given dataObject within its ROI.\nThe optinal flag to toggles if (flag==0) the deviation is calculated by 1/(n-1)*sqrt(sum(x-xm)^2)\nor (flag ==1) by 1/(n)*sqrt(sum(x-xm)^2)\n\
\n\
The filter do not work with RGBA32, Complex64 and Complex128, but with all other data-types\n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::devValue(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObj = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );

    if(dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toLatin1().data());
    }


    ito::float64 meanResult = 0.0;
    ito::float64 devResult = 0.0;
    
    int flag = (*paramsOpt)[0].getVal<int>();
    bool toggleInf = (*paramsOpt)[1].getVal<int>() > 0 ? true : false;

    if(dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

// new Version using the SDK-minValueHelper
  
    retval += ito::dObjHelper::devValue(dObj, flag, meanResult, devResult, toggleInf);

    (*paramsOut)[0] = ParamBase("mean",ParamBase::Double | ParamBase::Out, static_cast<double>(meanResult));
    (*paramsOut)[1] = ParamBase("dev",ParamBase::Double | ParamBase::Out, static_cast<double>(devResult));

    return retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
const char* DataObjectArithmetic::areEqualDoc = "Check pixel-wise wether two dataObjects are equal. \n\
The filter returns 1 if both objects are pixel-wise equal, else returns 0.\n\
\n\
The filter do not work with RGBA32, Complex64 and Complex128, but with all other data-types\n\
\n";

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Type> bool areEqualHelper(_Type* first, int xStep0, int yStep0, _Type* second, int xstep1, int ystep1, int rows, int cols)
{
    _Type* curSecond;
    _Type* curFirst;
    if(std::numeric_limits<_Type>::is_exact)
    {
        for(int y = 0; y < rows; y++)
        {
            curFirst = (_Type*)(((char*)first) + y * yStep0);
            curSecond =(_Type*)(((char*)second) + y * yStep0);
            for(int x = 0; x < cols - 1; x++)
            {
                if(first[x] != second[x]) return true;
            }
            if(*first != *second) return true;
        }
    }
    else
    {
        for(int y = 0; y < rows; y++)
        {
            curFirst = (_Type*)(((char*)first) + y * yStep0);
            curSecond =(_Type*)(((char*)second) + y * yStep0);
            for(int x = 0; x < cols - 1; x++)
            {
                if(ito::dObjHelper::isNotZero<_Type>(first[x] != second[x])) return true;
            }
        }
    }
    return false;
}

template<> bool areEqualHelper<complex64>(complex64* first, int xStep0, int yStep0, complex64* second, int xstep1, int ystep1, int rows, int cols)
{
    complex64* curSecond;
    complex64* curFirst;
    for(int y = 0; y < rows; y++)
    {
        curFirst = (complex64*)(((char*)first) + y * yStep0);
        curSecond =(complex64*)(((char*)second) + y * yStep0);
        for(int x = 0; x < cols - 1; x++)
        {
            if(ito::dObjHelper::isNotZero(first[x].real() - second[x].real())) return true;
            if(ito::dObjHelper::isNotZero(first[x].imag() - second[x].imag())) return true;
        }
    }

    return false;
}
template<> bool areEqualHelper<complex128>(complex128* first, int xStep0, int yStep0, complex128* second, int xstep1, int ystep1, int rows, int cols)
{

    complex128* curSecond;
    complex128* curFirst;
    for(int y = 0; y < rows; y++)
    {
        curFirst = (complex128*)(((char*)first) + y * yStep0);
        curSecond =(complex128*)(((char*)second) + y * yStep0);
        for(int x = 0; x < cols - 1; x++)
        {
            if(ito::dObjHelper::isNotZero(first[x].real() - second[x].real())) return true;
            if(ito::dObjHelper::isNotZero(first[x].imag() - second[x].imag())) return true;
        }
    }

    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::areEqual(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObj1 = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );
    if(dObj1 == NULL)
    {
        (*paramsOut)[0].setVal<int>(0);
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toLatin1().data());
    }

    ito::DataObject *dObj2 = static_cast<ito::DataObject*>( (*paramsMand)[1].getVal<void*>() );
    if(dObj2 == NULL)
    {
        (*paramsOut)[0].setVal<int>(0);
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toLatin1().data());
    }

    bool typeFlag;
    bool dimsFlag;
    bool last2DimsFlag;

    if(!ito::dObjHelper::dObjareEqualDetail(dObj1, dObj2, typeFlag, dimsFlag, last2DimsFlag))
    {
        //outVals->append(static_cast<bool>(false));
        (*paramsOut)[0].setVal<int>(0);
        return retOk;
    }
    /*
    ito::DataObject left = *dObj1;
    ito::DataObject right = *dObj2;

    ito::DataObject test = (left == right);
    int x, xSize = test.getSize(test.getDims()-1);
    int y, ySize = test.getSize(test.getDims()-2);

    for (int z = 0; z < test.calcNumMats(); z++)
    {
        cv::Mat_<unsigned char> * curMat = ((cv::Mat_<unsigned char> *)test.get_mdata()[test.seekMat(z)]);
        unsigned char* dataptr = NULL;
        for(y = 0; y < ySize; y++)
        {
            dataptr = curMat->ptr<unsigned char>(y);
            for(x = 0; x < xSize; x++)
            {
                if(!dataptr[x])
                {
                    (*paramsOut)[0].setVal<int>(0);
                    return retOk;
                }
            }
        }
    }
    */

    switch(dObj2->getType())
    {
        case ito::tInt8:
        for (int z = 0; z < dObj1->calcNumMats(); z++)
        {
            cv::Mat* mat1 = (cv::Mat*)(dObj1->get_mdata()[dObj1->seekMat(z)]);
            cv::Mat* mat2 = (cv::Mat*)(dObj2->get_mdata()[dObj1->seekMat(z)]);

            int stepX0 = static_cast<int>(mat1->step[1]);
            int stepX1 = static_cast<int>(mat2->step[1]);
            int stepY0 = static_cast<int>(mat1->step[0]);
            int stepY1 = static_cast<int>(mat2->step[0]);

            int8* first = mat1->ptr<int8>();
            int8* second = mat2->ptr<int8>();

            if(areEqualHelper(first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
            {
                (*paramsOut)[0].setVal<int>(0);
                return retOk;
            }
        }
        break;
        case ito::tUInt8:
        for (int z = 0; z < dObj1->calcNumMats(); z++)
        {
            cv::Mat* mat1 = (cv::Mat*)(dObj1->get_mdata()[dObj1->seekMat(z)]);
            cv::Mat* mat2 = (cv::Mat*)(dObj2->get_mdata()[dObj1->seekMat(z)]);

            int stepX0 = static_cast<int>(mat1->step[1]);
            int stepX1 = static_cast<int>(mat2->step[1]);
            int stepY0 = static_cast<int>(mat1->step[0]);
            int stepY1 = static_cast<int>(mat2->step[0]);

            uint8* first = mat1->ptr<uint8>();
            uint8* second = mat2->ptr<uint8>();

            if(areEqualHelper(first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
            {
                (*paramsOut)[0].setVal<int>(0);
                return retOk;
            }
        }
        break;
        case ito::tInt16:
        for (int z = 0; z < dObj1->calcNumMats(); z++)
        {
            cv::Mat* mat1 = (cv::Mat*)(dObj1->get_mdata()[dObj1->seekMat(z)]);
            cv::Mat* mat2 = (cv::Mat*)(dObj2->get_mdata()[dObj1->seekMat(z)]);

            int stepX0 = static_cast<int>(mat1->step[1]);
            int stepX1 = static_cast<int>(mat2->step[1]);
            int stepY0 = static_cast<int>(mat1->step[0]);
            int stepY1 = static_cast<int>(mat2->step[0]);

            int16* first = mat1->ptr<int16>();
            int16* second = mat2->ptr<int16>();

            if(areEqualHelper(first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
            {
                (*paramsOut)[0].setVal<int>(0);
                return retOk;
            }
        }
        break;
        case ito::tUInt16:
        for (int z = 0; z < dObj1->calcNumMats(); z++)
        {
            cv::Mat* mat1 = (cv::Mat*)(dObj1->get_mdata()[dObj1->seekMat(z)]);
            cv::Mat* mat2 = (cv::Mat*)(dObj2->get_mdata()[dObj1->seekMat(z)]);

            int stepX0 = static_cast<int>(mat1->step[1]);
            int stepX1 = static_cast<int>(mat2->step[1]);
            int stepY0 = static_cast<int>(mat1->step[0]);
            int stepY1 = static_cast<int>(mat2->step[0]);

            uint16* first = mat1->ptr<uint16>();
            uint16* second = mat2->ptr<uint16>();

            if(areEqualHelper(first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
            {
                (*paramsOut)[0].setVal<int>(0);
                return retOk;
            }
        }
        break;
        case ito::tInt32:
        for (int z = 0; z < dObj1->calcNumMats(); z++)
        {
            cv::Mat* mat1 = (cv::Mat*)(dObj1->get_mdata()[dObj1->seekMat(z)]);
            cv::Mat* mat2 = (cv::Mat*)(dObj2->get_mdata()[dObj1->seekMat(z)]);

            int stepX0 = static_cast<int>(mat1->step[1]);
            int stepX1 = static_cast<int>(mat2->step[1]);
            int stepY0 = static_cast<int>(mat1->step[0]);
            int stepY1 = static_cast<int>(mat2->step[0]);

            int32* first = mat1->ptr<int32>();
            int32* second = mat2->ptr<int32>();

            if(areEqualHelper(first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
            {
                (*paramsOut)[0].setVal<int>(0);
                return retOk;
            }
        }
        break;
        case ito::tFloat32:
        for (int z = 0; z < dObj1->calcNumMats(); z++)
        {
            cv::Mat* mat1 = (cv::Mat*)(dObj1->get_mdata()[dObj1->seekMat(z)]);
            cv::Mat* mat2 = (cv::Mat*)(dObj2->get_mdata()[dObj1->seekMat(z)]);

            int stepX0 = static_cast<int>(mat1->step[1]);
            int stepX1 = static_cast<int>(mat2->step[1]);
            int stepY0 = static_cast<int>(mat1->step[0]);
            int stepY1 = static_cast<int>(mat2->step[0]);

            float32* first = mat1->ptr<float32>();
            float32* second = mat2->ptr<float32>();

            if(areEqualHelper(first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
            {
                (*paramsOut)[0].setVal<int>(0);
                return retOk;
            }
        }
        break;
        case ito::tFloat64:
        for (int z = 0; z < dObj1->calcNumMats(); z++)
        {
            cv::Mat* mat1 = (cv::Mat*)(dObj1->get_mdata()[dObj1->seekMat(z)]);
            cv::Mat* mat2 = (cv::Mat*)(dObj2->get_mdata()[dObj1->seekMat(z)]);

            int stepX0 = static_cast<int>(mat1->step[1]);
            int stepX1 = static_cast<int>(mat2->step[1]);
            int stepY0 = static_cast<int>(mat1->step[0]);
            int stepY1 = static_cast<int>(mat2->step[0]);

            float64* first = mat1->ptr<float64>();
            float64* second = mat2->ptr<float64>();

            if(areEqualHelper(first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
            {
                (*paramsOut)[0].setVal<int>(0);
                return retOk;
            }
        }
        break;
        case ito::tComplex64:
        for (int z = 0; z < dObj1->calcNumMats(); z++)
        {
            cv::Mat* mat1 = (cv::Mat*)(dObj1->get_mdata()[dObj1->seekMat(z)]);
            cv::Mat* mat2 = (cv::Mat*)(dObj2->get_mdata()[dObj1->seekMat(z)]);

            int stepX0 = static_cast<int>(mat1->step[1]);
            int stepX1 = static_cast<int>(mat2->step[1]);
            int stepY0 = static_cast<int>(mat1->step[0]);
            int stepY1 = static_cast<int>(mat2->step[0]);

            complex64* first = mat1->ptr<complex64>();
            complex64* second = mat2->ptr<complex64>();

            if(areEqualHelper(first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
            {
                (*paramsOut)[0].setVal<int>(0);
                return retOk;
            }
        }
        break;
        case ito::tComplex128:
        for (int z = 0; z < dObj1->calcNumMats(); z++)
        {
            cv::Mat* mat1 = (cv::Mat*)(dObj1->get_mdata()[dObj1->seekMat(z)]);
            cv::Mat* mat2 = (cv::Mat*)(dObj2->get_mdata()[dObj1->seekMat(z)]);

            int stepX0 = static_cast<int>(mat1->step[1]);
            int stepX1 = static_cast<int>(mat2->step[1]);
            int stepY0 = static_cast<int>(mat1->step[0]);
            int stepY1 = static_cast<int>(mat2->step[0]);

            complex128* first = mat1->ptr<complex128>();
            complex128* second = mat2->ptr<complex128>();

            if(areEqualHelper(first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
            {
                (*paramsOut)[0].setVal<int>(0);
                return retOk;
            }
        }
        break;
        default:
            return ito::RetVal(ito::retError, 0, tr("type not supported").toLatin1().data());
    
    }
    
    (*paramsOut)[0].setVal<int>(1);

    return retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
const char* DataObjectArithmetic::centerOfGravityDoc = "This filter calculates the center of gravity of a 2D real image. \n\
\n\
The return value contains the column and row position in pixel and physical coordinates.\n\
\n\
For the determination, only values in the range [lowThreshold, highThreshold] are considered. The COG algorithm requires, that all values \n\
that do not belong to the required peak have values around zero. In order to achieve this, the 'lowThreshold' value is subtracted from each \n\
valid intensity value before calculating the COG with the following equations: \n\
\n\
cXI = \\frac{\\sum{idx_x * (I - lowThreshold)}}{\\sum{(I - lowThreshold)} \n\
cYI = \\frac{\\sum{idx_y * (I - lowThreshold)}}{\\sum{(I - lowThreshold)} \n\
\n\
The filter does not work with RGBA32, Complex64 and Complex128, but with all other data-types.";

RetVal DataObjectArithmetic::centerOfGravityParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr, NULL, tr("2D source image data object (u)int8, (u)int16, int32, float32 or float64 only.").toLatin1().data());
        paramsMand->append(param);
        param = Param("lowThreshold", ito::ParamBase::Double, -1*std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), 0.0, tr("values < lowThreshold are ignored. lowThreshold is subtracted from each valid value before COG determination.").toLatin1().data());
        paramsOpt->append(param);
        param = Param("highThreshold", ito::ParamBase::Double, -1*std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), tr("values > highThreshold are ignored.").toLatin1().data());
        paramsOpt->append(param);

        paramsOut->append( ito::Param("cYI", ito::ParamBase::Double | ito::ParamBase::Out, std::numeric_limits<ito::float64>::quiet_NaN(), NULL, tr("y-Coordinate of COG (index)").toLatin1().data()));
        paramsOut->append( ito::Param("cXI", ito::ParamBase::Double | ito::ParamBase::Out, std::numeric_limits<ito::float64>::quiet_NaN(), NULL, tr("x-Coordinate of COG (index)").toLatin1().data()));
        paramsOut->append( ito::Param("cY", ito::ParamBase::Double | ito::ParamBase::Out, std::numeric_limits<ito::float64>::quiet_NaN(), NULL, tr("y-Coordinate of COG (physical unit)").toLatin1().data()));
        paramsOut->append( ito::Param("cX", ito::ParamBase::Double | ito::ParamBase::Out, std::numeric_limits<ito::float64>::quiet_NaN(), NULL, tr("x-Coordinate of COG (physical unit)").toLatin1().data()));
    }

    return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::centerOfGravity(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObj = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );
    if(dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is NULL").toLatin1().data());
    }
    if(dObj->getDims() < 1)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is not initialized").toLatin1().data());    
    }

    ito::float64 cx = std::numeric_limits<ito::float64>::quiet_NaN();
    ito::float64 cy = std::numeric_limits<ito::float64>::quiet_NaN();
    ito::float64 cxPhys = std::numeric_limits<ito::float64>::quiet_NaN();
    ito::float64 cyPhys = std::numeric_limits<ito::float64>::quiet_NaN();

    ito::float64 lowThreshold = (*paramsOpt)[0].getVal<ito::float64>();
    ito::float64 highThreshold = (*paramsOpt)[1].getVal<ito::float64>();

    retval += ito::dObjHelper::verifyDataObjectType(dObj, "sourceImage", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if(dObj->getDims() > 2)
    {
        for(int i = 0; i < dObj->getDims() - 2; i++)
        {
            if(dObj->getSize(i) > 1)
            {
                return ito::RetVal(ito::retError, 0, tr("Error: source image must not have multiple planes").toLatin1().data());  
            }
        }    
    }    


    if(!retval.containsError())
    {
        const cv::Mat *plane = dObj->getCvPlaneMat(0);

        switch(dObj->getType())
        {
            case ito::tInt8:
                centroidHelper<ito::int8>(plane, cv::saturate_cast<ito::int8>(lowThreshold), cv::saturate_cast<ito::int8>(highThreshold), cx, cy);
            break;
            case ito::tUInt8:
                centroidHelper<ito::uint8>(plane, cv::saturate_cast<ito::uint8>(lowThreshold), cv::saturate_cast<ito::uint8>(highThreshold), cx, cy);
            break;
            case ito::tInt16:
                centroidHelper<ito::int16>(plane, cv::saturate_cast<ito::int16>(lowThreshold), cv::saturate_cast<ito::int16>(highThreshold), cx, cy);
            break;
            case ito::tUInt16:
                centroidHelper<ito::uint16>(plane, cv::saturate_cast<ito::uint16>(lowThreshold), cv::saturate_cast<ito::uint16>(highThreshold), cx, cy);
            break;
            case ito::tInt32:
                centroidHelper<ito::int32>(plane, cv::saturate_cast<ito::int32>(lowThreshold), cv::saturate_cast<ito::int32>(highThreshold), cx, cy);
            break;
            case ito::tFloat32:
                centroidHelper<ito::float32>(plane, cv::saturate_cast<ito::float32>(lowThreshold), cv::saturate_cast<ito::float32>(highThreshold), cx, cy);
            break;
            case ito::tFloat64:
                centroidHelper<ito::float64>(plane, cv::saturate_cast<ito::float64>(lowThreshold), cv::saturate_cast<ito::float64>(highThreshold), cx, cy);
            break;
            default:
                return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented for phase shifting evaluation").toLatin1().data());
        }
    }


    (*paramsOut)[0].setVal<ito::float64>(cy);
    (*paramsOut)[1].setVal<ito::float64>(cx);

    if(!retval.containsError() && ito::dObjHelper::isFinite<ito::float64>(cy))
    {
        bool test;
        cxPhys = dObj->getPixToPhys(dObj->getDims()-1, cx, test);
        cyPhys = dObj->getPixToPhys(dObj->getDims()-2, cy, test);
    }

    (*paramsOut)[2].setVal<ito::float64>(cyPhys);
    (*paramsOut)[3].setVal<ito::float64>(cxPhys);

    return retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal DataObjectArithmetic::centroidHelper(const cv::Mat *mat, const _Tp& lowThreshold, const _Tp &highThreshold, ito::float64 &xCOG, ito::float64 &yCOG)
{
    ito::int32 x, y;
    ito::float64 val = 0.0, sumva = 0.0, sumXv = 0.0, sumYv = 0.0;
    const _Tp *pValue = NULL;
    
    if(std::numeric_limits<_Tp>::is_exact)
    {
        for(y = 0; y < mat->rows; ++y)
        {
            pValue = mat->ptr<_Tp>(y);
            for(x = 0; x < mat->cols; ++x)
            {
                if (pValue[x] >= lowThreshold && pValue[x] <= highThreshold)
                {
                    val = (ito::float64) (pValue[x] - lowThreshold);
                    sumva += val;
                    sumXv += val * x;
                    sumYv += val * y;  
                }
            }
        }
    }
    else
    {
        for(y = 0; y < mat->rows; ++y)
        {
            pValue = mat->ptr<_Tp>(y);
            for(x = 0; x < mat->cols; ++x)
            {
                if(ito::dObjHelper::isFinite<_Tp>(pValue[x]) && pValue[x] >= lowThreshold && pValue[x] <= highThreshold)
                {
                    val = (ito::float64) (pValue[x] - lowThreshold);
                    sumva += val;
                    sumXv += val * x;
                    sumYv += val * y;
                }
            }
        }    
    }

    if(ito::dObjHelper::isNotZero<ito::float64>(sumva))
    {
        xCOG = sumXv / sumva;
        yCOG = sumYv / sumva;    
    }
    else
    {
        xCOG = std::numeric_limits<ito::float64>::quiet_NaN();
        yCOG = std::numeric_limits<ito::float64>::quiet_NaN();
    }

    return ito::retOk;

}


//----------------------------------------------------------------------------------------------------------------------------------
const char* DataObjectArithmetic::centerOfGravity1DimDoc = "Calculate center of gravity for each plane along the x- or y-direction. \n\
\n\
This methods creates the two given data objects 'destCOG' and 'destIntensity' in the following way: \n\
\n\
- destCOG, ito::float64, sizes: [nrOfPlanes x sizeOfElements], contains the sub-pixel wise one-dimensional coordinate of the center of gravity or NaN if it could not be determined. \n\
- destIntensity, same type than input object, sizes: [nrOfPlanes x sizeOfElements], contains the absolute maximum along the search direction. \n\
\n\
If the center of gravity should be calculated along each row of each plane inside of the given 'sourceStack' data object, the parameter 'columnWise' must be 0, for a column-wise \n\
calculation is must be set to 1. Along each search direction, the corresponding minimum and maximum value is determined and the center of gravity is determined using: \n\
\n\
cog = \\frac{\\sum{idx * (I - lowerBoundary)}}{\\sum{(I - lowerBoundary)} \n\
\n\
A value *I* is only valid and considered in the equation above if: \n\
\n\
- (max - min) > pvThreshold (peak-to-valley threshold, if not given, destCOG contains NaN at this position) \n\
- I > lowerThreshold (only checked if lowerThreshold > minimum possible value of the given data type) \n\
- I > (max + min) * dynamicThreshold (only checked if dynamicThreshold > 0.0) \n\
\n\
The value 'lowerBoundary' is set to the corresponding maximum of 'lowerThreshold' and 'dynamicThreshold' if one of those is checked; else the given data is considered that the values \n\
all drop to zero at the edge of each search range; for a valid cog determination, it is necessary to assume that all values that are far away from the cog position have values around zero; \n\
if this is not the case consider to set an appropriate value 'lowerThreshold' and / or 'dynamicThreshold'. \n\
\n\
The filter is not implemented for complex data types and the type rgba32.";
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::centerOfGravity1DimParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr, NULL, tr("source image data (2D or 3D) object for operation (u)int8, (u)int16, int32, float32 or float64").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destCOG", ito::ParamBase::DObjPtr, NULL, tr("destination image data object for operation, will contain evaluated COG (in physical coordinates)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destIntensity", ito::ParamBase::DObjPtr, NULL, tr("destination image data object for operation, will contain maximal intensity").toLatin1().data());
        paramsMand->append(param);
        param = Param("pvThreshold", ito::ParamBase::Double, 0.0, std::numeric_limits<ito::float64>::max(), 0.0, tr("if (max-min) along the search direction is lower or equal this pvThreshold (peak-to-valley), no cog is determined and a NaN value is set into the resulting position array (default: this threshold is not considered).").toLatin1().data());
        paramsOpt->append(param);
        param = Param("dynamicThreshold", ito::ParamBase::Double, 0.0, 0.999, 0.5, tr("If != 0.0, values <= (max+min)*dynamicThreshold will be ignored. To only consider values above the FWHM, set this value to 0.5 (default).").toLatin1().data());
        paramsOpt->append(param);
        param = Param("lowerThreshold", ito::ParamBase::Double, -std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), -std::numeric_limits<ito::float64>::max(), tr("values <= lowerThreshold will not be considered for the cog calculation (default: this threshold is not considered).").toLatin1().data());
        paramsOpt->append(param);
        param = Param("columnWise", ito::ParamBase::Int, 0, 1, 0, tr("The search direction is along each column if 1, else along each row (default, 0)").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail Writes "nelems" of object with size "elsize" in various byteordering into a file
 @ param[in] buf        signes to write
 @ param[in] elsize     size of the elements
 @ param[in] nelem      number of elements
 @ param[in] fp         handle to an open file
 @ param[in] swap       Byteordering in file swap=0: normal, swap=1: inverted
 @ detail This function writes signes to a file. It is copied from former ito mcpp
 */
//-----
ito::RetVal DataObjectArithmetic::centerOfGravity1Dim(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject *dObjIN = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject *dObjCogOut = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::DataObject *dObjIntOut = paramsMand->at(2).getVal<ito::DataObject*>();

    if(dObjIN == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is NULL").toLatin1().data());
    }
    if(dObjIN->getDims() < 1)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is not initialized").toLatin1().data());    
    }
    
    if(dObjCogOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: destCOG image is NULL").toLatin1().data());
    }
    
    if(dObjIntOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: destIntensity image is NULL").toLatin1().data());
    }

    if(dObjCogOut == dObjIntOut)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: destCOG and destIntensity must not be the same data objects.").toLatin1().data());
    }

    ito::float64 pvThreshold = (*paramsOpt)[0].getVal<ito::float64>();
    ito::float64 dynThreshold = (*paramsOpt)[1].getVal<ito::float64>();
    ito::float64 lowerThreshold = (*paramsOpt)[3].getVal<ito::float64>();
    bool columnWise = (*paramsOpt)[2].getVal<ito::int32>() > 0 ? true: false;

    ito::float64 scaleVert = 1.0;   // Scale along depth ScanAxis
    ito::float64 offsetVert = 0.0; //offset along depth ScanAxis
    ito::uint32 sizeZ;
    ito::uint32 sizeY;
    ito::uint32 sizeX;
    int dObjINDims = dObjIN->getDims();

    retval += ito::dObjHelper::verify3DDataObject(dObjIN, "sourceImage", 1, 100000, 1, 100000, 1, 100000, 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if(!retval.containsError())
    {
        sizeZ =  dObjIN->getSize(0);
        sizeY =  dObjIN->getSize(1);
        sizeX =  dObjIN->getSize(2);
    }
    else
    {
        retval = ito::dObjHelper::verify2DDataObject(dObjIN, "sourceImage", 1, 100000, 1, 100000, 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);

        if (retval.containsError())
        {
            return retval;
        }

        sizeZ =  1;
        sizeY =  dObjIN->getSize(0);
        sizeX =  dObjIN->getSize(1);
    }

    ito::RetVal retvalTemp = ito::retOk;
    
    if(columnWise)
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(dObjCogOut, "destCOG", sizeZ, sizeZ, sizeX, sizeX, 1, ito::tFloat64);
    }
    else
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(dObjCogOut, "destCOG", sizeZ, sizeZ, sizeY, sizeY, 1, ito::tFloat64);
    }

    if(retvalTemp.containsError())
    {
        if(columnWise)
        {
            scaleVert = dObjIN->getAxisScale(dObjIN->getDims()-2);
            offsetVert = dObjIN->getAxisOffset(dObjIN->getDims()-2); 
            *dObjCogOut = ito::DataObject(sizeZ, sizeX, ito::tFloat64);
        }
        else
        {
            scaleVert = dObjIN->getAxisScale(dObjIN->getDims()-1);
            offsetVert = dObjIN->getAxisOffset(dObjIN->getDims()-1);
            *dObjCogOut = ito::DataObject(sizeZ, sizeY, ito::tFloat64);
        }
    }

    if(columnWise)
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(dObjIntOut, "destIntensity", sizeZ, sizeZ, sizeX, sizeX, 1, dObjIN->getType());
    }
    else
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(dObjIntOut, "destIntensity", sizeZ, sizeZ, sizeY, sizeY, 1, dObjIN->getType());
    }

    if(retvalTemp.containsError())
    {
        if(columnWise)
        {
            *dObjIntOut = ito::DataObject(sizeZ, sizeX, dObjIN->getType());
        }
        else
        {
            *dObjIntOut = ito::DataObject(sizeZ, sizeY, dObjIN->getType());
        }
    }
 
    if(!retval.containsError())
    {
        cv::Mat sliceCOG;
        cv::Mat sliceInt;
        const cv::Mat *planeIn = NULL;
        cv::Mat *planeCogOut = dObjCogOut->getCvPlaneMat(0);
        cv::Mat *planeIntOut = dObjIntOut->getCvPlaneMat(0);

        switch(dObjIN->getType())
        {
            case ito::tInt8:
                for(ito::uint32 i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = dObjIN->getCvPlaneMat(i);
                    centroidHelperFor1D<ito::int8>(planeIn, planeCogOut->ptr<ito::float64>(i), planeIntOut->ptr<ito::int8>(i), cv::saturate_cast<ito::int8>(pvThreshold), cv::saturate_cast<ito::int8>(lowerThreshold), dynThreshold, scaleVert, offsetVert, columnWise);
                }
            break;
            case ito::tUInt8:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = dObjIN->getCvPlaneMat(i);
                    centroidHelperFor1D<ito::uint8>(planeIn, planeCogOut->ptr<ito::float64>(i), planeIntOut->ptr<ito::uint8>(i), cv::saturate_cast<ito::uint8>(pvThreshold), cv::saturate_cast<ito::uint8>(lowerThreshold), dynThreshold, scaleVert, offsetVert, columnWise);
                }
            break;
            case ito::tInt16:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = dObjIN->getCvPlaneMat(i);
                    centroidHelperFor1D<ito::int16>(planeIn, planeCogOut->ptr<ito::float64>(i), planeIntOut->ptr<ito::int16>(i), cv::saturate_cast<ito::int16>(pvThreshold), cv::saturate_cast<ito::int16>(lowerThreshold), dynThreshold, scaleVert, offsetVert, columnWise);
                }
            break;
            case ito::tUInt16:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = dObjIN->getCvPlaneMat(i);
                    centroidHelperFor1D<ito::uint16>(planeIn, planeCogOut->ptr<ito::float64>(i), planeIntOut->ptr<ito::uint16>(i), cv::saturate_cast<ito::uint16>(pvThreshold), cv::saturate_cast<ito::uint16>(lowerThreshold), dynThreshold, scaleVert, offsetVert, columnWise);
                }
            break;
            case ito::tInt32:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = dObjIN->getCvPlaneMat(i);
                    centroidHelperFor1D<ito::int32>(planeIn, planeCogOut->ptr<ito::float64>(i), planeIntOut->ptr<ito::int32>(i), cv::saturate_cast<ito::int32>(pvThreshold), cv::saturate_cast<ito::int32>(lowerThreshold), dynThreshold, scaleVert, offsetVert, columnWise);
                }
            break;
            case ito::tFloat32:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = dObjIN->getCvPlaneMat(i);
                    centroidHelperFor1D<ito::float32>(planeIn, planeCogOut->ptr<ito::float64>(i), planeIntOut->ptr<ito::float32>(i), cv::saturate_cast<ito::float32>(pvThreshold), cv::saturate_cast<ito::float32>(lowerThreshold), dynThreshold, scaleVert, offsetVert, columnWise);
                }
            break;
            case ito::tFloat64:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = dObjIN->getCvPlaneMat(i);
                    centroidHelperFor1D<ito::float64>(planeIn, planeCogOut->ptr<ito::float64>(i), planeIntOut->ptr<ito::float64>(i), cv::saturate_cast<ito::float64>(pvThreshold), cv::saturate_cast<ito::float64>(lowerThreshold), dynThreshold, scaleVert, offsetVert, columnWise);
                }
            break;
            default:
                return ito::RetVal(ito::retError, 0, tr("Center of gravity can only be calculated for (u)int8, (u)int16, (u)int32, float32 or float64 data objects.").toLatin1().data());
        }
    }

    if(!retval.containsError())
    {
        bool test;
        if(columnWise)
        {
            
            dObjCogOut->setAxisScale(1, dObjIN->getAxisScale(dObjINDims-1));
            dObjCogOut->setAxisOffset(1, dObjIN->getAxisOffset(dObjINDims-1));
            dObjCogOut->setAxisUnit(1, dObjIN->getAxisUnit(dObjINDims-1, test));
            dObjCogOut->setAxisDescription(1, dObjIN->getAxisDescription(dObjINDims-1, test));

            dObjCogOut->setValueUnit( dObjIN->getAxisDescription(dObjINDims - 2, test) );

            dObjIntOut->setAxisScale(1, dObjIN->getAxisScale(dObjINDims - 1));
            dObjIntOut->setAxisOffset(1, dObjIN->getAxisOffset(dObjINDims - 1));
            dObjIntOut->setAxisUnit(1, dObjIN->getAxisUnit(dObjINDims - 1, test));
            dObjIntOut->setAxisDescription(1, dObjIN->getAxisDescription(dObjINDims - 1, test));

        }
        else
        {
            dObjCogOut->setAxisScale(1,         dObjIN->getAxisScale(dObjINDims - 2));
            dObjCogOut->setAxisOffset(1,        dObjIN->getAxisOffset(dObjINDims - 2));
            dObjCogOut->setAxisUnit(1,          dObjIN->getAxisUnit(dObjINDims - 2, test));
            dObjCogOut->setAxisDescription(1,   dObjIN->getAxisDescription(dObjINDims - 2, test));

            dObjCogOut->setValueUnit( dObjIN->getAxisDescription(dObjINDims - 1, test) );

            dObjIntOut->setAxisScale(1,         dObjIN->getAxisScale(dObjINDims - 2));
            dObjIntOut->setAxisOffset(1,        dObjIN->getAxisOffset(dObjINDims - 2));
            dObjIntOut->setAxisUnit(1,          dObjIN->getAxisUnit(dObjINDims - 2, test));
            dObjIntOut->setAxisDescription(1,   dObjIN->getAxisDescription(dObjINDims - 2, test));

        }

        if (dObjINDims >= 3)
        {
            dObjCogOut->setAxisScale(0,         dObjIN->getAxisScale(dObjINDims - 3));
            dObjCogOut->setAxisOffset(0,        dObjIN->getAxisOffset(dObjINDims - 3));
            dObjCogOut->setAxisUnit(0,          dObjIN->getAxisUnit(dObjINDims - 3, test));
            dObjCogOut->setAxisDescription(0,   dObjIN->getAxisDescription(dObjINDims - 3, test));
            dObjCogOut->setValueDescription("Center of Gravity");

            dObjIntOut->setAxisScale(0,         dObjIN->getAxisScale(dObjINDims - 3));
            dObjIntOut->setAxisOffset(0,        dObjIN->getAxisOffset(dObjINDims - 3));
            dObjIntOut->setAxisUnit(0,          dObjIN->getAxisUnit(dObjINDims - 3, test));
            dObjIntOut->setAxisDescription(0,   dObjIN->getAxisDescription(dObjINDims - 3, test));
            dObjIntOut->setValueDescription("intensity");
            dObjIntOut->setValueUnit("a.u.");
        }
        else
        {
            dObjCogOut->setValueDescription("Center of Gravity");
            dObjIntOut->setValueDescription("intensity");
            dObjIntOut->setValueUnit("a.u.");
        }

        dObjIN->copyTagMapTo(*dObjCogOut);
        dObjIN->copyTagMapTo(*dObjIntOut);

        if(columnWise)
        {
            QString protocol("Center of gravity evaluated columnwise from intensity stack, ");
            protocol.append("pvThreshold: ");
            protocol.append(QString::number(pvThreshold));
            if (dynThreshold > std::numeric_limits<ito::float64>::epsilon())
            {
                protocol.append("dynThreshold: ");
                protocol.append(QString::number(dynThreshold));
            }
            if (lowerThreshold > (-std::numeric_limits<ito::float64>::max() + std::numeric_limits<ito::float64>::epsilon()))
            {
                protocol.append("lowerThreshold: ");
                protocol.append(QString::number(lowerThreshold));
            }

            dObjCogOut->addToProtocol(protocol.toLatin1().data());
            dObjIntOut->addToProtocol("Max intensity evaluated columnwise from intensity stack");
        }
        else
        {
            dObjCogOut->addToProtocol("Center of gravity evaluated linewise from intensity stack");
            dObjIntOut->addToProtocol("Max intensity evaluated linewise from intensity stack");        
        }
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal DataObjectArithmetic::centroidHelperFor1D(const cv::Mat *inMat, ito::float64 *outCOG, _Tp *outINT, const _Tp &pvThreshold, const _Tp &lowerThreshold, const ito::float64 &dynamicThreshold, const ito::float64 &scale, const ito::float64 &offset, bool alongCols)
{       
    ito::uint32 stepEvalInMat;
    ito::uint32 stepAddrInMat;
    ito::uint32 cogsToCalc;
    ito::uint32 pixelToEval;
    ito::uint32 cogCnt = 0;

    ito::float64 sumI, sumPxI;
    
    const _Tp *inMatPtrFirst = inMat->ptr<_Tp>(0);
    const _Tp *pInValue = NULL;
    _Tp sw;

    if(alongCols)
    {
        cogsToCalc = inMat->rows;
        pixelToEval =  inMat->cols;  
        stepEvalInMat = (ito::uint32)inMat->step1();
        stepAddrInMat = 1;
    }
    else
    {
        cogsToCalc = inMat->cols;
        pixelToEval =  inMat->rows;
        stepEvalInMat = 1;
        stepAddrInMat = (ito::uint32)inMat->step1();
    }

    _Tp typeMax;
    _Tp typeMin;
    bool runDynamic = dynamicThreshold > std::numeric_limits<ito::float64>::epsilon() ? true : false;
    bool runLower = false;

    if(std::numeric_limits<_Tp>::is_exact)
    {
        typeMax = std::numeric_limits<_Tp>::max();
        typeMin = std::numeric_limits<_Tp>::min();   

        if (lowerThreshold != std::numeric_limits<_Tp>::min())
        {
            runLower = true;
        }
    }
    else
    {
        typeMax = std::numeric_limits<_Tp>::max();
        typeMin = -std::numeric_limits<_Tp>::max();   

        if (lowerThreshold > (-std::numeric_limits<_Tp>::max() + std::numeric_limits<_Tp>::epsilon()))
        {
            runLower = true;
        }
    }
    
    for(ito::uint32 pixelCnt = 0; pixelCnt < pixelToEval; pixelCnt++)
    {
        sumI = 0.0; 
        sumPxI = 0.0;
        _Tp maxVal = typeMin;
        _Tp minVal = typeMax;
        _Tp val = 0;
        

        //determine the min/max value along the search direction (along each column or each row)
        pInValue  = inMatPtrFirst + stepAddrInMat * pixelCnt;

        for(cogCnt = 0; cogCnt < cogsToCalc; cogCnt++)
        {
            if(maxVal < *pInValue)
            {
                maxVal = *pInValue;
            }

            if(minVal > *pInValue)
            {
                minVal = *pInValue;
            }

            pInValue += stepEvalInMat;
        }

        //determine the cog along the search direction:
        /*
        conditions:
        - (max - min) > pvThreshold (peak-to-valley threshold)
        - val > lowerThreshold (if lowerThreshold > (minimum of data type _Tp))
        - val > (max + min) * dynamicThreshold (if dynamicThreshold > 0)

        If lowerThreshold or dynamicThreshold is active, its current maximum value is subtracted from each value within the search direction before further calculation.
        */
        pInValue = inMatPtrFirst + stepAddrInMat * pixelCnt;

        if((maxVal - minVal) > pvThreshold)
        {
            if(runDynamic || runLower)
            {
                if (runLower && runDynamic)
                {
                    sw = std::max(((ito::float64)maxVal + (ito::float64)minVal) * dynamicThreshold, (ito::float64)lowerThreshold);
                }
                else if (runLower)
                {
                    sw = (ito::float64)lowerThreshold;
                }
                else
                {
                    sw = ((ito::float64)maxVal + (ito::float64)minVal) * dynamicThreshold; 
                }

                for(cogCnt = 0; cogCnt < cogsToCalc; ++cogCnt)
                {
                    if(*pInValue > sw)
                    {
                        val = *pInValue - sw;
                        sumI += val;
                        sumPxI += (ito::float64)val * (ito::float64)cogCnt;                        
                    }
                    pInValue = pInValue + stepEvalInMat;
                }

            }
            else
            {
                //this version is only valid, if the minimum value converges towards zero, else the minimum value must be subtracted from each value first (consider to set a lowerThreshold)
                for(cogCnt = 0; cogCnt < cogsToCalc; ++cogCnt)
                 {
                    val = *pInValue;
                    sumI += val;
                    sumPxI += (ito::float64)val * (ito::float64)cogCnt;    
                    pInValue = pInValue + stepEvalInMat;
                }
            }
        }

        //save the currently found maximum along the search direction
        outINT[pixelCnt] = maxVal;

        //calculate and save current cog position
        if(ito::dObjHelper::isNotZero<ito::float64>(sumI))
        {
            outCOG[pixelCnt] = (sumPxI / sumI) / scale + offset; //sumPxI / sumI is in pixel-coordinates, outCOG is in physical coordinates: (px/scale + offset = phys)
        }
        else
        {
            outCOG[pixelCnt] = std::numeric_limits<ito::float64>::quiet_NaN();
        }    
    }

    return ito::retOk;

}

//----------------------------------------------------------------------------------------------------------------------------------
const char *DataObjectArithmetic::getPercentageThresholdDoc = "analyzes all values in the given data object and returns the value, which is at a given percentage in the sorted value list.";

/*static*/ ito::RetVal DataObjectArithmetic::getPercentageThresholdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    paramsMand->clear();
    paramsMand->append( ito::Param("data", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "valid non-complex data object") );
    paramsMand->append( ito::Param("percentage", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 100.0, 50.0, "percentage value [0.0,100.0]") );
    
    paramsOut->append( ito::Param("threshold", ito::ParamBase::Double | ito::ParamBase::Out, NULL, "threshold value (NaN if data object was empty or only contained invalid values)") );

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DataObjectArithmetic::getPercentageThreshold(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::DataObject *data = paramsMand->at(0).getVal<ito::DataObject*>();
    double percentage = paramsMand->at(1).getVal<double>();
    double threshold = std::numeric_limits<double>::quiet_NaN();
    ito::RetVal retval;

    if (!data)
    {
        retval += ito::RetVal(ito::retError,0,"input data is NULL");
    }
    else
    {
        switch (data->getType())
        {
        case ito::tUInt8:
            retval += getPercentageThresholdHelper<ito::uint8>(data, percentage, threshold);
            break;
        case ito::tInt8:
            retval += getPercentageThresholdHelper<ito::int8>(data, percentage, threshold);
            break;
        case ito::tUInt16:
            retval += getPercentageThresholdHelper<ito::uint16>(data, percentage, threshold);
            break;
        case ito::tInt16:
            retval += getPercentageThresholdHelper<ito::int16>(data, percentage, threshold);
            break;
        case ito::tUInt32:
            retval += getPercentageThresholdHelper<ito::uint32>(data, percentage, threshold);
            break;
        case ito::tInt32:
            retval += getPercentageThresholdHelper<ito::int32>(data, percentage, threshold);
            break;
        case ito::tFloat32:
            retval += getPercentageThresholdHelper<ito::float32>(data, percentage, threshold);
            break;
        case ito::tFloat64:
            retval += getPercentageThresholdHelper<ito::float64>(data, percentage, threshold);
            break;
        default:
            retval += ito::RetVal(ito::retError,0,"not implemented for complex64 or complex128");
        }
    }

    (*paramsOut)[0].setVal<double>(threshold);
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> /*static*/ ito::RetVal DataObjectArithmetic::getPercentageThresholdHelper(const ito::DataObject *dObj, double percentage, double &value)
{
    std::vector<_Tp> values;
    int numValues = 0;
    int planes = dObj->calcNumMats();
    int dims = dObj->getDims();
    int m = dObj->getSize(dims-2);
    int n = dObj->getSize(dims-1);

    if (std::numeric_limits<_Tp>::is_integer)
    {
        numValues = planes * m * n;
        values.resize(numValues);
        _Tp *vData = values.data();
        _Tp *rowPtr;

        //copies the entire content of the integer data object to the values vector.
        for (int p = 0; p < planes; ++p)
        {
            for (int mi = 0; mi < m; ++mi)
            {
                rowPtr = (_Tp*)(dObj->rowPtr(p, mi));
                memcpy(vData, rowPtr, sizeof(_Tp) * n);
                vData += n; 
            }
        }
    }
    else
    {
        values.reserve(planes * m * n);
        _Tp *rowPtr;

        for (int p = 0; p < planes; ++p)
        {
            for (int mi = 0; mi < m; ++mi)
            {
                rowPtr = (_Tp*)(dObj->rowPtr(p, mi));
                for (int ni = 0; ni < n; ++ni)
                {
                    if (ito::dObjHelper::isFinite(rowPtr[ni]))
                    {
                        values.push_back( rowPtr[ni] );
                        numValues++;
                    }
                }
            }
        }
    }

    if(numValues == 0 && (planes*m*n) > 0)
    {
        value = std::numeric_limits<double>::quiet_NaN();
        return ito::RetVal(ito::retWarning,0,"no valid values encountered");
    }
    else if(numValues == 0)
    {
        value = std::numeric_limits<double>::quiet_NaN();
        return ito::retOk;
    }
        
    if (percentage <= 50.0)
    {
        int selValue = floor( percentage * (double)numValues / 100.0 );
        selValue = std::max(0, selValue);
        selValue = std::min(numValues-1, selValue);
        std::nth_element (values.begin(), values.begin() + selValue, values.end(), DataObjectArithmetic::cmpLT<_Tp>);
        value = values[selValue];
    }
    else
    {
        int selValue = floor( (100.0-percentage) * (double)numValues / 100.0 );
        selValue = std::max(0, selValue);
        selValue = std::min(numValues-1, selValue);
        std::nth_element (values.begin(), values.begin() + selValue, values.end(), DataObjectArithmetic::cmpGT<_Tp>);
        value = values[selValue];
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectArithmetic::~DataObjectArithmetic()
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(DataObjectArithmetic::maxValue, DataObjectArithmetic::singleDObjInputValueAndPositionOutParams, tr(maxValueDoc));
    m_filterList.insert("maxValue", filter);

    filter = new FilterDef(DataObjectArithmetic::minValue, DataObjectArithmetic::singleDObjInputValueAndPositionOutParams, tr(minValueDoc));
    m_filterList.insert("minValue", filter);

    filter = new FilterDef(DataObjectArithmetic::minMaxValue, DataObjectArithmetic::minMaxValueParams, tr(minMaxValueDoc));
    m_filterList.insert("minMaxValue", filter);

    filter = new FilterDef(DataObjectArithmetic::meanValue, DataObjectArithmetic::singleDObjInputInfParams, tr(meanValueDoc));
    m_filterList.insert("meanValue", filter);

    filter = new FilterDef(DataObjectArithmetic::centerOfGravity, DataObjectArithmetic::centerOfGravityParams, tr(centerOfGravityDoc));
    m_filterList.insert("centroidXY", filter);

    filter = new FilterDef(DataObjectArithmetic::centerOfGravity1Dim, DataObjectArithmetic::centerOfGravity1DimParams, tr(centerOfGravity1DimDoc));
    m_filterList.insert("centroid1D", filter);

    filter = new FilterDef(DataObjectArithmetic::devValue, DataObjectArithmetic::devValueParams, tr(devValueDoc));
    m_filterList.insert("deviationValue", filter);

    filter = new FilterDef(DataObjectArithmetic::areEqual, DataObjectArithmetic::doubleDObjInputParams, tr(areEqualDoc));
    m_filterList.insert("areEqual", filter);

    filter = new FilterDef(getPercentageThreshold, getPercentageThresholdParams, tr(getPercentageThresholdDoc));
    m_filterList.insert("getPercentageThreshold", filter);

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------