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
    DataObjectArithmetic* newInst = new DataObjectArithmetic();
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
ito::RetVal DataObjectArithmeticInterface::closeThisInst(ito::AddInBase **addInInst)
{
    if (*addInInst)
    {
        DataObjectArithmetic * thisInst = qobject_cast<DataObjectArithmetic*>(*addInInst);
        if(thisInst)
        {
            delete thisInst;
            int idx = m_InstList.indexOf(*addInInst);
            m_InstList.removeAt(idx);
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("plugin-instance cannot be converted to class DataObjectArithmetic. Close operation failed").toAscii().data());
        }
    }

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
Q_EXPORT_PLUGIN2(DataObjectArithmeticInterface, DataObjectArithmeticInterface)

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
        ito::Param param = Param("sourceImage", ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object for operation").toAscii().data());
        paramsMand->append(param);
        paramsOut->append( Param("result", ParamBase::Double | ParamBase::Out, 0.0, NULL, tr("result of calculation. This param can be int or double").toAscii().data()));
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::singleDObjInputInfParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = Param("sourceImage", ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object for operation").toAscii().data());
        paramsMand->append(param);

        param = ito::Param("ignoreInf", ParamBase::Int | ParamBase::In, 0, 1, 0, tr("source image data object for operation").toAscii().data());
        paramsOpt->append(param);

        paramsOut->append( Param("result", ParamBase::Double | ParamBase::Out, 0.0, NULL, tr("result of calculation. This param can be int or double").toAscii().data()));
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::singleDObjInputValueAndPositionOutParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = Param("sourceImage", ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object for operation").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("ignoreInf", ParamBase::Int | ParamBase::In, 0, 1, 0, tr("Ignore invalid-Values for floating point").toAscii().data());
        paramsOpt->append(param);
        paramsOut->append( Param("result", ParamBase::Double | ParamBase::Out, 0.0, NULL, tr("result of calculation. This param can be int or double").toAscii().data()));
        paramsOut->append( Param("plane", ParamBase::Int | ParamBase::Out, 0, NULL, tr("Index of the plane, which contains the result.").toAscii().data()));
        paramsOut->append( Param("y", ParamBase::Int | ParamBase::Out, 0, NULL, tr("Pixelindex in y-direction.").toAscii().data()));
        paramsOut->append( Param("x", ParamBase::Int | ParamBase::Out, 0, NULL, tr("Pixelindex in x-direction.").toAscii().data()));
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::doubleDObjInputParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = Param("sourceImage1", ParamBase::DObjPtr, NULL, tr("1. source image data object for operation").toAscii().data());
        paramsMand->append(param);
        param = Param("sourceImage2", ParamBase::DObjPtr, NULL, tr("2. source image data object for operation").toAscii().data());
        paramsMand->append(param);

        paramsOut->append( Param("result", ParamBase::Int | ParamBase::Out, 0, tr("0 if both data objects are not equal, else 1").toAscii().data()));
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
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NUL").toAscii().data());
    }

    if(dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error, object dimensions must be unequal zero").toAscii().data());
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
        retval += ito::RetVal(retError, 0, tr("data type not supported").toAscii().data());
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
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NUL").toAscii().data());
    }

    if(dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error, object dimensions must be unequal zero").toAscii().data());
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
        retval += ito::RetVal(retError, 0, tr("data type not supported").toAscii().data());
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
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object for operation").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("ignoreInf", ParamBase::Int | ParamBase::In, 0, 1, 0, tr("Ignore invalid-Values for floating point").toAscii().data());
        paramsOpt->append(param);
        param = ito::Param("complexHandling", ParamBase::Int | ParamBase::In, 0, 3, 0, tr("Switch complex handling, 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value").toAscii().data());
        paramsOpt->append(param);
        paramsOut->append( Param("minimum", ParamBase::Double | ParamBase::Out, 0.0, NULL, tr("Minimal value, this parameter be int or double").toAscii().data()));
        paramsOut->append( Param("planeMin", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Index of the plane, which contains the result.").toAscii().data()));
        paramsOut->append( Param("yMin", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Pixelindex in y-direction.").toAscii().data()));
        paramsOut->append( Param("xMin", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Pixelindex in x-direction.").toAscii().data()));
        paramsOut->append( Param("maximum", ParamBase::Double | ParamBase::Out, 0.0, NULL, tr("Maximum value, this parameter. This param can be int or double").toAscii().data()));
        paramsOut->append( Param("planeMax", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Index of the plane, which contains the result.").toAscii().data()));
        paramsOut->append( Param("yMax", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Pixelindex in y-direction.").toAscii().data()));
        paramsOut->append( Param("xMax", ParamBase::Int | ParamBase::Out, 0.0, NULL, tr("Pixelindex in x-direction.").toAscii().data()));
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
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NUL").toAscii().data());
    }

    if(dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error, object dimensions must be unequal zero").toAscii().data());
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
        retval += ito::RetVal(retError, 0, tr("data type not supported").toAscii().data());
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
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NUL").toAscii().data());
    }

    if(dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error, object dimensions must be unequal zero").toAscii().data());
    }

// new Version using the SDK-minValueHelper


    ito::float64 result = 0.0;
    bool toogleInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;
    

    retval += ito::dObjHelper::meanValue(dObj, result, toogleInf);

    (*paramsOut)[0] = ParamBase("result",ParamBase::Double | ParamBase::Out, static_cast<double>(result));

    return retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::devValueParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ParamBase::In, NULL, tr("source image data object for operation").toAscii().data());
        paramsMand->append(param);
        param = Param("flag", ito::ParamBase::Int | ParamBase::In, 0, 1, 0, tr("Toggles the calculation mode of standard deviation over N or N-1 elements").toAscii().data());
        paramsOpt->append(param);
        param = Param("ignoreInf", ParamBase::Int | ParamBase::In, 0, 1, 1, tr("Ignore invalid-Values for floating point").toAscii().data());
        paramsOpt->append(param);

        paramsOut->append( ito::Param("mean", ito::ParamBase::Double | ito::ParamBase::Out, 0.0, NULL, tr("mean result").toAscii().data()));
        paramsOut->append( ito::Param("dev", ito::ParamBase::Double | ito::ParamBase::Out, 0.0, NULL, tr("deviation result").toAscii().data()));
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
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toAscii().data());
    }


    ito::float64 meanResult = 0.0;
    ito::float64 devResult = 0.0;
    
    int flag = (*paramsOpt)[0].getVal<int>();
    bool toogleInf = (*paramsOpt)[1].getVal<int>() > 0 ? true : false;

    if(dObj->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error, object dimensions must be unequal zero").toAscii().data());
    }

// new Version using the SDK-minValueHelper
  
    retval += ito::dObjHelper::devValue(dObj, flag, meanResult, devResult, toogleInf);

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
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toAscii().data());
    }

    ito::DataObject *dObj2 = static_cast<ito::DataObject*>( (*paramsMand)[1].getVal<void*>() );
    if(dObj2 == NULL)
    {
        (*paramsOut)[0].setVal<int>(0);
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toAscii().data());
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
            return ito::RetVal(ito::retError, 0, tr("type not supported").toAscii().data());
    
    }
    
    (*paramsOut)[0].setVal<int>(1);

    return retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::centerOfGravityParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr, NULL, tr("source image data object for operation").toAscii().data());
        paramsMand->append(param);
        param = Param("lowTreshold", ito::ParamBase::Double, -1*std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), 0.0, tr("Ingnore values lower than lowTreshold").toAscii().data());
        paramsOpt->append(param);
        param = Param("highTreshold", ito::ParamBase::Double, -1*std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), 0.0, tr("Ingnore values above highTreshold").toAscii().data());
        paramsOpt->append(param);

        paramsOut->append( ito::Param("cYI", ito::ParamBase::Double | ito::ParamBase::Out, std::numeric_limits<ito::float64>::signaling_NaN(), NULL, tr("y-Coordinate of COG (index)").toAscii().data()));
        paramsOut->append( ito::Param("cXI", ito::ParamBase::Double | ito::ParamBase::Out, std::numeric_limits<ito::float64>::signaling_NaN(), NULL, tr("x-Coordinate of COG (index)").toAscii().data()));
        paramsOut->append( ito::Param("cY", ito::ParamBase::Double | ito::ParamBase::Out, std::numeric_limits<ito::float64>::signaling_NaN(), NULL, tr("y-Coordinate of COG (unit)").toAscii().data()));
        paramsOut->append( ito::Param("cX", ito::ParamBase::Double | ito::ParamBase::Out, std::numeric_limits<ito::float64>::signaling_NaN(), NULL, tr("x-Coordinate of COG (unit)").toAscii().data()));
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
const char* DataObjectArithmetic::centerOfGravityDoc = "This filter calculates the center of gravity within a image plane. \n\
The return value contains the column and row position in pixel and physical coordinates.\n\
\n\
The filter do not work with RGBA32, Complex64 and Complex128, but with all other data-types.\n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal DataObjectArithmetic::centroidHelper(cv::Mat *mat, const _Tp lowTreshold, const _Tp highTreshold, ito::float64 &xCOG, ito::float64 &yCOG)
{
    ito::int32 x, y;
    ito::float64 val = 0.0, sumva = 0.0, sumXv = 0.0, sumYv = 0.0;
    _Tp *pValue = NULL;
    
    if(std::numeric_limits<_Tp>::is_exact && lowTreshold == std::numeric_limits<_Tp>::min() && highTreshold == std::numeric_limits<_Tp>::max())
    {
        for(y = 0; y < mat->rows; y++)
        {
            pValue = mat->ptr<_Tp>(y);
            for(x = 0; x < mat->cols; x++)
            {
                val = (ito::float64)pValue[x];
                sumva = sumva + val;
                sumXv = sumXv + val * x;
                sumYv = sumYv + val * y;                        
            }
        }
    }
    else
    {
        for(y = 0; y < mat->rows; y++)
        {
            pValue = mat->ptr<_Tp>(y);
            for(x = 0; x < mat->cols; x++)
            {
                if(ito::dObjHelper::isFinite<_Tp>(pValue[x]) && pValue[x] > lowTreshold && pValue[x] < highTreshold)
                {
                    val = (ito::float64)pValue[x];
                    sumva = sumva + val;
                    sumXv = sumXv + val * x;
                    sumYv = sumYv + val * y;
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
        xCOG = std::numeric_limits<ito::float64>::signaling_NaN();
        yCOG = std::numeric_limits<ito::float64>::signaling_NaN();
    }

    return ito::retOk;

}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::centerOfGravity(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObj = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );
    if(dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toAscii().data());
    }
    if(dObj->getDims() < 1)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is not initialized").toAscii().data());    
    }

    ito::float64 cx = std::numeric_limits<ito::float64>::signaling_NaN();
    ito::float64 cy = std::numeric_limits<ito::float64>::signaling_NaN();
    ito::float64 cxPhys = std::numeric_limits<ito::float64>::signaling_NaN();
    ito::float64 cyPhys = std::numeric_limits<ito::float64>::signaling_NaN();

    ito::float64 lowTresHold = (*paramsOpt)[0].getVal<ito::float64>();
    ito::float64 highTresHold = (*paramsOpt)[1].getVal<ito::float64>();

    retval += ito::dObjHelper::verifyDataObjectType(dObj, "sourceImage", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if(dObj->getDims() > 2)
    {
        for(int i = 0; i < dObj->getDims() - 2; i++)
        {
            if(dObj->getSize(i) > 1)
            {
                return ito::RetVal(ito::retError, 0, tr("Error: source image must not have multiple planes").toAscii().data());  
            }
        }    
    }    


    if(!retval.containsError())
    {
        cv::Mat *plane = (cv::Mat*)(dObj->get_mdata()[dObj->seekMat(0)]);

        switch(dObj->getType())
        {
            case ito::tInt8:
                centroidHelper<ito::int8>(plane, cv::saturate_cast<ito::int8>(lowTresHold), cv::saturate_cast<ito::int8>(highTresHold), cx, cy);
            break;
            case ito::tUInt8:
                centroidHelper<ito::uint8>(plane, cv::saturate_cast<ito::uint8>(lowTresHold), cv::saturate_cast<ito::uint8>(highTresHold), cx, cy);
            break;
            case ito::tInt16:
                centroidHelper<ito::int16>(plane, cv::saturate_cast<ito::int16>(lowTresHold), cv::saturate_cast<ito::int16>(highTresHold), cx, cy);
            break;
            case ito::tUInt16:
                centroidHelper<ito::uint16>(plane, cv::saturate_cast<ito::uint16>(lowTresHold), cv::saturate_cast<ito::uint16>(highTresHold), cx, cy);
            break;
            case ito::tInt32:
                centroidHelper<ito::int32>(plane, cv::saturate_cast<ito::int32>(lowTresHold), cv::saturate_cast<ito::int32>(highTresHold), cx, cy);
            break;
            case ito::tFloat32:
                centroidHelper<ito::float32>(plane, cv::saturate_cast<ito::float32>(lowTresHold), cv::saturate_cast<ito::float32>(highTresHold), cx, cy);
            break;
            case ito::tFloat64:
                centroidHelper<ito::float64>(plane, cv::saturate_cast<ito::float64>(lowTresHold), cv::saturate_cast<ito::float64>(highTresHold), cx, cy);
            break;
            default:
                return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented for phase shifting evaluation").toAscii().data());
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
const char* DataObjectArithmetic::centerOfGravity1DimDoc = "This filter calculates the center of gravity along y or x direction. \n\
\n\
The filter do not work with RGBA32, Complex64 and Complex128, but with all other data-types\n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::centerOfGravity1DimParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("sourceStack", ito::ParamBase::DObjPtr, NULL, tr("source image data object for operation").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("cogPlane", ito::ParamBase::DObjPtr, NULL, tr("destination image data object for operation, will contain evaluated COG").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("intPlane", ito::ParamBase::DObjPtr, NULL, tr("destination image data object for operation, will contain maximal Intensity").toAscii().data());
        paramsMand->append(param);
        param = Param("lowTreshold", ito::ParamBase::Double, -1*std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), 0.0, tr("Ingnore values lower than lowTreshold").toAscii().data());
        paramsOpt->append(param);
        param = Param("dynamicTreshold", ito::ParamBase::Double, 0.0, 0.999, 0.5, tr("Dynamic treshold, e.g. half maximum").toAscii().data());
        paramsOpt->append(param);
        param = Param("columnWise", ito::ParamBase::Int, 0, 1, 0, tr("Evaluate along Columns").toAscii().data());
        paramsOpt->append(param);
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal DataObjectArithmetic::centroidHelperFor1D(cv::Mat *inMat, cv::Mat *outCOG, cv::Mat *outINT, const _Tp lowTreshold, const ito::float64 dynamicTreshold, const ito::float64 scale, bool alongCols)
{       
    ito::uint32 stepEvalInMat = 1;
    ito::uint32 stepAddrInMat = 1;
    ito::uint32 stepOutMat = 1;
    ito::uint32 cogsToCalc = 0;
    ito::uint32 pixelToEval = 0;
    ito::uint32 cogCnt = 0;
    ito::uint32 pixelCnt = 0;
    
    _Tp *pInValue = NULL;
    _Tp *pDstInt = NULL;
    ito::float64 *pDstValue = NULL;

    if(alongCols)
    {
        cogsToCalc = inMat->rows;
        pixelToEval =  inMat->cols;  
        stepEvalInMat = (ito::uint32)inMat->step1();
        stepAddrInMat = 1;
        //stepOutMat = outCOG->step1();
        stepOutMat = 1;

    }
    else
    {
        cogsToCalc = inMat->cols;
        pixelToEval =  inMat->rows;
        stepEvalInMat = 1;
        stepAddrInMat = (ito::uint32)inMat->step1();
        stepOutMat = 1;
    }

    _Tp typeMax = 0;
    _Tp typeMin = 0;

    if(std::numeric_limits<_Tp>::is_exact)
    {
        typeMax = std::numeric_limits<_Tp>::max();
        typeMin = std::numeric_limits<_Tp>::min();    
    }
    else
    {
        typeMax = std::numeric_limits<_Tp>::max();
        typeMin = -std::numeric_limits<_Tp>::max();    
    }

    bool runDynamic = dynamicTreshold > 0.0 ? true : false;
    bool runWithOutTresHold = false;
    
    for(pixelCnt = 0; pixelCnt < pixelToEval; pixelCnt++)
    {
        ito::float64 sumI = 0.0, sumPxI = 0.0;
        _Tp maxVal = typeMin;
        _Tp minVal = typeMax;
        _Tp val = 0;

        _Tp sw = typeMin;

        pInValue  = inMat->ptr<_Tp>(0) + stepAddrInMat * pixelCnt;
        pDstInt   = outINT->ptr<_Tp>(0) + stepOutMat * pixelCnt;
        pDstValue = outCOG->ptr<ito::float64>(0) + stepOutMat * pixelCnt;


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

            pInValue = pInValue + stepEvalInMat;
        }
        pInValue = inMat->ptr<_Tp>(0) + stepAddrInMat * pixelCnt;
        
        *pDstInt = maxVal;

        if((maxVal-minVal) > lowTreshold)
        {
            if(runDynamic)
            {
                sw = cv::saturate_cast<_Tp>(((ito::float64)maxVal + (ito::float64)minVal) * dynamicTreshold); 

                for(cogCnt = 0; cogCnt < cogsToCalc; cogCnt++)
                {
                    val = (ito::float64)(*pInValue-sw);
                    if(val > 0.0)
                    {
                        
                        sumI = sumI + val;
                        sumPxI = sumPxI + val * cogCnt;                        
                    }
                    pInValue = pInValue + stepEvalInMat;
                }

            }
            else
            {
                for(cogCnt = 0; cogCnt < cogsToCalc; cogCnt++)
                 {
                    val = (ito::float64)(*pInValue);
                    sumI = sumI + val;
                    sumPxI = sumPxI + val * cogCnt;    
                    pInValue = pInValue + stepEvalInMat;
                }
            }
        }

        if(ito::dObjHelper::isNotZero<ito::float64>(sumI))
        {
            *pDstValue = sumPxI / sumI * scale;    
        }
        else
        {
            *pDstValue = std::numeric_limits<ito::float64>::signaling_NaN();
        }
    
    }

    return ito::retOk;

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
    ito::DataObject *dObjIN = static_cast<ito::DataObject*>( (*paramsMand)[0].getVal<void*>() );
    if(dObjIN == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toAscii().data());
    }
    if(dObjIN->getDims() < 1)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is not initialized").toAscii().data());    
    }
    ito::DataObject *dObjCogOut = static_cast<ito::DataObject*>( (*paramsMand)[1].getVal<void*>() );
    if(dObjCogOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: destination COG image is NULL").toAscii().data());
    }
    ito::DataObject *dObjIntOut = static_cast<ito::DataObject*>( (*paramsMand)[2].getVal<void*>() );
    if(dObjIntOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: destination Int image is NULL").toAscii().data());
    }

    if(dObjCogOut == dObjIntOut)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: cogPlane and intPlane has the same pointer address").toAscii().data());
    }

    ito::float64 lowTresHold = (*paramsOpt)[0].getVal<ito::float64>();
    ito::float64 dynTresHold = (*paramsOpt)[1].getVal<ito::float64>();
    bool columnWise = (*paramsOpt)[2].getVal<ito::int32>() > 0 ? true: false;

    ito::float64 scaleVert = 1.0;   // Scale along depth ScanAxis

    retval += ito::dObjHelper::verify3DDataObject(dObjIN, "sourceImage", 1, 10000, 1, 10000, 1, 10000, 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if(retval.containsError())
    {
        return retval;
    }

    ito::uint32 sizeZ =  dObjIN->getSize(0);
    ito::uint32 sizeY =  dObjIN->getSize(1);
    ito::uint32 sizeX =  dObjIN->getSize(2);

    ito::RetVal retvalTemp = ito::retOk;
    
    if(columnWise)
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(dObjCogOut, "destinationCOG", sizeZ, sizeZ, sizeX, sizeX, 1, ito::tFloat64);
    }
    else
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(dObjCogOut, "destinationCOG", sizeZ, sizeZ, sizeY, sizeY, 1, ito::tFloat64);
    }

    if(retvalTemp.containsError())
    {
        if(columnWise)
        {
            scaleVert = dObjIN->getAxisScale(dObjIN->getDims()-2);
            *dObjCogOut = ito::DataObject(sizeZ, sizeX, ito::tFloat64);
        }
        else
        {
            scaleVert = dObjIN->getAxisScale(dObjIN->getDims()-1);
            *dObjCogOut = ito::DataObject(sizeZ, sizeY, ito::tFloat64);
        }
    }

    if(columnWise)
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(dObjIntOut, "destinationINT", sizeZ, sizeZ, sizeX, sizeX, 1, dObjIN->getType());
    }
    else
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(dObjIntOut, "destinationINT", sizeZ, sizeZ, sizeY, sizeY, 1, dObjIN->getType());
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
        cv::Mat *planeIn = NULL;
        cv::Mat *planeCogOut = (cv::Mat*)(dObjCogOut->get_mdata()[0]);
        cv::Mat *planeIntOut = (cv::Mat*)(dObjIntOut->get_mdata()[0]);

        switch(dObjIN->getType())
        {
            case ito::tInt8:
                for(ito::uint32 i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = (cv::Mat*)(dObjIN->get_mdata()[dObjIN->seekMat(i)]);
                    sliceCOG = planeCogOut->rowRange(i,i+1);
                    sliceInt = planeIntOut->rowRange(i,i+1);
                    centroidHelperFor1D<ito::int8>(planeIn, &sliceCOG, &sliceInt, cv::saturate_cast<ito::int8>(lowTresHold), dynTresHold, scaleVert, columnWise);
                }
            break;
            case ito::tUInt8:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = (cv::Mat*)(dObjIN->get_mdata()[dObjIN->seekMat(i)]);
                    sliceCOG = planeCogOut->rowRange(i,i+1);
                    sliceInt = planeIntOut->rowRange(i,i+1);
                    centroidHelperFor1D<ito::uint8>(planeIn, &sliceCOG, &sliceInt, cv::saturate_cast<ito::uint8>(lowTresHold), dynTresHold, scaleVert, columnWise);
                }
            break;
            case ito::tInt16:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = (cv::Mat*)(dObjIN->get_mdata()[dObjIN->seekMat(i)]);
                    sliceCOG = planeCogOut->rowRange(i,i+1);
                    sliceInt = planeIntOut->rowRange(i,i+1);
                    centroidHelperFor1D<ito::int16>(planeIn, &sliceCOG, &sliceInt, cv::saturate_cast<ito::int16>(lowTresHold), dynTresHold, scaleVert, columnWise);
                }
            break;
            case ito::tUInt16:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = (cv::Mat*)(dObjIN->get_mdata()[dObjIN->seekMat(i)]);
                    sliceCOG = planeCogOut->rowRange(i,i+1);
                    sliceInt = planeIntOut->rowRange(i,i+1);
                    centroidHelperFor1D<ito::uint16>(planeIn, &sliceCOG, &sliceInt, cv::saturate_cast<ito::uint16>(lowTresHold), dynTresHold, scaleVert, columnWise);
                }
            break;
            case ito::tInt32:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = (cv::Mat*)(dObjIN->get_mdata()[dObjIN->seekMat(i)]);
                    sliceCOG = planeCogOut->rowRange(i,i+1);
                    sliceInt = planeIntOut->rowRange(i,i+1);
                    centroidHelperFor1D<ito::int32>(planeIn, &sliceCOG, &sliceInt, cv::saturate_cast<ito::int32>(lowTresHold), dynTresHold, scaleVert, columnWise);
                }
            break;
            case ito::tFloat32:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = (cv::Mat*)(dObjIN->get_mdata()[dObjIN->seekMat(i)]);
                    sliceCOG = planeCogOut->rowRange(i,i+1);
                    sliceInt = planeIntOut->rowRange(i,i+1);
                    centroidHelperFor1D<ito::float32>(planeIn, &sliceCOG, &sliceInt, cv::saturate_cast<ito::float32>(lowTresHold), dynTresHold, scaleVert, columnWise);
                }
            break;
            case ito::tFloat64:
                for(ito::uint32  i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
                {
                    planeIn = (cv::Mat*)(dObjIN->get_mdata()[dObjIN->seekMat(i)]);
                    sliceCOG = planeCogOut->rowRange(i,i+1);
                    sliceInt = planeIntOut->rowRange(i,i+1);
                    centroidHelperFor1D<ito::float64>(planeIn, &sliceCOG, &sliceInt, cv::saturate_cast<ito::float64>(lowTresHold), dynTresHold, scaleVert, columnWise);
                }
            break;
            default:
                return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented for phase shifting evaluation").toAscii().data());
        }
    }

    if(!retval.containsError())
    {
        bool test;
        if(columnWise)
        {
            
            dObjCogOut->setAxisScale(dObjCogOut->getDims()-1, dObjIN->getAxisScale(dObjIN->getDims()-1));
            dObjCogOut->setAxisOffset(dObjCogOut->getDims()-1, dObjIN->getAxisOffset(dObjIN->getDims()-1));
            dObjCogOut->setAxisUnit(dObjCogOut->getDims()-1, dObjIN->getAxisUnit(dObjIN->getDims()-1, test));
            dObjCogOut->setAxisDescription(dObjCogOut->getDims()-1, dObjIN->getAxisDescription(dObjIN->getDims()-1, test));

            dObjCogOut->setValueUnit( dObjIN->getAxisDescription(dObjIN->getDims() - 2, test) );

            dObjIntOut->setAxisScale(dObjIntOut->getDims()-1, dObjIN->getAxisScale(dObjIN->getDims()-1));
            dObjIntOut->setAxisOffset(dObjIntOut->getDims()-1, dObjIN->getAxisOffset(dObjIN->getDims()-1));
            dObjIntOut->setAxisUnit(dObjIntOut->getDims()-1, dObjIN->getAxisUnit(dObjIN->getDims()-1, test));
            dObjIntOut->setAxisDescription(dObjIntOut->getDims()-1, dObjIN->getAxisDescription(dObjIN->getDims()-1, test));

        }
        else
        {
            dObjCogOut->setAxisScale(dObjCogOut->getDims()-1, dObjIN->getAxisScale(dObjIN->getDims()-2));
            dObjCogOut->setAxisOffset(dObjCogOut->getDims()-1, dObjIN->getAxisOffset(dObjIN->getDims()-2));
            dObjCogOut->setAxisUnit(dObjCogOut->getDims()-1, dObjIN->getAxisUnit(dObjIN->getDims()-2, test));
            dObjCogOut->setAxisDescription(dObjCogOut->getDims()-1, dObjIN->getAxisDescription(dObjIN->getDims()-2, test));

            dObjCogOut->setValueUnit( dObjIN->getAxisDescription(dObjIN->getDims() - 1, test) );

            dObjIntOut->setAxisScale(dObjIntOut->getDims()-1, dObjIN->getAxisScale(dObjIN->getDims()-2));
            dObjIntOut->setAxisOffset(dObjIntOut->getDims()-1, dObjIN->getAxisOffset(dObjIN->getDims()-2));
            dObjIntOut->setAxisUnit(dObjIntOut->getDims()-1, dObjIN->getAxisUnit(dObjIN->getDims()-2, test));
            dObjIntOut->setAxisDescription(dObjIntOut->getDims()-1, dObjIN->getAxisDescription(dObjIN->getDims()-2, test));

        }
        dObjCogOut->setAxisScale(dObjCogOut->getDims()-2, dObjIN->getAxisScale(dObjIN->getDims()-3));
        dObjCogOut->setAxisOffset(dObjCogOut->getDims()-2, dObjIN->getAxisOffset(dObjIN->getDims()-3));
        dObjCogOut->setAxisUnit(dObjCogOut->getDims()-2, dObjIN->getAxisUnit(dObjIN->getDims()-3, test));
        dObjCogOut->setAxisDescription(dObjCogOut->getDims()-2, dObjIN->getAxisDescription(dObjIN->getDims()-3, test));
        dObjCogOut->setValueDescription("Center of Gravity");

        dObjIntOut->setAxisScale(dObjIntOut->getDims()-2, dObjIN->getAxisScale(dObjIN->getDims()-3));
        dObjIntOut->setAxisOffset(dObjIntOut->getDims()-2, dObjIN->getAxisOffset(dObjIN->getDims()-3));
        dObjIntOut->setAxisUnit(dObjIntOut->getDims()-2, dObjIN->getAxisUnit(dObjIN->getDims()-3, test));
        dObjIntOut->setAxisDescription(dObjIntOut->getDims()-2, dObjIN->getAxisDescription(dObjIN->getDims()-3, test));

        dObjIntOut->setValueDescription("intensity");
        dObjIntOut->setValueUnit("a.u.");

        dObjIN->copyTagMapTo(*dObjCogOut);
        dObjIN->copyTagMapTo(*dObjIntOut);

        if(columnWise)
        {
            QString protocol("Center of gravity evaluated columnwise from intensity stack, ");
            protocol.append("minTreshold: ");
            protocol.append(QString::number(lowTresHold));
            protocol.append("dynTresHold: ");
            protocol.append(QString::number(dynTresHold));

            dObjCogOut->addToProtocol(protocol.toAscii().data());
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
    double threshold = std::numeric_limits<double>::signaling_NaN();
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
        value = std::numeric_limits<double>::signaling_NaN();
        return ito::RetVal(ito::retWarning,0,"no valid values encountered");
    }
    else if(numValues == 0)
    {
        value = std::numeric_limits<double>::signaling_NaN();
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