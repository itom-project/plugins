/* ********************************************************************
    Plugin "dataobjectarithmetic" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2021, Institut fuer Technische Optik (ITO),
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

#include "dataobjectarithmetic.h"

#include "common/numeric.h"
#include <QtCore/QtPlugin>
#include <qnumeric.h>
#include <qstringlist.h>
#include <qthread.h>
#include <qvariant.h>

#include "common/helperCommon.h"

#include "DataObject/dataObjectFuncs.h"

#include "gitVersion.h"
#include "pluginVersion.h"

#ifdef USEOPENMP
#include <omp.h>
#endif

using namespace ito;

//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmeticInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(DataObjectArithmetic)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmeticInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(DataObjectArithmetic)
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
DataObjectArithmeticInterface::DataObjectArithmeticInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("DataObjectArithmetic");

    m_description = QObject::tr("Operations and arithmetic calculations of dataObject.");
    m_detaildescription = QObject::tr(
        "This plugin provides several arithmetic calculations for dataObject. These are for instance: \n\
- min- or maximum value\n\
- centroid along dimensions or inplane \n\
\n\
This plugin does not have any unusual dependencies.");

    m_author = "W. Lyda, M. Gronle, ITO, University Stuttgart";
    m_license = QObject::tr("LGPL");
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_aboutThis = tr(GITVERSION);
}

//-------------------------------------------------------------------------------------
DataObjectArithmeticInterface::~DataObjectArithmeticInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//-------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
DataObjectArithmetic::DataObjectArithmetic() : AddInAlgo()
{
}

//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::singleDObjInputParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = Param(
            "sourceImage",
            ParamBase::DObjPtr | ParamBase::In,
            NULL,
            tr("source image data object for operation").toLatin1().data());
        paramsMand->append(param);
        paramsOut->append(Param(
            "result",
            ParamBase::Double | ParamBase::Out,
            0.0,
            NULL,
            tr("result of calculation. This param can be int or double").toLatin1().data()));
    }
    return retval;
}
//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::singleDObjInputInfParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);

    if (!retval.containsError())
    {
        ito::Param param = Param(
            "sourceImage",
            ParamBase::DObjPtr | ParamBase::In,
            NULL,
            tr("source image data object for operation").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param(
            "ignoreInf",
            ParamBase::Int | ParamBase::In,
            0,
            1,
            0,
            tr("source image data object for operation").toLatin1().data());
        paramsOpt->append(param);

        paramsOut->append(Param(
            "result",
            ParamBase::Double | ParamBase::Out,
            0.0,
            NULL,
            tr("result of calculation. This param can be int or double").toLatin1().data()));
    }

    return retval;
}

//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::singleDObjInputValueAndPositionOutParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);

    if (!retval.containsError())
    {
        ito::Param param = Param(
            "sourceImage",
            ParamBase::DObjPtr | ParamBase::In,
            NULL,
            tr("source image data object for operation").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "ignoreInf",
            ParamBase::Int | ParamBase::In,
            0,
            1,
            0,
            tr("Ignore invalid-Values for floating point").toLatin1().data());
        paramsOpt->append(param);
        paramsOut->append(Param(
            "result",
            ParamBase::Double | ParamBase::Out,
            0.0,
            NULL,
            tr("result of calculation. This param can be int or double").toLatin1().data()));
        paramsOut->append(Param(
            "plane",
            ParamBase::Int | ParamBase::Out,
            0,
            NULL,
            tr("Index of the plane, which contains the result.").toLatin1().data()));
        paramsOut->append(Param(
            "y",
            ParamBase::Int | ParamBase::Out,
            0,
            NULL,
            tr("Pixelindex in y-direction.").toLatin1().data()));
        paramsOut->append(Param(
            "x",
            ParamBase::Int | ParamBase::Out,
            0,
            NULL,
            tr("Pixelindex in x-direction.").toLatin1().data()));
    }

    return retval;
}

//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::doubleDObjInputParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);

    if (!retval.containsError())
    {
        ito::Param param = Param(
            "sourceImage1",
            ParamBase::DObjPtr | ParamBase::In,
            NULL,
            tr("1. source image data object for operation").toLatin1().data());
        paramsMand->append(param);
        param = Param(
            "sourceImage2",
            ParamBase::DObjPtr | ParamBase::In,
            NULL,
            tr("2. source image data object for operation").toLatin1().data());
        paramsMand->append(param);

        paramsOut->append(Param(
            "result",
            ParamBase::Int | ParamBase::Out,
            0,
            tr("0 if both data objects are not equal, else 1").toLatin1().data()));
    }

    return retval;
}

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::minValueDoc = QObject::tr(
    "This filter calculates the global minimum value and its first location within the dataObject. \n\
\n\
The returned value will be an integer for all fixed-point data types and float for all floating point types. \n\
\n\
The filter is implemented for all data types besides RGBA32, Complex64 and Complex128\n\
\n");

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::minValue(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject* dObj = (*paramsMand)[0].getVal<const ito::DataObject*>();

    if (dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULLL").toLatin1().data());
    }

    if (dObj->getDims() < 2)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

    ito::float64 result = 0.0;
    ito::uint32 location[3] = {0, 0, 0};
    bool ignoreInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;

    retval += ito::dObjHelper::minValue(dObj, result, location, ignoreInf);

    switch (dObj->getType())
    {
    case tUInt8:
    case tInt8:
    case tUInt16:
    case tInt16:
    case tUInt32:
    case tInt32:
        ignoreInf = false;
        (*paramsOut)[0] =
            ParamBase("result", ParamBase::Int | ParamBase::Out, static_cast<int>(result));
        break;
    case tFloat32:
    case tFloat64:
    case tComplex64:
    case tComplex128:
        (*paramsOut)[0] =
            ParamBase("result", ParamBase::Double | ParamBase::Out, static_cast<double>(result));
        break;
    default:
        retval += ito::RetVal(retError, 0, tr("data type not supported").toLatin1().data());
        // outVals->clear();
    }

    (*paramsOut)[1] =
        ParamBase("plane", ParamBase::Int | ParamBase::Out, static_cast<int>(location[0]));
    (*paramsOut)[2] =
        ParamBase("y", ParamBase::Int | ParamBase::Out, static_cast<int>(location[1]));
    (*paramsOut)[3] =
        ParamBase("x", ParamBase::Int | ParamBase::Out, static_cast<int>(location[2]));

    return retval;
}

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::maxValueDoc = QObject::tr(
    "This filter calculates the global maximum value and its first location within the dataObject. \n\
\n\
The returned value will be an integer for all fixed-point data types and float for all floating point types. \n\
The global maximum of complex data types is defined to be the global maximum of all absolute values. \n\
\n\
The filter is implemented for all data types besides RGBA32");

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::maxValue(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject* dObj = (*paramsMand)[0].getVal<const ito::DataObject*>();

    if (dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toLatin1().data());
    }

    if (dObj->getDims() < 2)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

    ito::float64 result = 0.0;
    ito::uint32 location[3] = {0, 0, 0};

    bool ignoreInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;

    retval += ito::dObjHelper::maxValue(dObj, result, location, ignoreInf);

    switch (dObj->getType())
    {
    case tUInt8:
    case tInt8:
    case tUInt16:
    case tInt16:
    case tUInt32:
    case tInt32:
        ignoreInf = false;
        (*paramsOut)[0] =
            ParamBase("result", ParamBase::Int | ParamBase::Out, static_cast<int>(result));
        break;
    case tFloat32:
    case tFloat64:
    case tComplex64:
    case tComplex128:
        (*paramsOut)[0] =
            ParamBase("result", ParamBase::Double | ParamBase::Out, static_cast<double>(result));
        break;
    default:
        retval += ito::RetVal(retError, 0, tr("data type not supported").toLatin1().data());
        // outVals->clear();
    }

    (*paramsOut)[1] =
        ParamBase("plane", ParamBase::Int | ParamBase::Out, static_cast<int>(location[0]));
    (*paramsOut)[2] =
        ParamBase("y", ParamBase::Int | ParamBase::Out, static_cast<int>(location[1]));
    (*paramsOut)[3] =
        ParamBase("x", ParamBase::Int | ParamBase::Out, static_cast<int>(location[2]));

    return retval;
}

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::minMaxValueDoc = QObject::tr(
    "This filter calculates the minimal and maximal value and its first location within the dataObject. \n\
\n\
The returned values will be integer for all fixed-point data types or float for all floating point types. \n\
\n\
The filter does not work with RGBA32 but with all other datatypes.\n\
\n");

//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::minMaxValueParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "sourceImage",
            ito::ParamBase::DObjPtr | ParamBase::In,
            NULL,
            tr("source image data object for operation").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "ignoreInf",
            ParamBase::Int | ParamBase::In,
            0,
            1,
            0,
            tr("Ignore invalid-Values for floating point").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param(
            "complexHandling",
            ParamBase::Int | ParamBase::In,
            0,
            3,
            0,
            tr("Switch complex handling, 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: "
               "argument-Value")
                .toLatin1()
                .data());
        paramsOpt->append(param);
        paramsOut->append(Param(
            "minimum",
            ParamBase::Double | ParamBase::Out,
            0.0,
            NULL,
            tr("Minimal value, this parameter be int or double").toLatin1().data()));
        paramsOut->append(Param(
            "planeMin",
            ParamBase::Int | ParamBase::Out,
            0.0,
            NULL,
            tr("Index of the plane, which contains the result.").toLatin1().data()));
        paramsOut->append(Param(
            "yMin",
            ParamBase::Int | ParamBase::Out,
            0.0,
            NULL,
            tr("Pixelindex in y-direction.").toLatin1().data()));
        paramsOut->append(Param(
            "xMin",
            ParamBase::Int | ParamBase::Out,
            0.0,
            NULL,
            tr("Pixelindex in x-direction.").toLatin1().data()));
        paramsOut->append(Param(
            "maximum",
            ParamBase::Double | ParamBase::Out,
            0.0,
            NULL,
            tr("Maximum value, this parameter. This param can be int or double")
                .toLatin1()
                .data()));
        paramsOut->append(Param(
            "planeMax",
            ParamBase::Int | ParamBase::Out,
            0.0,
            NULL,
            tr("Index of the plane, which contains the result.").toLatin1().data()));
        paramsOut->append(Param(
            "yMax",
            ParamBase::Int | ParamBase::Out,
            0.0,
            NULL,
            tr("Pixelindex in y-direction.").toLatin1().data()));
        paramsOut->append(Param(
            "xMax",
            ParamBase::Int | ParamBase::Out,
            0.0,
            NULL,
            tr("Pixelindex in x-direction.").toLatin1().data()));
    }

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::minMaxValue(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject* dObj = (*paramsMand)[0].getVal<const ito::DataObject*>();

    if (dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toLatin1().data());
    }

    if (dObj->getDims() < 2)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

    ito::float64 minVal = 0.0;
    ito::float64 maxVal = 0.0;
    ito::uint32 locationMin[3] = {0, 0, 0};
    ito::uint32 locationMax[3] = {0, 0, 0};

    bool ignoreInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;
    int cmplxState = (*paramsOpt)[1].getVal<int>();

    retval += ito::dObjHelper::minMaxValue(
        dObj, minVal, locationMin, maxVal, locationMax, ignoreInf, cmplxState);

    switch (dObj->getType())
    {
    case tUInt8:
    case tInt8:
    case tUInt16:
    case tInt16:
    case tUInt32:
    case tInt32:
        ignoreInf = false;
        (*paramsOut)[0] =
            ParamBase("minimum", ParamBase::Int | ParamBase::Out, static_cast<int>(minVal));
        (*paramsOut)[4] =
            ParamBase("maximum", ParamBase::Int | ParamBase::Out, static_cast<int>(maxVal));
        break;
    case tFloat32:
    case tFloat64:
    case tComplex64:
    case tComplex128:
        (*paramsOut)[0] =
            ParamBase("minimum", ParamBase::Double | ParamBase::Out, static_cast<double>(minVal));
        (*paramsOut)[4] =
            ParamBase("maximum", ParamBase::Double | ParamBase::Out, static_cast<double>(maxVal));
        break;
    default:
        retval += ito::RetVal(retError, 0, tr("data type not supported").toLatin1().data());
        // outVals->clear();
    }

    (*paramsOut)[1] =
        ParamBase("planeMin", ParamBase::Int | ParamBase::Out, static_cast<int>(locationMin[0]));
    (*paramsOut)[2] =
        ParamBase("yMin", ParamBase::Int | ParamBase::Out, static_cast<int>(locationMin[1]));
    (*paramsOut)[3] =
        ParamBase("xMin", ParamBase::Int | ParamBase::Out, static_cast<int>(locationMin[2]));

    (*paramsOut)[5] =
        ParamBase("planeMax", ParamBase::Int | ParamBase::Out, static_cast<int>(locationMax[0]));
    (*paramsOut)[6] =
        ParamBase("yMax", ParamBase::Int | ParamBase::Out, static_cast<int>(locationMax[1]));
    (*paramsOut)[7] =
        ParamBase("xMax", ParamBase::Int | ParamBase::Out, static_cast<int>(locationMax[2]));

    return retval;
}

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::meanValueDoc =
    QObject::tr("This filter calculates the mean value within the dataObject. \n\
\n\
The return value containing the mean value of the dataObject.\n\
\n\
The filter is implemented for all datatypes besides RGBA32, Complex64 and Complex128\n\
\n");

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::meanValue(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject* dObj = (*paramsMand)[0].getVal<const ito::DataObject*>();

    if (dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toLatin1().data());
    }

    if (dObj->getDims() < 2)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

    ito::float64 result = 0.0;
    bool toggleInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;

    retval += ito::dObjHelper::meanValue(dObj, result, toggleInf);

    (*paramsOut)[0] =
        ParamBase("result", ParamBase::Double | ParamBase::Out, static_cast<double>(result));

    return retval;
}

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::medianValueDoc =
    QObject::tr("This method calculates the median value over all values in the data object. \n\
\n\
The returned median values is given as double.\n\
\n\
This method is implemented for all datatypes besides RGBA32, Complex64 and Complex128\n\
\n");

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::medianValue(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval;
    const ito::DataObject* dObj = (*paramsMand)[0].getVal<const ito::DataObject*>();

    if (dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULL").toLatin1().data());
    }

    if (dObj->getDims() < 2)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

    ito::float64 result = 0.0;
    bool toggleInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;

    retval += ito::dObjHelper::medianValue(dObj, result, toggleInf);

    (*paramsOut)[0] = ParamBase("result", ParamBase::Double | ParamBase::Out, result);

    return retval;
}


//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::devValueParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "sourceImage",
            ito::ParamBase::DObjPtr | ParamBase::In,
            NULL,
            tr("source image data object for operation").toLatin1().data());
        paramsMand->append(param);
        param = Param(
            "flag",
            ito::ParamBase::Int | ParamBase::In,
            0,
            1,
            0,
            tr("Toggles the calculation mode of standard deviation over N or N-1 elements")
                .toLatin1()
                .data());
        paramsOpt->append(param);
        param = Param(
            "ignoreInf",
            ParamBase::Int | ParamBase::In,
            0,
            1,
            1,
            tr("Ignore invalid-Values for floating point").toLatin1().data());
        paramsOpt->append(param);

        paramsOut->append(ito::Param(
            "mean",
            ito::ParamBase::Double | ito::ParamBase::Out,
            0.0,
            NULL,
            tr("mean result").toLatin1().data()));
        paramsOut->append(ito::Param(
            "dev",
            ito::ParamBase::Double | ito::ParamBase::Out,
            0.0,
            NULL,
            tr("deviation result").toLatin1().data()));
    }

    return retval;
}

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::devValueDoc = QObject::tr(
    "This method returns the arithmetic mean and the standard deviation of the given dataObject within its ROI.\n\
\n\
Use the optional argument 'flag' to choose between two formulas for the determination of the standard deviation. \n\
Either (flag = 0): \n\
\n\
\n    \\sqrt(\\sum{(x-xm)^2} / (n-1))\n\
\n\
or (flag = 1):\n\
\n\
\n    \\sqrt(\\sum{(x-xm)^2} / n)\n\
\n\
This method is implemented for all datatypes besides RGBA32, Complex64 and Complex128\n\
\n");

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::devValue(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject* dObj = (*paramsMand)[0].getVal<const ito::DataObject*>();

    if (dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULLL").toLatin1().data());
    }

    ito::float64 meanResult = 0.0;
    ito::float64 devResult = 0.0;

    int flag = (*paramsOpt)[0].getVal<int>();
    bool toggleInf = (*paramsOpt)[1].getVal<int>() > 0 ? true : false;

    if (dObj->getDims() < 2)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("Error, object dimensions must be unequal zero").toLatin1().data());
    }

    // new Version using the SDK-minValueHelper

    retval += ito::dObjHelper::devValue(dObj, flag, meanResult, devResult, toggleInf);

    (*paramsOut)[0] =
        ParamBase("mean", ParamBase::Double | ParamBase::Out, static_cast<double>(meanResult));
    (*paramsOut)[1] =
        ParamBase("dev", ParamBase::Double | ParamBase::Out, static_cast<double>(devResult));

    return retOk;
}

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::areEqualDoc =
    QObject::tr("Element-wise check if two dataObjects are equal. \n\
The filter returns 1 if all values of both objects are equal, else 0.\n\
\n\
The filter is implemented for all datatypes besides RGBA32, Complex64 and Complex128.");

//-------------------------------------------------------------------------------------
template <typename _Type>
bool areEqualHelper(
    _Type* first, int xStep0, int yStep0, _Type* second, int xstep1, int ystep1, int rows, int cols)
{
    _Type* curSecond;
    _Type* curFirst;
    if (std::numeric_limits<_Type>::is_exact)
    {
        for (int y = 0; y < rows; y++)
        {
            curFirst = (_Type*)(((char*)first) + y * yStep0);
            curSecond = (_Type*)(((char*)second) + y * yStep0);
            for (int x = 0; x < cols - 1; x++)
            {
                if (first[x] != second[x])
                    return true;
            }
            if (*first != *second)
                return true;
        }
    }
    else
    {
        for (int y = 0; y < rows; y++)
        {
            curFirst = (_Type*)(((char*)first) + y * yStep0);
            curSecond = (_Type*)(((char*)second) + y * yStep0);
            for (int x = 0; x < cols - 1; x++)
            {
                if (ito::isNotZero<_Type>(first[x] != second[x]))
                    return true;
            }
        }
    }
    return false;
}

//-------------------------------------------------------------------------------------
template <>
bool areEqualHelper<complex64>(
    complex64* first,
    int xStep0,
    int yStep0,
    complex64* second,
    int xstep1,
    int ystep1,
    int rows,
    int cols)
{
    complex64* curSecond;
    complex64* curFirst;
    for (int y = 0; y < rows; y++)
    {
        curFirst = (complex64*)(((char*)first) + y * yStep0);
        curSecond = (complex64*)(((char*)second) + y * yStep0);
        for (int x = 0; x < cols - 1; x++)
        {
            if (ito::isNotZero(first[x].real() - second[x].real()))
                return true;
            if (ito::isNotZero(first[x].imag() - second[x].imag()))
                return true;
        }
    }

    return false;
}

//-------------------------------------------------------------------------------------
template <>
bool areEqualHelper<complex128>(
    complex128* first,
    int xStep0,
    int yStep0,
    complex128* second,
    int xstep1,
    int ystep1,
    int rows,
    int cols)
{
    complex128* curSecond;
    complex128* curFirst;
    for (int y = 0; y < rows; y++)
    {
        curFirst = (complex128*)(((char*)first) + y * yStep0);
        curSecond = (complex128*)(((char*)second) + y * yStep0);
        for (int x = 0; x < cols - 1; x++)
        {
            if (ito::isNotZero(first[x].real() - second[x].real()))
                return true;
            if (ito::isNotZero(first[x].imag() - second[x].imag()))
                return true;
        }
    }

    return false;
}

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::areEqual(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* /*paramsOpt*/,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject* dObj1 = static_cast<ito::DataObject*>((*paramsMand)[0].getVal<void*>());
    if (dObj1 == NULL)
    {
        (*paramsOut)[0].setVal<int>(0);
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULLL").toLatin1().data());
    }

    ito::DataObject* dObj2 = static_cast<ito::DataObject*>((*paramsMand)[1].getVal<void*>());
    if (dObj2 == NULL)
    {
        (*paramsOut)[0].setVal<int>(0);
        return ito::RetVal(ito::retError, 0, tr("Error: source image is NULLL").toLatin1().data());
    }

    bool typeFlag;
    bool dimsFlag;
    bool last2DimsFlag;

    if (!ito::dObjHelper::dObjareEqualDetail(dObj1, dObj2, typeFlag, dimsFlag, last2DimsFlag))
    {
        // outVals->append(static_cast<bool>(false));
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
        cv::Mat_<unsigned char> * curMat = ((cv::Mat_<unsigned char>
    *)test.get_mdata()[test.seekMat(z)]); unsigned char* dataptr = NULL; for (y = 0; y < ySize; y++)
        {
            dataptr = curMat->ptr<unsigned char>(y);
            for (x = 0; x < xSize; x++)
            {
                if (!dataptr[x])
                {
                    (*paramsOut)[0].setVal<int>(0);
                    return retOk;
                }
            }
        }
    }
    */

    switch (dObj2->getType())
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

            if (areEqualHelper(
                    first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
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

            if (areEqualHelper(
                    first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
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

            if (areEqualHelper(
                    first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
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

            if (areEqualHelper(
                    first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
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

            if (areEqualHelper(
                    first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
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

            if (areEqualHelper(
                    first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
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

            if (areEqualHelper(
                    first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
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

            if (areEqualHelper(
                    first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
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

            if (areEqualHelper(
                    first, stepX0, stepY0, second, stepX1, stepY1, mat1->rows, mat1->cols))
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

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::centerOfGravityDoc =
    QObject::tr("This filter calculates the center of gravity of a 2D real image. \n\
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
The filter does not work with RGBA32, Complex64 and Complex128, but with all other datatypes.");

//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::centerOfGravityParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "sourceImage",
            ito::ParamBase::DObjPtr,
            NULL,
            tr("2D source image data object (u)int8, (u)int16, int32, float32 or float64 only.")
                .toLatin1()
                .data());
        paramsMand->append(param);
        param = Param(
            "lowThreshold",
            ito::ParamBase::Double,
            -1 * std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            0.0,
            tr("values < lowThreshold are ignored. lowThreshold is subtracted from each valid "
               "value before COG determination.")
                .toLatin1()
                .data());
        paramsOpt->append(param);
        param = Param(
            "highThreshold",
            ito::ParamBase::Double,
            -1 * std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            tr("values > highThreshold are ignored.").toLatin1().data());
        paramsOpt->append(param);

        paramsOut->append(ito::Param(
            "cYI",
            ito::ParamBase::Double | ito::ParamBase::Out,
            std::numeric_limits<ito::float64>::quiet_NaN(),
            NULL,
            tr("y-Coordinate of COG (index)").toLatin1().data()));
        paramsOut->append(ito::Param(
            "cXI",
            ito::ParamBase::Double | ito::ParamBase::Out,
            std::numeric_limits<ito::float64>::quiet_NaN(),
            NULL,
            tr("x-Coordinate of COG (index)").toLatin1().data()));
        paramsOut->append(ito::Param(
            "cY",
            ito::ParamBase::Double | ito::ParamBase::Out,
            std::numeric_limits<ito::float64>::quiet_NaN(),
            NULL,
            tr("y-Coordinate of COG (physical unit)").toLatin1().data()));
        paramsOut->append(ito::Param(
            "cX",
            ito::ParamBase::Double | ito::ParamBase::Out,
            std::numeric_limits<ito::float64>::quiet_NaN(),
            NULL,
            tr("x-Coordinate of COG (physical unit)").toLatin1().data()));
    }

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::centerOfGravity(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject* dObj = static_cast<ito::DataObject*>((*paramsMand)[0].getVal<void*>());
    if (dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is NULL").toLatin1().data());
    }
    if (dObj->getDims() < 1)
    {
        return ito::RetVal(
            ito::retError, 0, tr("Error: sourceImage is not initialized").toLatin1().data());
    }

    ito::float64 cx = std::numeric_limits<ito::float64>::quiet_NaN();
    ito::float64 cy = std::numeric_limits<ito::float64>::quiet_NaN();
    ito::float64 cxPhys = std::numeric_limits<ito::float64>::quiet_NaN();
    ito::float64 cyPhys = std::numeric_limits<ito::float64>::quiet_NaN();

    ito::float64 lowThreshold = (*paramsOpt)[0].getVal<ito::float64>();
    ito::float64 highThreshold = (*paramsOpt)[1].getVal<ito::float64>();

    retval += ito::dObjHelper::verifyDataObjectType(
        dObj,
        "sourceImage",
        7,
        ito::tInt8,
        ito::tUInt8,
        ito::tInt16,
        ito::tUInt16,
        ito::tInt32,
        ito::tFloat32,
        ito::tFloat64);
    if (dObj->getDims() > 2)
    {
        for (int i = 0; i < dObj->getDims() - 2; i++)
        {
            if (dObj->getSize(i) > 1)
            {
                return ito::RetVal(
                    ito::retError,
                    0,
                    tr("Error: source image must not have multiple planes").toLatin1().data());
            }
        }
    }

    if (!retval.containsError())
    {
        const cv::Mat* plane = dObj->getCvPlaneMat(0);

        switch (dObj->getType())
        {
        case ito::tInt8:
            centroidHelper<ito::int8>(plane, lowThreshold, highThreshold, cx, cy);
            break;
        case ito::tUInt8:
            centroidHelper<ito::uint8>(plane, lowThreshold, highThreshold, cx, cy);
            break;
        case ito::tInt16:
            centroidHelper<ito::int16>(plane, lowThreshold, highThreshold, cx, cy);
            break;
        case ito::tUInt16:
            centroidHelper<ito::uint16>(plane, lowThreshold, highThreshold, cx, cy);
            break;
        case ito::tInt32:
            centroidHelper<ito::int32>(plane, lowThreshold, highThreshold, cx, cy);
            break;
        case ito::tFloat32:
            centroidHelper<ito::float32>(plane, lowThreshold, highThreshold, cx, cy);
            break;
        case ito::tFloat64:
            centroidHelper<ito::float64>(plane, lowThreshold, highThreshold, cx, cy);
            break;
        default:
            return ito::RetVal(
                ito::retError,
                0,
                tr("Unknown type or type not implemented for phase shifting evaluation")
                    .toLatin1()
                    .data());
        }
    }

    (*paramsOut)[0].setVal<ito::float64>(cy);
    (*paramsOut)[1].setVal<ito::float64>(cx);

    if (!retval.containsError() && ito::isFinite<ito::float64>(cy))
    {
        bool test;
        cxPhys = dObj->getPixToPhys(dObj->getDims() - 1, cx, test);
        cyPhys = dObj->getPixToPhys(dObj->getDims() - 2, cy, test);
    }

    (*paramsOut)[2].setVal<ito::float64>(cyPhys);
    (*paramsOut)[3].setVal<ito::float64>(cxPhys);

    return retOk;
}

//-------------------------------------------------------------------------------------
template <typename _Tp>
ito::RetVal DataObjectArithmetic::centroidHelper(
    const cv::Mat* mat,
    const ito::float64& lowThreshold,
    const ito::float64& highThreshold,
    ito::float64& xCOG,
    ito::float64& yCOG)
{
    ito::int32 x, y;
    ito::float64 val = 0.0, sumva = 0.0, sumXv = 0.0, sumYv = 0.0;
    const _Tp* pValue = NULL;

    if (std::numeric_limits<_Tp>::is_exact)
    {
        const _Tp lowThres = cv::saturate_cast<_Tp>(qBound(
            (ito::float64)(std::numeric_limits<_Tp>::min()),
            lowThreshold,
            (ito::float64)(std::numeric_limits<_Tp>::max())));
        const _Tp highThres = cv::saturate_cast<_Tp>(qBound(
            (ito::float64)(std::numeric_limits<_Tp>::min()),
            highThreshold,
            (ito::float64)(std::numeric_limits<_Tp>::max())));

        for (y = 0; y < mat->rows; ++y)
        {
            pValue = mat->ptr<_Tp>(y);
            for (x = 0; x < mat->cols; ++x)
            {
                if (pValue[x] >= lowThres && pValue[x] <= highThres)
                {
                    val = (ito::float64)(pValue[x] - lowThres);
                    sumva += val;
                    sumXv += val * x;
                    sumYv += val * y;
                }
            }
        }
    }
    else
    {
        const _Tp lowThres = cv::saturate_cast<_Tp>(qBound(
            (ito::float64)(-std::numeric_limits<_Tp>::max()),
            lowThreshold,
            (ito::float64)(std::numeric_limits<_Tp>::max())));
        const _Tp highThres = cv::saturate_cast<_Tp>(qBound(
            (ito::float64)(-std::numeric_limits<_Tp>::max()),
            highThreshold,
            (ito::float64)(std::numeric_limits<_Tp>::max())));

        for (y = 0; y < mat->rows; ++y)
        {
            pValue = mat->ptr<_Tp>(y);
            for (x = 0; x < mat->cols; ++x)
            {
                if (ito::isFinite<_Tp>(pValue[x]) && pValue[x] >= lowThres &&
                    pValue[x] <= highThres)
                {
                    val = (ito::float64)(pValue[x] - lowThres);
                    sumva += val;
                    sumXv += val * x;
                    sumYv += val * y;
                }
            }
        }
    }

    if (ito::isNotZero<ito::float64>(sumva))
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

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::localCenterOfGravityDoc =
    QObject::tr("This filter determines the sub-pixel \n\
spot position of multiple spots in an image. The pixel-precise spot position must be given including the size \n\
of the area around the coarse spot position over which the center of gravity algorithm is applied. \n\
\n\
The area can either be a rectangle (width and height, odd values) or a circle (odd diameter). \n\
\n\
The COG is calculated by the following algorithm: \n\
\n\
cXI = \\frac{\\sum{idx_x * (I - lowThreshold)}}{\\sum{(I - lowThreshold)} \n\
cYI = \\frac{\\sum{idx_y * (I - lowThreshold)}}{\\sum{(I - lowThreshold)} \n\
\n\
The lowThreshold can either be given or (if it is NaN), the minimum value of each area will be taken as local lower threshold. \n\
Only values <= highThreshold are considered, set highThreshold to NaN or Inf in order to do not consider this constraint. \n\
\n\
Usually, the resulting 'centroids' object contains the sub-pixel x and y position as well as the number of valid pixels in each row. \n\
If no or only one valid pixel has been encountered, the coarse pixel x and y position as well as 0 or 1 (for no or one valid pixel) is returned. \n\
\n\
If the coarse spot position lies outside of the image, the resulting row in 'centroids' contains NaN coordinates. \n\
Please consider, that all input and output coordinates are assumed to be pixel values, the scaling and offset of the image are not considered.");

ito::RetVal DataObjectArithmetic::localCenterOfGravityParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "sourceImage",
            ito::ParamBase::DObjPtr,
            NULL,
            tr("2D source image data object (u)int8, (u)int16, int32, float32 or float64 only.")
                .toLatin1()
                .data());
        paramsMand->append(param);

        param = ito::Param(
            "coarseSpots",
            ito::ParamBase::DObjPtr,
            NULL,
            tr("Mx3 or Mx4 2D data object of type uint16, each row corresponds to one spot. The "
               "line contains [px_x, px_y, circle_diameter] if the cog should be determined within "
               "a circle or [px_x, px_y, width, height] if the cog should be determined within a "
               "rectangle. circle_diameter, width or height have to be odd.")
                .toLatin1()
                .data());
        paramsMand->append(param);

        param = ito::Param(
            "centroids",
            ito::ParamBase::DObjPtr,
            NULL,
            tr("resulting Mx3 data object of type float64 with the sub-pixel precise position of "
               "the spots (all is given in pixel coordinates, never physical coordinates). Each "
               "row is [subpix_x, subpix_y, nr_of_valid_elements_within_search_mask] or [px_x, "
               "px_y, 0 | 1] if the spot only contained one or no valid values.")
                .toLatin1()
                .data());
        paramsMand->append(param);

        param = Param(
            "lowThreshold",
            ito::ParamBase::Double,
            -1 * std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::quiet_NaN(),
            tr("values < lowThreshold are ignored. lowThreshold is subtracted from each valid "
               "value before COG determination. if lowThreshold is NaN (default), the lowest value "
               "within each spot search area is taken as local minimum value.")
                .toLatin1()
                .data());
        paramsOpt->append(param);
        param = Param(
            "highThreshold",
            ito::ParamBase::Double,
            -1 * std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            tr("values > highThreshold are ignored.").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

template <typename _Tp>
ito::RetVal localCenterOfGravityHelper(
    const ito::DataObject& source,
    const ito::DataObject& coarse,
    ito::DataObject& centroids,
    const ito::float64& lowThreshold,
    const ito::float64& highThreshold)
{
    ito::RetVal retval;
    const cv::Mat* source_ = source.getCvPlaneMat(0);
    const cv::Mat* coarse_ = coarse.getCvPlaneMat(0);

    bool circleNotRect = (coarse_->cols == 3);

    int max_half_width = source_->cols / 2; // will be max. radius for circular roi
    int max_half_height = source_->rows / 2;

    if (circleNotRect)
    {
        max_half_width = std::min(max_half_width, max_half_height);
    }

    _Tp high = cv::saturate_cast<_Tp>(
        std::min((ito::float64)std::numeric_limits<_Tp>::max(), highThreshold));
    if (!qIsFinite(highThreshold))
    {
        high = std::numeric_limits<_Tp>::max();
    }

#define LCOGRADIUS(r, c)                                                                           \
    std::sqrt(                                                                                     \
        (float)(coarseRow[1] - r) * (float)(coarseRow[1] - r) +                                    \
        (float)(coarseRow[0] - c) * (float)(coarseRow[0] - c))

#ifdef USEOPENMP
    omp_set_num_threads(ito::AddInBase::getMaximumThreadCount());
#pragma omp parallel
    {
#endif
        // in a parallel for loop, these variables have to be created for each thread
        std::vector<_Tp> vals;
        std::vector<int> x_px;
        std::vector<int> y_px;
        int count;
        const _Tp* sourceRow;
        ito::float64* centroidsRow;
        const ito::uint16* coarseRow;
        int half_width; // will be radius for circular roi
        int half_height;
        int max_vals;
        ito::float64 roiMinimum;
        ito::float64 denomx, denomy, nom;
        int start_row, end_row, start_col, end_col;

#ifdef USEOPENMP
#pragma omp for schedule(dynamic, 100)
#endif
        for (int row = 0; row < coarse_->rows; ++row)
        {
            count = 0;
            coarseRow = coarse_->ptr<ito::uint16>(row);
            centroidsRow = centroids.rowPtr<ito::float64>(0, row);
            roiMinimum = std::numeric_limits<ito::float64>::max();

            if (coarseRow[0] < 0 || coarseRow[0] >= source_->cols || coarseRow[1] < 0 ||
                coarseRow[1] >= source_->rows)
            {
                centroidsRow[0] = std::numeric_limits<ito::float64>::quiet_NaN();
                centroidsRow[1] = std::numeric_limits<ito::float64>::quiet_NaN();
                centroidsRow[2] = 0;
                continue;
            }

            if (circleNotRect)
            {
                half_width =
                    qBound(0, (int)std::ceil((float)(coarseRow[2] - 1) / 2.0), max_half_width);
                start_row = qBound(0, coarseRow[1] - half_width, source_->rows - 1);
                end_row = qBound(0, coarseRow[1] + half_width, source_->rows - 1);
                start_col = qBound(0, coarseRow[0] - half_width, source_->cols - 1);
                end_col = qBound(0, coarseRow[0] + half_width, source_->cols - 1);
            }
            else
            {
                half_width =
                    qBound(0, (int)std::ceil((float)(coarseRow[2] - 1) / 2.0), max_half_width);
                half_height =
                    qBound(0, (int)std::ceil((float)(coarseRow[3] - 1) / 2.0), max_half_height);
                start_row = qBound(0, coarseRow[1] - half_height, source_->rows - 1);
                end_row = qBound(0, coarseRow[1] + half_height, source_->rows - 1);
                start_col = qBound(0, coarseRow[0] - half_width, source_->cols - 1);
                end_col = qBound(0, coarseRow[0] + half_width, source_->cols - 1);
            }

            max_vals = (end_col - start_col + 1) * (end_row - start_row + 1);
            vals.clear();
            vals.reserve(max_vals);
            x_px.clear();
            x_px.reserve(max_vals);
            y_px.clear();
            y_px.reserve(max_vals);


            if (circleNotRect)
            {
                for (int r = start_row; r <= end_row; ++r)
                {
                    sourceRow = &(source_->ptr<_Tp>(r)[start_col]);

                    for (int c = start_col; c <= end_col; ++c)
                    {
                        if (LCOGRADIUS(r, c) <= half_width)
                        {
                            if (ito::isFinite<_Tp>(*sourceRow) && *sourceRow <= high)
                            {
                                roiMinimum = std::min(roiMinimum, (ito::float64)*sourceRow);
                                vals.push_back(*sourceRow);
                                x_px.push_back(c);
                                y_px.push_back(r);
                                count++;
                            }
                        }

                        ++sourceRow;
                    }
                }
            }
            else
            {
                for (int r = start_row; r <= end_row; ++r)
                {
                    sourceRow = &(source_->ptr<_Tp>(r)[start_col]);

                    for (int c = start_col; c <= end_col; ++c)
                    {
                        if (ito::isFinite<_Tp>(*sourceRow) && *sourceRow <= high)
                        {
                            roiMinimum = std::min(roiMinimum, (ito::float64)*sourceRow);
                            vals.push_back(*sourceRow);
                            x_px.push_back(c);
                            y_px.push_back(r);
                            count++;
                        }

                        ++sourceRow;
                    }
                }
            }

            if (count > 1)
            {
                denomx = 0.0;
                denomy = 0.0;
                nom = 0.0;

                if (ito::isFinite<ito::float64>(lowThreshold))
                {
                    for (int c = 0; c < count; ++c)
                    {
                        if (vals[c] > lowThreshold)
                        {
                            denomx += x_px[c] * (vals[c] - lowThreshold);
                            denomy += y_px[c] * (vals[c] - lowThreshold);
                            nom += (vals[c] - lowThreshold);
                        }
                        // else
                        //{
                        // count--;
                        //}
                    }
                }
                else
                {
                    for (int c = 0; c < count; ++c)
                    {
                        denomx += x_px[c] * (vals[c] - roiMinimum);
                        denomy += y_px[c] * (vals[c] - roiMinimum);
                        nom += (vals[c] - roiMinimum);
                    }
                }

                if (ito::isZeroValue<ito::float64>(
                        nom, std::numeric_limits<ito::float64>::epsilon()))
                {
                    centroidsRow[0] = coarseRow[0];
                    centroidsRow[1] = coarseRow[1];
                    centroidsRow[2] = count;
                }
                else
                {
                    centroidsRow[0] = denomx / nom;
                    centroidsRow[1] = denomy / nom;
                    centroidsRow[2] = count;
                }
            }
            else
            {
                centroidsRow[0] = coarseRow[0];
                centroidsRow[1] = coarseRow[1];
                centroidsRow[2] = count;
            }
        }

#ifdef USEOPENMP
    }
#endif

    return retval;
}

ito::RetVal DataObjectArithmetic::localCenterOfGravity(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval;
    ito::DataObject source = ito::dObjHelper::squeezeConvertCheck2DDataObject(
        paramsMand->at(0).getVal<const ito::DataObject*>(),
        "sourceImage",
        ito::Range::all(),
        ito::Range::all(),
        retval,
        -1,
        7,
        ito::tUInt8,
        ito::tInt8,
        ito::tUInt16,
        ito::tInt16,
        ito::tFloat32,
        ito::tFloat64);
    ito::DataObject coarseSpots = ito::dObjHelper::squeezeConvertCheck2DDataObject(
        paramsMand->at(1).getVal<const ito::DataObject*>(),
        "coarseSpots",
        ito::Range::all(),
        ito::Range(3, 4),
        retval,
        -1,
        1,
        ito::tUInt16);
    ito::float64 low = paramsOpt->at(0).getVal<ito::float64>();
    ito::float64 high = paramsOpt->at(1).getVal<ito::float64>();

    int numSpots = coarseSpots.getSize(0);

    if (source.getSize(0) < 2 || source.getSize(1) < 2)
    {
        retval += ito::RetVal(ito::retError, 0, "sourceImage must have a minimum size of 2x2.");
    }

    if (!retval.containsError())
    {
        if (numSpots > 0)
        {
            ito::DataObject centroids(numSpots, 3, ito::tFloat64);

            switch (source.getType())
            {
            case ito::tUInt8:
                retval += localCenterOfGravityHelper<ito::uint8>(
                    source, coarseSpots, centroids, low, high);
                break;
            case ito::tInt8:
                retval += localCenterOfGravityHelper<ito::int8>(
                    source, coarseSpots, centroids, low, high);
                break;
            case ito::tUInt16:
                retval += localCenterOfGravityHelper<ito::uint16>(
                    source, coarseSpots, centroids, low, high);
                break;
            case ito::tInt16:
                retval += localCenterOfGravityHelper<ito::int16>(
                    source, coarseSpots, centroids, low, high);
                break;
            case ito::tUInt32:
                retval += localCenterOfGravityHelper<ito::uint32>(
                    source, coarseSpots, centroids, low, high);
                break;
            case ito::tInt32:
                retval += localCenterOfGravityHelper<ito::int32>(
                    source, coarseSpots, centroids, low, high);
                break;
            case ito::tFloat32:
                retval += localCenterOfGravityHelper<ito::float32>(
                    source, coarseSpots, centroids, low, high);
                break;
            case ito::tFloat64:
                retval += localCenterOfGravityHelper<ito::float64>(
                    source, coarseSpots, centroids, low, high);
                break;
            }

            if (!retval.containsError())
            {
                *((*paramsMand)[2].getVal<ito::DataObject*>()) = centroids;
            }
        }
        else
        {
            *((*paramsMand)[2].getVal<ito::DataObject*>()) = ito::DataObject();
        }
    }

    return retval;
}


//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::centerOfGravity1DimDoc =
    QObject::tr("Calculate center of gravity for each plane along the x- or y-direction. \n\
\n\
This methods creates the two given data objects 'destCOG' and 'destIntensity' in the following way: \n\
\n\
- destCOG, ito::float64, sizes: [nrOfPlanes x sizeOfElements], contains the sub-pixel wise one-dimensional coordinate of the center of gravity (in physical coordinates) or NaN if it could not be determined. \n\
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
The filter is not implemented for complex data types and the type rgba32 since there is no maximum value defined for these types.");

//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::centerOfGravity1DimParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "sourceImage",
            ito::ParamBase::DObjPtr | ito::ParamBase::In,
            NULL,
            tr("source image data (2D or 3D) object for operation (u)int8, (u)int16, int32, "
               "float32 or float64")
                .toLatin1()
                .data());
        paramsMand->append(param);
        param = ito::Param(
            "destCOG",
            ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out,
            NULL,
            tr("destination object for center of gravity values (in physical coordinates), "
               "float64, size: [numPlanes x sizeOfElements]")
                .toLatin1()
                .data());
        paramsMand->append(param);
        param = ito::Param(
            "destIntensity",
            ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out,
            NULL,
            tr("destination object for the absolute maximum along the search direction, same type "
               "than source image, size: [numPlanes x sizeOfElements]")
                .toLatin1()
                .data());
        paramsMand->append(param);
        param = Param(
            "pvThreshold",
            ito::ParamBase::Double | ito::ParamBase::In,
            0.0,
            std::numeric_limits<ito::float64>::max(),
            0.0,
            tr("if (max-min) along the search direction is lower or equal this pvThreshold "
               "(peak-to-valley), no cog is determined and a NaN value is set into the resulting "
               "position array (default: this threshold is not considered).")
                .toLatin1()
                .data());
        paramsOpt->append(param);
        param = Param(
            "dynamicThreshold",
            ito::ParamBase::Double | ito::ParamBase::In,
            0.0,
            0.999,
            0.5,
            tr("If != 0.0, values <= (max+min)*dynamicThreshold will be ignored. To only consider "
               "values above the FWHM, set this value to 0.5 (default).")
                .toLatin1()
                .data());
        paramsOpt->append(param);
        param = Param(
            "lowerThreshold",
            ito::ParamBase::Double | ito::ParamBase::In,
            -std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            -std::numeric_limits<ito::float64>::max(),
            tr("values <= lowerThreshold will not be considered for the cog calculation (default: "
               "this threshold is not considered).")
                .toLatin1()
                .data());
        paramsOpt->append(param);
        param = Param(
            "columnWise",
            ito::ParamBase::Int | ito::ParamBase::In,
            0,
            1,
            0,
            tr("0: COG search along each row (default), 1: along each column").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::centerOfGravity1Dim(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject* dObjIN = paramsMand->at(0).getVal<ito::DataObject*>();
    ito::DataObject* dObjCogOut = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::DataObject* dObjIntOut = paramsMand->at(2).getVal<ito::DataObject*>();

    if (dObjIN == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is NULL").toLatin1().data());
    }
    if (dObjIN->getDims() < 1)
    {
        return ito::RetVal(
            ito::retError, 0, tr("Error: sourceImage is not initialized").toLatin1().data());
    }

    if (dObjCogOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: destCOG image is NULL").toLatin1().data());
    }

    if (dObjIntOut == NULL)
    {
        return ito::RetVal(
            ito::retError, 0, tr("Error: destIntensity image is NULL").toLatin1().data());
    }

    if (dObjCogOut == dObjIntOut)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("Error: destCOG and destIntensity must not be the same data objects.")
                .toLatin1()
                .data());
    }

    ito::float64 pvThreshold = (*paramsOpt)[0].getVal<ito::float64>();
    ito::float64 dynThreshold = (*paramsOpt)[1].getVal<ito::float64>();
    ito::float64 lowerThreshold = (*paramsOpt)[2].getVal<ito::float64>();
    bool columnWise = (*paramsOpt)[3].getVal<ito::int32>() > 0 ? true : false;

    ito::float64 scaleVert = 1.0; // Scale along depth ScanAxis
    ito::float64 offsetVert = 0.0; // offset along depth ScanAxis
    ito::uint32 sizeZ;
    ito::uint32 sizeY;
    ito::uint32 sizeX;
    int dObjINDims = dObjIN->getDims();

    retval += ito::dObjHelper::verify3DDataObject(
        dObjIN,
        "sourceImage",
        1,
        100000,
        1,
        100000,
        1,
        100000,
        7,
        ito::tInt8,
        ito::tUInt8,
        ito::tInt16,
        ito::tUInt16,
        ito::tInt32,
        ito::tFloat32,
        ito::tFloat64);
    if (!retval.containsError())
    {
        sizeZ = dObjIN->getSize(0);
        sizeY = dObjIN->getSize(1);
        sizeX = dObjIN->getSize(2);
    }
    else
    {
        retval = ito::dObjHelper::verify2DDataObject(
            dObjIN,
            "sourceImage",
            1,
            100000,
            1,
            100000,
            7,
            ito::tInt8,
            ito::tUInt8,
            ito::tInt16,
            ito::tUInt16,
            ito::tInt32,
            ito::tFloat32,
            ito::tFloat64);

        if (retval.containsError())
        {
            return retval;
        }

        sizeZ = 1;
        sizeY = dObjIN->getSize(0);
        sizeX = dObjIN->getSize(1);
    }

    ito::RetVal retvalTemp = ito::retOk;

    if (columnWise)
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(
            dObjCogOut, "destCOG", sizeZ, sizeZ, sizeX, sizeX, 1, ito::tFloat64);

        if (retvalTemp.containsError())
        {
            *dObjCogOut = ito::DataObject(sizeZ, sizeX, ito::tFloat64);
        }

        scaleVert = dObjIN->getAxisScale(dObjIN->getDims() - 2);
        offsetVert = dObjIN->getAxisOffset(dObjIN->getDims() - 2);
    }
    else
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(
            dObjCogOut, "destCOG", sizeZ, sizeZ, sizeY, sizeY, 1, ito::tFloat64);

        if (retvalTemp.containsError())
        {
            *dObjCogOut = ito::DataObject(sizeZ, sizeY, ito::tFloat64);
        }

        scaleVert = dObjIN->getAxisScale(dObjIN->getDims() - 1);
        offsetVert = dObjIN->getAxisOffset(dObjIN->getDims() - 1);
    }

    if (columnWise)
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(
            dObjIntOut, "destIntensity", sizeZ, sizeZ, sizeX, sizeX, 1, dObjIN->getType());
    }
    else
    {
        retvalTemp = ito::dObjHelper::verify2DDataObject(
            dObjIntOut, "destIntensity", sizeZ, sizeZ, sizeY, sizeY, 1, dObjIN->getType());
    }

    if (retvalTemp.containsError())
    {
        if (columnWise)
        {
            *dObjIntOut = ito::DataObject(sizeZ, sizeX, dObjIN->getType());
        }
        else
        {
            *dObjIntOut = ito::DataObject(sizeZ, sizeY, dObjIN->getType());
        }
    }

    if (!retval.containsError())
    {
        cv::Mat sliceCOG;
        cv::Mat sliceInt;
        const cv::Mat* planeIn = NULL;
        cv::Mat* planeCogOut = dObjCogOut->getCvPlaneMat(0);
        cv::Mat* planeIntOut = dObjIntOut->getCvPlaneMat(0);

        switch (dObjIN->getType())
        {
        case ito::tInt8:
            for (ito::uint32 i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
            {
                planeIn = dObjIN->getCvPlaneMat(i);
                centroidHelperFor1D<ito::int8>(
                    planeIn,
                    planeCogOut->ptr<ito::float64>(i),
                    planeIntOut->ptr<ito::int8>(i),
                    cv::saturate_cast<ito::int8>(pvThreshold),
                    cv::saturate_cast<ito::int8>(lowerThreshold),
                    dynThreshold,
                    scaleVert,
                    offsetVert,
                    columnWise);
            }
            break;
        case ito::tUInt8:
            for (ito::uint32 i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
            {
                planeIn = dObjIN->getCvPlaneMat(i);
                centroidHelperFor1D<ito::uint8>(
                    planeIn,
                    planeCogOut->ptr<ito::float64>(i),
                    planeIntOut->ptr<ito::uint8>(i),
                    cv::saturate_cast<ito::uint8>(pvThreshold),
                    cv::saturate_cast<ito::uint8>(lowerThreshold),
                    dynThreshold,
                    scaleVert,
                    offsetVert,
                    columnWise);
            }
            break;
        case ito::tInt16:
            for (ito::uint32 i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
            {
                planeIn = dObjIN->getCvPlaneMat(i);
                centroidHelperFor1D<ito::int16>(
                    planeIn,
                    planeCogOut->ptr<ito::float64>(i),
                    planeIntOut->ptr<ito::int16>(i),
                    cv::saturate_cast<ito::int16>(pvThreshold),
                    cv::saturate_cast<ito::int16>(lowerThreshold),
                    dynThreshold,
                    scaleVert,
                    offsetVert,
                    columnWise);
            }
            break;
        case ito::tUInt16:
            for (ito::uint32 i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
            {
                planeIn = dObjIN->getCvPlaneMat(i);
                centroidHelperFor1D<ito::uint16>(
                    planeIn,
                    planeCogOut->ptr<ito::float64>(i),
                    planeIntOut->ptr<ito::uint16>(i),
                    cv::saturate_cast<ito::uint16>(pvThreshold),
                    cv::saturate_cast<ito::uint16>(lowerThreshold),
                    dynThreshold,
                    scaleVert,
                    offsetVert,
                    columnWise);
            }
            break;
        case ito::tInt32:
            for (ito::uint32 i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
            {
                planeIn = dObjIN->getCvPlaneMat(i);
                centroidHelperFor1D<ito::int32>(
                    planeIn,
                    planeCogOut->ptr<ito::float64>(i),
                    planeIntOut->ptr<ito::int32>(i),
                    cv::saturate_cast<ito::int32>(pvThreshold),
                    cv::saturate_cast<ito::int32>(lowerThreshold),
                    dynThreshold,
                    scaleVert,
                    offsetVert,
                    columnWise);
            }
            break;
        case ito::tFloat32:
            for (ito::uint32 i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
            {
                planeIn = dObjIN->getCvPlaneMat(i);
                centroidHelperFor1D<ito::float32>(
                    planeIn,
                    planeCogOut->ptr<ito::float64>(i),
                    planeIntOut->ptr<ito::float32>(i),
                    cv::saturate_cast<ito::float32>(pvThreshold),
                    cv::saturate_cast<ito::float32>(lowerThreshold),
                    dynThreshold,
                    scaleVert,
                    offsetVert,
                    columnWise);
            }
            break;
        case ito::tFloat64:
            for (ito::uint32 i = 0; i < (ito::uint32)dObjIN->calcNumMats(); i++)
            {
                planeIn = dObjIN->getCvPlaneMat(i);
                centroidHelperFor1D<ito::float64>(
                    planeIn,
                    planeCogOut->ptr<ito::float64>(i),
                    planeIntOut->ptr<ito::float64>(i),
                    cv::saturate_cast<ito::float64>(pvThreshold),
                    cv::saturate_cast<ito::float64>(lowerThreshold),
                    dynThreshold,
                    scaleVert,
                    offsetVert,
                    columnWise);
            }
            break;
        default:
            return ito::RetVal(
                ito::retError,
                0,
                tr("Center of gravity can only be calculated for (u)int8, (u)int16, (u)int32, "
                   "float32 or float64 data objects.")
                    .toLatin1()
                    .data());
        }
    }

    if (!retval.containsError())
    {
        bool test;
        if (columnWise)
        {
            dObjCogOut->setAxisScale(1, dObjIN->getAxisScale(dObjINDims - 1));
            dObjCogOut->setAxisOffset(1, dObjIN->getAxisOffset(dObjINDims - 1));
            dObjCogOut->setAxisUnit(1, dObjIN->getAxisUnit(dObjINDims - 1, test));
            dObjCogOut->setAxisDescription(1, dObjIN->getAxisDescription(dObjINDims - 1, test));

            dObjIntOut->setAxisScale(1, dObjIN->getAxisScale(dObjINDims - 1));
            dObjIntOut->setAxisOffset(1, dObjIN->getAxisOffset(dObjINDims - 1));
            dObjIntOut->setAxisUnit(1, dObjIN->getAxisUnit(dObjINDims - 1, test));
            dObjIntOut->setAxisDescription(1, dObjIN->getAxisDescription(dObjINDims - 1, test));
        }
        else
        {
            dObjCogOut->setAxisScale(1, dObjIN->getAxisScale(dObjINDims - 2));
            dObjCogOut->setAxisOffset(1, dObjIN->getAxisOffset(dObjINDims - 2));
            dObjCogOut->setAxisUnit(1, dObjIN->getAxisUnit(dObjINDims - 2, test));
            dObjCogOut->setAxisDescription(1, dObjIN->getAxisDescription(dObjINDims - 2, test));

            dObjIntOut->setAxisScale(1, dObjIN->getAxisScale(dObjINDims - 2));
            dObjIntOut->setAxisOffset(1, dObjIN->getAxisOffset(dObjINDims - 2));
            dObjIntOut->setAxisUnit(1, dObjIN->getAxisUnit(dObjINDims - 2, test));
            dObjIntOut->setAxisDescription(1, dObjIN->getAxisDescription(dObjINDims - 2, test));
        }

        if (dObjINDims >= 3)
        {
            dObjCogOut->setAxisScale(0, dObjIN->getAxisScale(dObjINDims - 3));
            dObjCogOut->setAxisOffset(0, dObjIN->getAxisOffset(dObjINDims - 3));
            dObjCogOut->setAxisUnit(0, dObjIN->getAxisUnit(dObjINDims - 3, test));
            dObjCogOut->setAxisDescription(0, dObjIN->getAxisDescription(dObjINDims - 3, test));

            dObjIntOut->setAxisScale(0, dObjIN->getAxisScale(dObjINDims - 3));
            dObjIntOut->setAxisOffset(0, dObjIN->getAxisOffset(dObjINDims - 3));
            dObjIntOut->setAxisUnit(0, dObjIN->getAxisUnit(dObjINDims - 3, test));
            dObjIntOut->setAxisDescription(0, dObjIN->getAxisDescription(dObjINDims - 3, test));
        }

        dObjCogOut->setValueUnit("a. u.");
        dObjCogOut->setValueDescription("center of gravity");
        dObjIntOut->setValueDescription("intensity");
        dObjIntOut->setValueUnit("a.u.");

        dObjIN->copyTagMapTo(*dObjCogOut);
        dObjIN->copyTagMapTo(*dObjIntOut);

        if (columnWise)
        {
            QString protocol("Center of gravity evaluated columnwise from intensity stack, ");
            protocol.append("pvThreshold: ");
            protocol.append(QString::number(pvThreshold));
            if (dynThreshold > std::numeric_limits<ito::float64>::epsilon())
            {
                protocol.append("dynThreshold: ");
                protocol.append(QString::number(dynThreshold));
            }
            if (lowerThreshold > (-std::numeric_limits<ito::float64>::max() +
                                  std::numeric_limits<ito::float64>::epsilon()))
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

//-------------------------------------------------------------------------------------
template <typename _Tp>
ito::RetVal DataObjectArithmetic::centroidHelperFor1D(
    const cv::Mat* inMat,
    ito::float64* outCOG,
    _Tp* outINT,
    const _Tp& pvThreshold,
    const _Tp& lowerThreshold,
    const ito::float64& dynamicThreshold,
    const ito::float64& scale,
    const ito::float64& offset,
    bool alongCols)
{
    ito::uint32 stepEvalInMat;
    ito::uint32 stepAddrInMat;
    ito::uint32 cogsToCalc;
    ito::uint32 pixelToEval;
    ito::uint32 cogCnt = 0;

    ito::float64 sumI, sumxi;

    const _Tp* inMatPtrFirst = inMat->ptr<_Tp>(0);
    const _Tp* pInValue = NULL;
    _Tp sw;

    if (alongCols)
    {
        cogsToCalc = inMat->rows;
        pixelToEval = inMat->cols;
        stepEvalInMat = (ito::uint32)inMat->step1();
        stepAddrInMat = 1;
    }
    else
    {
        cogsToCalc = inMat->cols;
        pixelToEval = inMat->rows;
        stepEvalInMat = 1;
        stepAddrInMat = (ito::uint32)inMat->step1();
    }

    _Tp typeMax;
    _Tp typeMin;
    bool runDynamic =
        dynamicThreshold > std::numeric_limits<ito::float64>::epsilon() ? true : false;
    bool runLower = false;

    if (std::numeric_limits<_Tp>::is_exact)
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

        if (lowerThreshold >
            (-std::numeric_limits<_Tp>::max() + std::numeric_limits<_Tp>::epsilon()))
        {
            runLower = true;
        }
    }

    for (ito::uint32 pixelCnt = 0; pixelCnt < pixelToEval; pixelCnt++)
    {
        sumI = 0.0;
        sumxi = 0.0;
        _Tp maxVal = typeMin;
        _Tp minVal = typeMax;
        _Tp val = 0;


        // determine the min/max value along the search direction (along each column or each row)
        pInValue = inMatPtrFirst + stepAddrInMat * pixelCnt;

        for (cogCnt = 0; cogCnt < cogsToCalc; cogCnt++)
        {
            if (maxVal < *pInValue)
            {
                maxVal = *pInValue;
            }

            if (minVal > *pInValue)
            {
                minVal = *pInValue;
            }

            pInValue += stepEvalInMat;
        }

        // determine the cog along the search direction:
        /*
        conditions:
        - (max - min) > pvThreshold (peak-to-valley threshold)
        - val > lowerThreshold (if lowerThreshold > (minimum of data type _Tp))
        - val > (max + min) * dynamicThreshold (if dynamicThreshold > 0)

        If lowerThreshold or dynamicThreshold is active, its current maximum value is subtracted
        from each value within the search direction before further calculation.
        */
        pInValue = inMatPtrFirst + stepAddrInMat * pixelCnt;

        if ((maxVal - minVal) > pvThreshold)
        {
            if (runDynamic || runLower)
            {
                if (runLower && runDynamic)
                {
                    sw = std::max(
                        ((ito::float64)maxVal + (ito::float64)minVal) * dynamicThreshold,
                        (ito::float64)lowerThreshold);
                }
                else if (runLower)
                {
                    sw = (ito::float64)lowerThreshold;
                }
                else
                {
                    sw = ((ito::float64)maxVal + (ito::float64)minVal) * dynamicThreshold;
                }

                for (cogCnt = 0; cogCnt < cogsToCalc; ++cogCnt)
                {
                    if (*pInValue > sw)
                    {
                        val = *pInValue - sw;
                        sumI += val;
                        sumxi += (ito::float64)val * (ito::float64)cogCnt;
                    }
                    pInValue = pInValue + stepEvalInMat;
                }
            }
            else
            {
                // this version is only valid, if the minimum value converges towards zero, else the
                // minimum value must be subtracted from each value first (consider to set a
                // lowerThreshold)
                for (cogCnt = 0; cogCnt < cogsToCalc; ++cogCnt)
                {
                    val = *pInValue;
                    sumI += val;
                    sumxi += (ito::float64)val * (ito::float64)cogCnt;
                    pInValue = pInValue + stepEvalInMat;
                }
            }
        }

        // save the currently found maximum along the search direction
        outINT[pixelCnt] = maxVal;

        // calculate and save current cog position
        if (ito::isNotZero<ito::float64>(sumI))
        {
            outCOG[pixelCnt] = ((sumxi / sumI) - offset) *
                scale; // sumxi / sumI is in pixel-coordinates, outCOG is in physical coordinates:
                       // (px - offset) * scaling = phys
        }
        else
        {
            outCOG[pixelCnt] = std::numeric_limits<ito::float64>::quiet_NaN();
        }
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::gaussianFit1DimDoc =
    QObject::tr("Fits Gaussian for given x and y values. \n\
\n\
This method fits a Gaussian curve of the form: \n\
\n\
y = A * exp(-(x - mu)^2 / (2 * sigma^2)) \n\
\n\
It implements the iterative method, described in the paper \n\
\n\
Hongwei Guo et. al, A Simple Algorithm for Fitting a Gaussian Function, \n\
IEEE Signal Processing Magazine, 28(5), 2011 \n\
\n\
There is no additional bias or offset considered, such that a fit will provide \n\
valid results only if the y-values trend towards zero at the edges. If less \n\
than three values > 0 are given, the fit will fail. \n\
\n\
The returned values are the coefficients A, mu and sigma. \n\
\n\
For the calculation, an internal accumulator has to be created. To avoid a buffer \n\
overflow of this accumulator, do not use to big values. Possibly downscale the \n\
values of the source objects.");

//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::gaussianFit1DimParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "x",
            ito::ParamBase::DObjPtr | ito::ParamBase::In,
            NULL,
            tr("1xN, float64 x coordinates").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param(
            "y",
            ito::ParamBase::DObjPtr | ito::ParamBase::In,
            NULL,
            tr("1xN, float64 y coordinates").toLatin1().data());
        paramsMand->append(param);

        param = Param(
            "tolerance",
            ito::ParamBase::Double | ito::ParamBase::In,
            0.0,
            10000.0,
            0.001,
            tr("Stop criteria for the iterative approach. If the norm of the difference of the internal \n\
vector (a,b,c) (see referenced paper) is smaller than this tolerance, the iteration is stopped. \n\
The maximum iteration count is 20 in any case.")
                .toLatin1()
                .data());
        paramsOpt->append(param);

        param = Param(
            "ignoreNaN",
            ito::ParamBase::Int | ito::ParamBase::In,
            0,
            1,
            1,
            tr("If 1, NaN values in x and / or y are ignored.").toLatin1().data());
        paramsOpt->append(param);

        param = Param(
            "A",
            ito::ParamBase::Double | ito::ParamBase::Out,
            0.0,
            nullptr,
            tr("The magnitude A").toLatin1().data());
        paramsOut->append(param);

        param = Param(
            "mu",
            ito::ParamBase::Double | ito::ParamBase::Out,
            0.0,
            nullptr,
            tr("The center mu").toLatin1().data());
        paramsOut->append(param);

        param = Param(
            "sigma",
            ito::ParamBase::Double | ito::ParamBase::Out,
            0.0,
            nullptr,
            tr("The value sigma").toLatin1().data());
        paramsOut->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::gaussianFit1Dim(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject* x_ = paramsMand->at(0).getVal<const ito::DataObject*>();
    const ito::DataObject* y_ = paramsMand->at(1).getVal<const ito::DataObject*>();

    if (x_ == nullptr || y_ == nullptr || x_->getDims() < 2 || y_->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: x or y is empty").toLatin1().data());
    }

    ito::DataObject x = ito::dObjHelper::squeezeConvertCheck2DDataObject(
        x_,
        "x",
        ito::Range(1, 1),
        ito::Range(3, 1000000),
        retval,
        ito::tFloat64,
        7,
        ito::tInt8,
        ito::tUInt8,
        ito::tInt16,
        ito::tUInt16,
        ito::tInt32,
        ito::tFloat32,
        ito::tFloat64);

    int cols = x.getSize(1);

    ito::DataObject y = ito::dObjHelper::squeezeConvertCheck2DDataObject(
        y_,
        "y",
        ito::Range(1, 1),
        ito::Range(cols, cols),
        retval,
        ito::tFloat64,
        7,
        ito::tInt8,
        ito::tUInt8,
        ito::tInt16,
        ito::tUInt16,
        ito::tInt32,
        ito::tFloat32,
        ito::tFloat64);

    double tolerance = paramsOpt->at(0).getVal<double>();
    bool ignoreNaN = paramsOpt->at(1).getVal<int>() > 0;

    if (!retval.containsError())
    {
        // accumulators and solve results
        cv::Mat_<double> M = cv::Mat_<double>::zeros(3, 3);
        cv::Mat_<double> u = cv::Mat_<double>::zeros(3, 1);
        float temp;
        float y_square_k;
        cv::Mat_<double> p = cv::Mat_<double>::zeros(3, 1);
        cv::Mat_<double> p_old = cv::Mat_<double>::zeros(3, 1);
        int count;

        double* u_ptr = u.ptr<double>();
        double* M_ptr = M.ptr<double>();

        const int iterMax = 20;
        double y_square;
        double sigma_square;
        double n; // the norm
        const double thres = 1e-4;

        const double* y_ptr = y.rowPtr<double>(0, 0);
        const double* x_ptr = x.rowPtr<double>(0, 0);

        // iterative approach
        for (int iter = 0; iter < iterMax; ++iter)
        {
            M.setTo(0);
            u.setTo(0);
            count = 0;

            for (int idx = 0; idx < cols; ++idx)
            {
                if (y_ptr[idx] < 0)
                {
                    retval += ito::RetVal(
                        ito::retError, 0, "Negative value encountered. This value is ignored.");
                }

                if (ignoreNaN && (qIsNaN(y_ptr[idx]) || qIsNaN(x_ptr[idx])))
                {
                    continue;
                }

                y_square = (y_ptr[idx]) * (y_ptr[idx]);

                // the threshold is important, since ln(y) with y == 0 is -inf.
                if (y_square > thres)
                {
                    count++;

                    if (iter == 0)
                    {
                        y_square_k = y_square;
                    }
                    else
                    {
                        y_square_k = std::pow(
                            std::exp(
                                p_old(0) + p_old(1) * x_ptr[idx] +
                                p_old(2) * x_ptr[idx] * x_ptr[idx]),
                            2);
                    }

                    // modify M

                    // y^2
                    temp = y_square_k;
                    M_ptr[0] += temp;

                    // y^2 * x
                    temp *= x_ptr[idx];
                    M_ptr[1] += temp;
                    M_ptr[3] += temp;

                    // y^2 * x^2
                    temp *= x_ptr[idx];
                    M_ptr[2] += temp;
                    M_ptr[4] += temp;
                    M_ptr[6] += temp;

                    // y^2 * x^3
                    temp *= x_ptr[idx];
                    M_ptr[5] += temp;
                    M_ptr[7] += temp;

                    // y^2 * x^4
                    temp *= x_ptr[idx];
                    M_ptr[8] += temp;

                    // modify u
                    temp = std::log(y_ptr[idx]) * y_square_k;
                    u_ptr[0] += temp;
                    u_ptr[1] += x_ptr[idx] * temp;
                    u_ptr[2] += x_ptr[idx] * x_ptr[idx] * temp;
                }
            }

            if (count < 3)
            {
                retval += ito::RetVal(
                    ito::retError,
                    0,
                    "Error fitting the Gaussian curve. At least three valid values > 0 required.");
                p.setTo(std::numeric_limits<double>::quiet_NaN());
                break;
            }

            // result of solve, Solve  M * p = u
            if (!cv::solve(M, u, p, cv::DECOMP_SVD))
            {
                retval += ito::RetVal(
                    ito::retError, 0, "Error fitting the Gaussian curve. Solve failed.");
                p.setTo(std::numeric_limits<double>::quiet_NaN());
                break;
            }

            n = cv::norm(p_old - p);
            p_old = p.clone();

            if (n < tolerance)
            {
                // it is enough!
                break;
            }
        }

        sigma_square = -1.0 / (2.0 * p(2));

        (*paramsOut)[0].setVal<double>(std::exp(p(0) - p(1) * p(1) / (4 * p(2))));
        (*paramsOut)[1].setVal<double>(p(1) * sigma_square);
        (*paramsOut)[2].setVal<double>(std::sqrt(sigma_square));
    }

    return retval;
}


//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::getPercentageThresholdDoc =
    QObject::tr("analyzes all values in the given data object and returns the value, which is at a "
                "given percentage in the sorted value list.");

//-------------------------------------------------------------------------------------
/*static*/ ito::RetVal DataObjectArithmetic::getPercentageThresholdParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError())
        return retval;

    paramsMand->clear();
    paramsMand->append(ito::Param(
        "data",
        ito::ParamBase::DObjPtr | ito::ParamBase::In,
        NULL,
        tr("valid non-complex data object").toLatin1().data()));
    paramsMand->append(ito::Param(
        "percentage",
        ito::ParamBase::Double | ito::ParamBase::In,
        0.0,
        100.0,
        50.0,
        tr("percentage value [0.0, 100.0]").toLatin1().data()));

    paramsOut->append(ito::Param(
        "threshold",
        ito::ParamBase::Double | ito::ParamBase::Out,
        NULL,
        tr("threshold value (NaN if data object was empty or only contained invalid values)")
            .toLatin1()
            .data()));

    return retval;
}

//-------------------------------------------------------------------------------------
/*static*/ ito::RetVal DataObjectArithmetic::getPercentageThreshold(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::DataObject* data = paramsMand->at(0).getVal<ito::DataObject*>();
    double percentage = paramsMand->at(1).getVal<double>();
    double threshold = std::numeric_limits<double>::quiet_NaN();
    ito::RetVal retval;

    if (!data)
    {
        retval += ito::RetVal(ito::retError, 0, tr("input data is NULL").toLatin1().data());
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
            retval += ito::RetVal(
                ito::retError,
                0,
                tr("not implemented for complex64 or complex128").toLatin1().data());
        }
    }

    (*paramsOut)[0].setVal<double>(threshold);
    return retval;
}

//-------------------------------------------------------------------------------------
template <typename _Tp>
/*static*/ ito::RetVal DataObjectArithmetic::getPercentageThresholdHelper(
    const ito::DataObject* dObj, double percentage, double& value)
{
    std::vector<_Tp> values;
    int numValues = 0;
    int planes = dObj->calcNumMats();
    int dims = dObj->getDims();
    int m = dObj->getSize(dims - 2);
    int n = dObj->getSize(dims - 1);

    if (std::numeric_limits<_Tp>::is_integer)
    {
        numValues = planes * m * n;
        values.resize(numValues);
        _Tp* vData = values.data();
        _Tp* rowPtr;

        // copies the entire content of the integer data object to the values vector.
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
        _Tp* rowPtr;

        for (int p = 0; p < planes; ++p)
        {
            for (int mi = 0; mi < m; ++mi)
            {
                rowPtr = (_Tp*)(dObj->rowPtr(p, mi));
                for (int ni = 0; ni < n; ++ni)
                {
                    if (ito::isFinite(rowPtr[ni]))
                    {
                        values.push_back(rowPtr[ni]);
                        numValues++;
                    }
                }
            }
        }
    }

    if (numValues == 0 && (planes * m * n) > 0)
    {
        value = std::numeric_limits<double>::quiet_NaN();
        return ito::RetVal(ito::retWarning, 0, "no valid values encountered");
    }
    else if (numValues == 0)
    {
        value = std::numeric_limits<double>::quiet_NaN();
        return ito::retOk;
    }

    if (percentage <= 50.0)
    {
        int selValue = floor(percentage * (double)numValues / 100.0);
        selValue = std::max(0, selValue);
        selValue = std::min(numValues - 1, selValue);
        std::nth_element(
            values.begin(),
            values.begin() + selValue,
            values.end(),
            DataObjectArithmetic::cmpLT<_Tp>);
        value = values[selValue];
    }
    else
    {
        int selValue = floor((100.0 - percentage) * (double)numValues / 100.0);
        selValue = std::max(0, selValue);
        selValue = std::min(numValues - 1, selValue);
        std::nth_element(
            values.begin(),
            values.begin() + selValue,
            values.end(),
            DataObjectArithmetic::cmpGT<_Tp>);
        value = values[selValue];
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::boundingBoxDoc = QObject::tr(
    "This filter calculates the minimum ROI that contains all values within a lower and optional upper threshold. \n\
\n\
The return value contains the [x0,y0,width,height] of the minimum ROI.\n\
\n\
Values of the data object belong to the ROI if they are >= lowThreshold and <= highThreshold. \n\
The highThreshold is only checked, if it is different than the default value (maximum value of double). \n\
\n\
The filter does not work with RGBA32, Complex64 and Complex128, but with all other datatypes. This filter has got a fast \n\
implementation for fixed-point data types without an higher threshold (since version 0.0.3).");

//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::boundingBoxParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "sourceImage",
            ito::ParamBase::DObjPtr,
            NULL,
            tr("2D source image data object (u)int8, (u)int16, int32, float32 or float64 only.")
                .toLatin1()
                .data());
        paramsMand->append(param);
        param = Param(
            "lowThreshold",
            ito::ParamBase::Double,
            -1 * std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            0.0,
            tr("only values >= lowThreshold are considered for the ROI").toLatin1().data());
        paramsMand->append(param);
        param = Param(
            "highThreshold",
            ito::ParamBase::Double,
            -1 * std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            tr("if given, only values <= highThreshold are considered for the ROI")
                .toLatin1()
                .data());
        paramsOpt->append(param);

        paramsOut->append(ito::Param(
            "roi",
            ito::ParamBase::IntArray | ito::ParamBase::Out,
            NULL,
            tr("ROI of bounding box [x0,y0,width,height]").toLatin1().data()));
    }

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::boundingBox(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject* dObj = (*paramsMand)[0].getVal<ito::DataObject*>();
    if (dObj == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is NULL").toLatin1().data());
    }

    if (dObj->getDims() < 1)
    {
        return ito::RetVal(
            ito::retError, 0, tr("Error: sourceImage is not initialized").toLatin1().data());
    }

    ito::float64 lowThreshold = (*paramsMand)[1].getVal<ito::float64>();
    ito::float64 highThreshold = (*paramsOpt)[0].getVal<ito::float64>();

    retval += ito::dObjHelper::verifyDataObjectType(
        dObj,
        "sourceImage",
        7,
        ito::tInt8,
        ito::tUInt8,
        ito::tInt16,
        ito::tUInt16,
        ito::tInt32,
        ito::tFloat32,
        ito::tFloat64);
    if (dObj->getNumPlanes() != 1)
    {
        return ito::RetVal(
            ito::retError, 0, tr("Error: source image must have one plane").toLatin1().data());
    }

    int roi[] = {0, 0, 0, 0};

    if (!retval.containsError())
    {
        const cv::Mat* plane = dObj->getCvPlaneMat(0);

        switch (dObj->getType())
        {
        case ito::tInt8:
            boundingBoxHelper<ito::int8>(plane, lowThreshold, highThreshold, roi);
            break;
        case ito::tUInt8:
            boundingBoxHelper<ito::uint8>(plane, lowThreshold, highThreshold, roi);
            break;
        case ito::tInt16:
            boundingBoxHelper<ito::int16>(plane, lowThreshold, highThreshold, roi);
            break;
        case ito::tUInt16:
            boundingBoxHelper<ito::uint16>(plane, lowThreshold, highThreshold, roi);
            break;
        case ito::tInt32:
            boundingBoxHelper<ito::int32>(plane, lowThreshold, highThreshold, roi);
            break;
        case ito::tFloat32:
            boundingBoxHelper<ito::float32>(plane, lowThreshold, highThreshold, roi);
            break;
        case ito::tFloat64:
            boundingBoxHelper<ito::float64>(plane, lowThreshold, highThreshold, roi);
            break;
        default:
            return ito::RetVal(
                ito::retError,
                0,
                tr("Unknown type or type not implemented for phase shifting evaluation")
                    .toLatin1()
                    .data());
        }
    }

    (*paramsOut)[0].setVal<int*>(roi, 4);

    return retOk;
}

//-------------------------------------------------------------------------------------
template <typename _Tp>
ito::RetVal DataObjectArithmetic::boundingBoxHelper(
    const cv::Mat* mat,
    const ito::float64& lowThreshold,
    const ito::float64& highThreshold,
    int* roi)
{
    unsigned int x, y;
    unsigned int x0 = mat->cols - 1;
    unsigned int x1 = 0;
    unsigned int y0 = mat->rows - 1;
    unsigned int y1 = 0;

    const _Tp* pValue = NULL;

    if (std::numeric_limits<_Tp>::is_exact)
    {
        const _Tp lowThres = cv::saturate_cast<_Tp>(qBound(
            (ito::float64)(std::numeric_limits<_Tp>::min()),
            lowThreshold,
            (ito::float64)(std::numeric_limits<_Tp>::max())));
        const _Tp highThres = cv::saturate_cast<_Tp>(qBound(
            (ito::float64)(std::numeric_limits<_Tp>::min()),
            highThreshold,
            (ito::float64)(std::numeric_limits<_Tp>::max())));

        if (highThreshold < std::numeric_limits<ito::float64>::max())
        {
            for (y = 0; y < (unsigned int)mat->rows; ++y)
            {
                pValue = mat->ptr<_Tp>(y);
                for (x = 0; x < (unsigned int)mat->cols; ++x)
                {
                    if (pValue[x] >= lowThres && pValue[x] <= highThres)
                    {
                        x0 = std::min(x0, x);
                        x1 = std::max(x1, x);
                        y0 = std::min(y0, y);
                        y1 = std::max(y1, y);
                    }
                }
            }
        }
        else
        {
            // this is a faster version for the special case of integer based numbers without a
            // higher threshold this version tries to detect the first valid line beginning from
            // line 0, then the last valid line beginning from the last line, and finally it detects
            // the x0 and x1 boundary from left and right only in between the valid line range.
            for (y = 0; y < (unsigned int)mat->rows; ++y)
            {
                pValue = mat->ptr<_Tp>(y);
                for (x = 0; x < (unsigned int)mat->cols; ++x)
                {
                    if (pValue[x] >= lowThres)
                    {
                        y0 = y;
                        y1 = y;
                        x0 = x;
                        x1 = x;

                        // search for last valid x-value in this row
                        for (x = (unsigned int)mat->cols - 1; x > x0; --x)
                        {
                            if (pValue[x] >= lowThres)
                            {
                                x1 = x;
                                break;
                            }
                        }

                        goto step2; // goto command in order to leave both nested for loops
                    }
                }
            }

        step2:
            // search for last line with at least one valid value
            for (y = (unsigned int)mat->rows - 1; y > y0; --y)
            {
                pValue = mat->ptr<_Tp>(y);
                for (x = 0; x < (unsigned int)mat->cols; ++x)
                {
                    if (pValue[x] >= lowThres)
                    {
                        y1 = y;
                        x0 = std::min(x, x0);
                        x1 = std::max(x, x1);

                        // search for last valid x-value in this row
                        for (x = (unsigned int)mat->cols - 1; x > x0; --x)
                        {
                            if (pValue[x] >= lowThres)
                            {
                                x1 = std::max(x, x1);
                                break;
                            }
                        }

                        goto step3; // goto command in order to leave both nested for loops
                    }
                }
            }

        step3:
            // search for smaller valid x-values than recent x0 in lines (y0+1::x1-1)
            // and for bigger valid x-values than recent x1 in the same lines
            for (y = y0 + 1; y < y1; ++y)
            {
                pValue = mat->ptr<_Tp>(y);
                for (x = 0; x < (unsigned int)mat->cols; ++x)
                {
                    if (pValue[x] >= lowThres)
                    {
                        x0 = std::min(x0, x);
                        x1 = std::max(x1, x);
                        break;
                    }
                }

                for (x = (unsigned int)mat->cols - 1; x > x0; --x)
                {
                    if (pValue[x] >= lowThres)
                    {
                        x1 = std::max(x1, x);
                        break;
                    }
                }
            }
        }
    }
    else
    {
        const _Tp lowThres = cv::saturate_cast<_Tp>(qBound(
            (ito::float64)(-std::numeric_limits<_Tp>::max()),
            lowThreshold,
            (ito::float64)(std::numeric_limits<_Tp>::max())));
        const _Tp highThres = cv::saturate_cast<_Tp>(qBound(
            (ito::float64)(-std::numeric_limits<_Tp>::max()),
            highThreshold,
            (ito::float64)(std::numeric_limits<_Tp>::max())));

        if (highThreshold < std::numeric_limits<ito::float64>::max())
        {
            for (y = 0; y < (unsigned int)mat->rows; ++y)
            {
                pValue = mat->ptr<_Tp>(y);
                for (x = 0; x < (unsigned int)mat->cols; ++x)
                {
                    if (pValue[x] >= lowThres && pValue[x] <= highThres)
                    {
                        x0 = std::min(x0, x);
                        x1 = std::max(x1, x);
                        y0 = std::min(y0, y);
                        y1 = std::max(y1, y);
                    }
                }
            }
        }
        else
        {
            for (y = 0; y < (unsigned int)mat->rows; ++y)
            {
                pValue = mat->ptr<_Tp>(y);
                for (x = 0; x < (unsigned int)mat->cols; ++x)
                {
                    if (pValue[x] >= lowThres)
                    {
                        x0 = std::min(x0, x);
                        x1 = std::max(x1, x);
                        y0 = std::min(y0, y);
                        y1 = std::max(y1, y);
                    }
                }
            }
        }
    }

    roi[0] = x0;
    roi[1] = y0;
    roi[2] = 1 + x1 - x0;
    roi[3] = 1 + y1 - y0;

    return ito::retOk;
}


//-------------------------------------------------------------------------------------
const QString DataObjectArithmetic::findMultiSpotsDoc = QObject::tr(
    "This method determines the sub-pixel peak position of multiple spots in an image. \n\
\n\
This algorithm is implemented for 2D or 3D input images of type uint8 or uint16 only and has been developped with \
respect to a fast implementation. At first, the image is analyzed line-wise with a line distancen of 'searchStepSize'. \n\
In every line the coarse peak position of every 1D peak is analyzed. This can be done in two different ways (depending on the \n\
parameter 'mode' (0, 2 or 4): \n\
\n\
In mode 0 (slightly slower) pixels belong to the background if their distance to the previous pixel (the search step size is also \
considered in each line) is smaller than 'backgroundNoise'. If this is not the case, a potential peak starts. However this peak \
is only a true peak, if the peak's height is bigger than 'minPeakHeight'. \n\
\n\
In mode 2, a peak consists of a sequence of pixels whose gray-value are all >= 'maxBackgroundLevel' (fast, but requires homogeneous background and peak levels). \n\
\n\
In mode 4, a peak can only start if a current gray-value is >= 'minPeakHeight' and if the difference to its previous pixel \n\
is bigger than 'backgroundNoise'. The peak is only finished and hence stopped if the difference between its highest gray-value \n\
and the start-value has been at least 'minPeakHeight', checked at the moment if the gradient is currently negative and its current \n\
gray value is either below 'minPeakHeight' or its difference to the previous value is <= 'backgroundNoise'. \n\
\n\
After all peaks in all analyzed lines have been detected, peaks in adjacent lines(step size of 'searchStepSize') are clustered \
considering the parameter 'maxPeakDiameter'.Finally the center of gravity is determined around each local maximum using 'maxPeakDiameter' \
as rectangular size of the search rectangle around the coarse maximum position.The results are stored in the data object 'spots'. \
The 'spots' object is two dimensional for a 2D input image, else 3D where the first dimension corresponds to the number of planes in 'input'. \
Each line corresponds to one peak and contains its sub - pixel precise row and column as well as the coarse intensity value and the area of the peak. \
This value may differ from the real peak value due to the search grid size of 'searchStepSize'. \n\
\n\
The parameter 'searchStepSize' is a list of two values, the first describes the vertical step size, the second the horizontal step size.");

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::findMultiSpotsParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError())
        return retval;

    paramsMand->append(ito::Param(
        "image",
        ito::ParamBase::DObjPtr | ito::ParamBase::In,
        NULL,
        tr("input 2D or 3D uint8 or uint16 data object (in case of 3D, every plane is analyzed "
           "independently and the resulting spot object is 3D as well. Indicate parameter "
           "'maxNrOfSpots' in case of 3D.")
            .toLatin1()
            .data()));
    paramsMand->append(ito::Param(
        "spots",
        ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out,
        NULL,
        tr("resulting data object with spot coordinates. Every line consists of the following "
           "entries: [sub-pixel wise row (physical coordinates), sub-pixel wise column (physical "
           "coordinates), coarse intensity of the peak, area of the peak (nr of pixels brighter "
           "than background)].")
            .toLatin1()
            .data()));

    paramsOpt->append(ito::Param(
        "backgroundNoise",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        4095,
        3,
        tr("maximum difference between two adjacent background values (used for deciding if pixel "
           "belongs to background or peak, only necessary in mode 0)")
            .toLatin1()
            .data()));
    paramsOpt->append(ito::Param(
        "minPeakHeight",
        ito::ParamBase::Int | ito::ParamBase::In,
        2,
        4095,
        7,
        tr("minimum height of a peak (its maximum and the neighbouring background, only necessary "
           "in mode 0).")
            .toLatin1()
            .data()));
    paramsOpt->append(ito::Param(
        "maxPeakDiameter",
        ito::ParamBase::Int | ito::ParamBase::In,
        3,
        65000,
        15,
        tr("maximum diameter of a peak (this is used to distinguish between neighbouring peaks and "
           "the determination of the sub-pixel peak position).")
            .toLatin1()
            .data()));
    int searchStepSize[] = {2, 2};
    ito::Param p = ito::Param(
        "searchStepSize",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        2,
        searchStepSize,
        tr("step size in pixel for the coarse search of peaks (for rows and columns)")
            .toLatin1()
            .data());
    p.setMeta(new ito::IntArrayMeta(0, 1000, 1, 2, 2, 1), true);
    paramsOpt->append(p);
    paramsOpt->append(ito::Param(
        "maxBackgroundLevel",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        4095,
        5,
        tr("maximum background level for subpixel determination, in mode 2 this value is the "
           "single value used to determine if value is a peak.")
            .toLatin1()
            .data()));
    paramsOpt->append(ito::Param(
        "mode",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        4,
        0,
        tr("implemented modes are 0, 2 or 4. Depending on each mode, the search strategy of "
           "possible points in each line is kindly different and varies in speed and accuracy.")
            .toLatin1()
            .data()));
    paramsOpt->append(ito::Param(
        "maxNrOfSpots",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        std::numeric_limits<int>::max(),
        0,
        tr("if > 0 the resulting spots object is limited to the maximum number of spots "
           "(unsorted), else it contains as many lines as detected spots. In case of a 3D image, "
           "every plane is analyzed. Then it becomes necessary to indicate this parameter. If "
           "'spots' is then allocated with a bigger number of lines than detected peaks, the "
           "additional lines are filled with 0.0.")
            .toLatin1()
            .data()));

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal DataObjectArithmetic::findMultiSpots(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut)
{
#ifdef TIMEIT
    float times[6];
    int64 tick = cv::getTickCount();
#endif

    ito::RetVal retval;
    const ito::DataObject* image = paramsMand->at(0).getVal<ito::DataObject*>();
    int type = image->getType();
    int dims = image->getDims();
    const cv::Mat* plane = NULL;

    MultiSpotParameters params;
    params.backgroundNoise = paramsOpt->at(0).getVal<int>();
    params.minPeakHeight = paramsOpt->at(1).getVal<int>();
    params.maxPeakDiameter = paramsOpt->at(2).getVal<int>();
    params.searchStepSizeHeight = paramsOpt->at(3).getVal<int*>()[0];
    params.searchStepSizeWidth = paramsOpt->at(3).getVal<int*>()[1];
    params.maxBackgroundLevel = paramsOpt->at(4).getVal<int>();
    params.mode = paramsOpt->at(5).getVal<int>();

    int maxNrOfSpots = paramsOpt->at(6).getVal<int>();
    int nrOfPlanes = image->getNumPlanes();

    if (type != ito::tUInt8 && type != ito::tUInt16)
    {
        retval += ito::RetVal(
            ito::retError, 0, tr("image must be of type uint8 or uint16").toLatin1().data());
    }
    else if (nrOfPlanes < 1)
    {
        retval += ito::RetVal(
            ito::retError, 0, tr("image must have at least one plane.").toLatin1().data());
    }
    else if (nrOfPlanes > 1 && maxNrOfSpots == 0)
    {
        retval += ito::RetVal(
            ito::retError,
            0,
            tr("in case of a 3D input image please indicate the optional parameter 'maxNrOfSpots'")
                .toLatin1()
                .data());
    }

    if (!retval.containsError())
    {
        int rows = image->getSize(dims - 2);
        int cols = image->getSize(dims - 1);
        int cols_reduced = std::ceil((double)cols / (double)params.maxPeakDiameter);
        int rows_reduced = std::ceil((double)rows / (double)params.searchStepSizeHeight);
        ito::DataObject result;
        ito::uint16 area;
        int count;
        ito::float32* linePtr;

        if (maxNrOfSpots > 0 && nrOfPlanes > 1)
        {
            result.zeros(nrOfPlanes, maxNrOfSpots, 4, ito::tFloat32);
        }
        else if (maxNrOfSpots > 0)
        {
            result.zeros(maxNrOfSpots, 4, ito::tFloat32);
        }

        if (type == ito::tUInt8)
        {
            // spots is completely uninitialized, therefore don't trust any value. The number of
            // spots should be sufficient to hold all possible spots
            Spot<ito::uint8>* spots =
                (Spot<ito::uint8>*)malloc(rows_reduced * cols_reduced * sizeof(Spot<ito::uint8>));
            Spot<ito::uint8>* spots_temp;

            for (int planeIdx = 0; planeIdx < nrOfPlanes; ++planeIdx)
            {
                plane = image->getCvPlaneMat(planeIdx);

#ifdef TIMEIT
                times[0] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
#endif
                spots_temp = spots;
                for (int r = 0; r < rows; r += params.searchStepSizeHeight)
                {
                    // the spot value after the last valid one is marked with row = -1 by
                    // findMultiSpots1D!
                    findMultiSpots1D<ito::uint8>(
                        (ito::uint8*)plane->ptr(r), r, cols, spots_temp, params);
                    spots_temp += cols_reduced;
                }
#ifdef TIMEIT
                times[1] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
#endif

                Spot<ito::uint8> finalSpotsRoot, tempSpotsRoot;
                Spot<ito::uint8>* finalSpotsLast = &finalSpotsRoot;
                Spot<ito::uint8>* firstSpotInLine;

                for (int r = 0; r < rows_reduced; ++r)
                {
                    firstSpotInLine = &spots[r * cols_reduced];
                    if (firstSpotInLine[0].row > -1)
                    {
                        clusterSpots<ito::uint8>(
                            firstSpotInLine,
                            &finalSpotsLast,
                            &tempSpotsRoot,
                            r,
                            cols_reduced,
                            params);
                    }
                }
#ifdef TIMEIT
                times[2] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
#endif

                finalSpotsLast->next =
                    tempSpotsRoot
                        .next /*first element of tempSpotsRoot, root is only a dummy element */;

                if (maxNrOfSpots == 0)
                {
                    // count final spots
                    finalSpotsLast = finalSpotsRoot.next;
                    while (finalSpotsLast)
                    {
                        maxNrOfSpots++;
                        finalSpotsLast = finalSpotsLast->next;
                    }

                    result = ito::DataObject(maxNrOfSpots, 4, ito::tFloat32);
                }

                finalSpotsLast = finalSpotsRoot.next;
                count = 0;

                // get subpixel peak for each spot
                while (finalSpotsLast && count < maxNrOfSpots)
                {
                    linePtr = (ito::float32*)result.rowPtr(planeIdx, count);
                    ++count;
                    fastCOG<ito::uint8>(
                        plane,
                        finalSpotsLast->row,
                        finalSpotsLast->col,
                        params.maxPeakDiameter / 2,
                        params.maxBackgroundLevel,
                        linePtr[0],
                        linePtr[1],
                        area);
                    linePtr[2] = finalSpotsLast->value;
                    linePtr[3] = area;
                    finalSpotsLast = finalSpotsLast->next;
                    linePtr[0] = image->getPixToPhys(dims - 2, linePtr[0]);
                    linePtr[1] = image->getPixToPhys(dims - 1, linePtr[1]);
                }
#ifdef TIMEIT
                times[3] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
#endif
            }

            free(spots);
            spots = NULL;

#ifdef TIMEIT
            times[4] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
#endif
        }
        else // type uint16
        {
            // spots is completely uninitialized, therefore don't trust any value. The number of
            // spots should be sufficient to hold all possible spots
            Spot<ito::uint16>* spots =
                (Spot<ito::uint16>*)malloc(rows_reduced * cols_reduced * sizeof(Spot<ito::uint16>));
            Spot<ito::uint16>* spots_temp;

            for (int planeIdx = 0; planeIdx < nrOfPlanes; ++planeIdx)
            {
                plane = image->getCvPlaneMat(planeIdx);

#ifdef TIMEIT
                times[0] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
#endif
                spots_temp = spots;
                for (int r = 0; r < rows; r += params.searchStepSizeHeight)
                {
                    // the spot value after the last valid one is marked with row = -1 by
                    // findMultiSpots1D!
                    findMultiSpots1D<ito::uint16>(
                        (ito::uint16*)plane->ptr(r), r, cols, spots_temp, params);
                    spots_temp += cols_reduced;
                }
#ifdef TIMEIT
                times[1] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
#endif

                Spot<ito::uint16> finalSpotsRoot, tempSpotsRoot;
                Spot<ito::uint16>* finalSpotsLast = &finalSpotsRoot;
                Spot<ito::uint16>* firstSpotInLine;

                for (int r = 0; r < rows_reduced; ++r)
                {
                    firstSpotInLine = &spots[r * cols_reduced];
                    if (firstSpotInLine[0].row > -1)
                    {
                        clusterSpots<ito::uint16>(
                            firstSpotInLine,
                            &finalSpotsLast,
                            &tempSpotsRoot,
                            r,
                            cols_reduced,
                            params);
                    }
                }
#ifdef TIMEIT
                times[2] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
#endif

                finalSpotsLast->next =
                    tempSpotsRoot
                        .next /*first element of tempSpotsRoot, root is only a dummy element */;

                if (maxNrOfSpots == 0)
                {
                    // count final spots
                    finalSpotsLast = finalSpotsRoot.next;
                    while (finalSpotsLast)
                    {
                        maxNrOfSpots++;
                        finalSpotsLast = finalSpotsLast->next;
                    }

                    result = ito::DataObject(maxNrOfSpots, 4, ito::tFloat32);
                }

                finalSpotsLast = finalSpotsRoot.next;
                count = 0;

                // get subpixel peak for each spot
                while (finalSpotsLast && count < maxNrOfSpots)
                {
                    linePtr = (ito::float32*)result.rowPtr(planeIdx, count);
                    ++count;
                    fastCOG<ito::uint16>(
                        plane,
                        finalSpotsLast->row,
                        finalSpotsLast->col,
                        params.maxPeakDiameter / 2,
                        params.maxBackgroundLevel,
                        linePtr[0],
                        linePtr[1],
                        area);
                    linePtr[2] = finalSpotsLast->value;
                    linePtr[3] = area;
                    finalSpotsLast = finalSpotsLast->next;
                    linePtr[0] = image->getPixToPhys(dims - 2, linePtr[0]);
                    linePtr[1] = image->getPixToPhys(dims - 1, linePtr[1]);
                }
#ifdef TIMEIT
                times[3] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
#endif
            }

            free(spots);
            spots = NULL;

#ifdef TIMEIT
            times[4] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
#endif
        }

        *(paramsMand->at(1).getVal<ito::DataObject*>()) = result;
#ifdef TIMEIT
        times[5] = double(cv::getTickCount() - tick) / cv::getTickFrequency();
        std::cout << times[0] * 1000.0 << " -> " << times[1] * 1000.0 << " -> " << times[2] * 1000.0
                  << " -> " << times[3] * 1000.0 << " -> " << times[4] * 1000.0 << " -> "
                  << times[5] * 1000.0 << " \n"
                  << std::endl;
#endif
    }

    return retval;
}


//--------------------------------------------------------------------------------------
// only _Tp uint8 and uint16 are allowed
// spots points to the first spot item that belongs to the analyzed row. spots must be long enough!
template <typename _Tp>
void DataObjectArithmetic::findMultiSpots1D(
    const _Tp* lineData,
    const int& row,
    const int& cols,
    Spot<_Tp>* spots,
    const MultiSpotParameters& params)
{
    if (params.mode & 0x02)
    {
        bool mode = 0; // false: need to find start of peak, true: start already found
        _Tp peakPeakVal; // value of the potential maximum of the peak
        int peakPeakCol1 = 0; // col index of the potential peak
        int peakPeakCol2 = 0;
        int idx = 0;

        for (int c = 0; c <= cols - params.searchStepSizeWidth; c += params.searchStepSizeWidth)
        {
            if (!mode)
            {
                if (lineData[c] > params.maxBackgroundLevel)
                {
                    peakPeakCol1 = peakPeakCol2 = c;
                    peakPeakVal = lineData[c];
                    mode = true;
                }
            }
            else
            {
                if (lineData[c] >= peakPeakVal)
                {
                    if (lineData[c] == peakPeakVal)
                    {
                        peakPeakCol2 = c;
                    }
                    else
                    {
                        peakPeakCol1 = peakPeakCol2 = c;
                        peakPeakVal = lineData[c];
                    }
                }
                else if (lineData[c] <= params.maxBackgroundLevel)
                {
                    mode = false;
                    spots[idx].row = row;
                    spots[idx].col = (peakPeakCol1 + peakPeakCol2) / 2;
                    spots[idx++].value = peakPeakVal;
                }
            }
        }

        spots[idx].row = -1; // mark the item after the last valid item with row = -1
    }
    else if (params.mode & 0x04)
    {
        int idx = 0;
        int mode = 0; // 0: no peak in sight yet, 1: peak started, 2: maximum reached, check the
                      // falling edge
        _Tp peakStartVal = lineData[0]; // first, left potential value of a peak
        _Tp peakPeakVal = lineData[0]; // value of the potential maximum of the peak
        int peakPeakCol = 0; // col index of the potential peak
        int diff;

        for (int c = 0; c <= cols - params.searchStepSizeWidth; c += params.searchStepSizeWidth)
        {
            if (mode == 0) // below background
            {
                // search for start of raising flank of peak
                if (lineData[c] >= params.maxBackgroundLevel)
                {
                    peakPeakVal = lineData[c];
                    peakStartVal = lineData[c];
                    peakPeakCol = c;
                    mode = 1;
                }
                else
                {
                    peakStartVal = lineData[c];
                }
            }
            else if (mode == 1) // raising edge
            {
                if (lineData[c] < params.maxBackgroundLevel)
                {
                    peakStartVal = lineData[c];
                    mode = 0;
                }
                else if ((lineData[c] - peakStartVal) >= params.minPeakHeight)
                {
                    peakPeakVal = lineData[c];
                    peakPeakCol = c;
                    mode = 2;
                }
            }
            else if (mode == 2)
            {
                if (lineData[c] < params.maxBackgroundLevel)
                {
                    spots[idx].row = row;
                    spots[idx].col = peakPeakCol;
                    spots[idx].value = peakPeakVal;
                    idx++;
                    mode = 0;
                    c = peakPeakCol + params.maxPeakDiameter;
                    peakStartVal = lineData[c];
                }
                else if (lineData[c] > peakPeakVal)
                {
                    peakPeakVal = lineData[c];
                    peakPeakCol = c;
                }
            }
        }

        spots[idx].row = -1; // mark the item after the last valid item with row = -1
    }
    else
    {
        int idx = 0;
        int mode = 0; // 0: no peak in sight yet, 1: peak started
        _Tp peakStartVal = lineData[0]; // first, left potential value of a peak
        _Tp peakPeakVal = lineData[0]; // value of the potential maximum of the peak
        int peakPeakCol = 0; // col index of the potential peak
        int peakPeakColTemp = 0; // col index of the potential peak (if the peak is a plateau, this
                                 // index is always the begin of the plateau)
        int diff;

        for (int c = 0; c <= cols - params.searchStepSizeWidth; c += params.searchStepSizeWidth)
        {
            if (mode == 0)
            {
                // search for start of raising flank of peak

                diff = lineData[c] - peakStartVal;
                if (diff > params.backgroundNoise)
                {
                    peakPeakVal = lineData[c];
                    peakPeakCol = peakPeakColTemp = c;
                    mode = 1;
                }
                else
                {
                    peakStartVal = lineData[c];
                }
            }
            else
            {
                // search for falling flank of peak

                diff = lineData[c] - peakPeakVal;
                if (diff > 0) // peak weiter steigend
                {
                    peakPeakVal = lineData[c];
                    peakPeakCol = peakPeakColTemp = c;
                }
                else if (diff == 0) // stagniert
                {
                    peakPeakCol = int((c + peakPeakColTemp) / 2);
                }
                else // peak faellt, ist er schon vorbei?
                {
                    if (std::abs(lineData[c] - peakStartVal) <=
                        params.backgroundNoise) // peak ist vorbei, war er aber auch hoch genug?
                    {
                        if (peakPeakVal - peakStartVal >= params.minPeakHeight)
                        {
                            spots[idx].row = row;
                            spots[idx].col = peakPeakCol;
                            spots[idx].value = peakPeakVal;
                            idx++;
                            mode = 0;
                            c += params.maxPeakDiameter;
                            peakStartVal = lineData[c];
                        }
                        else // peak war nicht hoch genug
                        {
                            mode = 0;
                            peakStartVal = lineData[c];
                        }
                    }
                }
            }
        }

        spots[idx].row = -1; // mark the item after the last valid item with row = -1
    }
}

//-------------------------------------------------------------------------------------
// only _Tp uint8 and uint16 are allowed
template <typename _Tp>
void DataObjectArithmetic::clusterSpots(
    Spot<_Tp>* spots,
    Spot<_Tp>** finalSpotsLast,
    Spot<_Tp>* tempSpotsRoot,
    const int rowIdx,
    const int& spotsSlice,
    const MultiSpotParameters& params)
{
    Spot<_Tp>* tempSpotsLast =
        tempSpotsRoot; // root is dummy, the next element is the first real temporary spot

    // 1. check if any tempSpots are out of currentRow and therefore final
    if (tempSpotsRoot->next) // temporary elements exist
    {
        int rowLimit = rowIdx * params.searchStepSizeHeight - params.maxPeakDiameter;

        while (tempSpotsLast->next)
        {
            if (tempSpotsLast->next->row < rowLimit)
            {
                (*finalSpotsLast)->next = tempSpotsLast->next;
                *finalSpotsLast =
                    (*finalSpotsLast)->next; // move finalSpotsLast to newly appended element
                tempSpotsLast->next = (*finalSpotsLast)->next;
                (*finalSpotsLast)->next = NULL;
            }
            else
            {
                tempSpotsLast = tempSpotsLast->next;
            }
        }
    }

    // 2. now all items in temporaryPeaks are in the range of any peak in currentRow
    //    check all peaks in currentRow and verify if they are the same than one in temporaryPeaks
    //    if so, use the higher one, else: append the new peak to temporaryPeaks
    Spot<_Tp>* item;
    Spot<_Tp>* tempItem = tempSpotsRoot->next;
    bool found;
    for (int i = 0; i < spotsSlice; ++i)
    {
        found = false;
        item = &(spots[i]);
        if (item->row >= 0) // valid spot
        {
            tempItem = tempSpotsRoot->next;

            while (tempItem)
            {
                if (std::abs(item->col - tempItem->col) <= params.maxPeakDiameter)
                {
                    // found
                    if (item->value >= tempItem->value)
                    {
                        tempItem->col = item->col;
                        tempItem->row = item->row;
                        tempItem->value = item->value;
                    }

                    found = true;
                    break;
                }

                tempItem = tempItem->next;
            }

            if (!found)
            {
                tempSpotsLast->next = item;
                tempSpotsLast = item;
                item->next = NULL;
            }
        }
        else
        {
            break;
        }
    }
}

//-------------------------------------------------------------------------------------
template <typename _Tp>
void DataObjectArithmetic::fastCOG(
    const cv::Mat* img,
    const int row,
    const int col,
    const int halfSize,
    const _Tp lowThreshold,
    ito::float32& rowSubPix,
    ito::float32& colSubPix,
    ito::uint16& area)
{
    int startRow = std::max(0, row - halfSize);
    int startCol = std::max(0, col - halfSize);
    int width = std::min(1 + 2 * halfSize, img->cols - startCol);
    int height = std::min(1 + 2 * halfSize, img->rows - startRow);
    int dm, dn;

    _Tp val;
    unsigned int sumva = 0, sumMv = 0, sumNv = 0;
    size_t step_Tp = img->step[0] / sizeof(_Tp);
    area = 0;

    const _Tp* rowPtr = &(((_Tp*)(img->ptr(startRow)))[startCol]);

    for (dm = 0; dm < height; ++dm)
    {
        for (dn = 0; dn < width; ++dn)
        {
            if (rowPtr[dn] >= lowThreshold)
            {
                val = rowPtr[dn] - lowThreshold;
                sumva += val;
                sumMv += val * dm;
                sumNv += val * dn;
                area += 1;
            }
        }
        rowPtr += step_Tp;
    }

    if (sumva != 0)
    {
        rowSubPix = (ito::float32)startRow + ((ito::float32)sumMv / (ito::float32)sumva);
        colSubPix = (ito::float32)startCol + ((ito::float32)sumNv / (ito::float32)sumva);
    }
    else
    {
        rowSubPix = std::numeric_limits<ito::float32>::quiet_NaN();
        colSubPix = std::numeric_limits<ito::float32>::quiet_NaN();
    }
}


//-------------------------------------------------------------------------------------
DataObjectArithmetic::~DataObjectArithmetic()
{
    FilterDef* filter;
    foreach (filter, m_filterList)
    {
        delete filter;
    }

    m_filterList.clear();
}

//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::init(
    QVector<ito::ParamBase>* /*paramsMand*/,
    QVector<ito::ParamBase>* /*paramsOpt*/,
    ItomSharedSemaphore* /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef* filter = NULL;

    filter = new FilterDef(
        DataObjectArithmetic::maxValue,
        DataObjectArithmetic::singleDObjInputValueAndPositionOutParams,
        maxValueDoc);
    m_filterList.insert("maxValue", filter);

    filter = new FilterDef(
        DataObjectArithmetic::minValue,
        DataObjectArithmetic::singleDObjInputValueAndPositionOutParams,
        minValueDoc);
    m_filterList.insert("minValue", filter);

    filter = new FilterDef(
        DataObjectArithmetic::minMaxValue, DataObjectArithmetic::minMaxValueParams, minMaxValueDoc);
    m_filterList.insert("minMaxValue", filter);

    filter = new FilterDef(
        DataObjectArithmetic::meanValue,
        DataObjectArithmetic::singleDObjInputInfParams,
        meanValueDoc);
    m_filterList.insert("meanValue", filter);

    filter = new FilterDef(
        DataObjectArithmetic::medianValue,
        DataObjectArithmetic::singleDObjInputInfParams,
        medianValueDoc);
    m_filterList.insert("medianValue", filter);

    filter = new FilterDef(
        DataObjectArithmetic::centerOfGravity,
        DataObjectArithmetic::centerOfGravityParams,
        centerOfGravityDoc);
    m_filterList.insert("centroidXY", filter);

    filter = new FilterDef(
        DataObjectArithmetic::localCenterOfGravity,
        DataObjectArithmetic::localCenterOfGravityParams,
        localCenterOfGravityDoc);
    m_filterList.insert("localCentroidXY", filter);

    filter = new FilterDef(
        DataObjectArithmetic::boundingBox, DataObjectArithmetic::boundingBoxParams, boundingBoxDoc);
    m_filterList.insert("boundingBox", filter);

    filter = new FilterDef(
        DataObjectArithmetic::centerOfGravity1Dim,
        DataObjectArithmetic::centerOfGravity1DimParams,
        centerOfGravity1DimDoc);
    m_filterList.insert("centroid1D", filter);

    filter = new FilterDef(
        DataObjectArithmetic::gaussianFit1Dim,
        DataObjectArithmetic::gaussianFit1DimParams,
        gaussianFit1DimDoc);
    m_filterList.insert("gaussianFit1D", filter);

    filter = new FilterDef(
        DataObjectArithmetic::devValue, DataObjectArithmetic::devValueParams, devValueDoc);
    m_filterList.insert("deviationValue", filter);

    filter = new FilterDef(
        DataObjectArithmetic::areEqual, DataObjectArithmetic::doubleDObjInputParams, areEqualDoc);
    m_filterList.insert("areEqual", filter);

    filter = new FilterDef(
        getPercentageThreshold, getPercentageThresholdParams, getPercentageThresholdDoc);
    m_filterList.insert("getPercentageThreshold", filter);

    filter = new FilterDef(autoFocusEstimate, autoFocusEstimateParams, autoFocusEstimateDoc);
    m_filterList.insert("autofocusEstimate", filter);

    filter = new FilterDef(findMultiSpots, findMultiSpotsParams, findMultiSpotsDoc);
    m_filterList.insert("findMultiSpots", filter);

    setInitialized(true); // init method has been finished (independent on retval)
    return retval;
}

//-------------------------------------------------------------------------------------
RetVal DataObjectArithmetic::close(ItomSharedSemaphore* /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    return retval;
}

//-------------------------------------------------------------------------------------
