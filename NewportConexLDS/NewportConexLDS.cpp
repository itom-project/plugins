/* ********************************************************************
    Plugin "NewportConexLDS" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
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

#include "NewportConexLDS.h"
#include "common/helperCommon.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#include <qdatetime.h>
#include <qelapsedtimer.h>
#include <qmap.h>
#include <qmessagebox.h>
#include <qplugin.h>
#include <qregularexpression.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qthread.h>
#include <qwaitcondition.h>

#include "dockWidgetNewportConexLDS.h"

//----------------------------------------------------------------------------------------------------------------------------------
NewportConexLDSInterface::NewportConexLDSInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("NewportConexLDS");

    m_description = QObject::tr("NewportConexLDS");
    char docstring[] =
"NewportConexLDS is an itom-plugin to use the Newport Conex-LDS autocollimator.\n\
For further information go to: https://www.newport.com/p/CONEX-LDS\n\
\n\
This plugin has been developed using SerialIO interface with following default parameters:\n\
\n\
========== ======================================================\n\
Baud Rate  921600 (default for RS232)\n\
Data Bits  8\n\
Parity     None\n\
Stop bits  1\n\
endline    \\r\\n\n\
========== ======================================================\n\
";
    m_detaildescription = QObject::tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal(
        "serialIOInstance",
        ito::ParamBase::HWRef | ito::ParamBase::In,
        nullptr,
        tr("An opened serial port.").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param(
        "controllerAddress",
        ito::ParamBase::Int,
        1,
        new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Device parameter"),
        tr("Controller address.").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
NewportConexLDSInterface::~NewportConexLDSInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDSInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(NewportConexLDS)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDSInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(NewportConexLDS)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
NewportConexLDS::NewportConexLDS() :
    AddInDataIO(), m_pSerialIO(nullptr), m_delayAfterSendCommandMS(10), m_requestTimeOutMS(5000)
{
    ito::Param paramVal = ito::Param(
        "name",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "NewportConexLDS",
        tr("Plugin name.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------------------------- SerialIO parameter
    paramVal = ito::Param(
        "requestTimeout",
        ito::ParamBase::Int,
        m_requestTimeOutMS,
        new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "SerialIO parameter"),
        tr("Request timeout in ms for the SerialIO interface.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------------------------- device parameter
    paramVal = ito::Param(
        "deviceName",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Device name.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------------------------- device parameter
    paramVal = ito::Param(
        "commandError",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Command error string.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    paramVal = ito::Param(
        "version",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Controller version.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    paramVal = ito::Param(
        "laserPowerState",
        ito::ParamBase::Int,
        0,
        new ito::IntMeta(0, 1, 1, "Device parameter"),
        tr("Enable/Disable laser power (0==OFF, 1==ON).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    paramVal = ito::Param(
        "configurationState",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Configuration state (MEASURE, READY, CONFIGURATION).").toLatin1().data());
    ito::StringMeta sm(ito::StringMeta::String, "", "Device parameter");
    sm.addItem("MEASURE");
    sm.addItem("READY");
    sm.addItem("CONFIGURATION");
    paramVal.setMeta(&sm, false);
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    paramVal = ito::Param(
        "enableConfiguration",
        ito::ParamBase::Int,
        0,
        new ito::IntMeta(0, 1, 1, "Device parameter"),
        tr("Enable/Disable configuration (0==OFF, 1==ON). Laser is disabled when it is on.")
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    paramVal = ito::Param(
        "factoryCalibrationState",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Factory calibration information.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    ito::float64 gain[2] = {1.0, 1.0};
    paramVal = ito::Param(
        "gain",
        ito::ParamBase::DoubleArray,
        2,
        gain,
        new ito::DoubleArrayMeta(0.0, 200.0, 0.0, 0, 2, 2, "Device parameter"),
        tr("Gain of x and y axis.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------------------------- Measurement
    ito::float64 offset[2] = {1.0, 1.0};
    paramVal = ito::Param(
        "offset",
        ito::ParamBase::DoubleArray,
        2,
        gain,
        new ito::DoubleArrayMeta(
            -std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::max(),
            0.0,
            0,
            2,
            2,
            "Measurement"),
        tr("Offset values of x and y axis.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "frequency",
        ito::ParamBase::Double,
        1.0,
        new ito::DoubleMeta(1.0, 2000, 1, "Measurement"),
        tr("Low pass filter frequency as response time before ouputing measurement that is "
           "inversely proportional to the low pass filter frequency. Following frequencies [Hz] "
           "corresponds to a resolution [%1rad] (RMS noise): 1 == 0.03, 20 == 0.013, 50 == "
           "0.021, "
           "100 == 0.030, 200 == 0.042, 500 == 0.067, 1000 == 0.095, 2000 == 0.134.")
            .arg(QLatin1String("\u00B5"))
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    ito::float64 calibration[2] = {1.0, 1.0};
    paramVal = ito::Param(
        "calibrationCoefficients",
        ito::ParamBase::DoubleArray,
        2,
        gain,
        new ito::DoubleArrayMeta(
            0.0, std::numeric_limits<ito::float64>::max(), 0.0, 0, 2, 2, "Measurement"),
        tr("Calibration coefficients of x and y axis.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    paramVal = ito::Param(
        "range",
        ito::ParamBase::Int,
        2000,
        new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Measurement"),
        tr("Value range.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    paramVal = ito::Param(
        "lowLevelPowerThreshold",
        ito::ParamBase::Int,
        0,
        new ito::IntMeta(0, 2000, 1, "Measurement"),
        tr("Low level power threshold for valid measurement.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    paramVal = ito::Param(
        "highLevelPowerThreshold",
        ito::ParamBase::Int,
        0,
        new ito::IntMeta(0, 2000, 1, "Measurement"),
        tr("High level power threshold for valid measurement.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------------------------- Measurement
    paramVal = ito::Param(
        "unit", ito::ParamBase::String, "unknown", tr("Measurement unit.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Measurement"), true);
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------------------------- EXEC functions
    QVector<ito::Param> pMand = QVector<ito::Param>();
    QVector<ito::Param> pOpt = QVector<ito::Param>();
    QVector<ito::Param> pOut = QVector<ito::Param>();

    ito::float64 values[3] = {0.0, 0.0, 0.0};
    paramVal = ito::Param(
        "positionAndPower",
        ito::ParamBase::DoubleArray | ito::ParamBase::Out,
        3,
        values,
        new ito::DoubleArrayMeta(
            std::numeric_limits<ito::float64>::min(),
            std::numeric_limits<ito::float64>::max(),
            std::numeric_limits<ito::float64>::epsilon(),
            0,
            3,
            3,
            "Measurement"),
        tr("Positions of x and y axis.").toLatin1().data());
    pOut.append(paramVal);

    paramVal = ito::Param(
        "timeStamp",
        ito::ParamBase::String | ito::ParamBase::Out,
        "unknown",
        tr("TimeStamp of measurement.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Measurement"), true);
    pOut.append(paramVal);

    registerExecFunc(
        "getPositionAndPower",
        pMand,
        pOpt,
        pOut,
        tr("Measure the position and laser power.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //-------------------------------------------------
    paramVal = ito::Param(
        "data",
        ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out,
        nullptr,
        tr("Measruement data X, Y, position and laser power.").toLatin1().data());
    pMand.append(paramVal);

    paramVal = ito::Param(
        "interval",
        ito::ParamBase::Int | ito::ParamBase::In,
        200,
        new ito::IntMeta(200, std::numeric_limits<int>::max(), 1, "Measurement"),
        tr("Interval between measruement points in ms.").toLatin1().data());
    pOpt.append(paramVal);

    paramVal = ito::Param(
        "timeStamps",
        ito::ParamBase::StringList | ito::ParamBase::Out,
        nullptr,
        tr("TimeStamps corresponding to the measruement data.").toLatin1().data());
    pOut.append(paramVal);

    registerExecFunc(
        "getPositionAndPowerMeasurement",
        pMand,
        pOpt,
        pOut,
        tr("Measure the position and laser power. "
           "It will fill the input dataObject with positions, laser power and timestamps. Please "
           "note that this function blocks itom until the entire measurement has been carried out.")
            .toLatin1()
            .data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    DockWidgetNewportConexLDS* dw = new DockWidgetNewportConexLDS(getID(), this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
        QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
NewportConexLDS::~NewportConexLDS()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (reinterpret_cast<ito::AddInBase*>((*paramsMand)[0].getVal<void*>())
            ->getBasePlugin()
            ->getType() &
        (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSerialIO = (ito::AddInDataIO*)(*paramsMand)[0].getVal<void*>();
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Input parameter is not a dataIO instance of the SerialIO Plugin!")
                .toLatin1()
                .data());
    }

    if (!retValue.containsError())
    {
        retValue += m_pSerialIO->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 921600)),
            nullptr);
        retValue += m_pSerialIO->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)),
            nullptr);
        retValue += m_pSerialIO->setParam(
            QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)),
            nullptr);
        retValue += m_pSerialIO->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)),
            nullptr);
        retValue += m_pSerialIO->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 0)),
            nullptr);
        retValue += m_pSerialIO->setParam(
            QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("endline", ito::ParamBase::String, "\r\n")),
            nullptr);

        QSharedPointer<QVector<ito::ParamBase>> _dummy;
        m_pSerialIO->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
        m_pSerialIO->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);
    }

    if (!retValue.containsError())
    {
        m_controllerAddress = (*paramsOpt)[0].getVal<int>();
    }

    if (!retValue.containsError())
    {
        ConfigurationState state;
        retValue += getConfigurationState(state);

        if (!retValue.containsError())
        {
            m_params["configurationState"].setVal<char*>(
                configurationEnumToString(state).toUtf8().data());
        }

        switch (state)
        {
        case NewportConexLDS::MEASURE:
            m_params["laserPowerState"].setVal<int>(1);
            break;
        case NewportConexLDS::READY:
            m_params["laserPowerState"].setVal<int>(0);
            break;
        case NewportConexLDS::CONFIGURATION:
            m_params["laserPowerState"].setVal<int>(0);
            break;
        default:
            break;
        }
    }

    if (!retValue.containsError())
    {
        QString version;
        QString deviceName;
        retValue += getVersion(version, deviceName);
        if (!retValue.containsError())
        {
            m_params["deviceName"].setVal<char*>(version.toUtf8().data());
            m_params["version"].setVal<char*>(deviceName.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        QString error;
        retValue += getError(error);
        if (!retValue.containsError())
        {
            m_params["commandError"].setVal<char*>(error.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        QString state;
        retValue += getFactoryCalibrationState(state);
        if (!retValue.containsError())
        {
            m_params["factoryCalibrationState"].setVal<char*>(state.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        ito::float64* gain = new double[2];
        retValue += getGain(gain);
        if (!retValue.containsError())
        {
            m_params["gain"].setVal<double*>(gain, 2);
        }
        DELETE_AND_SET_NULL_ARRAY(gain);
    }

    if (!retValue.containsError())
    {
        ito::float64* offset = new double[2];
        retValue += getOffset(offset);
        if (!retValue.containsError())
        {
            m_params["offset"].setVal<double*>(offset, 2);
        }
        DELETE_AND_SET_NULL_ARRAY(offset);
    }

    if (!retValue.containsError())
    {
        ito::float64* calibration = new double[2];
        retValue += getCalibrationCoefficients(calibration);
        if (!retValue.containsError())
        {
            m_params["calibrationCoefficients"].setVal<double*>(calibration, 2);
        }
        DELETE_AND_SET_NULL_ARRAY(calibration);
    }

    if (!retValue.containsError())
    {
        int range;
        retValue += getRange(range);
        if (!retValue.containsError())
        {
            m_params["range"].setVal<int>(range);
        }
    }

    if (!retValue.containsError())
    {
        int level;
        retValue += getLowLevelPowerThreshold(level);
        if (!retValue.containsError())
        {
            m_params["lowLevelPowerThreshold"].setVal<int>(level);
        }
    }

    if (!retValue.containsError())
    {
        int level;
        retValue += getHighLevelPowerThreshold(level);
        if (!retValue.containsError())
        {
            m_params["highLevelPowerThreshold"].setVal<int>(level);
        }
    }

    if (!retValue.containsError())
    {
        QString state;
        retValue += getUnit(state);
        if (!retValue.containsError())
        {
            m_params["unit"].setVal<char*>(state.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); // init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    dockWidgetVisibilityChanged(false);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::execFunc(
    const QString funcName,
    QSharedPointer<QVector<ito::ParamBase>> paramsMand,
    QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
    QSharedPointer<QVector<ito::ParamBase>> paramsOut,
    ItomSharedSemaphore* waitCond)
{
    ito::RetVal retValue = ito::retOk;


    if (funcName == "getPositionAndPower")
    {
        ito::ParamBase* positionAndPower =
            ito::getParamByName(&(*paramsOut), "positionAndPower", &retValue);

        ito::ParamBase* timeStamp = ito::getParamByName(&(*paramsOut), "timeStamp", &retValue);

        if (!retValue.containsError())
        {
            retValue += NewportConexLDS::execGetPositionAndPower(*positionAndPower, *timeStamp);
        }
    }
    else if (funcName == "getPositionAndPowerMeasurement")
    {
        ito::DataObject* dObj = (ito::DataObject*)(*paramsMand)[0].getVal<ito::DataObject*>();
        ito::ParamBase* timeStamps = ito::getParamByName(&(*paramsOut), "timeStamps", &retValue);
        int interval = (*paramsOpt)[0].getVal<int>();

        if (!retValue.containsError())
        {
            retValue += NewportConexLDS::execGetPositionAndPowerArray(*dObj, *timeStamps, interval);
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::autoGrabbing(
    QSharedPointer<ito::float64> values, ItomSharedSemaphore* waitCond)
{
    ito::RetVal retval(ito::retOk);
    retval += getPositionAndLaserPower(values.data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        // gets the parameter key from m_params map (read-only is allowed, since we only want to get
        // the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (key == "laserPowerState")
        {
            int laser = 0;
            ConfigurationState state;
            retValue += getConfigurationState(state);

            if (!retValue.containsError())
            {
                switch (state)
                {
                case NewportConexLDS::MEASURE:
                    laser = 1;
                    break;
                case NewportConexLDS::READY:
                    laser = 0;
                    break;
                case NewportConexLDS::CONFIGURATION:
                    laser = 0;
                    break;
                default:
                    break;
                }
                it->setVal<int>(state);
            }
        }
        else if (key == "gain")
        {
            ito::float64* gain = new double[2]{0.0, 0.0};
            retValue += getGain(gain);
            if (!retValue.containsError())
            {
                it->setVal<ito::float64*>(gain, 2);
            }
            DELETE_AND_SET_NULL_ARRAY(gain);
        }
        else if (key == "configurationState")
        {
            ConfigurationState state;
            retValue += getConfigurationState(state);

            if (!retValue.containsError())
            {
                it->setVal<char*>(configurationEnumToString(state).toUtf8().data());
            }
        }
        else if (key == "offset")
        {
            ito::float64* offset = new double[2]{0.0, 0.0};
            retValue += getOffset(offset);
            if (!retValue.containsError())
            {
                it->setVal<ito::float64*>(offset, 2);
            }
            DELETE_AND_SET_NULL_ARRAY(offset);
        }
        else if (key == "frequency")
        {
            ito::float64 frequency;
            retValue += getFrequency(frequency);
            if (!retValue.containsError())
            {
                it->setVal<ito::float64>(frequency);
            }
        }
        else if (key == "calibrationCoefficients")
        {
            ito::float64* calibration = new double[2];
            retValue += getCalibrationCoefficients(calibration);
            if (!retValue.containsError())
            {
                it->setVal<ito::float64*>(calibration, 2);
            }
            DELETE_AND_SET_NULL_ARRAY(calibration);
        }
        else if (key == "range")
        {
            int range;
            retValue += getRange(range);
            if (!retValue.containsError())
            {
                it->setVal<int>(range);
            }
        }
        else if (key == "lowLevelPowerThreshold")
        {
            int level;
            retValue += getLowLevelPowerThreshold(level);
            if (!retValue.containsError())
            {
                it->setVal<int>(level);
            }
        }
        else if (key == "highLevelPowerThreshold")
        {
            int level;
            retValue += getHighLevelPowerThreshold(level);
            if (!retValue.containsError())
            {
                it->setVal<int>(level);
            }
        }
        else if (key == "unit")
        {
            QString unit;
            retValue += getUnit(unit);
            if (!retValue.containsError())
            {
                it->setVal<char*>(unit.toUtf8().data());
            }
        }
        else if (key == "commandError")
        {
            QString error;
            retValue += getError(error);
            if (!retValue.containsError())
            {
                it->setVal<char*>(error.toUtf8().data());
            }
        }
        *val = it.value();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setParam(
    QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        // gets the parameter key from m_params map (read-only is not allowed and leads to
        // ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        // here the new parameter is checked whether its type corresponds or can be cast into the
        //  value in m_params and whether the new type fits to the requirements of any possible
        //  meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    ConfigurationState config = READY;
    retValue += getConfigurationState(config);

    if (config != CONFIGURATION)
    {
        if (!(key == "laserPowerState" || key == "enableConfiguration"))
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("The parameter '%1' cannot be set since the configuration state is '%2'.")
                    .arg(key)
                    .arg(configurationEnumToString(config))
                    .toUtf8()
                    .data());
        }
    }

    if (!retValue.containsError())
    {
        if (key == "laserPowerState")
        {
            if (config == CONFIGURATION)
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("The parameter '%1' cannot be set since the configuration state is "
                       "'%2'.")
                        .arg(key)
                        .arg(configurationEnumToString(config))
                        .toUtf8()
                        .data());
            }

            int state = val->getVal<int>();
            if (!(state && config == MEASURE))
            {
                if (!retValue.containsError())
                {
                    retValue += setLaserPowerState(state);
                    ConfigurationState config;

                    retValue += getConfigurationState(config);
                    m_params["configurationState"].setVal<char*>(
                        configurationEnumToString(config).toUtf8().data());
                    retValue += it->copyValueFrom(&(*val));
                }
            }
        }
        else if (key == "gain")
        {
            retValue += setGain(val->getVal<ito::float64*>());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "offset")
        {
            retValue += setOffset(val->getVal<ito::float64*>());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "frequency")
        {
            retValue += setFrequency(val->getVal<ito::float64>());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "calibrationCoefficients")
        {
            retValue += setCalibrationCoefficients(val->getVal<ito::float64*>());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "lowLevelPowerThreshold")
        {
            retValue += setLowLevelPowerThreshold(val->getVal<int>());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "highLevelPowerThreshold")
        {
            retValue += setHighLevelPowerThreshold(val->getVal<int>());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "enableConfiguration")
        {
            int requiredState = val->getVal<int>();
            if (config == MEASURE)
            {
                retValue += setLaserPowerState(0);
            }
            retValue += setConfigurationState(requiredState);
            retValue += it->copyValueFrom(&(*val));
        }
        else
        {
            // all parameters that don't need further checks can simply be assigned
            // to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(
            m_params); // send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void NewportConexLDS::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget* widget = getDockWidget()->widget();
        if (visible)
        {
            connect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                widget,
                SLOT(parametersChanged(QMap<QString, ito::Param>)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                widget,
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal NewportConexLDS::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogNewportConexLDS(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::sendCommand(const QByteArray& command)
{
    ito::RetVal retVal;
    retVal += m_pSerialIO->setVal(command.data(), command.length(), nullptr);

    if (m_delayAfterSendCommandMS > 0)
    {
        QMutex mutex;
        mutex.lock();
        QWaitCondition waitCondition;
        waitCondition.wait(&mutex, m_delayAfterSendCommandMS);
        mutex.unlock();
    }
    setAlive();
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::readString(QByteArray& result, int& len)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;

    bool done = false;
    int curFrom = 0;
    int pos = 0;

    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";

    QByteArray endline;

    QSharedPointer<ito::Param> param(new ito::Param("endline"));
    retValue += m_pSerialIO->getParam(param, nullptr);

    if (param->getType() == (ito::ParamBase::String & ito::paramTypeMask))
    {
        char* temp = param->getVal<char*>(); // borrowed reference
        int len = temp[0] == 0 ? 0 : (temp[1] == 0 ? 1 : (temp[2] == 0 ? 2 : 3));
        endline = QByteArray::fromRawData(temp, len);
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("could not read endline parameter from serial port").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        len = 0;
        timer.start();
        QThread::msleep(m_delayAfterSendCommandMS);

        while (!done && !retValue.containsError())
        {
            *curBufLen = buflen;
            retValue += m_pSerialIO->getVal(curBuf, curBufLen, nullptr);


            if (!retValue.containsError())
            {
                result += QByteArray(curBuf.data(), *curBufLen);
                pos = result.indexOf(endline, curFrom);
                curFrom = qMax(0, result.length() - 3);

                if (pos >= 0) // found
                {
                    done = true;
                    result = result.left(pos);
                }
            }

            if (!done && timer.elapsed() > m_requestTimeOutMS && m_requestTimeOutMS >= 0)
            {
                retValue += ito::RetVal(
                    ito::retError,
                    m_delayAfterSendCommandMS,
                    tr("timeout during read string.").toLatin1().data());
            }
        }

        len = result.length();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::sendQuestionWithAnswerString(
    const QByteArray& questionCommand, QByteArray& answer)
{
    QByteArray questionCommand_ = QString::number(m_controllerAddress).toUtf8() + questionCommand;
    int readSigns;
    ito::RetVal retValue = sendCommand(questionCommand_);
    retValue += readString(answer, readSigns);
    filterCommand(questionCommand, answer);
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::sendQuestionWithAnswerDouble(
    const QByteArray& questionCommand, double& answer)
{
    QByteArray questionCommand_ = QString::number(m_controllerAddress).toUtf8() + questionCommand;
    int readSigns;
    QByteArray answerStr;
    bool ok;

    ito::RetVal retValue = sendCommand(questionCommand_);
    retValue += readString(answerStr, readSigns);

    if (questionCommand_.contains("?"))
    {
        questionCommand_.replace("?", "");
    }

    filterCommand(questionCommand_, answerStr);
    answer = answerStr.toDouble(&ok);

    if (!ok)
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Error during SendQuestionWithAnswerDouble, converting %1 to double value.")
                .arg(answerStr.constData())
                .toLatin1()
                .data());
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::sendQuestionWithAnswerDoubleArray(
    const QByteArray& questionCommand, double* answer, const int number)
{
    QByteArray questionCommand_ = QString::number(m_controllerAddress).toUtf8() + questionCommand;
    int readSigns;
    QByteArray answerStr;
    bool ok = true;
    ito::RetVal retValue = sendCommand(questionCommand_);
    retValue += readString(answerStr, readSigns);

    if (questionCommand_.contains("?"))
    {
        questionCommand_.replace("?", "");
    }

    filterCommand(questionCommand_, answerStr);

    QRegularExpression regex("-?\\d+(\\.\\d+)?");
    QRegularExpressionMatchIterator matchIterator = regex.globalMatch(answerStr);

    // Extract and print all matched values
    int n = 0;
    QRegularExpressionMatch match;
    QString matchedValue;
    while (matchIterator.hasNext() && ok)
    {
        match = matchIterator.next();
        matchedValue = match.captured();
        answer[n] = matchedValue.toDouble(&ok);
        n++;
    }

    if (!ok)
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Error during SendQuestionWithAnswerDouble, converting %1 to double value.")
                .arg(answerStr.constData())
                .toLatin1()
                .data());
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::sendQuestionWithAnswerInteger(
    const QByteArray& questionCommand, int& answer)
{
    QByteArray questionCommand_ = QString::number(m_controllerAddress).toUtf8() + questionCommand;
    int readSigns;
    QByteArray _answer;
    bool ok;
    ito::RetVal retValue = sendCommand(questionCommand_);
    retValue += readString(_answer, readSigns);

    if (questionCommand_.contains("?"))
    {
        questionCommand_.replace("?", "");
    }

    filterCommand(questionCommand_, _answer);
    answer = _answer.toInt(&ok);

    if (!ok)
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Error during SendQuestionWithAnswerInteger, converting %1 to double value.")
                .arg(_answer.constData())
                .toLatin1()
                .data());
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void NewportConexLDS::filterCommand(const QByteArray& questionCommand, QByteArray& answer)
{
    QRegularExpression regex("^(" + questionCommand + ")\\s*(.+)$");
    QRegularExpressionMatch match = regex.match(answer);

    if (match.hasMatch())
    {
        int index = answer.indexOf(match.captured(0).toUtf8().data());

        if (index != -1)
        {
            answer.remove(index, questionCommand.length());
        }
        answer = answer.trimmed();
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getVersion(QString& version, QString& deviceName)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray answer;
    retVal += sendQuestionWithAnswerString("VE", answer);

    QString error;
    retVal += getError(error);

    if (!retVal.containsError())
    {
        QRegularExpression regex("([A-Z\\-]+)\\s([0-9.]+)");
        QRegularExpressionMatch match = regex.match(answer);

        if (match.hasMatch())
        {
            version = match.captured(1);
            deviceName = match.captured(2);
        }
        else
        {
            retVal += ito::RetVal(
                ito::retError,
                0,
                tr("Error during version request with answer: '%1'")
                    .arg(answer.constData())
                    .toUtf8()
                    .data());
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getLaserPowerState(int& state)
{
    ito::RetVal retVal = ito::retOk;

    ConfigurationState config;
    retVal += getConfigurationState(config);

    state = (config == MEASURE) ? 1 : 0;

    if (retVal.containsError())
    {
        retVal +=
            ito::RetVal(ito::retError, 0, tr("Error during laser power request.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getFactoryCalibrationState(QString& state)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray answer;
    retVal += sendQuestionWithAnswerString("CD?", answer);

    QString error;
    retVal += getError(error);

    if (!retVal.containsError())
    {
        filterCommand(QString::number(m_controllerAddress).toUtf8() + "CD", answer);
        state = answer;
    }
    else
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during factory calibration state request with answer: '%1'")
                .arg(answer.constData())
                .toUtf8()
                .data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getGain(ito::float64* gain)
{
    ito::RetVal retVal = ito::retOk;

    retVal += sendQuestionWithAnswerDouble("GX?", gain[0]);

    QString error;
    retVal += getError(error);

    retVal += sendQuestionWithAnswerDouble("GY?", gain[1]);

    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal +=
            ito::RetVal(ito::retError, 0, tr("Error during getting gain values.").toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getOffset(ito::float64* offset)
{
    ito::RetVal retVal = ito::retOk;

    retVal += sendQuestionWithAnswerDouble("OX?", offset[0]);

    QString error;
    retVal += getError(error);
    retVal += sendQuestionWithAnswerDouble("OY?", offset[1]);

    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(
            ito::retError, 0, tr("Error during getting offset values.").toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getFrequency(ito::float64& frequency)
{
    ito::RetVal retVal = ito::retOk;
    retVal += sendQuestionWithAnswerDouble("LF?", frequency);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal +=
            ito::RetVal(ito::retError, 0, tr("Error during getting gain values.").toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getPositionAndLaserPower(ito::float64* values)
{
    ito::RetVal retVal = ito::retOk;
    retVal += sendQuestionWithAnswerDoubleArray("GP", values, 3);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal +=
            ito::RetVal(ito::retError, 0, tr("Error during getting gain values.").toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getCalibrationCoefficients(ito::float64* calibrationCoefficients)
{
    ito::RetVal retVal = ito::retOk;
    retVal += sendQuestionWithAnswerDouble("PX?", calibrationCoefficients[0]);

    QString error;
    retVal += getError(error);
    retVal += sendQuestionWithAnswerDouble("PY?", calibrationCoefficients[1]);

    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during getting calibration coefficients values.").toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getRange(int& range)
{
    ito::RetVal retVal = ito::retOk;

    retVal += sendQuestionWithAnswerInteger("RG?", range);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal +=
            ito::RetVal(ito::retError, 0, tr("Error during range value request.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getLowLevelPowerThreshold(int& level)
{
    ito::RetVal retVal = ito::retOk;

    retVal += sendQuestionWithAnswerInteger("SL?", level);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during low level power threshold request.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getHighLevelPowerThreshold(int& level)
{
    ito::RetVal retVal = ito::retOk;

    retVal += sendQuestionWithAnswerInteger("SR?", level);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during high level power threshold request.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getUnit(QString& unit)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray answer;
    QByteArray questionCommand_ = QString::number(m_controllerAddress).toUtf8() + "SU?";
    retVal += sendQuestionWithAnswerString("SU?", answer);

    QString error;
    retVal += getError(error);

    if (!retVal.containsError())
    {
        if (questionCommand_.contains("?"))
        {
            questionCommand_.replace("?", "");
        }
        filterCommand(questionCommand_, answer);
        unit = answer;
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getError(QString& error)
{
    ito::RetVal retVal = ito::retOk;
    QByteArray answer;
    retVal += sendQuestionWithAnswerString("TB", answer);

    if (!retVal.containsError())
    {
        filterCommand(QString::number(m_controllerAddress).toUtf8() + "TB@", answer);
        error = answer;
    }

    if (error != "No error")
    {
        retVal += ito::RetVal(ito::retError, 0, error.toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getConfigurationState(ConfigurationState& state)
{
    ito::RetVal retVal = ito::retOk;
    int answer;
    retVal += sendQuestionWithAnswerInteger("PW?", answer);
    state = static_cast<ConfigurationState>(answer);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(
            ito::retError, 0, tr("Error during configuration state request.").toUtf8().data());
    }
    else
    {
        state = static_cast<ConfigurationState>(answer);
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setLaserPowerState(const int state)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray sendStr =
        QString::number(m_controllerAddress).toUtf8() + "LB" + QString::number(state).toUtf8();
    retVal += sendCommand(sendStr);

    QThread::msleep(5000);
    setAlive();

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal +=
            ito::RetVal(ito::retError, 0, tr("Error during laser power enabling.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setGain(const ito::float64* gain)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray sendStr =
        QString::number(m_controllerAddress).toUtf8() + "GX" + QString::number(gain[0]).toUtf8();
    retVal += sendCommand(sendStr);

    QString error;
    retVal += getError(error);

    sendStr =
        QString::number(m_controllerAddress).toUtf8() + "GY" + QString::number(gain[1]).toUtf8();
    retVal += sendCommand(sendStr);

    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Error during gain setting.").toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setOffset(const ito::float64* offset)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray sendStr =
        QString::number(m_controllerAddress).toUtf8() + "OX" + QString::number(offset[0]).toUtf8();
    retVal += sendCommand(sendStr);

    QString error;
    retVal += getError(error);

    sendStr =
        QString::number(m_controllerAddress).toUtf8() + "OY" + QString::number(offset[1]).toUtf8();
    retVal += sendCommand(sendStr);

    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Error during offset setting.").toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setFrequency(const ito::float64 frequency)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray sendStr =
        QString::number(m_controllerAddress).toUtf8() + "LF" + QString::number(frequency).toUtf8();
    retVal += sendCommand(sendStr);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal +=
            ito::RetVal(ito::retError, 0, tr("Error during setting frequency.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setCalibrationCoefficients(const ito::float64* calibrationCoefficients)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray sendStr = QString::number(m_controllerAddress).toUtf8() + "PX" +
        QString::number(calibrationCoefficients[0]).toUtf8();
    retVal += sendCommand(sendStr);

    QString error;
    retVal += getError(error);

    sendStr = QString::number(m_controllerAddress).toUtf8() + "PY" +
        QString::number(calibrationCoefficients[1]).toUtf8();
    retVal += sendCommand(sendStr);

    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(
            ito::retError, 0, tr("Error during calibrationCoefficients setting.").toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setRange(const int& range)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray sendStr =
        QString::number(m_controllerAddress).toUtf8() + "RG" + QString::number(range).toUtf8();
    retVal += sendCommand(sendStr);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Error during range setting.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setLowLevelPowerThreshold(const int& level)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray sendStr =
        QString::number(m_controllerAddress).toUtf8() + "SL" + QString::number(level).toUtf8();
    retVal += sendCommand(sendStr);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during low level power threshold setting.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setHighLevelPowerThreshold(const int& level)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray sendStr =
        QString::number(m_controllerAddress).toUtf8() + "SR" + QString::number(level).toUtf8();
    retVal += sendCommand(sendStr);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during high level power threshold setting.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setUnit(const QString& unit)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray sendStr = QString::number(m_controllerAddress).toUtf8() + "SU" + unit.toUtf8();
    retVal += sendCommand(sendStr);

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Error during unit setting.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::setConfigurationState(const int& state)
{
    ito::RetVal retVal = ito::retOk;

    QByteArray sendStr =
        QString::number(m_controllerAddress).toUtf8() + "PW" + QString::number(state).toUtf8();

    retVal += sendCommand(sendStr);

    if (!state)
    {
        QThread::msleep(5000);
        setAlive();
    }

    QString error;
    retVal += getError(error);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(
            ito::retError, 0, tr("Error during configuration state setting.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::execGetPositionAndPower(
    ito::ParamBase& positionAndPower, ito::ParamBase& timeStamp)
{
    ito::RetVal retValue(ito::retOk);

    ConfigurationState state;
    retValue += getConfigurationState(state);
    if (state != MEASURE)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("Measurement cannot be executed since laser power is not ON. Disable configuration "
               "state using the parameter 'configurationState' and enable the laser using "
               "'laserPowerState'")
                .toUtf8()
                .data());
    }

    ito::float64* values = new ito::float64[3]{0.0, 0.0, 0.0};
    retValue += getPositionAndLaserPower(values);

    if (!retValue.containsError())
    {
        positionAndPower.setVal<ito::float64*>(values, 3);
        timeStamp.setVal<char*>(
            QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz").toUtf8().data());
    }
    DELETE_AND_SET_NULL_ARRAY(values);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::execGetPositionAndPowerArray(
    ito::DataObject& dObj, ito::ParamBase& timeStamps, const int& interval)
{
    ito::RetVal retValue(ito::retOk);

    if (dObj.getDims() != 2)
    {
        return ito::RetVal(
            ito::retError, 0, tr("data dObj must be 2 dimensional").toLatin1().data());
    }

    if (dObj.getType() != ito::tFloat64)
    {
        return ito::RetVal(
            ito::retError, 0, tr("type of dObj must by 'float64'.").toLatin1().data());
    }

    if (dObj.getSize(0) != 3)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("first axis of dObj must be 3 of shape [x, y, power level].").toLatin1().data());
    }

    ConfigurationState state;
    retValue += getConfigurationState(state);
    if (state != MEASURE)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("Measurement cannot be executed since laser power is not ON. Disable configuration "
               "state using the parameter 'configurationState' and enable the laser using "
               "'laserPowerState")
                .toUtf8()
                .data());
    }

    if (!retValue.containsError())
    {
        ito::float64* values = new ito::float64[3]{0.0, 0.0, 0.0};
        int planeID = dObj.seekMat(0);
        ito::float64* xPtr = dObj.rowPtr<ito::float64>(0, 0);
        ito::float64* yPtr = dObj.rowPtr<ito::float64>(0, 1);
        ito::float64* powerPtr = dObj.rowPtr<ito::float64>(0, 2);

        int length = dObj.getSize(1);
        ito::ByteArray* time = new ito::ByteArray[length];

        QElapsedTimer timer;
        timer.start();

        int elapsed = timer.elapsed();
        int remaining = interval - elapsed;

        for (int i = 0; i < dObj.getSize(1); i++)
        {
            retValue += getPositionAndLaserPower(values);
            xPtr[i] = values[0];
            yPtr[i] = values[1];
            powerPtr[i] = values[2];
            time[i] =
                QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz").toUtf8().data();
            setAlive();

            elapsed = timer.elapsed();
            remaining = interval - elapsed;
            if (remaining > 0)
                QThread::msleep(remaining);

            timer.restart();
        }
        dObj.setTag("legendTitle0", "x position");
        dObj.setTag("legendTitle1", "y position");
        dObj.setTag("legendTitle2", "laser power");

        timeStamps.setVal<ito::ByteArray*>(time, length);
        DELETE_AND_SET_NULL_ARRAY(values);
        DELETE_AND_SET_NULL_ARRAY(time);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString NewportConexLDS::configurationEnumToString(const ConfigurationState& state)
{
    QString conf;
    switch (state)
    {
    case MEASURE:
        conf = "MEASURE";
        break;
    case READY:
        conf = "READY";
        break;
    case CONFIGURATION:
        conf = "CONFIGURATION";
        break;
    default:
        break;
    }
    return conf;
}
