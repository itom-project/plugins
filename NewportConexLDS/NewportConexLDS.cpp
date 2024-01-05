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
#include <qmessagebox.h>
#include <qplugin.h>
#include <qregularexpression.h>
#include <qstring.h>
#include <qstringlist.h>
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
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
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
    AddInDataIO(), m_pSerialIO(nullptr), m_delayAfterSendCommandMS(100), m_requestTimeOutMS(5000)
{
    ito::Param paramVal(
        "name", ito::ParamBase::String | ito::ParamBase::Readonly, "NewportConexLDS", nullptr);
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
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    paramVal = ito::Param(
        "version",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Controller version.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
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
        "factoryCalibrationState",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Factory calibration information.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    ito::float64 gain[2] = {1.0, 1.0};
    paramVal = ito::Param(
        "gain",
        ito::ParamBase::DoubleArray,
        2,
        gain,
        new ito::DoubleArrayMeta(
            0.0, std::numeric_limits<ito::float64>::max(), 0.0, 0, 2, 2, "Device parameter"),
        tr("Gain of x and y axis.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------------------------- Measurement
    paramVal = ito::Param(
        "frequency",
        ito::ParamBase::Double,
        0.20,
        new ito::DoubleMeta(0.0, std::numeric_limits<ito::float64>::max(), 0.20, "Measurement"),
        tr("Low pass filter frequency.").toLatin1().data());
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
        new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Measurement"),
        tr("Low level power threshold for valid measurement.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //-------------------------------------------------
    paramVal = ito::Param(
        "highLevelPowerThreshold",
        ito::ParamBase::Int,
        0,
        new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Measurement"),
        tr("High level power threshold for valid measurement.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------------------------- Measurement
    paramVal = ito::Param(
        "unit", ito::ParamBase::String, "unknown", tr("Measurement unit.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Measurement"), true);
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
        "timeStemp",
        ito::ParamBase::String | ito::ParamBase::Out,
        "unknown",
        tr("Timestemp of measurement.").toLatin1().data());
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
        "timeStemps",
        ito::ParamBase::StringList | ito::ParamBase::Out,
        nullptr,
        tr("Timestemps corresponding to the measruement data.").toLatin1().data());
    pOut.append(paramVal);

    registerExecFunc(
        "getPositionAndPowerMeasurement",
        pMand,
        pOpt,
        pOut,
        tr("Measure the position and laser power. "
           "It will fill the input dataObject with positions, laser power and timestemps.")
            .toLatin1()
            .data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // the following lines create and register the plugin's dock widget. Delete these lines if the
    // plugin does not have a dock widget.
    DockWidgetNewportConexLDS* dw = new DockWidgetNewportConexLDS(this);

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
        QString state;
        retValue += getFactoryCalibrationState(state);
        if (!retValue.containsError())
        {
            m_params["factoryCalibrationState"].setVal<char*>(state.toUtf8().data());
        }
    }

    if (!retValue.containsError())
    {
        int state;
        retValue += getLaserPowerState(state);
        if (!retValue.containsError())
        {
            m_params["laserPowerState"].setVal<int>(state);
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

        ito::ParamBase* timeStemp = ito::getParamByName(&(*paramsOut), "timeStemp", &retValue);

        if (!retValue.containsError())
        {
            retValue += NewportConexLDS::execGetPositionAndPower(*positionAndPower, *timeStemp);
        }
    }
    else if (funcName == "getPositionAndPowerMeasurement")
    {
        ito::DataObject* dObj = (ito::DataObject*)(*paramsMand)[0].getVal<ito::DataObject*>();
        ito::ParamBase* timeStemps = ito::getParamByName(&(*paramsOut), "timeStemps", &retValue);

        if (!retValue.containsError())
        {
            retValue += NewportConexLDS::execGetPositionAndPowerArray(*dObj, *timeStemps);
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
            int state;
            retValue += getLaserPowerState(state);
            if (!retValue.containsError())
            {
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

    if (!retValue.containsError())
    {
        if (key == "laserPowerState")
        {
            retValue += setLaserPowerState(val->getVal<int>());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "gain")
        {
            retValue += setGain(val->getVal<ito::float64*>());
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
        _sleep(m_delayAfterSendCommandMS);

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
    while (matchIterator.hasNext() and ok)
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
    retVal += sendQuestionWithAnswerInteger("LB?", state);
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
    retVal += sendQuestionWithAnswerDouble("GY?", gain[1]);

    if (retVal.containsError())
    {
        retVal +=
            ito::RetVal(ito::retError, 0, tr("Error during getting gain values.").toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getFrequency(ito::float64& frequency)
{
    ito::RetVal retVal = ito::retOk;
    retVal += sendQuestionWithAnswerDouble("LF?", frequency);

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
    retVal += sendQuestionWithAnswerDouble("PY?", calibrationCoefficients[1]);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during getting calibration coeffients values.").toUtf8().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::getRange(int& range)
{
    ito::RetVal retVal = ito::retOk;
    retVal += sendQuestionWithAnswerInteger("RG?", range);
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
        filterCommand("TB", answer);
        error = answer;
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

    sendStr =
        QString::number(m_controllerAddress).toUtf8() + "GY" + QString::number(gain[1]).toUtf8();
    retVal += sendCommand(sendStr);

    if (retVal.containsError())
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Error during gain setting.").toUtf8().data());
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

    sendStr = QString::number(m_controllerAddress).toUtf8() + "PY" +
        QString::number(calibrationCoefficients[1]).toUtf8();
    retVal += sendCommand(sendStr);

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
    if (retVal.containsError())
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Error during unit setting.").toUtf8().data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::execGetPositionAndPower(
    ito::ParamBase& positionAndPower, ito::ParamBase& timeStemp)
{
    ito::RetVal retValue(ito::retOk);
    ito::float64* values = new double[3]{0.0, 0.0, 0.0};
    retValue += getPositionAndLaserPower(values);

    if (!retValue.containsError())
    {
        positionAndPower.setVal<ito::float64*>(values, 3);
        timeStemp.setVal<char*>(
            QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz").toUtf8().data());
    }
    DELETE_AND_SET_NULL_ARRAY(values);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NewportConexLDS::execGetPositionAndPowerArray(
    ito::DataObject& dObj, ito::ParamBase& timeStemps)
{
    ito::RetVal retValue(ito::retOk);

    if (dObj.getDims() != 2)
    {
        retValue +=
            ito::RetVal(ito::retError, 0, tr("data dObj must be 2 dimensional").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        ito::float64* values = new double[3]{0.0, 0.0, 0.0};
        int planeID = dObj.seekMat(0);
        ito::float64* xPtr = dObj.rowPtr<ito::float64>(0, 0);
        ito::float64* yPtr = dObj.rowPtr<ito::float64>(0, 1);
        ito::float64* powerPtr = dObj.rowPtr<ito::float64>(0, 2);

        int length = dObj.getSize(1);
        ito::ByteArray* time = new ito::ByteArray[length];

        for (int i = 0; i < dObj.getSize(1); i++)
        {
            retValue += getPositionAndLaserPower(values);
            xPtr[i] = values[0];
            yPtr[i] = values[1];
            powerPtr[i] = values[2];
            time[i] =
                QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz").toUtf8().data();
            setAlive();
        }
        dObj.setTag("legendTitle0", "x position");
        dObj.setTag("legendTitle1", "y position");
        dObj.setTag("legendTitle2", "laser power");

        timeStemps.setVal<ito::ByteArray*>(time, length);
        DELETE_AND_SET_NULL_ARRAY(values);
        DELETE_AND_SET_NULL_ARRAY(time);
    }

    return retValue;
}
