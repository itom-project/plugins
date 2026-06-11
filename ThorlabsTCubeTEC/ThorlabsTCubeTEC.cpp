/* ********************************************************************
Plugin "ThorlabsTCubeTEC" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2022, Institut für Technische Optik (ITO),
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

#include "ThorlabsTCubeTEC.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#define NOMINMAX // https://stackoverflow.com/questions/22744262/cant-call-stdmax-because-minwindef-h-defines-max

#include <qmessagebox.h>
#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>

#include "dockWidgetThorlabsTCubeTEC.h"

#include "Thorlabs.MotionControl.TCube.TEC.h"

QList<QByteArray> ThorlabsTCubeTEC::openedDevices = QList<QByteArray>();
int ThorlabsTCubeTEC::numberOfKinesisSimulatorConnections = 0;

//-------------------------------------------------------------------------------------
ThorlabsTCubeTECInterface::ThorlabsTCubeTECInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO; // any grabber is a dataIO device AND its subtype
                                               // grabber (bitmask -> therefore the OR-combination).
    setObjectName("ThorlabsTCubeTEC");

    m_description = QObject::tr("ThorlabsTCubeTEC");

    m_detaildescription = QObject::tr(
"ThorlabsTCubeTEC is an plugin to control the Thorlabs T-Cube TEC controller for thermoelectric coolers. \n\
\n\
It requires the new Kinesis driver package from Thorlabs and implements the interface Thorlabs.MotionControl.TCube.TEC.\n\
\n\
Please install the Kinesis driver package in advance with the same bit-version (32/64bit) than itom. \n\
\n\
This plugin has been tested with both the simulated T-Cube TEC controller of Kinesis and a real device.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);
    m_initParamsOpt.append(ito::Param(
        "serialNo",
        ito::ParamBase::String,
        "",
        tr("Serial number of the device to be loaded, if empty, the first device that can be "
           "opened will be opened")
            .toLatin1()
            .data()));

    auto p = ito::Param(
        "sensorType",
        ito::ParamBase::String,
        "Transducer",
        tr("Connected sensor type.").toLatin1().data());
    auto sm = new ito::StringMeta(ito::StringMeta::String, "Transducer");
    sm->addItem("TH20kOhm");
    sm->addItem("TH200kOhm");
    p.setMeta(sm, true);
    m_initParamsOpt.append(p);

    m_initParamsOpt.append(ito::Param(
        "connectToKinesisSimulator",
        ito::ParamBase::Int,
        0,
        1,
        0,
        tr("If 1, a connection to the running Kinesis Simulator is established before starting to "
           "search for devices.")
            .toLatin1()
            .data()));
}

//-------------------------------------------------------------------------------------
ThorlabsTCubeTECInterface::~ThorlabsTCubeTECInterface()
{
}

//-------------------------------------------------------------------------------------
ito::RetVal ThorlabsTCubeTECInterface::getAddInInst(ito::AddInBase** addInInst)
{
    // the argument of the macro is the classname of the plugin
    NEW_PLUGININSTANCE(ThorlabsTCubeTEC)
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
ito::RetVal ThorlabsTCubeTECInterface::closeThisInst(ito::AddInBase** addInInst)
{
    // the argument of the macro is the classname of the plugin
    REMOVE_PLUGININSTANCE(ThorlabsTCubeTEC)
    return ito::retOk;
}


//-------------------------------------------------------------------------------------
ThorlabsTCubeTEC::ThorlabsTCubeTEC() : AddInDataIO(), m_opened(false)
{
    m_params.insert(
        "name",
        ito::Param(
            "name",
            ito::ParamBase::String | ito::ParamBase::Readonly,
            "ThorlabsTCubeTEC",
            tr("name of the plugin").toLatin1().data()));

    m_params.insert(
        "deviceName",
        ito::Param(
            "deviceName",
            ito::ParamBase::String | ito::ParamBase::Readonly,
            "",
            tr("description of the device").toLatin1().data()));


    m_params.insert(
        "serialNumber",
        ito::Param(
            "serialNumber",
            ito::ParamBase::String | ito::ParamBase::Readonly,
            "",
            tr("serial number of the device").toLatin1().data()));

    auto p = ito::Param(
        "sensorType",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "Transducer",
        tr("Connected sensor type.").toLatin1().data());
    auto sm = new ito::StringMeta(ito::StringMeta::String, "Transducer");
    sm->addItem("TH20kOhm");
    sm->addItem("TH200kOhm");
    p.setMeta(sm, true);
    m_params.insert("sensorType", p);

    m_params.insert(
        "pollingInterval",
        ito::Param(
            "pollingInterval",
            ito::ParamBase::Int | ito::ParamBase::Readonly,
            200,
            new ito::IntMeta(1, 10000, 1, "pollingInterval"),
            tr("device polling interval in ms").toLatin1().data()));

    m_params.insert(
        "firmwareVersion",
        ito::Param(
            "firmwareVersion",
            ito::ParamBase::Int | ito::ParamBase::Readonly,
            0,
            tr("firmware version of the connected device").toLatin1().data()));
    m_params.insert(
        "softwareVersion",
        ito::Param(
            "softwareVersion",
            ito::ParamBase::Int | ito::ParamBase::Readonly,
            0,
            tr("software version of the connected device").toLatin1().data()));

    m_params.insert(
        "currentTemperature",
        ito::Param(
            "currentTemperature",
            ito::ParamBase::Double | ito::ParamBase::Readonly,
            -45.0,
            200.0,
            0.0,
            tr("The current temperature in \xc2\xb0\x43 or kOhm, depending on the sensor type")
                .toLatin1()
                .data()));

    m_params.insert(
        "targetTemperature",
        ito::Param(
            "targetTemperature",
            ito::ParamBase::Double,
            -45.0,
            200.0,
            0.0,
            tr("The target temperature in \xc2\xb0\x43 or kOhm, depending on the sensor type.")
                .toLatin1()
                .data()));

    m_params.insert(
        "derivativeGain",
        ito::Param(
            "derivativeGain",
            ito::ParamBase::Double,
            0.0,
            100.0,
            0.0,
            tr("The derivative gain term for the temperature loop parameters.").toLatin1().data()));

    m_params.insert(
        "integralGain",
        ito::Param(
            "integralGain",
            ito::ParamBase::Double,
            0.0,
            100.0,
            0.0,
            tr("The integral gain term for the temperature loop parameters.").toLatin1().data()));

    m_params.insert(
        "proportionalGain",
        ito::Param(
            "proportionalGain",
            ito::ParamBase::Double,
            1.0,
            100.0,
            1.0,
            tr("The proportional gain term for the temperature loop parameters.")
                .toLatin1()
                .data()));

    m_params.insert(
        "currentLimit",
        ito::Param(
            "currentLimit",
            ito::ParamBase::Double,
            0.0,
            2000.0,
            0.0,
            tr("The maximum current limit in mA.").toLatin1().data()));

    m_params.insert(
        "enableControl",
        ito::Param(
            "enableControl",
            ito::ParamBase::Int,
            0,
            1,
            1,
            tr("Enable (1) or disable (0) cube for computer control.").toLatin1().data()));

    if (hasGuiSupport())
    {
        // now create dock widget for this plugin
        DockWidgetThorlabsTCubeTEC* dockWidget = new DockWidgetThorlabsTCubeTEC(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
            QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, dockWidget);
    }
    memset(m_serialNo, '\0', sizeof(m_serialNo));
}

//-------------------------------------------------------------------------------------
ThorlabsTCubeTEC::~ThorlabsTCubeTEC()
{
}

//-------------------------------------------------------------------------------------
ito::RetVal ThorlabsTCubeTEC::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    QByteArray serialNo = paramsOpt->at(0).getVal<const char*>();
    QByteArray sensorType = paramsOpt->at(1).getVal<const char*>();
    bool connectToKinesisSimulator = paramsOpt->at(2).getVal<int>() > 0;

    if (connectToKinesisSimulator)
    {
        numberOfKinesisSimulatorConnections++;
        TLI_InitializeSimulations();
    }

    retValue += checkError(TLI_BuildDeviceList(), "build device list");
    QByteArray existingSerialNumbers("", 256);
    TLI_DeviceInfo deviceInfo;

    if (!retValue.containsError())
    {
        short numDevices = TLI_GetDeviceListSize();

        if (numDevices == 0)
        {
            retValue += ito::RetVal(ito::retError, 0, "no Thorlabs device detected");
        }
        else
        {
            retValue += checkError(
                TLI_GetDeviceListExt(existingSerialNumbers.data(), existingSerialNumbers.size()),
                "get device list");
        }
    }

    if (!retValue.containsError())
    {
        int idx = existingSerialNumbers.indexOf('\0');

        if (idx > 0)
        {
            existingSerialNumbers = existingSerialNumbers.left(idx);
        }

        QList<QByteArray> serialNumbers;

        // filter all serial numbers for supported devices only
        foreach (const QByteArray& serialNumber, existingSerialNumbers.split(','))
        {
            if (TLI_GetDeviceInfo(serialNumber.constData(), &deviceInfo) > 0)
            {
                if (deviceInfo.isKnownType && deviceInfo.typeID == 87 /*TCube TEC*/)
                {
                    serialNumbers << serialNumber;
                }
            }
        }

        if (serialNo == "")
        {
            bool found = false;

            for (int i = 0; i < serialNumbers.size(); ++i)
            {
                if (!openedDevices.contains(serialNumbers[i]))
                {
                    serialNo = serialNumbers[i];
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                retValue += ito::RetVal(ito::retError, 0, "No free Thorlabs TCube TEC detected.");
            }
        }
        else
        {
            bool found = false;

            foreach (const QByteArray& s, serialNumbers)
            {
                if (s == serialNo && s != "")
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                retValue += ito::RetVal::format(
                    ito::retError,
                    0,
                    "no device with the serial number '%s' found",
                    serialNo.data());
            }
            else if (openedDevices.contains(serialNo))
            {
                retValue += ito::RetVal::format(
                    ito::retError,
                    0,
                    "Thorlabs device with the serial number '%s' already in use.",
                    serialNo.data());
            }
        }
    }

    if (!retValue.containsError()) // open the device
    {
        if (TLI_GetDeviceInfo(serialNo.data(), &deviceInfo) == 0)
        {
            retValue += ito::RetVal(ito::retError, 0, "error obtaining device information.");
        }
        else
        {
            // bug: deviceInfo.serialNo is sometimes wrong if more than one of
            // the same devices are connected
            m_params["serialNumber"].setVal<char*>(serialNo.data());
            setIdentifier(QLatin1String(serialNo.data()));
            m_params["deviceName"].setVal<char*>(deviceInfo.description);
            m_params["sensorType"].setVal<char*>(sensorType.data());
        }
    }

    if (!retValue.containsError()) // open device
    {
        if (deviceInfo.isKnownType && (deviceInfo.typeID == 87 /*TCube TEC*/))
        {
            memcpy(
                m_serialNo, serialNo.data(), std::min((size_t)serialNo.size(), sizeof(m_serialNo)));
            retValue += checkError(TC_Open(m_serialNo), "open device");

            if (!retValue.containsError())
            {
                if (!TC_LoadSettings(m_serialNo))
                {
                    retValue +=
                        ito::RetVal(ito::retWarning, 0, "settings of device could not be loaded");
                }
            }

            if (!retValue.containsError())
            {
                m_opened = true;
                openedDevices.append(m_serialNo);
            }

            if (sensorType == "Transducer")
            {
                retValue +=
                    checkError(TC_SetSensorType(m_serialNo, TC_Transducer), "TC_SetSensorType");
            }
            else if (sensorType == "TH20kOhm")
            {
                retValue +=
                    checkError(TC_SetSensorType(m_serialNo, TC_TH20kOhm), "TC_SetSensorType");
            }
            if (sensorType == "TH200kOhm")
            {
                retValue +=
                    checkError(TC_SetSensorType(m_serialNo, TC_TH200kOhm), "TC_SetSensorType");
            }

            // start device polling at pollingInterval intervals
            TC_StartPolling(m_serialNo, m_params["pollingInterval"].getVal<int>());

            if (m_params["enableControl"].getVal<int>() > 0)
            {
                retValue += checkError(TC_Enable(m_serialNo), "TC_Enable");
            }
            else
            {
                retValue += checkError(TC_Disable(m_serialNo), "TC_Disable");
            }
        }
        else
        {
            retValue += ito::RetVal(
                ito::retError, 0, "The given device serial number is no TCube TEC device");
        }
    }

    if (!retValue.containsError()) // get current parameters
    {
        TC_RequestStatus(m_serialNo);
        Sleep(100);

        m_params["firmwareVersion"].setVal<int>((int)TC_GetFirmwareVersion(m_serialNo));
        m_params["softwareVersion"].setVal<int>((int)TC_GetSoftwareVersion(m_serialNo));
        m_params["currentTemperature"].setVal<double>(
            (double)TC_GetTemperatureReading(m_serialNo) / 100.0);
        m_params["targetTemperature"].setVal<double>(
            (double)TC_GetTemperatureSet(m_serialNo) / 100.0);

        TC_LoopParameters pidParams;
        TC_GetTempLoopParams(m_serialNo, &pidParams);

        // it seems that the documentation is wrong, and a differentialGain, integralGain,
        // proportionalGain value of 100.0 is equal to 100%
        m_params["derivativeGain"].setVal<double>(
            /*100.0 * */ (double)pidParams.differentialGain); // / 32767.0);
        m_params["integralGain"].setVal<double>(
            /*100.0 * */ (double)pidParams.integralGain); // / 32767.0);
        m_params["proportionalGain"].setVal<double>(
            /*100.0 * */ (double)pidParams.proportionalGain); // / 32767.0);

        double limit = TC_GetCurrentLimit(m_serialNo);
        m_params["currentLimit"].setVal<double>(limit);

        switch (TC_GetSensorType(m_serialNo))
        {
        case TC_Transducer:
            sensorType = "Transducer";
            m_params["currentTemperature"].getMetaT<ito::DoubleMeta>()->setMin(-45.0);
            m_params["currentTemperature"].getMetaT<ito::DoubleMeta>()->setMax(145.0);
            m_params["targetTemperature"].getMetaT<ito::DoubleMeta>()->setMin(-45.0);
            m_params["targetTemperature"].getMetaT<ito::DoubleMeta>()->setMax(145.0);
            break;
        case TC_TH20kOhm:
            sensorType = "TH20kOhm";
            m_params["currentTemperature"].getMetaT<ito::DoubleMeta>()->setMin(0.0);
            m_params["currentTemperature"].getMetaT<ito::DoubleMeta>()->setMax(20.0);
            m_params["targetTemperature"].getMetaT<ito::DoubleMeta>()->setMin(0.0);
            m_params["targetTemperature"].getMetaT<ito::DoubleMeta>()->setMax(20.0);
            break;
        case TC_TH200kOhm:
            sensorType = "TH200kOhm";
            m_params["currentTemperature"].getMetaT<ito::DoubleMeta>()->setMin(0.0);
            m_params["currentTemperature"].getMetaT<ito::DoubleMeta>()->setMax(200.0);
            m_params["targetTemperature"].getMetaT<ito::DoubleMeta>()->setMin(0.0);
            m_params["targetTemperature"].getMetaT<ito::DoubleMeta>()->setMax(200.0);
            break;
        default:
            retValue += ito::RetVal(ito::retError, 0, "unsupported sensor type");
            break;
        }

        m_params["sensorType"].setVal<char*>(sensorType.data());
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

//-------------------------------------------------------------------------------------
ito::RetVal ThorlabsTCubeTEC::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_opened)
    {
        TC_StopPolling(m_serialNo);
        Sleep(100);
        TC_Close(m_serialNo);
        m_opened = false;
        openedDevices.removeOne(m_serialNo);
    }

    if (numberOfKinesisSimulatorConnections > 0)
    {
        numberOfKinesisSimulatorConnections--;

        if (numberOfKinesisSimulatorConnections == 0)
        {
            TLI_UninitializeSimulations();
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
ito::RetVal ThorlabsTCubeTEC::getParam(
    QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
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
        if (key == "currentTemperature")
        {
            it->setVal<double>((double)TC_GetTemperatureReading(m_serialNo) / 100.0);
            *val = it.value();
            emit parametersChanged(m_params);
        }
        else if (key == "targetTemperature")
        {
            it->setVal<double>((double)TC_GetTemperatureSet(m_serialNo) / 100.0);
            *val = it.value();
            emit parametersChanged(m_params);
        }
        else if (key == "derivativeGain")
        {
            TC_LoopParameters pidParams;
            TC_GetTempLoopParams(m_serialNo, &pidParams);
            // it seems that the documentation is wrong, and a differentialGain value of 100.0 is
            // equal to 100%
            it->setVal<double>(/*100.0 * */(double)pidParams.differentialGain); // / 32767.0);
            *val = it.value();
        }
        else if (key == "integralGain")
        {
            TC_LoopParameters pidParams;
            TC_GetTempLoopParams(m_serialNo, &pidParams);
            // it seems that the documentation is wrong, and a integralGain value of 100.0 is equal
            // to 100%
            it->setVal<double>(/*100.0 * */(double)pidParams.integralGain); // / 32767.0);
            *val = it.value();
        }
        else if (key == "proportionalGain")
        {
            TC_LoopParameters pidParams;
            TC_GetTempLoopParams(m_serialNo, &pidParams);
            // it seems that the documentation is wrong, and a proportionalGain value of 100.0 is
            // equal to 100%
            it->setVal<double>(/*100.0 * */(double)pidParams.proportionalGain); // / 32767.0);
            *val = it.value();
        }
        else if (key == "currentLimit")
        {
            double limit = TC_GetCurrentLimit(m_serialNo);
            it->setVal<double>(limit);
            *val = it.value();
        }
        else
        {
            *val = it.value();
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
ito::RetVal ThorlabsTCubeTEC::setParam(
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
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "targetTemperature")
        {
            double temp = val->getVal<double>();
            retValue +=
                checkError(TC_SetTemperature(m_serialNo, temp * 100.0), "TC_SetTemperature");

            if (!retValue.containsError())
            {
                it->setVal<double>(temp);
            }
            else
            {
                Sleep(300);
                it->setVal<double>((double)TC_GetTemperatureSet(m_serialNo) / 100.0);
            }
        }
        else if (key == "derivativeGain")
        {
            double gain = val->getVal<double>();

            TC_LoopParameters pidParams;
            TC_GetTempLoopParams(m_serialNo, &pidParams);
            // it seems that the documentation is wrong, and a differentialGain value of 100.0 is
            // equal to 100%
            pidParams.differentialGain = /*32767 * */ gain /*/ 100.0*/;

            retValue +=
                checkError(TC_SetTempLoopParams(m_serialNo, &pidParams), "TC_SetTempLoopParams");

            if (!retValue.containsError())
            {
                it->setVal<double>(gain);
            }
        }
        else if (key == "integralGain")
        {
            double gain = val->getVal<double>();

            TC_LoopParameters pidParams;
            TC_GetTempLoopParams(m_serialNo, &pidParams);
            // it seems that the documentation is wrong, and a integralGain value of 100.0 is equal
            // to 100%
            pidParams.integralGain = /*32767 **/ gain /*/ 100.0*/;

            retValue +=
                checkError(TC_SetTempLoopParams(m_serialNo, &pidParams), "TC_SetTempLoopParams");

            if (!retValue.containsError())
            {
                it->setVal<double>(gain);
            }
        }
        else if (key == "proportionalGain")
        {
            double gain = val->getVal<double>();

            TC_LoopParameters pidParams;
            TC_GetTempLoopParams(m_serialNo, &pidParams);
            // it seems that the documentation is wrong, and a proportionalGain value of 100.0 is
            // equal to 100%
            pidParams.proportionalGain = /*32767 **/ gain /*/ 100.0*/;

            retValue +=
                checkError(TC_SetTempLoopParams(m_serialNo, &pidParams), "TC_SetTempLoopParams");

            if (!retValue.containsError())
            {
                it->setVal<double>(gain);
            }
        }
        else if (key == "currentLimit")
        {
            double limit = val->getVal<double>();

            retValue += checkError(TC_SetCurrentLimit(m_serialNo, limit), "TC_SetCurrentLimit");

            if (!retValue.containsError())
            {
                it->setVal<double>(limit);
            }
        }
        else if (key == "enableControl")
        {
            int state = val->getVal<int>();

            if (state > 0)
            {
                retValue += checkError(TC_Enable(m_serialNo), "enable control");
            }
            else
            {
                retValue += checkError(TC_Enable(m_serialNo), "disable control");
            }

            retValue += it->copyValueFrom(&(*val));
        }
        else
        {
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError())
    {
        // send changed parameters to any connected dialogs or dock-widgets
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
/* if a dock widget is displayed, this timer is activated to continuously update
the current temperature and emit it via parametersChanged. */
void ThorlabsTCubeTEC::timerEvent(QTimerEvent* event)
{
    if (m_opened)
    {
        auto temp = TC_GetTemperatureReading(m_serialNo);
        m_params["currentTemperature"].setVal<double>((double)temp / 100.0);
        emit parametersChanged(m_params);
    }
}

//-------------------------------------------------------------------------------------
void ThorlabsTCubeTEC::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        DockWidgetThorlabsTCubeTEC* widget =
            qobject_cast<DockWidgetThorlabsTCubeTEC*>(getDockWidget()->widget());

        if (visible)
        {
            connect(
                this,
                &AddInBase::parametersChanged,
                widget,
                &DockWidgetThorlabsTCubeTEC::parametersChanged);

            emit parametersChanged(m_params);

            if (m_updateTimerId < 0)
            {
                m_updateTimerId = startTimer(1000);
            }
        }
        else
        {
            disconnect(
                this,
                &AddInBase::parametersChanged,
                widget,
                &DockWidgetThorlabsTCubeTEC::parametersChanged);

            if (m_updateTimerId >= 0)
            {
                killTimer(m_updateTimerId);
                m_updateTimerId = -1;
            }
        }
    }
}

//-------------------------------------------------------------------------------------
ito::RetVal ThorlabsTCubeTEC::checkError(short value, const char* message)
{
    if (value == 0)
    {
        return ito::retOk;
    }
    else
    {
        switch (value)
        {
        case 1:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The FTDI functions have not been initialized.", message);
        case 2:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: The Device could not be found. This can be generated if the function "
                "TLI_BuildDeviceList() has not been called.",
                message);
        case 3:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: The Device must be opened before it can be accessed. See the appropriate Open "
                "function for your device.",
                message);
        case 4:
            return ito::RetVal::format(
                ito::retError, 1, "%s: An I/O Error has occurred in the FTDI chip.", message);
        case 5:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: There are Insufficient resources to run this application.",
                message);
        case 6:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: An invalid parameter has been supplied to the device.",
                message);
        case 7:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: The Device is no longer present. The device may have been disconnected since "
                "the last TLI_BuildDeviceList() call.",
                message);
        case 8:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The device detected does not match that expected.", message);
        case 16:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The library for this device could not be found.", message);
        case 17:
            return ito::RetVal::format(
                ito::retError, 1, "%s: No functions available for this device.", message);
        case 18:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The function is not available for this device.", message);
        case 19:
            return ito::RetVal::format(
                ito::retError, 1, "%s: Bad function pointer detected.", message);
        case 20:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The function failed to complete successfully.", message);
        case 21:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The function failed to complete successfully.", message);
        case 32:
            return ito::RetVal::format(
                ito::retError, 1, "%s: Attempt to open a device that was already open.", message);
        case 33:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The device has stopped responding.", message);
        case 34:
            return ito::RetVal::format(
                ito::retError, 1, "%s: This function has not been implemented.", message);
        case 35:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The device has reported a fault.", message);
        case 36:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The function could not be completed at this time.", message);
        case 40:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: The function could not be completed because the device is disconnected.",
                message);
        case 41:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The firmware has thrown an error", message);
        case 42:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The device has failed to initialize", message);
        case 43:
            return ito::RetVal::format(
                ito::retError, 1, "%s: An Invalid channel address was supplied", message);
        case 37:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: The device cannot perform this function until it has been Homed.",
                message);
        case 38:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: The function cannot be performed as it would result in an illegal position.",
                message);
        case 39:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: An invalid velocity parameter was supplied. The velocity must be greater than "
                "zero.",
                message);
        case 44:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: This device does not support Homing. Check the Limit switch parameters are "
                "correct.",
                message);
        case 45:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: An invalid jog mode was supplied for the jog function.",
                message);
        case 46:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: There is no Motor Parameters available to convert Real World Units.",
                message);
        case 47:
            return ito::RetVal::format(
                ito::retError,
                1,
                "%s: Command temporarily unavailable, Device may be busy.",
                message);
        default:
            return ito::RetVal::format(
                ito::retError, value, "%s: unknown error %i.", message, value);
        }
    }
}
