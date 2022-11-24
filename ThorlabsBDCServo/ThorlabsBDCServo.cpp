/* ********************************************************************
    Plugin "ThorlabsBDCServo" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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

#include "ThorlabsBDCServo.h"

#define NOMINMAX

#include "gitVersion.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include <qdatetime.h>
#include <qelapsedtimer.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qtimer.h>
#include <qwaitcondition.h>

#include "Thorlabs.MotionControl.Benchtop.DCServo.h"

#include <iostream>
#include <qdebug.h>

QList<QByteArray> ThorlabsBDCServo::openedDevices = QList<QByteArray>();
int ThorlabsBDCServo::numberOfKinesisSimulatorConnections = 0;

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of ThorlabsBDCServoInterface and returns the instance-pointer
    \param [in,out] addInInst is a ito::float64 pointer of type ito::AddInBase. The newly created
   ThorlabsBDCServoInterface-instance is stored in *addInInst \return retOk \sa ThorlabsBDCServo
*/
ito::RetVal ThorlabsBDCServoInterface::getAddInInst(ito::AddInBase** addInInst)
{
    ito::RetVal retValue;
    NEW_PLUGININSTANCE(ThorlabsBDCServo)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of ThorlabsBPInterface. This instance is given by parameter
   addInInst. \param [in] ito::float64 pointer to the instance which should be deleted. \return
   retOk \sa ThorlabsBDCServo
*/
ito::RetVal ThorlabsBDCServoInterface::closeThisInst(ito::AddInBase** addInInst)
{
    ito::RetVal retValue;
    REMOVE_PLUGININSTANCE(ThorlabsBDCServo)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is
   initialized (e.g. by a Python call) with mandatory or optional parameters (m_initParamsMand and
   m_initParamsOpt).
*/
ThorlabsBDCServoInterface::ThorlabsBDCServoInterface()
{
    m_type = ito::typeActuator;

    setObjectName("ThorlabsBDCServo");

    m_description = QObject::tr("ThorlabsBDCServo");
    m_detaildescription = QObject::tr(
        "ThorlabsBDCServo is an acutator plugin to control the following integrated devices from Thorlabs: \n\
\n\
* Benchtop DC Servo (M30XY) \n\
\n\
It requires the new Kinesis driver package from Thorlabs and implements the interface Thorlabs.MotionControl.Benchtop.DCServo.\n\
\n\
Please install the Kinesis driver package in advance with the same bit-version (32/64bit) than itom. \n\
\n\
This plugin has been tested with the Benchtop DC Servo M30XY. \n");

    m_author = "J. Krauter, TRUMPF SE + Co. KG, Ditzingen";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsOpt.append(ito::Param(
        "serialNo",
        ito::ParamBase::String,
        "",
        tr("Serial number of the device to be loaded, if empty, the first device that can be "
           "opened will be opened")
            .toLatin1()
            .data()));
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

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class ThorlabsBDCServoInterface with the name ThorlabsBDCServoInterface
// as plugin for the Qt-System (see Qt-DOC)


//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized
   (e.g. by a Python call) with mandatory or optional parameters (m_initParamsMand and
   m_initParamsOpt) by the ThorlabsBDCServo::init. The widged window is created at this position.
*/
ThorlabsBDCServo::ThorlabsBDCServo() :
    AddInActuator(), m_async(0), m_opened(false), m_numChannels(0)
{
    m_params.insert(
        "name",
        ito::Param(
            "name",
            ito::ParamBase::String | ito::ParamBase::Readonly,
            "ThorlabsBDCServo",
            tr("name of plugin").toLatin1().data()));
    m_params.insert(
        "numaxis",
        ito::Param(
            "numaxis",
            ito::ParamBase::Int | ito::ParamBase::Readonly,
            0,
            100,
            0,
            tr("number of axes (channels)").toLatin1().data()));
    m_params.insert(
        "deviceName",
        ito::Param(
            "deviceName",
            ito::ParamBase::String | ito::ParamBase::Readonly,
            "",
            tr("Description of the device").toLatin1().data()));
    m_params.insert(
        "firmwareVersion",
        ito::Param(
            "firmwareVersion",
            ito::ParamBase::Int | ito::ParamBase::Readonly,
            0,
            tr("Firmware version of the device").toLatin1().data()));
    m_params.insert(
        "serialNumber",
        ito::Param(
            "serialNumber",
            ito::ParamBase::String | ito::ParamBase::Readonly,
            "",
            tr("Serial number of the device").toLatin1().data()));
    m_params.insert(
        "channel",
        ito::Param(
            "channel",
            ito::ParamBase::IntArray | ito::ParamBase::Readonly,
            NULL,
            tr("Channel number of each axis.").toLatin1().data()));
    m_params.insert(
        "timeout",
        ito::Param(
            "timeout",
            ito::ParamBase::Double,
            0.0,
            200.0,
            15.0,
            tr("Timeout for positioning in seconds.").toLatin1().data()));

    m_params.insert(
        "enabled",
        ito::Param(
            "enabled",
            ito::ParamBase::IntArray,
            NULL,
            tr("If 1, the axis is enabled and power is applied to the motor. 0: disabled, the "
               "motor can be turned by hand.")
                .toLatin1()
                .data()));
    m_params.insert(
        "homed",
        ito::Param(
            "homed",
            ito::ParamBase::IntArray | ito::ParamBase::Readonly,
            NULL,
            tr("If 0, the axis is not homed. 1: homed.").toLatin1().data()));
    m_params.insert(
        "acceleration",
        ito::Param(
            "acceleration",
            ito::ParamBase::DoubleArray,
            NULL,
            tr("Acceleration values for each axis in mm/s^2.").toLatin1().data()));
    m_params.insert(
        "velocity",
        ito::Param(
            "velocity",
            ito::ParamBase::DoubleArray,
            NULL,
            tr("Velocity values for each axis in mm/s.").toLatin1().data()));
    m_params.insert(
        "maximumTravelPosition",
        ito::Param(
            "maximumTravelPosition",
            ito::ParamBase::DoubleArray | ito::ParamBase::Readonly,
            NULL,
            tr("Maximum travel position for each axis in mm.").toLatin1().data()));
    m_params.insert(
        "minimumTravelPosition",
        ito::Param(
            "minimumTravelPosition",
            ito::ParamBase::DoubleArray | ito::ParamBase::Readonly,
            NULL,
            tr("Minimum travel position for each axis in mm. ").toLatin1().data()));
    m_params.insert(
        "backlash",
        ito::Param(
            "backlash",
            ito::ParamBase::DoubleArray,
            NULL,
            tr("Backlash distance setting in mm (used to control hysteresis).").toLatin1().data()));
    m_params.insert(
        "async",
        ito::Param(
            "async",
            ito::ParamBase::Int,
            0,
            1,
            m_async,
            tr("asychronous (1) or synchronous (0) mode").toLatin1().data()));

    m_currentPos.fill(0.0, 1);
    m_currentStatus.fill(0, 1);
    m_targetPos.fill(0.0, 1);

    if (hasGuiSupport())
    {
        // now create dock widget for this plugin
        DockWidgetThorlabsBDCServo* dockWidget = new DockWidgetThorlabsBDCServo(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
            QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, dockWidget);
    }

    memset(m_serialNo, '\0', sizeof(m_serialNo));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsBDCServo::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    QByteArray serial = paramsOpt->at(0).getVal<char*>();

    bool connectToKinesisSimulator = paramsOpt->at(1).getVal<int>() > 0;

    if (connectToKinesisSimulator)
    {
        numberOfKinesisSimulatorConnections++;
        TLI_InitializeSimulations();
    }

    retval += checkError(TLI_BuildDeviceList(), "build device list");
    QByteArray existingSerialNumbers("", 256);
    TLI_DeviceInfo deviceInfo;
    int maxChannels = 0;

    if (!retval.containsError())
    {
        short numDevices = TLI_GetDeviceListSize();

        if (numDevices == 0)
        {
            retval += ito::RetVal(ito::retError, 0, "no Thorlabs devices detected.");
        }
        else
        {
            retval += checkError(
                TLI_GetDeviceListExt(existingSerialNumbers.data(), existingSerialNumbers.size()),
                "get device list");
        }
    }

    if (!retval.containsError())
    {
        int idx = existingSerialNumbers.indexOf('\0');
        if (idx > 0)
        {
            existingSerialNumbers = existingSerialNumbers.left(idx);
        }

        QList<QByteArray> serialNumbers = existingSerialNumbers.split(',');

        if (serial == "")
        {
            bool found = false;
            for (int i = 0; i < serialNumbers.size(); ++i)
            {
                if (!openedDevices.contains(serialNumbers[i]))
                {
                    serial = serialNumbers[i];
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                retval += ito::RetVal(ito::retError, 0, "no free Thorlabs devices found.");
            }
        }
        else
        {
            bool found = false;
            foreach (const QByteArray& s, serialNumbers)
            {
                if (s == serial && s != "")
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                retval += ito::RetVal::format(
                    ito::retError, 0, "no device with the serial number '%s' found", serial.data());
            }
            else if (openedDevices.contains(serial))
            {
                retval += ito::RetVal::format(
                    ito::retError,
                    0,
                    "Thorlabs device with the serial number '%s' already in use.",
                    serial.data());
            }
        }
    }

    if (!retval.containsError())
    {
        if (TLI_GetDeviceInfo(serial.data(), &deviceInfo) == 0)
        {
            retval += ito::RetVal(ito::retError, 0, "error obtaining device information.");
        }
        else
        {
            m_params["serialNumber"].setVal<char*>(serial.data());
            setIdentifier(QLatin1String(serial.data()));
            m_params["deviceName"].setVal<char*>(deviceInfo.description);
        }
    }

    if (!retval.containsError())
    {
        if (deviceInfo.isKnownType && (deviceInfo.typeID == 101 /*Benchtop DC Servo (2 channels)*/))
        {
            memcpy(m_serialNo, serial.data(), std::min((size_t)serial.size(), sizeof(m_serialNo)));
            retval += checkError(BDC_Open(m_serialNo), "open device");

            if (!retval.containsError())
            {
                m_opened = true;
                openedDevices.append(m_serialNo);
            }
        }
        else
        {
            retval += ito::RetVal(
                ito::retError,
                0,
                "the type of the device is not among the supported devices (Benchtop DC Servo)");
        }
    }

    if (!retval.containsError())
    {
        if (BDC_GetNumChannels(m_serialNo) <= 0)
        {
            retval += ito::RetVal(ito::retError, 0, "The device has no connected channels.");
        }
        else
        {
            m_channelIndices.resize(BDC_GetNumChannels(m_serialNo));
            int i = 0;
            for (int c = 1; c <= m_channelIndices.length(); ++c)
            {
                if (BDC_EnableChannel(m_serialNo, c) == 0)
                {
                    m_channelIndices[i] = c;
                    i++;
                }
            }
        }
    }

    if (!retval.containsError())
    {
        foreach (int channel, m_channelIndices)
        {
            // int err = BDC_LoadSettings(m_serialNo, channel);
            int err = BDC_LoadSettings(m_serialNo, channel);
            if (!err)
            {
                retval += ito::RetVal(
                    ito::retWarning,
                    0,
                    "settings of device could not be loaded. You may must check 'persist setting "
                    "to device' in Kinesis software.");
            }

            if (!BDC_StartPolling(m_serialNo, channel, 200))
            {
                retval +=
                    ito::RetVal(ito::retError, 0, "error starting position and status polling.");
            }
        }

        m_numChannels = m_channelIndices.size();
        m_dummyValues =
            QSharedPointer<QVector<ito::float64>>(new QVector<ito::float64>(m_numChannels, 0.0));

        m_params["numaxis"].setVal<int>(m_numChannels);
        m_currentPos.fill(0.0, m_numChannels);
        m_currentStatus.fill(
            ito::actuatorAvailable | ito::actuatorEnabled | ito::actuatorAtTarget, m_numChannels);
        m_targetPos.fill(0.0, m_numChannels);

        int* intDummy = new int[m_numChannels];
        memset(intDummy, 0, m_numChannels * sizeof(int));
        m_params["enabled"].setMeta(new ito::IntArrayMeta(0, 1, 1, 0, m_numChannels, 1), true);

        m_params["enabled"].setVal<int*>(intDummy, m_numChannels);
        m_params["homed"].setMeta(new ito::IntArrayMeta(0, 1, 1, 0, m_numChannels, 1), true);


        ito::float64* minPos = new ito::float64[m_numChannels];
        ito::float64* maxPos = new ito::float64[m_numChannels];
        for (int i = 0; i < m_numChannels; ++i)
        {
            BDC_GetMotorTravelLimits(m_serialNo, m_channelIndices[i], &minPos[i], &maxPos[i]);
        }
        m_params["maximumTravelPosition"].setVal<ito::float64*>(maxPos, m_numChannels);
        m_params["minimumTravelPosition"].setVal<ito::float64*>(minPos, m_numChannels);

        m_params["maximumTravelPosition"].setMeta(
            new ito::DoubleArrayMeta(maxPos[0], maxPos[0], 0.0, m_numChannels, m_numChannels, 1),
            true);
        m_params["minimumTravelPosition"].setMeta(
            new ito::DoubleArrayMeta(minPos[0], minPos[0], 0.0, m_numChannels, m_numChannels, 1),
            true);

        for (int i = 0; i < m_numChannels; ++i)
        {
            intDummy[i] = m_channelIndices[i];
        }
        m_params["channel"].setVal<int*>(intDummy, m_numChannels);

        int currentVelocity, currentAcceleration;
        ito::float64* acceleration = new ito::float64[m_numChannels];
        ito::float64* velocity = new ito::float64[m_numChannels];
        ito::float64* maxVelocity = new ito::float64[m_numChannels];
        ito::float64* maxAcceleration = new ito::float64[m_numChannels];

        for (int i = 0; i < m_numChannels; ++i)
        {
            BDC_GetVelParams(
                m_serialNo, m_channelIndices[i], &currentAcceleration, &currentVelocity);
            BDC_GetRealValueFromDeviceUnit(
                m_serialNo, m_channelIndices[i], currentAcceleration, &acceleration[i], 2);
            BDC_GetRealValueFromDeviceUnit(
                m_serialNo, m_channelIndices[i], currentVelocity, &velocity[i], 1);
            BDC_GetMotorVelocityLimits(
                m_serialNo, m_channelIndices[i], &maxVelocity[i], &maxAcceleration[i]);
        }

        m_params["acceleration"].setVal<ito::float64*>(acceleration, m_numChannels);
        m_params["acceleration"].setMeta(
            new ito::DoubleArrayMeta(0.01, maxAcceleration[0], 0.01, m_numChannels, m_numChannels));
        m_params["velocity"].setVal<ito::float64*>(velocity, m_numChannels);
        m_params["velocity"].setMeta(
            new ito::DoubleArrayMeta(0.01, maxVelocity[0], 0.01, m_numChannels, m_numChannels));

        ito::float64* doubleDummy = new ito::float64[m_numChannels];
        for (int i = 0; i < m_numChannels; ++i)
        {
            doubleDummy[i] = (ito::float64)BDC_GetBacklash(m_serialNo, m_channelIndices[i]);
        }
        m_params["backlash"].setVal<ito::float64*>(doubleDummy, m_numChannels);

        m_params["backlash"].setMeta(
            new ito::DoubleArrayMeta(0.0, 5.0, 0.0, m_numChannels, m_numChannels));

        m_params["firmwareVersion"].setVal<int>(
            (int)BDC_GetFirmwareVersion(m_serialNo, m_channelIndices[0]));

        DELETE_AND_SET_NULL_ARRAY(intDummy);
        DELETE_AND_SET_NULL_ARRAY(doubleDummy);
        DELETE_AND_SET_NULL_ARRAY(minPos);
        DELETE_AND_SET_NULL_ARRAY(maxPos);
        DELETE_AND_SET_NULL_ARRAY(acceleration);
        DELETE_AND_SET_NULL_ARRAY(velocity);
        DELETE_AND_SET_NULL_ARRAY(maxVelocity);
        DELETE_AND_SET_NULL_ARRAY(maxAcceleration);
    }

    if (!retval.containsError())
    {
        Sleep(160);
        QSharedPointer<QVector<int>> status(new QVector<int>(m_numChannels, 0));
        retval += getStatus(status, NULL);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); // init method has been finished (independent on retval)
    return retval;
}


//---------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal ThorlabsBDCServo::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogThorlabsBDCServo(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the
   ThorlabsBDCServoInterface notice that this method is called in the actual thread of this
   instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsBDCServo::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (m_opened)
    {
        foreach (int channel, m_channelIndices)
        {
            BDC_StopPolling(m_serialNo, channel);
        }
        Sleep(200);

        BDC_Close(m_serialNo); // Kinesis 1.6.0 and Kinesis 1.7.0 crash in this line! bug. todo.
        m_opened = false;
        openedDevices.removeOne(m_serialNo);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type int/double with key "name" stored in m_params
   and the corresponding member variabels. This function is defined by the actuator class and
   overwritten at this position.

    \param[in] *name        Name of parameter
    \param[out] val            New parameter value as double
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal ThorlabsBDCServo::getParam(
    QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        *val = apiGetParam(*it, hasIndex, index, retValue);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type char* with key "name" stored in m_params and the
   corresponding member variabels. This function is defined by the actuator class and overwritten at
   this position. If the "ctrl-type" is set, ThorlabsBDCServo::SMCSwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal ThorlabsBDCServo::setParam(
    QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index = 0;
    QString suffix;
    ParamMapIterator it;

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
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError() && hasIndex)
    {
        if (index < 0 || index >= m_numChannels)
        {
            retValue += ito::RetVal(ito::retError, 0, "The given index is out of bounds.");
        }
    }

    if (!retValue.containsError())
    {
        //---------------------------
        if (key == "async")
        {
            m_async = val->getVal<int>();
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "enabled")
        {
            if (hasIndex)
            {
                if (val->getVal<int>())
                {
                    retValue += checkError(
                        BDC_EnableChannel(m_serialNo, m_channelIndices[index]), "enable channel");
                }
                else
                {
                    retValue += checkError(
                        BDC_DisableChannel(m_serialNo, m_channelIndices[index]), "disable channel");
                }

                if (!retValue.containsError())
                {
                    it->getVal<int*>()[index] = val->getVal<int>();
                }
                else
                {
                    Sleep(160);
                    QSharedPointer<QVector<int>> status(new QVector<int>(m_numChannels, 0));
                    retValue += getStatus(status, NULL);
                }
            }
            else
            {
                const int* values =
                    val->getVal<int*>(); // are alway m_numChannels values due to meta information
                for (int i = 0; i < m_numChannels; ++i)
                {
                    if (values[i])
                    {
                        retValue += checkError(
                            BDC_EnableChannel(m_serialNo, m_channelIndices[i]), "enable channel");
                    }
                    else
                    {
                        retValue += checkError(
                            BDC_DisableChannel(m_serialNo, m_channelIndices[i]), "disable channel");
                    }
                }

                if (!retValue.containsError())
                {
                    it->copyValueFrom(&(*val));
                }
                else
                {
                    Sleep(160);
                    QSharedPointer<QVector<int>> status(new QVector<int>(m_numChannels, 0));
                    retValue += getStatus(status, NULL);
                }
            }
        }
        else if (key == "velocity")
        {
            ito::float64* acceleration = m_params["acceleration"].getVal<ito::float64*>();
            ito::float64* velocity = val->getVal<ito::float64*>();
            int newAcceleration, newVelocity;

            for (int i = 0; i < m_numChannels; ++i)
            {
                retValue += checkError(
                    BDC_GetDeviceUnitFromRealValue(
                        m_serialNo, m_channelIndices[i], acceleration[i], &newAcceleration, 2),
                    "acceleration devie to real value");
                retValue += checkError(
                    BDC_GetDeviceUnitFromRealValue(
                        m_serialNo, m_channelIndices[i], velocity[i], &newVelocity, 1),
                    "velocity device to real value");
                retValue += checkError(
                    BDC_SetVelParams(m_serialNo, m_channelIndices[i], newAcceleration, newVelocity),
                    "set velocity");
            }

            if (!retValue.containsError())
            {
                it->copyValueFrom(&(*val));
            }
            else
            {
                Sleep(160);
                QSharedPointer<QVector<int>> status(new QVector<int>(m_numChannels, 0));
                retValue += getStatus(status, NULL);
            }
        }
        else if (key == "acceleration")
        {
            ito::float64* velocity = m_params["velocity"].getVal<ito::float64*>();
            ito::float64* acceleration = val->getVal<ito::float64*>();
            int newAcceleration, newVelocity;

            for (int i = 0; i < m_numChannels; ++i)
            {
                retValue += checkError(
                    BDC_GetDeviceUnitFromRealValue(
                        m_serialNo, m_channelIndices[i], acceleration[i], &newAcceleration, 2),
                    "acceleration devie to real value");
                retValue += checkError(
                    BDC_GetDeviceUnitFromRealValue(
                        m_serialNo, m_channelIndices[i], velocity[i], &newVelocity, 1),
                    "velocity device to real value");
                retValue += checkError(
                    BDC_SetVelParams(m_serialNo, m_channelIndices[i], newAcceleration, newVelocity),
                    "set acceleration");
            }

            if (!retValue.containsError())
            {
                it->copyValueFrom(&(*val));
            }
            else
            {
                Sleep(160);
                QSharedPointer<QVector<int>> status(new QVector<int>(m_numChannels, 0));
                retValue += getStatus(status, NULL);
            }
        }
        else if (key == "backlash")
        {
            ito::float64* backlash = val->getVal<ito::float64*>();

            for (int i = 0; i < m_numChannels; ++i)
            {
                retValue += checkError(
                    BDC_SetBacklash(m_serialNo, m_channelIndices[i], backlash[i]), "set backlash");
            }

            if (!retValue.containsError())
            {
                it->copyValueFrom(&(*val));
            }
            else
            {
                Sleep(160);
                QSharedPointer<QVector<int>> status(new QVector<int>(m_numChannels, 0));
                retValue += getStatus(status, NULL);
            }
        }
        //---------------------------
        else
        {
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
/*! \detail This function executes a calibration routine for one axis spezified by "axis". In the
   case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Number of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsBDCServo::calib(const int axis, ItomSharedSemaphore* waitCond)
{
    return calib(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for a set of axis spezified by "axis". In
   the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Vector this numbers of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsBDCServo::calib(const QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    bool done = false;
    DWORD s;

    foreach (int ax, axis)
    {
        if (ax < 0 || ax >= m_numChannels)
        {
            retval += ito::RetVal::format(ito::retError, 0, "invalid axis index %i.", ax);
        }
        else
        {
            s = BDC_GetStatusBits(m_serialNo, m_channelIndices[ax]);
            if ((s & 0x80000000) == 0)
            {
                retval += ito::RetVal::format(
                    ito::retError, 0, "axis %i is disabled and cannot be zeroed.", ax);
            }
        }
    }

    if (!retval.containsError())
    {
        foreach (int ax, axis)
        {
            retval += checkError(BDC_Home(m_serialNo, m_channelIndices[ax]), "set homed");
        }

        replaceStatus(axis, ito::actuatorAtTarget, ito::actuatorMoving);
        sendStatusUpdate(true);

        Sleep(400); // to be sure that the first requested status is correct

        while (!done && !retval.containsError())
        {
            done = true;

            foreach (int ax, axis)
            {
                s = BDC_GetStatusBits(m_serialNo, m_channelIndices[ax]);
                if (s & 0x00000200)
                {
                    done = false;
                }
            }

            if (!done)
            {
                setAlive();
                Sleep(160);
            }
        }

        if (done)
        { // Position reached and movement done
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate(true);

            QSharedPointer<QVector<int>> status(new QVector<int>(m_numChannels, 0));
            getStatus(status, NULL);

            QVector<int> allAxes;
            getPos(allAxes, m_dummyValues, NULL);
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsBDCServo::setOrigin(const int axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, "set origin not implemented.");

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsBDCServo::setOrigin(QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, "set origin not implemented.");

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function gets the status of the device. The GetStatusBits function is called
   internally.

    \param [out] status        Status of System. 0: okay, 1: error
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk \todo define the status value
*/
ito::RetVal ThorlabsBDCServo::getStatus(
    QSharedPointer<QVector<int>> status, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    DWORD s;

    int* homed = new int[m_numChannels];
    int* enabled = new int[m_numChannels];

    for (int idx = 0; idx < m_numChannels; ++idx)
    {
        s = BDC_GetStatusBits(m_serialNo, m_channelIndices[idx]);

        m_currentStatus[idx] = ito::actuatorAtTarget;
        m_currentStatus[idx] |= ito::actuatorAvailable; // the connected flag is not always set

        if (s & 0x80000000)
        {
            m_currentStatus[idx] |= ito::actuatorEnabled;
        }

        enabled[idx] = (s & 0x80000000) ? 1 : 0;

        if (memcmp(m_params["enabled"].getVal<int*>(), enabled, m_numChannels * sizeof(int)) != 0)
        {
            m_params["enabled"].setVal<int*>(enabled, m_numChannels);
            emit parametersChanged(
                m_params); // send changed parameters to any connected dialogs or dock-widgets
        }

        for (int i = 0; i < m_numChannels; ++i)
        {
            homed[i] = !BDC_NeedsHoming(m_serialNo, m_channelIndices[i]);
        }
        m_params["homed"].setVal<int*>(homed, m_numChannels);

        *status = m_currentStatus;
    }

    DELETE_AND_SET_NULL_ARRAY(homed);
    DELETE_AND_SET_NULL_ARRAY(enabled);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a single axis spezified by axis. The value in device independet in
   mm.

    \param [in] axis        Axisnumber
    \param [out] pos        Current position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk
*/
ito::RetVal ThorlabsBDCServo::getPos(
    const int axis, QSharedPointer<ito::float64> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QVector<int> axes(1, axis);
    ito::RetVal retval = getPos(axes, m_dummyValues, NULL);
    *pos = m_dummyValues->at(0);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a set of axis spezified by "axis". The value in device independet in
   mm. In this case if more than one axis is specified this function returns an error.

    \param [in] axis        Vector with axis numbers
    \param [out] pos        Current positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk
*/
ito::RetVal ThorlabsBDCServo::getPos(
    const QVector<int> axis,
    QSharedPointer<QVector<ito::float64>> pos,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    int idx;

    for (int i = 0; i < axis.size(); ++i)
    {
        idx = axis[i];

        if (idx < 0 || idx >= m_numChannels)
        {
            retval += ito::RetVal::format(ito::retError, 0, "invalid axis index %i", idx);
            break;
        }
        // in mm
        int deviceUnit = BDC_GetPosition(m_serialNo, m_channelIndices[idx]);
        ito::float64 posMM;
        BDC_GetRealValueFromDeviceUnit(m_serialNo, m_channelIndices[idx], deviceUnit, &posMM, 0);
        m_currentPos[idx] = posMM;
        (*pos)[i] = m_currentPos[idx];
    }

    sendStatusUpdate(false);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a one axis spezified by "axis" to the position "pos" . The
   value in device independet in mm. This function calls ThorlabsBDCServo::SetPos(axis, pos,
   "ABSOLUTCOMMAND")

    \param [in] axis     axis number
    \param [in] pos      absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \sa SMCSetPos \return retOk
*/
ito::RetVal ThorlabsBDCServo::setPosAbs(
    const int axis, const ito::float64 pos, ItomSharedSemaphore* waitCond)
{
    QVector<int> axes(1, axis);
    QVector<ito::float64> positions(1, pos);
    return setPosAbs(axes, positions, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos"
   . The value in device independet in mm. If the size of the vector is more then 1 element, this
   function returns an error. This function calls ThorlabsBDCServo::SetPos(axis, pos,
   "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with absolute positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \sa SMCSetPos \return retOk
*/
ito::RetVal ThorlabsBDCServo::setPosAbs(
    const QVector<int> axis, QVector<ito::float64> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    const ito::float64* maximumTravelPosition =
        m_params["maximumTravelPosition"].getVal<ito::float64*>();

    const ito::float64* minimumTravelPosition =
        m_params["minimumTravelPosition"].getVal<ito::float64*>();
    int idx;
    int flags = 0;
    DWORD s;
    bool released = false;

    if (isMotorMoving())
    {
        retval += ito::RetVal(
            ito::retError,
            0,
            tr("Any motor axis is moving. The motor is locked.").toLatin1().data());
    }
    else
    {
        foreach (int a, axis)
        {
            if (a < 0 || a >= m_numChannels)
            {
                retval += ito::RetVal::format(ito::retError, 0, "invalid axis index %i.", a);
            }
            else
            {
                s = BDC_GetStatusBits(m_serialNo, m_channelIndices[a]);
                if ((s & 0x80000000) == 0)
                {
                    retval += ito::RetVal::format(ito::retError, 0, "axis %i is disabled.", a);
                }
            }
        }
    }

    if (!retval.containsError())
    {
        for (int i = 0; i < axis.size(); ++i)
        {
            idx = axis[i];

            if (pos[i] < minimumTravelPosition[idx] || pos[i] > maximumTravelPosition[idx])
            {
                retval +=
                    ito::RetVal::format(ito::retError, 0, "target of axis %i out of bounds.", idx);
                break;
            }
            else
            {
                idx = axis[i];
                m_targetPos[idx] = pos[i];
                flags += (1 << idx);
            }
        }
    }

    if (!retval.containsError())
    {
        // set status of all given axes to moving and keep all flags related to the status and
        // switches
        setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);

        for (int i = 0; i < axis.size(); ++i)
        {
            idx = axis[i];

            BDC_ClearMessageQueue(m_serialNo, m_channelIndices[idx]);

            int deviceVal;
            BDC_GetDeviceUnitFromRealValue(
                m_serialNo, m_channelIndices[idx], m_targetPos[idx], &deviceVal, 0);
            retval += checkError(
                BDC_MoveToPosition(
                    m_serialNo, m_channelIndices[idx], deviceVal),
                "set absolute position");
        }

        if (!retval.containsError())
        {
            sendTargetUpdate();
            sendStatusUpdate();

            // release the wait condition now, if async is true (itom considers this method to be
            // finished now due to the threaded call)
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }

            retval += waitForDone(
                m_params["timeout"].getVal<ito::float64>() * 1e3,
                axis,
                flags); // drops into timeout

            // release the wait condition now, if async is false (itom waits until now if async is
            // false, hence in the synchronous mode)
            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }
        }
    }

    // if the wait condition has not been released yet, do it now
    if (waitCond && !released)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the relativ position of a one axis spezified by "axis" to the position "pos" . The
   value in device independet in mm. This function calls ThorlabsBDCServo::SetPos(axis, pos,
   "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        relative target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \sa SMCSetPos \return retOk
*/
ito::RetVal ThorlabsBDCServo::setPosRel(
    const int axis, const ito::float64 pos, ItomSharedSemaphore* waitCond)
{
    QVector<int> axes(1, axis);
    QVector<ito::float64> positions(1, pos);
    return setPosRel(axes, positions, waitCond);

    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos"
   . The value in device independet in mm. If the size of the vector is more then 1 element, this
   function returns an error. This function calls ThorlabsBDCServo::SetPos(axis, pos,
   "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with relative positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \sa SMCSetPos \return retOk
*/
ito::RetVal ThorlabsBDCServo::setPosRel(
    const QVector<int> axis, QVector<ito::float64> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    int idx;
    ito::float64 newAbsPos;
    const ito::float64* maximumTravelPosition =
        m_params["maximumTravelPosition"].getVal<ito::float64*>();
    const ito::float64* minimumTravelPosition =
        m_params["minimumTravelPosition"].getVal<ito::float64*>();
    int flags = 0;
    DWORD s;
    bool released = false;

    if (isMotorMoving())
    {
        retval += ito::RetVal(
            ito::retError,
            0,
            tr("Any motor axis is moving. The motor is locked.").toLatin1().data());
    }
    else
    {
        foreach (int a, axis)
        {
            if (a < 0 || a >= m_numChannels)
            {
                retval += ito::RetVal::format(ito::retError, 0, "invalid axis index %i.", a);
            }
            else
            {
                s = BDC_GetStatusBits(m_serialNo, m_channelIndices[a]);
                if ((s & 0x80000000) == 0)
                {
                    retval += ito::RetVal::format(ito::retError, 0, "axis %i is disabled.", a);
                }
            }
        }

        retval += getPos(axis, m_dummyValues, NULL);
    }

    if (!retval.containsError())
    {
        for (int i = 0; i < axis.size(); ++i)
        {
            idx = axis[i];
            newAbsPos = m_currentPos[idx] + pos[i];
            if (newAbsPos < minimumTravelPosition[idx] || newAbsPos > maximumTravelPosition[idx])
            {
                retval +=
                    ito::RetVal::format(ito::retError, 0, "target of axis %i out of bounds.", idx);
                break;
            }
            else
            {
                m_targetPos[idx] = newAbsPos;
            }
            flags += (1 << idx);
        }
    }

    if (!retval.containsError())
    {
        setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);

        for (int i = 0; i < axis.size(); ++i)
        {
            idx = axis[i];

            BDC_ClearMessageQueue(m_serialNo, m_channelIndices[idx]);

            int deviceVal;
            BDC_GetDeviceUnitFromRealValue(
                m_serialNo, m_channelIndices[idx], pos[i], &deviceVal, 0);
            retval += checkError(
                BDC_MoveRelative(m_serialNo, m_channelIndices[idx], deviceVal),
                "set relative position");
        }

        if (!retval.containsError())
        {
            sendTargetUpdate();
            sendStatusUpdate();

            // release the wait condition now, if async is true (itom considers this method to be
            // finished now due to the threaded call)
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }

            retval += waitForDone(
                m_params["timeout"].getVal<ito::float64>() * 1e3,
                axis,
                flags); // drops into timeout

            // release the wait condition now, if async is false (itom waits until now if async is
            // false, hence in the synchronous mode)
            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }
        }
    }

    if (waitCond && !released)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the
   position after ever positioning command. It sends the current postion and the status to the
   world.

    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsBDCServo::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    if (sendCurrentPos)
    {
        QVector<int> axes(m_numChannels, 0);
        for (int i = 0; i < m_numChannels; ++i)
        {
            axes[i] = i;
        }
        retval += getPos(axes, m_dummyValues, NULL);
    }
    else
    {
        sendStatusUpdate(true);
    }

    if (sendTargetPos)
    {
        sendTargetUpdate();
    }

    return retval;
}


//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsBDCServo::waitForDone(
    const int timeoutMS, const QVector<int> axis, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    QElapsedTimer timer;
    QMutex waitMutex;
    QWaitCondition waitCondition;
    WORD messageType;
    WORD messageId;
    DWORD messageData;
    QSharedPointer<ito::float64> pos_(new ito::float64);

    timer.start();
    while (!done && !retVal.containsWarningOrError())
    {
        if (!retVal.containsError())
        {
            // short delay of 60ms
            waitMutex.lock();
            waitCondition.wait(&waitMutex, 60);
            waitMutex.unlock();
            setAlive();
        }

        for (int i = 0; i < axis.size(); ++i)
        {
            retVal += getPos(i, pos_, nullptr);
            m_currentPos[i] = *pos_;

            BDC_WaitForMessage(
                m_serialNo, m_channelIndices[axis[i]], &messageType, &messageId, &messageData);
            if ((messageType != 2 || messageId != 1)) // wait for completion
            {
                foreach (const int& i, axis)  // update status to moving
                {
                    setStatus(
                        m_currentStatus[i],
                        ito::actuatorMoving,
                        ito::actSwitchesMask | ito::actStatusMask);
                }
                done = false;
            }
            else
            {
                foreach (const int& i, axis)  // update status to at target
                {
                    setStatus(
                        m_currentStatus[i],
                        ito::actuatorAtTarget,
                        ito::actSwitchesMask | ito::actStatusMask);
                }
                done = true;
            }
        }

        sendStatusUpdate(false);

        if (!done && isInterrupted()) // interrupt the movement
        {
            for (int i = 0; i < axis.size(); ++i)
            {
                retVal += checkError(
                    BDC_StopImmediate(m_serialNo, m_channelIndices[axis[i]]),
                    "stop immediate");
            }
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorInterrupted);
            sendStatusUpdate(true);
            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            return retVal;
        }

        if (timer.hasExpired(timeoutMS))  // timeout during movement
        {
            timeout = true;
            // timeout occured, set the status of all currently moving axes to timeout
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
            retVal += ito::RetVal(ito::retError, 9999, "timeout occurred during movement");
            sendStatusUpdate(true);
        }
    }

    foreach (const int& i, axis)  // update current positions
    {
        retVal += getPos(i, pos_, nullptr);
        m_currentPos[i] = *pos_;
    }

    sendStatusUpdate(true);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ThorlabsBDCServo::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget* w = getDockWidget()->widget();

        if (w)
        {
            if (visible)
            {
                connect(
                    this,
                    SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                    w,
                    SLOT(parametersChanged(QMap<QString, ito::Param>)));
                emit parametersChanged(m_params);
            }
            else
            {
                QObject::disconnect(
                    this,
                    SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                    w,
                    SLOT(parametersChanged(QMap<QString, ito::Param>)));
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsBDCServo::checkError(short value, const char* message)
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
                ito::retError, 1, "%s: An I/O Error has occured in the FTDI chip.", message);
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
                ito::retError, 1, "%s: The function failed to complete succesfully.", message);
        case 21:
            return ito::RetVal::format(
                ito::retError, 1, "%s: The function failed to complete succesfully.", message);
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
