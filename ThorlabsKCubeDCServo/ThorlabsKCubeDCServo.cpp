/* ********************************************************************
    Plugin "ThorlabsISM" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2020, TRUMPF Laser- und Systemtechnik GmbH

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

#include "ThorlabsKCubeDCServo.h"

#include "dialogThorlabsKCubeDCServo.h"
#include "dockWidgetThorlabsKCubeDCServo.h"

#include "pluginVersion.h"
#include "gitVersion.h"

#include <iostream>

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qelapsedtimer.h>
#include <qwaitcondition.h>
#include <qelapsedtimer.h>
#include <qdatetime.h>

#include <qdebug.h>


QList<QByteArray> ThorlabsKCubeDCServo::openedDevices = QList<QByteArray>();
int ThorlabsKCubeDCServo::numberOfKinesisSimulatorConnections = 0;

//-------------------------------------------------------------------------------------
/*!
    \detail creates new instance of ThorlabsKCubeDCServoInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created ThorlabsKCubeDCServoInterface-instance is stored in *addInInst
    \return retOk
    \sa ThorlabsISM
*/
ito::RetVal ThorlabsKCubeDCServoInterface::getAddInInst(ito::AddInBase **addInInst)
{
    ito::RetVal retValue;
    NEW_PLUGININSTANCE(ThorlabsKCubeDCServo)
    return retValue;
}

//-------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of ThorlabsKCubeDCServoInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa ThorlabsISM
*/
ito::RetVal ThorlabsKCubeDCServoInterface::closeThisInst(ito::AddInBase **addInInst)
{

    ito::RetVal retValue;
    REMOVE_PLUGININSTANCE(ThorlabsKCubeDCServo)
    return retValue;
}

//-------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
ThorlabsKCubeDCServoInterface::ThorlabsKCubeDCServoInterface()
{
    m_type = ito::typeActuator;

    setObjectName("ThorlabsKCubeDCServo");

    m_description = QObject::tr("ThorlabsKCubeDCServo");
    m_detaildescription = QObject::tr("ThorlabsKCubeDCServo is an acutator plugin to control the following integrated devices from Thorlabs: \n\
\n\
* K-Cube Controller for Brushed DC Servo Motors (e.g. KDC101) \n\
\n\
It requires the new Kinesis driver package from Thorlabs and implements the interface Thorlabs.MotionControl.KCube.DCServo.\n\
\n\
Please install the Kinesis driver package in advance with the same bit-version (32/64bit) than itom. \n\
\n\
This plugin has been tested with the motorized translation stage MTS25-Z8.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsOpt.append(ito::Param("serialNo", ito::ParamBase::String, "", tr("Serial number of the device to be loaded, if empty, the first device that can be opened will be opened").toLatin1().data()));
    m_initParamsOpt.append(ito::Param("connectToKinesisSimulator", ito::ParamBase::Int, 0, 1, 0, tr("If 1, a connection to the running Kinesis Simulator is established before starting to search for devices.").toLatin1().data()));
}


//-------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans).
    The plugin is initialized (e.g. by a Python call) with mandatory or optional
    parameters (m_initParamsMand and m_initParamsOpt) by
    ThorlabsKCubeDCServo::init. The widget window is created at this position.
*/
ThorlabsKCubeDCServo::ThorlabsKCubeDCServo() :
    AddInActuator(),
    m_async(0),
    m_opened(false),
    m_numaxis(1)
{
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ThorlabsKCubeDCServo", tr("Name of the plugin").toLatin1().data()));
    m_params.insert("numaxis", ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1, 1, tr("Number of axes").toLatin1().data()));
    m_params.insert("deviceName", ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Description of the device").toLatin1().data()));
    m_params.insert("serialNumber", ito::Param("serialNumber", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Serial number of the device").toLatin1().data()));
    m_params.insert("firmwareVersion", ito::Param("firmwareVersion", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Firmware version of the device").toLatin1().data()));
    m_params.insert("hardwareVersion", ito::Param("hardwareVersion", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Hardware version of the device").toLatin1().data()));
    m_params.insert("homingAvailable", ito::Param("homingAvailable", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("1 if actuator supports a home drive, else 0").toLatin1().data()));

    m_params.insert("acceleration", ito::Param("acceleration", ito::ParamBase::Double, 0.0, 10000.0, 1.0, tr("acceleration in real world units (e.g. mm/s^2)").toLatin1().data()));
    m_params.insert("speed", ito::Param("speed", ito::ParamBase::Double, 0.0, 10000.0, 1.0, tr("speed in real world units (e.g. mm/s)").toLatin1().data()));

    m_params.insert("async", ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("synchronous (0, default) or asynchronous (1) mode").toLatin1().data()));
    m_params.insert("timeout", ito::Param("timeout", ito::ParamBase::Double, 0.0, 200.0, 100.0, tr("timeout for move operations in sec").toLatin1().data()));

    m_params.insert("lockFrontPanel", ito::Param("lockFrontPanel", ito::ParamBase::Int, 0, 1, 0, tr("1 to lock the front panel, else 0 (default)").toLatin1().data()));

    m_params.insert("enableAxis", ito::Param("enableAxis", ito::ParamBase::Int, 0, 1, 1, tr("If enabled (1, default), power is applied to the motor so it is fixed in position. Else (0), the motor can be freely moved.").toLatin1().data()));

    m_currentPos.fill(0.0, 1);
    m_currentStatus.fill(0, 1);
    m_targetPos.fill(0.0, 1);
    m_originPositions.fill(0.0, 1);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetThorlabsKCubeDCServo *dockWidget = new DockWidgetThorlabsKCubeDCServo(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<const char*>()), features, areas, dockWidget);
    }

    memset(m_serialNo, '\0', sizeof(m_serialNo));
}

//-------------------------------------------------------------------------------------
/*! \detail Init method which is called by the addInManager after the initiation of a new instance of DummyGrabber.
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: nullptr), which is released if this method has been terminated
    \todo check if (*paramsMand)[0] is a serial port
    \return retOk
    */
ito::RetVal ThorlabsKCubeDCServo::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    short ret;

    QByteArray serial = paramsOpt->at(0).getVal<const char*>();
    bool connectToKinesisSimulator = paramsOpt->at(1).getVal<int>() > 0;

    if (connectToKinesisSimulator)
    {
        numberOfKinesisSimulatorConnections++;
        TLI_InitializeSimulations();
    }

    retval += checkError(TLI_BuildDeviceList(), "build device list");
    QByteArray existingSerialNumbers("", 512);
    TLI_DeviceInfo deviceInfo;

    if (!retval.containsError())
    {
        short numDevices = TLI_GetDeviceListSize();

        if (numDevices == 0)
        {
            retval += ito::RetVal(ito::retError, 0, "no Thorlabs devices detected");
        }
        else
        {
            // check for devices of type KCube DC Servo (27) only
            retval += checkError(
                TLI_GetDeviceListByTypeExt(existingSerialNumbers.data(), existingSerialNumbers.size(), 27),
                "get device list"
            );
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
                retval += ito::RetVal(ito::retError, 0, "no free Thorlabs devices found");
            }
        }
        else
        {
            bool found = false;

            foreach(const QByteArray &s, serialNumbers)
            {
                if (s == serial && s != "")
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                retval += ito::RetVal::format(ito::retError, 0, "no device with the serial number '%s' found", serial.data());
            }
            else if (openedDevices.contains(serial))
            {
                retval += ito::RetVal::format(ito::retError, 0, "Thorlabs device with the serial number '%s' already in use", serial.data());
            }
        }
    }

    if (!retval.containsError())
    {
        if (TLI_GetDeviceInfo(serial.data(), &deviceInfo) == 0)
        {
            retval += ito::RetVal(ito::retError, 0, "error obtaining device information");
        }
        else
        {
            m_params["serialNumber"].setVal<const char*>(serial.data()); // bug: deviceInfo.serialNo is sometimes wrong if more than one of the same devices are connected
            setIdentifier(QLatin1String(serial.data())); // bug: deviceInfo.serialNo is sometimes wrong if more than one of the same devices are connected
            m_params["deviceName"].setVal<const char*>(deviceInfo.description);
        }
    }

    if (!retval.containsError())
    {
        memcpy(m_serialNo, serial.data(), std::min((size_t)serial.size(), sizeof(m_serialNo)));
        retval += checkError(CC_Open(m_serialNo), "open device");

        if (!retval.containsError())
        {
            m_opened = true;
            openedDevices.append(m_serialNo);
        }
    }

    if (!retval.containsError())
    {
        if (!CC_LoadSettings(m_serialNo))
        {
            retval += ito::RetVal(ito::retWarning, 0, "settings of device could not be loaded");
        }

        if (!CC_StartPolling(m_serialNo, 150))
        {
            retval += ito::RetVal(ito::retError, 0, "error starting position and status polling");
        }

        // get firmware version
        TLI_HardwareInformation hardwareInfo;
        ret = CC_GetHardwareInfoBlock(m_serialNo, &hardwareInfo);

        if (ret != FT_OK)
        {
            retval += ito::RetVal(ito::retError, ret, "could not fetch hardware information of the controller");
        }
        else
        {
            m_params["firmwareVersion"].setVal<const char*>(QByteArray::number((int)hardwareInfo.firmwareVersion).data());
            m_params["hardwareVersion"].setVal<const char*>(QByteArray::number((int)hardwareInfo.hardwareVersion).data());
            /*m_params["numaxis"].setVal<int>(hardwareInfo.numChannels);
            m_numaxis = hardwareInfo.numChannels;

            m_currentPos.fill(0.0, m_numaxis);
            m_currentStatus.fill(0, m_numaxis);
            m_targetPos.fill(0.0, m_numaxis);*/

        }

        m_params["homingAvailable"].setVal<int>(CC_CanHome(m_serialNo) ? 1 : 0);
    }

    if (!retval.containsError())
    {
        // get the device parameter here
        bool deviceCanLockFrontPanel = CC_CanDeviceLockFrontPanel(m_serialNo) ? 1 : 0;

        if (deviceCanLockFrontPanel)
        {
            m_params["lockFrontPanel"].setVal<int>(CC_GetFrontPanelLocked(m_serialNo) ? 1 : 0);
        }
        else
        {
            m_params["lockFrontPanel"].setFlags(ito::ParamBase::Readonly);
        }

        double real_unit;
        int pos = CC_GetPosition(m_serialNo);
        retval += checkError(CC_GetRealValueFromDeviceUnit(m_serialNo, pos, &real_unit, 0 /*Distance*/), "getRealValueFromDeviceUnit");
        m_currentPos[0] = real_unit;
        m_currentStatus[0] = ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
        m_targetPos[0] = real_unit;

        // get velocity and acceleration limits
        double maxVelocity;
        double maxAcceleration;
        int velocity;
        int acceleration;
        retval += checkError(CC_GetMotorVelocityLimits(m_serialNo, &maxVelocity, &maxAcceleration), "getMotorVelocityLimits");
        retval += checkError(CC_GetVelParams(m_serialNo, &acceleration, &velocity), "getMotorVelocityLimits");

        m_params["speed"].getMetaT<ito::DoubleMeta>()->setMax(maxVelocity);
        m_params["acceleration"].getMetaT<ito::DoubleMeta>()->setMax(maxAcceleration);

        retval += checkError(CC_GetRealValueFromDeviceUnit(m_serialNo, acceleration, &real_unit, 2 /*Acceleration*/), "getRealValueFromDeviceUnit");
        m_params["acceleration"].setVal<double>(real_unit);
        retval += checkError(CC_GetRealValueFromDeviceUnit(m_serialNo, velocity, &real_unit, 1 /*Velocity*/), "getRealValueFromDeviceUnit");
        m_params["speed"].setVal<double>(real_unit);

        int enable = m_params["enableAxis"].getVal<int>();

        if (enable > 0)
        {
            retval += checkError(CC_EnableChannel(m_serialNo), "enable axis");
        }
        else
        {
            retval += checkError(CC_DisableChannel(m_serialNo), "disable axis");
        }

    }

    if (!retval.containsError())
    {
        Sleep(200);
        QSharedPointer<QVector<int> > status(new QVector<int>(1, 0));
        retval += getStatus(status, nullptr);
    }

    if (!retval.containsError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal ThorlabsKCubeDCServo::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogThorlabsKCubeDCServo(this));
}

//-------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the ThorlabsISMInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: nullptr), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsKCubeDCServo::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (m_opened)
    {
        CC_StopPolling(m_serialNo);
        Sleep(300);

        CC_Close(m_serialNo);
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
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//-------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type int/double with key "name" stored in m_params and the corresponding member variables.
            This function is defined by the actuator class and overwritten at this position.

    \param[in] *name        Name of parameter
    \param[out] val            New parameter value as double
    \param[in/out] *waitCond    Waitcondition between this thread and the callers thread,

    \return retOk
*/
ito::RetVal ThorlabsKCubeDCServo::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

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

//-------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type char* with key "name" stored in m_params and the corresponding member variables.
            This function is defined by the actuator class and overwritten at this position.
            If the "ctrl-type" is set, ThorlabsISM::SMCSwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers thread

    \return retOk
*/
ito::RetVal ThorlabsKCubeDCServo::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index = 0;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;
    QVector<QPair<int, QByteArray> > lastitError;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }
    if (!retValue.containsError())
    {
        //here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        //---------------------------
        if (key == "async")
        {
            m_async = val->getVal<int>();
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "lockFrontPanel")
        {
            retValue += checkError(CC_SetFrontPanelLock(m_serialNo, val->getVal<int>() ? true : false), "setParam to lock frontpanel");
            val->setVal<int>(CC_GetFrontPanelLocked(m_serialNo) ? 1 : 0);
        }
        else if (key == "acceleration")
        {
            double accel = val->getVal<double>();
            int accelDev, velocityDev;
            CC_GetVelParams(m_serialNo, &accelDev, &velocityDev);
            CC_GetDeviceUnitFromRealValue(m_serialNo, accel, &accelDev, 2 /*Acceleration*/);
            retValue += checkError(CC_SetVelParams(m_serialNo, accelDev, velocityDev), "set acceleration");
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "speed")
        {
            double speed = val->getVal<double>();
            int accelDev, velocityDev;
            CC_GetVelParams(m_serialNo, &accelDev, &velocityDev);
            CC_GetDeviceUnitFromRealValue(m_serialNo, speed, &velocityDev, 1 /*Velocity*/);
            retValue += checkError(CC_SetVelParams(m_serialNo, accelDev, velocityDev), "set velocity");
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "enableAxis")
        {
            if (val->getVal<int>() > 0)
            {
                retValue += checkError(CC_EnableChannel(m_serialNo), "enable axes");
            }
            else
            {
                retValue += checkError(CC_DisableChannel(m_serialNo), "disable axes");
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
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
//! calib
/*!
the given axis should be calibrated (e.g. by moving to a reference switch).
*/
ito::RetVal ThorlabsKCubeDCServo::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    retValue += calib(QVector<int>(1, axis));

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
//! calib
/*!
the given axes should be calibrated (e.g. by moving to a reference switch).
*/
ito::RetVal ThorlabsKCubeDCServo::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible").toLatin1().data());
    }
    else if (m_params["homingAvailable"].getVal<int>() == 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "This device cannot be homed / calibrated.");
    }
    else
    {
        CC_ClearMessageQueue(m_serialNo);
        isInterrupted();

        retValue += checkError(CC_Home(m_serialNo), "home axis");

        QElapsedTimer timer;
        timer.start();
        double timeoutMs = m_params["timeout"].getVal<double>() * 1000.0;
        bool done = false;
        WORD messageType, messageId;
        DWORD messageData;
        QSharedPointer<double> pos(new double);

        setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        sendStatusUpdate();

        while (!done)
        {
            //now check if the interrupt flag has been set (e.g. by a button click on its dock widget)
            if (!done && isInterrupted())
            {
                //todo: force all axes to stop
                retValue += checkError(CC_StopProfiled(m_serialNo), "stop profiled");

                //set the status of all axes from moving to interrupted (only if moving was set before)
                QVector<int> axis;
                axis << 0;
                replaceStatus(axis, ito::actuatorMoving, ito::actuatorInterrupted);
                sendStatusUpdate(true);

                retValue += ito::RetVal(ito::retError, 0, "interrupt occurred");

                if (waitCond)
                {
                    waitCond->returnValue = retValue;
                    waitCond->release();
                }

                return retValue;
            }

            setAlive();

            while (CC_MessageQueueSize(m_serialNo) > 0)
            {
                if (CC_GetNextMessage(m_serialNo, &messageType, &messageId, &messageData))
                {
                    if (messageType == 2 && messageId == 0)
                    {
                        // homed
                        setStatus(m_currentStatus[0], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                        sendStatusUpdate();
                        done = true;
                        retValue += getPos(0, pos, nullptr);
                    }
                }
            }

            if (timeoutMs < timer.elapsed())
            {
                retValue += ito::RetVal(ito::retError, 0, "timeout while homing / calibrating.");
                done = true;
            }
            else
            {
                retValue += getPos(0, pos, nullptr);
            }

            Sleep(50);
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubeDCServo::setOrigin(const int axis, ItomSharedSemaphore * waitCond)
{
    return setOrigin(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubeDCServo::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    QSharedPointer<double> originPos(new double);
    double previous = m_originPositions[0];
    m_originPositions[0] = 0.0;

    ito::RetVal retValue = getPos(0, originPos, nullptr);

    if (!retValue.containsError())
    {
        m_originPositions[0] = *originPos;
    }
    else
    {
        m_originPositions[0] = previous;
    }

    retValue += getPos(0, originPos, nullptr);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
/*! \detail This function gets the status of the device. The SMCStatus function is called internally.

    \param [out] status        Status of System. 0: okay, 1: error
    \param [in] waitCond is the semaphore (default: nullptr), which is released if this method has been terminated
    \return retOk
    \todo define the status value
*/
ito::RetVal ThorlabsKCubeDCServo::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    *status = m_currentStatus;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//-------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubeDCServo::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if ((axis >= m_numaxis) || axis < 0)
    {
        retValue = ito::RetVal(ito::retError, 1, tr("axis index is out of bound").toLatin1().data());
    }
    else
    {
        double real_unit;
        int counts = CC_GetPosition(m_serialNo);
        retValue += checkError(CC_GetRealValueFromDeviceUnit(m_serialNo, counts, &real_unit, 0 /*Distance*/), "getRealValueFromDeviceUnit");
        m_currentPos[axis] = real_unit - m_originPositions[0];
        *pos = m_currentPos[axis];
    }

    sendStatusUpdate(false);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubeDCServo::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    for (int naxis = 0; naxis < axis.size(); naxis++)
    {
        if ((axis[naxis] >= m_numaxis) || axis[naxis] < 0)
        {
            retValue = ito::RetVal(ito::retError, 1, tr("at least one axis index is out of bound").toLatin1().data());
        }
        else
        {
            QSharedPointer<double> pos_(new double);
            retValue += getPos(axis[naxis], pos_, nullptr);
            (*pos)[naxis] = *pos_;
        }
    }

    sendStatusUpdate(false);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
//! setPosAbs
/*!
starts moving the given axis to the desired absolute target position

depending on m_async this method directly returns after starting the movement (async = 1) or
only returns if the axis reached the given target position (async = 0)

In some cases only relative movements are possible, then get the current position, determine the
relative movement and call the method relatively move the axis.
*/
ito::RetVal ThorlabsKCubeDCServo::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosAbs(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//-------------------------------------------------------------------------------------
//! setPosAbs
/*!
starts moving all given axes to the desired absolute target positions

depending on m_async this method directly returns after starting the movement (async = 1) or
only returns if all axes reached their given target positions (async = 0)

In some cases only relative movements are possible, then get the current position, determine the
relative movement and call the method relatively move the axis.
*/
ito::RetVal ThorlabsKCubeDCServo::setPosAbs(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else
    {
        int cntPos = 0;

        foreach(const int i, axis)
        {
            if (i < 0 || i >= m_numaxis)
            {
                retValue += ito::RetVal::format(ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                m_targetPos[i] = pos[cntPos]; //todo: set the absolute target position to the desired value in mm or degree
            }

            cntPos++;
        }

        sendTargetUpdate();

        if (retValue.containsError())
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            if (m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }

            CC_ClearMessageQueue(m_serialNo);

            foreach(const int axisNum, axis)
            {
                int deviceUnit;
                CC_GetDeviceUnitFromRealValue(m_serialNo, m_targetPos[axisNum] + m_originPositions[axisNum], &deviceUnit, 0 /*Position*/);
                retValue += checkError(CC_MoveToPosition(m_serialNo, deviceUnit), "move absolute");

                //call waitForDone in order to wait until all axes reached their target or a given timeout expired
                //the m_currentPos and m_currentStatus vectors are updated within this function
                retValue += waitForDone(m_params["timeout"].getVal<double>() * 1000.0, axisNum); //WaitForAnswer(60000, axis);
            }

            if (!m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }

    }

    return retValue;
}

//-------------------------------------------------------------------------------------
//! setPosRel
/*!
starts moving the given axis by the given relative distance

depending on m_async this method directly returns after starting the movement (async = 1) or
only returns if the axis reached the given target position (async = 0)

In some cases only absolute movements are possible, then get the current position, determine the
new absolute target position and call setPosAbs with this absolute target position.
*/
ito::RetVal ThorlabsKCubeDCServo::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosRel(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//-------------------------------------------------------------------------------------
//! setPosRel
/*!
starts moving the given axes by the given relative distances

depending on m_async this method directly returns after starting the movement (async = 1) or
only returns if all axes reached the given target positions (async = 0)

In some cases only absolute movements are possible, then get the current positions, determine the
new absolute target positions and call setPosAbs with these absolute target positions.
*/
ito::RetVal ThorlabsKCubeDCServo::setPosRel(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else
    {
        int cntPos = 0;

        foreach(const int i, axis)
        {
            if (i < 0 || i >= m_numaxis)
            {
                retValue += ito::RetVal::format(ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                m_targetPos[i] = m_currentPos[i] + pos[cntPos]; //todo: set the absolute target position to the desired value in mm or degree
            }
            cntPos++;
        }

        sendTargetUpdate();


        if (retValue.containsError())
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            if (m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }

            CC_ClearMessageQueue(m_serialNo);

            foreach(const int axisNum, axis)
            {
                int deviceUnit;
                CC_GetDeviceUnitFromRealValue(m_serialNo, m_targetPos[axisNum] + m_originPositions[axisNum], &deviceUnit, 0 /*Position*/);
                retValue += checkError(CC_MoveToPosition(m_serialNo, deviceUnit), "move relative");

                //call waitForDone in order to wait until all axes reached their target or a given timeout expired
                //the m_currentPos and m_currentStatus vectors are updated within this function
                retValue += waitForDone(m_params["timeout"].getVal<double>() * 1000.0, axisNum); //WaitForAnswer(60000, axis);
            }

            if (!m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
    }

    return retValue;
}
//-------------------------------------------------------------------------------------
/*! \detail This slot is triggered by the request signal from the dockingwidget dialog to update the position after ever positioning command.
            It sends the current position and the status to the world.

    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsKCubeDCServo::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    if (sendCurrentPos)
    {
        QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);

        for (int axis = 0; axis < m_numaxis; axis++)
        {
            retval += getPos(axis, sharedpos, nullptr);
        }
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
ito::RetVal ThorlabsKCubeDCServo::waitForDone(const int timeoutMS, const int axis, const int flags)
{
    ito::RetVal retVal;
    retVal += waitForDone(timeoutMS, QVector<int>(1, axis), flags);
    return retVal;
}

//-------------------------------------------------------------------------------------
//! method must be overwritten from ito::AddInActuator
/*!
WaitForDone should wait for a moving motor until the indicated axes (or all axes of nothing is indicated) have stopped or a timeout or user interruption
occurred. The timeout can be given in milliseconds, or -1 if no timeout should be considered. The flag-parameter can be used for your own purpose.
*/
ito::RetVal ThorlabsKCubeDCServo::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
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
    QSharedPointer<double> pos_(new double);

    //reset interrupt flag
    isInterrupted();

    long delay = 60; //[ms]

    timer.start();

    //if axis is empty, all axes should be observed by this method
    QVector<int> _axis = axis;

    if (_axis.size() == 0) //all axis
    {
        for (int i = 0; i < m_numaxis; i++)
        {
            _axis.append(i);
        }
    }

    while (!done && !timeout)
    {
        //now check if the interrupt flag has been set (e.g. by a button click on its dock widget)
        if (!done && isInterrupted())
        {
            //todo: force all axes to stop
            retVal += checkError(CC_StopProfiled(m_serialNo), "stop profiled");

            //set the status of all axes from moving to interrupted (only if moving was set before)
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            sendStatusUpdate(true);

            retVal += ito::RetVal(ito::retError, 0, "interrupt occurred");
            return retVal;
        }

        //short delay
        Sleep(delay);
        setAlive();

        if (timeoutMS > -1)
        {
            double currentTime = timer.elapsed();

            if (currentTime > timeoutMS) // timeout
            {
                //todo: obtain the current position, status... of all given axes
                foreach(const int &i, axis)
                {
                    retVal += getPos(i, pos_, nullptr);
                    m_currentPos[i] = *pos_;
                    m_targetPos[i] = m_currentPos[i];
                    setStatus(axis, ito::actuatorAtTarget,  ito::actStatusMask);
                }
                timeout = true;
            }
            else
            {
                while (CC_MessageQueueSize(m_serialNo) > 0)
                {
                    if (CC_GetNextMessage(m_serialNo, &messageType, &messageId, &messageData))
                    {
                        if (messageType == 2 && messageId == 1)
                        {
                            foreach(const int &i, axis)
                            {
                                retVal += getPos(i, pos_, nullptr);
                                m_currentPos[i] = *pos_;

                                if ((std::abs(m_targetPos[i] - m_currentPos[i]) < 0.05))
                                {
                                    setStatus(m_currentStatus[i], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                                    done = true; //not done yet
                                }
                                else
                                {
                                    setStatus(m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                                    done = false;
                                }
                            }
                        }
                    }
                }

                foreach(const int &i, axis)
                {
                    retVal += getPos(i, pos_, nullptr);
                    m_currentPos[i] = *pos_;
                }
            }
        }

        if (!timeout)
        {
            sendStatusUpdate();
            Sleep(20);
        }

    }

    if (timeout)
    {
        //timeout occurred, set the status of all currently moving axes to timeout
        replaceStatus(_axis, ito::actuatorMoving, ito::actuatorTimeout);
        retVal += ito::RetVal(ito::retError, 9999, "timeout occurred");
        sendStatusUpdate(true);
    }

    replaceStatus(_axis, ito::actuatorMoving, ito::actuatorAtTarget);
    sendStatusUpdate();

    return retVal;
}

//-------------------------------------------------------------------------------------
void ThorlabsKCubeDCServo::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        DockWidgetThorlabsKCubeDCServo *w = qobject_cast<DockWidgetThorlabsKCubeDCServo*>(getDockWidget()->widget());

        if (w)
        {
            if (visible)
            {
                connect(this, &AddInActuator::parametersChanged, w, &DockWidgetThorlabsKCubeDCServo::parametersChanged);
                connect(this, &AddInActuator::actuatorStatusChanged, w, &DockWidgetThorlabsKCubeDCServo::actuatorStatusChanged);
                connect(this, &AddInActuator::targetChanged, w, &DockWidgetThorlabsKCubeDCServo::targetChanged);

                emit parametersChanged(m_params);
                sendTargetUpdate();
                sendStatusUpdate(false);
            }
            else
            {
                disconnect(this, &AddInActuator::parametersChanged, w, &DockWidgetThorlabsKCubeDCServo::parametersChanged);
                disconnect(this, &AddInActuator::actuatorStatusChanged, w, &DockWidgetThorlabsKCubeDCServo::actuatorStatusChanged);
                disconnect(this, &AddInActuator::targetChanged, w, &DockWidgetThorlabsKCubeDCServo::targetChanged);
            }
        }
    }
}

//-------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubeDCServo::checkError(short value, const char* message)
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
            return ito::RetVal::format(ito::retError, 1, "%s: The FTDI functions have not been initialized", message);
        case 2:
            return ito::RetVal::format(ito::retError, 1, "%s: The device could not be found", message);
        case 3:
            return ito::RetVal::format(ito::retError, 1, "%s: The device must be opened before it can be accessed", message);
        case 37:
            return ito::RetVal::format(ito::retError, 1, "%s: The device cannot perform the function until it has been homed (call calib() before)", message);
        case 38:
            return ito::RetVal::format(ito::retError, 1, "%s: The function cannot be performed as it would result in an illegal position", message);
        case 39:
            return ito::RetVal::format(ito::retError, 1, "%s: An invalid velocity parameter was supplied. The velocity must be greater than zero", message);
        default:
            return ito::RetVal::format(ito::retError, value, "%s: unknown error %i", message, value);
        }
    }
}
