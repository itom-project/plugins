/* ********************************************************************
    Plugin "ThorlabsISM" for itom software
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

#include "ThorlabsISM.h"

#define NOMINMAX

#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qtimer.h>
#include <qwaitcondition.h>
#include <qelapsedtimer.h>
#include <qdatetime.h>

#include "Thorlabs.MotionControl.IntegratedStepperMotors.h"

#include <qdebug.h>

QList<QByteArray> ThorlabsISM::openedDevices = QList<QByteArray>();
int ThorlabsISM::numberOfKinesisSimulatorConnections = 0;


//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of ThorlabsISMInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created ThorlabsISMInterface-instance is stored in *addInInst
    \return retOk
    \sa ThorlabsISM
*/
ito::RetVal ThorlabsISMInterface::getAddInInst(ito::AddInBase **addInInst)
{
    ito::RetVal retValue;
    NEW_PLUGININSTANCE(ThorlabsISM)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of ThorlabsISMInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa ThorlabsISM
*/
ito::RetVal ThorlabsISMInterface::closeThisInst(ito::AddInBase **addInInst)
{

    ito::RetVal retValue;
    REMOVE_PLUGININSTANCE(ThorlabsISM)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
ThorlabsISMInterface::ThorlabsISMInterface()
{
    m_type = ito::typeActuator;

    setObjectName("ThorlabsISM");

    m_description = QObject::tr("ThorlabsISM");
    m_detaildescription = QObject::tr("ThorlabsISM is an acutator plugin to control the following integrated devices from Thorlabs: \n\
\n\
* Long Travel Stage (e.g. LTS150 and LTS300) \n\
* Lab Jack (e.g. MLJ050) \n\
* Cage Rotator (K10CR1) \n\
\n\
It requires the new Kinesis driver package from Thorlabs and implements the interface Thorlabs.MotionControl.IntegratedStepperMotors.\n\
\n\
Please install the Kinesis driver package in advance with the same bit-version (32/64bit) than itom. \n\
\n\
This plugin has been tested with the cage rotator K10CR1.");

    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsOpt.append(ito::Param("serialNo", ito::ParamBase::String, "", tr("Serial number of the device to be loaded, if empty, the first device that can be opened will be opened").toLatin1().data()));
    m_initParamsOpt.append(ito::Param("additionalGearFactor", ito::ParamBase::Double, 0.0000000001, 1.0e12, 1.0, tr("There seems to be an additional conversion factor for some devices between device and real world units. This can be given here.").toLatin1().data()));
    m_initParamsOpt.append(ito::Param("connectToKinesisSimulator", ito::ParamBase::Int, 0, 1, 0, tr("If 1, a connection to the running Kinesis Simulator is established before starting to search for devices.").toLatin1().data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class ThorlabsISMInterface with the name ThorlabsISMInterface as plugin for the Qt-System (see Qt-DOC)


//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the ThorlabsISM::init. The widged window is created at this position.
*/
ThorlabsISM::ThorlabsISM() :
AddInActuator(),
    m_async(0),
    m_opened(false),
    m_additionalFactor(1.0)
{
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ThorlabsISM", tr("Name of the plugin").toLatin1().data()));
    m_params.insert("numaxis", ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 100, 0, tr("number of axes (channels)").toLatin1().data()));
    m_params.insert("deviceName", ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Description of the device").toLatin1().data()));
    m_params.insert("serialNumber", ito::Param("serialNumber", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Serial number of the device").toLatin1().data()));
    m_params.insert("enabled", ito::Param("enabled", ito::ParamBase::Int, 0, 1, 1, tr("If 1, the axis is enabled and power is applied to the motor. 0: disabled, the motor can be turned by hand.").toLatin1().data()));

    m_params.insert("async", ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1) or synchronous (0) mode").toLatin1().data()));
    m_params.insert(
        "speed",
        ito::Param(
            "speed",
            ito::ParamBase::Double,
            0.0,
            std::numeric_limits<double>::infinity(),
            0.0,
            tr("Target speed in %1/s (travelMode == %2) or mm/s (travelMode == %3)")
                .arg(QLatin1String("\u00B0"))
                .arg(MOT_Rotational)
                .arg(MOT_Linear)
                .toLatin1()
                .data()));
    m_params.insert(
        "accel",
        ito::Param(
            "accel",
            ito::ParamBase::Double,
            0.0,
            std::numeric_limits<double>::infinity(),
            0.0,
            tr("Target acceleration in %1/s^2 (travelMode == %2) or mm/s^2 (travelMode == %3)")
                .arg(QLatin1String("\u00B0"))
                .arg(MOT_Rotational)
                .arg(MOT_Linear)
                .toLatin1()
                .data()));
    m_params.insert("timeout", ito::Param("timeout", ito::ParamBase::Double, 0.0, 200.0, 60.0, tr("timeout for move operations in sec").toLatin1().data()));

    m_params.insert("homingAvailable", ito::Param("homingAvailable", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("1 if actuator supports a home drive, else 0").toLatin1().data()));
    m_params.insert("homed", ito::Param("homed", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("1 if actuator is 'homed', else 0").toLatin1().data()));
    m_params.insert("travelMode", ito::Param("travelMode", ito::ParamBase::Int | ito::ParamBase::Readonly, MOT_TravelModeUndefined, MOT_Rotational, MOT_TravelModeUndefined, tr("travel mode: linear (%1), rotational (%2), undefined (%3)").arg(MOT_Linear).arg(MOT_Rotational).arg(MOT_TravelModeUndefined).toLatin1().data()));
    m_params.insert(
        "stagePosMin",
        ito::Param(
            "stagePosMin",
            ito::ParamBase::Double | ito::ParamBase::Readonly,
            0.0,
            std::numeric_limits<double>::infinity(),
            0.0,
            tr("Minimum stage position in mm (travelMode == %1) or %2 (travelMode == %3). For "
               "%4, given positions will be wrapped by 360%5 for absolute moves.")
                .arg(MOT_Linear)
                .arg(QLatin1String("\u00B0"))
                .arg(MOT_Rotational)
                .arg(QLatin1String("\u00B0"))
                .arg(QLatin1String("\u00B0"))
                .toLatin1()
                .data()));
    m_params.insert(
        "stagePosMax",
        ito::Param(
            "stagePosMax",
            ito::ParamBase::Double | ito::ParamBase::Readonly,
            0.0,
            std::numeric_limits<double>::infinity(),
            0.0,
            tr("Maximum stage position in mm (travelMode == %1) or %2 (travelMode == %3). For %4, "
               "given positions will be wrapped by 360%5 for absolute moves.")
                .arg(MOT_Linear)
                .arg(QLatin1String("\u00B0"))
                .arg(MOT_Rotational)
                .arg(QLatin1String("\u00B0"))
                .arg(QLatin1String("\u00B0"))
                .toLatin1()
                .data()));

    m_params.insert("restCurrent", ito::Param("restCurrent", ito::ParamBase::Int, 0, 100, 20, tr("Percentage of full power to give while not moving.").toLatin1().data()));
    m_params.insert("moveCurrent", ito::Param("moveCurrent", ito::ParamBase::Int, 0, 100, 100, tr("Percentage of full power to give while moving.").toLatin1().data()));

    m_currentPos.fill(0.0, 1);
    m_currentStatus.fill(0, 1);
    m_targetPos.fill(0.0, 1);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetThorlabsISM *dockWidget = new DockWidgetThorlabsISM(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
    }

    memset(m_serialNo, '\0', sizeof(m_serialNo));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Init method which is called by the addInManager after the initiation of a new instance of DummyGrabber.
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \todo check if (*paramsMand)[0] is a serial port
    \return retOk
    */
ito::RetVal ThorlabsISM::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    QByteArray serial = paramsOpt->at(0).getVal<char*>();
    double additionalGearFactor = paramsOpt->at(1).getVal<double>();
    bool connectToKinesisSimulator = paramsOpt->at(2).getVal<int>() > 0;

    if (connectToKinesisSimulator)
    {
        numberOfKinesisSimulatorConnections++;
        TLI_InitializeSimulations();
    }

    retval += checkError(TLI_BuildDeviceList(), "build device list");
    QByteArray existingSerialNumbers("", 256);
    TLI_DeviceInfo deviceInfo;

    if (!retval.containsError())
    {
        short numDevices = TLI_GetDeviceListSize();

        if (numDevices == 0)
        {
            retval += ito::RetVal(ito::retError, 0, "no Thorlabs devices detected.");
        }
        else
        {
            retval += checkError(TLI_GetDeviceListExt(existingSerialNumbers.data(), existingSerialNumbers.size()), "get device list");
        }
    }

    if (!retval.containsError())
    {
        int idx = existingSerialNumbers.indexOf('\0');
        if (idx > 0)
        {
            existingSerialNumbers = existingSerialNumbers.left(idx);
        }

        QList<QByteArray> serialNumbers;

        // filter all serial numbers for supported devices only
        foreach(const QByteArray &serialNumber, existingSerialNumbers.split(','))
        {
            if (TLI_GetDeviceInfo(serialNumber.constData(), &deviceInfo) > 0)
            {
                if (deviceInfo.isKnownType && (
                        deviceInfo.typeID == 45 /*Long Travel Stage*/ ||
                        deviceInfo.typeID == 46 /*Lab Jack*/ ||
                        deviceInfo.typeID == 49 /*Lab Jack*/ ||
                        deviceInfo.typeID == 55 /*Cage Rotator*/))
                {
                    serialNumbers << serialNumber;
                }
            }
        }

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
                retval += ito::RetVal(ito::retError, 0, "no free Thorlabs devices of the supported types Long Travel Stage, Labjack, Cage Rotator found.");
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
                retval += ito::RetVal::format(ito::retError, 0, "Thorlabs device with the serial number '%s' already in use.", serial.data());
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
            m_params["numaxis"].setVal<int>(1);
            m_params["serialNumber"].setVal<char*>(serial.data()); // bug: deviceInfo.serialNo is sometimes wrong if more than one of the same devices are connected
            setIdentifier(QLatin1String(serial.data())); // bug: deviceInfo.serialNo is sometimes wrong if more than one of the same devices are connected
            m_params["deviceName"].setVal<char*>(deviceInfo.description);
        }
    }

    if (!retval.containsError())
    {
        memcpy(m_serialNo, serial.data(), std::min((size_t)serial.size(), sizeof(m_serialNo)));
        retval += checkError(ISC_Open(m_serialNo), "open device");

        if (!retval.containsError())
        {
            m_opened = true;
            openedDevices.append(m_serialNo);
        }
    }

    if (!retval.containsError())
    {
        m_params["homingAvailable"].setVal<int>(ISC_CanHome(m_serialNo) ? 1 : 0);

        // itom crash during initalization of this plugin. Does not crash in debug. Some bug in ISC_LoadSettings. ask Robin?
        if (!ISC_LoadSettings(m_serialNo))
        {
            retval += ito::RetVal(ito::retWarning, 0, "settings of device could not be loaded.");
        }
        if (!ISC_StartPolling(m_serialNo, 150))
        {
            retval += ito::RetVal(ito::retError, 0, "error starting position and status polling.");
        }

        double stepsPerRev;
        double gearBoxRatio;
        double pitch;
        retval += checkError(ISC_GetMotorParamsExt(m_serialNo, &stepsPerRev, &gearBoxRatio, &pitch), "get motor parameters");
        m_additionalFactor = additionalGearFactor;

        m_params["travelMode"].setVal<int>(ISC_GetMotorTravelMode(m_serialNo));

        int accel, speed;


        if (deviceInfo.typeID == 49)
        {
            retval += checkError(ISC_SetButtonParams(m_serialNo, MOT_JogMode, 1, 1),"set button params");
            retval += checkError(ISC_SetVelParams(m_serialNo, realWorldUnit2DeviceUnit(8,2), realWorldUnit2DeviceUnit(3, 1)), "set speed and acceleration");
            retval += checkError(ISC_SetJogVelParams(m_serialNo, realWorldUnit2DeviceUnit(8, 2), realWorldUnit2DeviceUnit(3, 1)), "set jog velocity and acceleration");
            retval += checkError(ISC_SetJogStepSize(m_serialNo, realWorldUnit2DeviceUnit(1, 0)), "set jog step size");
        }


        retval += checkError(ISC_GetVelParams(m_serialNo, &accel, &speed), "get speed and acceleration");
        m_params["speed"].setVal<double>(deviceUnit2RealWorldUnit(speed,1));
        m_params["accel"].setVal<double>(deviceUnit2RealWorldUnit(accel,2));


        MOT_PowerParameters powerParams;
        retval += checkError(ISC_GetPowerParams(m_serialNo, &powerParams), "get motor power parameters");
        m_params["restCurrent"].setVal<int>(powerParams.restPercentage);
        m_params["moveCurrent"].setVal<int>(powerParams.movePercentage);

        m_params["stagePosMax"].setVal<double>(deviceUnit2RealWorldUnit(ISC_GetStageAxisMaxPos(m_serialNo),0));
        m_params["stagePosMin"].setVal<double>(deviceUnit2RealWorldUnit(ISC_GetStageAxisMinPos(m_serialNo),0));
    }

    if (!retval.containsError())
    {
        Sleep(200);
        QSharedPointer<QVector<int> > status(new QVector<int>(1, 0));
        retval += getStatus(status, NULL);

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
const ito::RetVal ThorlabsISM::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogThorlabsISM(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the ThorlabsISMInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsISM::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (m_opened)
    {
        ISC_StopImmediate(m_serialNo);
        ISC_StopPolling(m_serialNo);
        Sleep(300);

        ISC_Close(m_serialNo);
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

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type int/double with key "name" stored in m_params and the corresponding member variabels.
            This function is defined by the actuator class and overwritten at this position.

    \param[in] *name        Name of parameter
    \param[out] val            New parameter value as double
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal ThorlabsISM::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type char* with key "name" stored in m_params and the corresponding member variabels.
            This function is defined by the actuator class and overwritten at this position.
            If the "ctrl-type" is set, ThorlabsISM::SMCSwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal ThorlabsISM::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        else if (key == "enabled")
        {
            if (val->getVal<int>() > 0)
            {
                retValue += checkError(ISC_EnableChannel(m_serialNo), "enable axis");
            }
            else
            {
                retValue += checkError(ISC_DisableChannel(m_serialNo), "disable axis");
            }
            retValue += it->copyValueFrom(&(*val));

            Sleep(400);
            QSharedPointer<QVector<int> > status(new QVector<int>(1, 0));
            retValue += getStatus(status, NULL);
        }
        else if (key == "speed" || key == "accel")
        {
            int speed, accel;
            retValue += checkError(ISC_GetVelParams(m_serialNo, &accel, &speed), "get current speed and acceleration");
            if (!retValue.containsError())
            {
                if (key == "speed")
                {
                    speed = realWorldUnit2DeviceUnit(val->getVal<double>(), 1);
                }
                else
                {
                    accel = realWorldUnit2DeviceUnit(val->getVal<double>(), 2);
                }

                retValue += checkError(ISC_SetVelParams(m_serialNo, accel, speed), "set speed and acceleration");
                retValue += checkError(ISC_GetVelParams(m_serialNo, &accel, &speed), "get current speed and acceleration");
            }
            m_params["speed"].setVal<double>(deviceUnit2RealWorldUnit(speed,1));
            m_params["accel"].setVal<double>(deviceUnit2RealWorldUnit(accel,2));
        }
        else if (key == "moveCurrent" || key == "restCurrent")
        {
            MOT_PowerParameters powerParams;
            retValue += checkError(ISC_GetPowerParams(m_serialNo, &powerParams), "get motor power parameters");
            if (!retValue.containsError())
            {
                if (key == "moveCurrent")
                {
                    powerParams.movePercentage = val->getVal<int>();
                }
                else
                {
                    powerParams.restPercentage = val->getVal<int>();
                }

                retValue += checkError(ISC_SetPowerParams(m_serialNo, &powerParams), "set motor power parameters");
                retValue += checkError(ISC_GetPowerParams(m_serialNo, &powerParams), "get current speed and acceleration");
            }

            m_params["moveCurrent"].setVal<int>(powerParams.movePercentage);
            m_params["restCurrent"].setVal<int>(powerParams.restPercentage);
        }


        //---------------------------
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

//----------------------------------------------------------------------------------------------------------------------------------
double ThorlabsISM::deviceUnit2RealWorldUnit(int deviceUnit, int mode)
{
    double realValue;
    ISC_GetRealValueFromDeviceUnit(m_serialNo, deviceUnit, &realValue, mode);
    return (double)realValue / m_additionalFactor;
}

//----------------------------------------------------------------------------------------------------------------------------------
int ThorlabsISM::realWorldUnit2DeviceUnit(double realWorldUnit, int mode)
{
    int deviceUnit;
    ISC_GetDeviceUnitFromRealValue(m_serialNo, realWorldUnit, &deviceUnit, mode);
    return qRound(deviceUnit * m_additionalFactor);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for one axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Number of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsISM::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    // Home device
    ISC_ClearMessageQueue(m_serialNo);
    ito::RetVal retval;

    if (axis != 0)
    {
        retval += ito::RetVal(ito::retError, 0, "This motor device has only a single axis with index 0");
    }
    else if (m_params["homingAvailable"].getVal<int>() == 0)
    {
        retval += ito::RetVal(ito::retError, 0, "this device does not support homing");
    }
    else
    {
        ISC_ClearMessageQueue(m_serialNo);
        retval += checkError(ISC_Home(m_serialNo), "start homing");
    }

    DWORD s = ISC_GetStatusBits(m_serialNo);

    if ((s & 0x80000000) == 0)
    {
        retval += ito::RetVal(ito::retError, 0, "homing not possible if the axis is not enabled.");
    }

    if (!retval.containsError())
    {
        // wait for completion
        WORD messageType;
        WORD messageId;
        DWORD messageData;
        bool interrupted = false;

        do
        {
            while (ISC_MessageQueueSize(m_serialNo) == 0)
            {
                setAlive();

                if (isInterrupted())
                {
                    retval += checkError(ISC_StopImmediate(m_serialNo), "interrupt homing");
                    retval += ito::RetVal(ito::retError, 0, "Homing interrupted");
                    interrupted = true;
                    break;
                }
                Sleep(100);
            }

            if (!interrupted)
            {
                ISC_WaitForMessage(m_serialNo, &messageType, &messageId, &messageData);
            }

        } while (!interrupted && (messageType != 2 || messageId != 0));

        int accel, speed;
        retval += checkError(ISC_GetVelParams(m_serialNo, &accel, &speed), "get speed and acceleration");
        m_params["speed"].setVal<double>(deviceUnit2RealWorldUnit(speed,1));
        m_params["accel"].setVal<double>(deviceUnit2RealWorldUnit(accel,2));
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    }

    Sleep(200); //due to position polling, the current position will be available after 200ms at the latest
    QSharedPointer<double> pos(new double);
    retval += getPos(0, pos, NULL);

    QSharedPointer<QVector<int> > status(new QVector<int>(1, 0));
    retval += getStatus(status, NULL);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for a set of axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Vector this numbers of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsISM::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (axis.size() != 1 || axis[0] != 0)
    {
        retval += ito::RetVal(ito::retError, 0, "This motor device only supports one axis with index 0");
    }
    else
    {
        retval += calib(axis[0], NULL);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsISM::setOrigin(const int axis, ItomSharedSemaphore * waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, "set origin not implemented");

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsISM::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, "set origin not implemented");

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function gets the status of the device. The SMCStatus function is called internally.

    \param [out] status        Status of System. 0: okay, 1: error
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \todo define the status value
*/
ito::RetVal ThorlabsISM::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    DWORD s = ISC_GetStatusBits(m_serialNo);

    m_currentStatus[0] = (s & (0x00000010 | 0x00000020)) ? ito::actuatorMoving : ito::actuatorAtTarget;
    m_currentStatus[0] |= (s & (0x00000001 | 0x00000002 | 0x00000004 | 0x00000008)) ? ito::actuatorEndSwitch : 0;
    m_currentStatus[0] |= ito::actuatorAvailable; //the connected flag is not always set

    if (s & 0x80000000)
    {
        m_currentStatus[0] |= ito::actuatorEnabled;
    }

    int homed = (s & 0x00000400) ? 1 : 0;
    int enabled = (s & 0x80000000) ? 1 : 0;


    if (m_params["homed"].getVal<int>() != homed || m_params["enabled"].getVal<int>() != enabled)
    {
        m_params["homed"].setVal<int>(homed);
        m_params["enabled"].setVal<int>(enabled);
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    }

    *status = m_currentStatus;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a single axis spezified by axis. The value in device independet in mm.

    \param [in] axis        Axisnumber
    \param [out] pos        Current position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal ThorlabsISM::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (axis != 0)
    {
        retval += ito::RetVal(ito::retError, 0, "This motor device has only a single axis with index 0");
    }
    else
    {
        *pos = deviceUnit2RealWorldUnit(ISC_GetPosition(m_serialNo),0);
        m_currentPos[0] = *pos;
        sendStatusUpdate(false);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a set of axis spezified by "axis". The value in device independet in mm.
            In this case if more than one axis is specified this function returns an error.

    \param [in] axis        Vector with axis numbers
    \param [out] pos        Current positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal ThorlabsISM::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (axis.size() != 1 || axis[0] != 0)
    {
        retval += ito::RetVal(ito::retError, 0, "This motor device only supports one axis with index 0");
    }
    else
    {
        QSharedPointer<double> pos_(new double);
        retval += getPos(axis[0], pos_, NULL);
        (*pos)[0] = *pos_;
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            This function calls ThorlabsISM::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis     axis number
    \param [in] pos      absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsISM::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (axis != 0)
    {
        retval += ito::RetVal(ito::retError, 0, "This motor device has only a single axis with index 0");
    }
    else if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("Any motor axis is moving. The motor is locked.").toLatin1().data());

    }
    else
    {
        QVector<int> axis_(1, 0);
        double newPosition = pos; //real world unit

        if (m_params["travelMode"].getVal<int>() == MOT_Rotational)
        {
            int c = newPosition / 360;
            newPosition -= (c * 360);
            if (newPosition < 0.0)
                newPosition += 360.0;
        }

        if (newPosition < m_params["stagePosMin"].getVal<double>() || newPosition > m_params["stagePosMax"].getVal<double>())
        {
            retval += ito::RetVal::format(ito::retError, 0, "new position out of range of stage [%f,%f].", m_params["stagePosMin"].getVal<double>(), m_params["stagePosMax"].getVal<double>());
        }
        else
        {
            m_targetPos[0] = newPosition;
            sendTargetUpdate();
            int systemPos = realWorldUnit2DeviceUnit(pos,0);

            retval += checkError(ISC_SetMoveAbsolutePosition(m_serialNo, systemPos), "set move absolute position");

            if (!retval.containsError())
            {
                retval += checkError(ISC_MoveAbsolute(m_serialNo), "move absolute");

                if (!retval.containsError())
                {
                    setStatus(axis_, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                    sendStatusUpdate();

                    if (m_async && waitCond)
                    {
                        waitCond->returnValue = retval;
                        waitCond->release();
                        waitCond = NULL;
                    }

                    retval += waitForDone(m_params["timeout"].getVal<double>() * 1000.0, axis_); //drops into timeout
                }

                if (!retval.containsError())
                {
                    replaceStatus(axis_, ito::actuatorMoving, ito::actuatorAtTarget);
                    sendStatusUpdate();
                }

                if (!m_async && waitCond)
                {
                    waitCond->returnValue = retval;
                    waitCond->release();
                    waitCond = NULL;
                }
            }
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls ThorlabsISM::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with absolute positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsISM::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval;

    if (axis.size() != 1 || axis[0] != 0)
    {
        ItomSharedSemaphoreLocker locker(waitCond);
        retval += ito::RetVal(ito::retError, 0, "This motor device only supports one axis with index 0");
        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
    }
    else
    {
        retval += setPosAbs(axis[0], pos[0], waitCond);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the relativ position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            This function calls ThorlabsISM::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        relative target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsISM::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (axis != 0)
    {
        retval += ito::RetVal(ito::retError, 0, "This motor device has only a single axis with index 0");
    }
    else if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("Any motor axis is moving. The motor is locked.").toLatin1().data());

    }
    else
    {
        QVector<int> axis_(1, 0);
        m_targetPos[0] = m_currentPos[0] + pos;
        sendTargetUpdate();

        retval += checkError(ISC_SetMoveRelativeDistance(m_serialNo, realWorldUnit2DeviceUnit(pos, 0)), "set move relative position");

        if (!retval.containsError())
        {
            retval += checkError(ISC_MoveRelativeDistance(m_serialNo), "move relative");

            if (!retval.containsError())
            {
                setStatus(axis_, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                sendStatusUpdate();

                if (m_async && waitCond)
                {
                    waitCond->returnValue = retval;
                    waitCond->release();
                    waitCond = NULL;
                }

                retval += waitForDone(m_params["timeout"].getVal<double>() * 1000.0, axis_); //drops into timeout
            }

            if (!retval.containsError())
            {
                replaceStatus(axis_, ito::actuatorMoving, ito::actuatorAtTarget);
                sendStatusUpdate();
            }

            if (!m_async && waitCond)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                waitCond = NULL;
            }
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls ThorlabsISM::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with relative positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsISM::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval;

    if (axis.size() != 1 || axis[0] != 0)
    {
        ItomSharedSemaphoreLocker locker(waitCond);
        retval += ito::RetVal(ito::retError, 0, "This motor device only supports one axis with index 0");
        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
    }
    else
    {
        retval += setPosRel(axis[0], pos[0], waitCond);
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the position after ever positioning command.
            It sends the current postion and the status to the world.

    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsISM::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    if (sendCurrentPos)
    {
        QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);
        retval += getPos(0, sharedpos, NULL);
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
ito::RetVal ThorlabsISM::waitForDone(const int timeoutMS, const QVector<int> axis /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retval(ito::retOk);
    bool done = false;
    DWORD status;
    Sleep(250); //to be sure that the first requested status is correct
    QElapsedTimer timer;
    timer.start();
    qint64 lastTime = timer.elapsed();


    while (!done && !retval.containsWarningOrError())
    {
        if (isInterrupted())
        {
            retval += checkError(ISC_StopImmediate(m_serialNo), "stop immediate");
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorInterrupted);

            retval += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            done = true;

            sendStatusUpdate(true);
        }
        else if (timer.elapsed() > timeoutMS)
        {
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
            retval += checkError(ISC_StopImmediate(m_serialNo), "stop immediate");
            retval += ito::RetVal(ito::retError, 0, tr("timeout occurred").toLatin1().data());
            done = true;

            sendStatusUpdate(true);
        }
        else
        {
            status = ISC_GetStatusBits(m_serialNo);
            setStatus(axis, ito::actuatorAvailable | ((status & 0x80000000) ? ito::actuatorEnabled : 0), ito::actMovingMask | ito::actSwitchesMask);

            if (status & (0x00000001 | 0x00000002 | 0x00000004 | 0x00000008))
            {
                setStatus(axis, ito::actuatorEndSwitch, ito::actMovingMask | ito::actStatusMask);
                done = true;
                retval += ito::RetVal(ito::retError, 0, tr("end switch reached").toLatin1().data());
            }
            else if ((status & (0x00000010 | 0x00000020)) == 0)
            {
                //motor is at target
                done = true;
            }

            if (done)
            {   // Position reached and movement done
                replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
                sendStatusUpdate(true);
                m_currentPos[0] = deviceUnit2RealWorldUnit(ISC_GetPosition(m_serialNo),0);
                sendStatusUpdate(false);
                break;
            }
            else
            {
                if ((timer.elapsed() - lastTime) > 250.0)
                {
                    m_currentPos[0] = deviceUnit2RealWorldUnit(ISC_GetPosition(m_serialNo),0);
                    sendStatusUpdate(false);
                    lastTime = timer.elapsed();
                }

            }

            if (!retval.containsError())
            {
                setAlive();
            }
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ThorlabsISM::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *w = getDockWidget()->widget();

        if (w)
        {
            if (visible)
            {
                connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), w, SLOT(parametersChanged(QMap<QString, ito::Param>)));
                emit parametersChanged(m_params);

            }
            else
            {
                QObject::disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), w, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsISM::checkError(short value, const char* message)
{
    if (value == 0)
    {
        return ito::retOk;
    }
    else
    {
        switch (value)
        {
        case 1: return ito::RetVal::format(ito::retError, 1, "%s: The FTDI functions have not been initialized.", message);
        case 2: return ito::RetVal::format(ito::retError, 1, "%s: The Device could not be found. This can be generated if the function TLI_BuildDeviceList() has not been called.", message);
        case 3: return ito::RetVal::format(ito::retError, 1, "%s: The Device must be opened before it can be accessed. See the appropriate Open function for your device.", message);
        case 4: return ito::RetVal::format(ito::retError, 1, "%s: An I/O Error has occured in the FTDI chip.", message);
        case 5: return ito::RetVal::format(ito::retError, 1, "%s: There are Insufficient resources to run this application.", message);
        case 6: return ito::RetVal::format(ito::retError, 1, "%s: An invalid parameter has been supplied to the device.", message);
        case 7: return ito::RetVal::format(ito::retError, 1, "%s: The Device is no longer present. The device may have been disconnected since the last TLI_BuildDeviceList() call.", message);
        case 8: return ito::RetVal::format(ito::retError, 1, "%s: The device detected does not match that expected.", message);
        case 16: return ito::RetVal::format(ito::retError, 1, "%s: The library for this device could not be found.", message);
        case 17: return ito::RetVal::format(ito::retError, 1, "%s: No functions available for this device.", message);
        case 18: return ito::RetVal::format(ito::retError, 1, "%s: The function is not available for this device.", message);
        case 19: return ito::RetVal::format(ito::retError, 1, "%s: Bad function pointer detected.", message);
        case 20: return ito::RetVal::format(ito::retError, 1, "%s: The function failed to complete succesfully.", message);
        case 21: return ito::RetVal::format(ito::retError, 1, "%s: The function failed to complete succesfully.", message);
        case 32: return ito::RetVal::format(ito::retError, 1, "%s: Attempt to open a device that was already open.", message);
        case 33: return ito::RetVal::format(ito::retError, 1, "%s: The device has stopped responding.", message);
        case 34: return ito::RetVal::format(ito::retError, 1, "%s: This function has not been implemented.", message);
        case 35: return ito::RetVal::format(ito::retError, 1, "%s: The device has reported a fault.", message);
        case 36: return ito::RetVal::format(ito::retError, 1, "%s: The function could not be completed at this time.", message);
        case 40: return ito::RetVal::format(ito::retError, 1, "%s: The function could not be completed because the device is disconnected.", message);
        case 41: return ito::RetVal::format(ito::retError, 1, "%s: The firmware has thrown an error", message);
        case 42: return ito::RetVal::format(ito::retError, 1, "%s: The device has failed to initialize", message);
        case 43: return ito::RetVal::format(ito::retError, 1, "%s: An Invalid channel address was supplied", message);
        case 37: return ito::RetVal::format(ito::retError, 1, "%s: The device cannot perform this function until it has been Homed.", message);
        case 38: return ito::RetVal::format(ito::retError, 1, "%s: The function cannot be performed as it would result in an illegal position.", message);
        case 39: return ito::RetVal::format(ito::retError, 1, "%s: An invalid velocity parameter was supplied. The velocity must be greater than zero.", message);
        case 44: return ito::RetVal::format(ito::retError, 1, "%s: This device does not support Homing. Check the Limit switch parameters are correct.", message);
        case 45: return ito::RetVal::format(ito::retError, 1, "%s: An invalid jog mode was supplied for the jog function.", message);
        case 46: return ito::RetVal::format(ito::retError, 1, "%s: There is no Motor Parameters available to convert Real World Units.", message);
        case 47: return ito::RetVal::format(ito::retError, 1, "%s: Command temporarily unavailable, Device may be busy.", message);
        default:
            return ito::RetVal::format(
                ito::retError, value, "%s: unknown error %i.", message, value);
        }
    }
}
