/* ********************************************************************
    Plugin "ThorlabsISM" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2020, Institut fuer Technische Optik (ITO),
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

#include "ThorlabsKCubeIM.h"

#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qtimer.h>
#include <qwaitcondition.h>
#include <qelapsedtimer.h>
#include <qdatetime.h>

#include <qdebug.h>


QList<QByteArray> ThorlabsKCubeIM::openedDevices = QList<QByteArray>();

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of ThorlabsISMInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created ThorlabsISMInterface-instance is stored in *addInInst
    \return retOk
    \sa ThorlabsISM
*/
ito::RetVal ThorlabsKCubeIMInterface::getAddInInst(ito::AddInBase **addInInst)
{
    ito::RetVal retValue;
    NEW_PLUGININSTANCE(ThorlabsKCubeIM)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of ThorlabsISMInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa ThorlabsISM
*/
ito::RetVal ThorlabsKCubeIMInterface::closeThisInst(ito::AddInBase **addInInst)
{

    ito::RetVal retValue;
    REMOVE_PLUGININSTANCE(ThorlabsKCubeIM)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
ThorlabsKCubeIMInterface::ThorlabsKCubeIMInterface()
{
    m_type = ito::typeActuator;

    setObjectName("ThorlabsKCubeIM");

    m_description = QObject::tr("ThorlabsKCubeIM");
    m_detaildescription = QObject::tr("ThorlabsKCubeIM is an acutator plugin to control the following integrated devices from Thorlabs: \n\
\n\
* K-Cube Controller for Piezo Inertia Stages and Actuators (e.g. KIM101) \n\
\n\
It requires the new Kinesis driver package from Thorlabs and implements the interface Thorlabs.MotionControl.KCube.InertialMotor.\n\
\n\
Please install the Kinesis driver package in advance with the same bit-version (32/64bit) than itom. \n\
\n\
This plugin has been tested with the cage rotator KIM101.");

    m_author = "J. Krauter, Trumpf Lasersystems For Semiconductor Manufacturing Gmbh";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);    
    
    m_initParamsOpt.append(ito::Param("serialNo", ito::ParamBase::String, "", tr("Serial number of the device to be loaded, if empty, the first device that can be opened will be opened").toLatin1().data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class ThorlabsISMInterface with the name ThorlabsISMInterface as plugin for the Qt-System (see Qt-DOC)


//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the ThorlabsISM::init. The widged window is created at this position.
*/
ThorlabsKCubeIM::ThorlabsKCubeIM() :
    AddInActuator(),
    m_async(0),
    m_opened(false),
    m_numaxis(0)
{
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ThorlabsKCubeIM", tr("Name of the plugin").toLatin1().data()));
    m_params.insert("numaxis", ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 4, 4, tr("number of axes (channels), default 4").toLatin1().data()));
    m_params.insert("deviceName", ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Description of the device").toLatin1().data()));
    m_params.insert("serialNumber", ito::Param("serialNumber", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Serial number of the device").toLatin1().data()));
    m_params.insert("firmwareVersion", ito::Param("firmwareVersion", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Firmware version of the device").toLatin1().data()));
    m_params.insert("softwareVersion", ito::Param("softwareVersion", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("Software version of the device").toLatin1().data()));
    
    ito::int32 voltage[4] = { 112, 112, 112, 112};
    m_params.insert("maxVoltage", ito::Param("maxVoltage", ito::ParamBase::IntArray, 4, voltage, new ito::IntArrayMeta(85, 125, 1, 4, 4), tr("maximum voltage of axis").toLatin1().data()));
    
    ito::int32 steps[4] = { 500, 500, 500, 500 };
    m_params.insert("stepRate", ito::Param("stepRate", ito::ParamBase::IntArray, 4, steps, new ito::IntArrayMeta(1, 2000, 1, 4, 4), tr("step rate in Steps/s").toLatin1().data()));

    ito::int32 accel[4] = { 1000, 1000, 1000, 1000 };
    m_params.insert("acceleration", ito::Param("acceleration", ito::ParamBase::IntArray, 4, accel, new ito::IntArrayMeta(1, 100000, 1, 4, 4), tr("acceleration Steps/s\0x5E2").toLatin1().data()));

    m_params.insert("async", ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1) or sychronous (0) mode").toLatin1().data()));
    m_params.insert("timeout", ito::Param("timeout", ito::ParamBase::Double, 0.0, 200.0, 100.0, tr("timeout for move operations in sec").toLatin1().data()));

    m_params.insert("lockFrontPanel", ito::Param("lockFrontPanel", ito::ParamBase::Int, 0, 1, 0, tr("1 to lock the front panel, else 0").toLatin1().data()));

    m_currentPos.fill(0.0, 4);
    m_currentStatus.fill(0, 4);
    m_targetPos.fill(0.0, 4);
    
    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetThorlabsKCubeIM *dockWidget = new DockWidgetThorlabsKCubeIM(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
    }
    
    memset(m_serialNo, '\0', sizeof(m_serialNo));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! function to map axis number to KIM_Channels enum
*/
std::map<unsigned int, KIM_Channels> KIMChannelMap
{
    { 0, KIM_Channels::Channel1 },
    { 1, KIM_Channels::Channel2 },
    { 2, KIM_Channels::Channel3 },
    { 3, KIM_Channels::Channel4 }
};

KIM_Channels WhatChannel(unsigned int num) 
{
    return KIMChannelMap[num];
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
ito::RetVal ThorlabsKCubeIM::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    QByteArray serial = paramsOpt->at(0).getVal<char*>();

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
            m_params["serialNumber"].setVal<char*>(serial.data()); // bug: deviceInfo.serialNo is sometimes wrong if more than one of the same devices are connected
            setIdentifier(QLatin1String(serial.data())); // bug: deviceInfo.serialNo is sometimes wrong if more than one of the same devices are connected
            m_params["deviceName"].setVal<char*>(deviceInfo.description);
        }
    }

    if (!retval.containsError())
    {
        if (deviceInfo.isKnownType && (deviceInfo.typeID == 97 /*KCube Inertial Motor*/)) //TODO check typeID
        {
            memcpy(m_serialNo, serial.data(), std::min((size_t)serial.size(), sizeof(m_serialNo)));
            retval += checkError(KIM_Open(m_serialNo), "open device");

            if (!retval.containsError())
            {
                m_opened = true;
                openedDevices.append(m_serialNo);
            }
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "the type of the device is not among the supported devices (Long Travel Stage, Labjack, Cage Rotator)");
        }
    }

    if (!retval.containsError())
    {        
        if (!KIM_LoadSettings(m_serialNo))
        {
            retval += ito::RetVal(ito::retWarning, 0, "settings of device could not be loaded.");
        }
        if (!KIM_StartPolling(m_serialNo, 150))
        {
            retval += ito::RetVal(ito::retError, 0, "error starting position and status polling.");
        }

        // get firmware version
        std::string verionStr = std::to_string(KIM_GetFirmwareVersion(m_serialNo));
        char * versionChar = new char[verionStr.size() + 1];
        std::copy(verionStr.begin(), verionStr.end(), versionChar);
        versionChar[verionStr.size()] = '\0'; // don't forget the terminating 0

        m_params["firmwareVersion"].setVal<char*>(versionChar);
        delete[] versionChar;

        // get software version
        verionStr = std::to_string(KIM_GetSoftwareVersion(m_serialNo));
        versionChar = new char[verionStr.size() + 1];
        std::copy(verionStr.begin(), verionStr.end(), versionChar);
        versionChar[verionStr.size()] = '\0'; // don't forget the terminating 0

        m_params["softwareVersion"].setVal<char*>(versionChar);
        delete[] versionChar;

        //get num of axis
        // num = sizeof(KIM_Channels); does not work
        m_numaxis = m_params["numaxis"].getVal<int>();
        
        // get the device parameter here
        bool deviceCanLockFrontPanel = KIM_CanDeviceLockFrontPanel(m_serialNo) ? 1 : 0;
        m_params["lockFrontPanel"].setVal<int>(KIM_GetFrontPanelLocked(m_serialNo) ? 1 : 0);
        if (!deviceCanLockFrontPanel)
        {
            m_params["lockFrontPanel"].setFlags(ito::ParamBase::Readonly);
        }

        Sleep(200);

        // get current positions
        m_currentPos.resize(m_numaxis);
        m_currentStatus.resize(m_numaxis);
        m_targetPos.resize(m_numaxis);

        for (int i = 0; i < m_currentPos.size(); i++)
        {
            m_currentPos[i] = KIM_GetCurrentPosition(m_serialNo, WhatChannel(i));
            m_currentStatus[i] = ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
            m_targetPos[i] = 0.0;
        }

        // get the drive parameters
        Sleep(200);
        int maxVol[4];
        int stepRate[4];
        int accel[4];
        for(int axis = 0; axis < m_numaxis; axis++)
        {    
            short vol;
            retval += checkError(KIM_GetDriveOPParameters(m_serialNo, WhatChannel(axis), vol, stepRate[axis], accel[axis]), "get drive OP Parameter");
            maxVol[axis] = int(vol);
        }

        m_params["maxVoltage"].setVal<int*>(maxVol, 4);
        m_params["stepRate"].setVal<int*>(stepRate, 4);
        m_params["acceleration"].setVal<int*>(accel, 4);

        
    }

    if (!retval.containsError())
    {
        Sleep(200);
        QSharedPointer<QVector<int> > status(new QVector<int>(1, 0));
        retval += getStatus(status, NULL);
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
const ito::RetVal ThorlabsKCubeIM::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogThorlabsKCubeIM(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the ThorlabsISMInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ThorlabsKCubeIM::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (m_opened)
    {
        KIM_StopPolling(m_serialNo);
        Sleep(300);

        KIM_Close(m_serialNo);
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
    \detail It is used to set the parameter of type int/double with key "name" stored in m_params and the corresponding member variabels. 
            This function is defined by the actuator class and overwritten at this position.

    \param[in] *name        Name of parameter
    \param[out] val            New parameter value as double
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal ThorlabsKCubeIM::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal ThorlabsKCubeIM::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
            retValue += checkError(KIM_SetFrontPanelLock(m_serialNo, val->getVal<int>() ? true : false), "setParam to lock frontpanel");
            val->setVal<int>(KIM_GetFrontPanelLocked(m_serialNo) ? 1 : 0);
        }
        else if (key == "maxVoltage")
        {
            int *maxVol = val->getVal<int*>();

            for (int axis = 0; axis < m_numaxis; axis++)
            {
                retValue += checkError(KIM_SetDriveOPParameters(m_serialNo, WhatChannel(axis), maxVol[axis], m_params["stepRate"].getVal<int*>()[axis], m_params["acceleration"].getVal<int*>()[axis]), "set max voltage");
            }

            retValue += it->copyValueFrom(&(*val));

        }
        else if (key == "stepRate")
        {
            int *stepRate = val->getVal<int*>();

            for (int axis = 0; axis < m_numaxis; axis++)
            {
                retValue += checkError(KIM_SetDriveOPParameters(m_serialNo, WhatChannel(axis), m_params["maxVoltage"].getVal<int*>()[axis], stepRate[axis], m_params["acceleration"].getVal<int*>()[axis]), "set step rate");
            }

            retValue += it->copyValueFrom(&(*val));

        }
        else if (key == "acceleration")
        {
            int *accel = val->getVal<int*>();

            for (int axis = 0; axis < m_numaxis; axis++)
            {
                retValue += checkError(KIM_SetDriveOPParameters(m_serialNo, WhatChannel(axis), m_params["maxVoltage"].getVal<int*>()[axis], m_params["stepRate"].getVal<int*>()[axis], accel[axis]), "set acceleration");
            }

            retValue += it->copyValueFrom(&(*val));
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
//! calib
/*!
the given axis should be calibrated (e.g. by moving to a reference switch).
*/
ito::RetVal ThorlabsKCubeIM::calib(const int axis, ItomSharedSemaphore *waitCond)
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

//----------------------------------------------------------------------------------------------------------------------------------
//! calib
/*!
the given axes should be calibrated (e.g. by moving to a reference switch).
*/
ito::RetVal ThorlabsKCubeIM::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toLatin1().data());

    }
    else
    {
        //todo:
        //start calibrating the given axes and don't forget to regularily call setAlive().
        //this is important if the calibration needs more time than the timeout time of itom (e.g. 5sec).
        //itom regularily checks the alive flag and only drops to a timeout if setAlive() is not regularily called (at least all 3-4 secs).

        setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        sendStatusUpdate();

        foreach(const int& i, axis)
        {
            retValue += checkError(KIM_ZeroPosition(m_serialNo, WhatChannel(i)), "set position as new zero");
            m_targetPos[i] = 0.0;
            
            retValue += waitForDone(m_params["timeout"].getVal<double>() * 1000.0, axis); //should drop into timeout
            setStatus(m_currentStatus[i], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
        }

        sendStatusUpdate();

    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();

    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubeIM::setOrigin(const int axis, ItomSharedSemaphore * waitCond)
{
    return setOrigin(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubeIM::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        foreach(const int &i, axis)
        {
            if (i >= 0 && i < m_numaxis)
            {
                retValue += checkError(KIM_Home(m_serialNo, WhatChannel(i)), "home axis");
            }
            else
            {
                retValue += ito::RetVal::format(ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            m_currentPos[i] = 0.0;
            m_targetPos[i] = 0.0;
        }

        sendStatusUpdate();
        sendTargetUpdate();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function gets the status of the device. The SMCStatus function is called internally.

    \param [out] status        Status of System. 0: okay, 1: error
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \todo define the status value
*/
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubeIM::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubeIM::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if ((axis >= m_numaxis) || axis < 0)
    {
        retValue = ito::RetVal(ito::retError, 1, tr("axis index is out of bound").toLatin1().data());
    }
    else
    {
        m_currentPos[axis] = KIM_GetCurrentPosition(m_serialNo, WhatChannel(axis));
        retValue = ito::retOk;
        *pos = m_currentPos[axis];
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsKCubeIM::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
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
            retValue += getPos(axis[naxis], pos_, NULL);
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

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosAbs
/*!
starts moving the given axis to the desired absolute target position

depending on m_async this method directly returns after starting the movement (async = 1) or
only returns if the axis reached the given target position (async = 0)

In some cases only relative movements are possible, then get the current position, determine the
relative movement and call the method relatively move the axis.
*/
ito::RetVal ThorlabsKCubeIM::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosAbs(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosAbs
/*!
starts moving all given axes to the desired absolute target positions

depending on m_async this method directly returns after starting the movement (async = 1) or
only returns if all axes reached their given target positions (async = 0)

In some cases only relative movements are possible, then get the current position, determine the
relative movement and call the method relatively move the axis.
*/
ito::RetVal ThorlabsKCubeIM::setPosAbs(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible.").toLatin1().data());
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

        if (!retValue.containsError())
        {
            sendTargetUpdate();

            //set status of all given axes to moving and keep all flags related to the status and switches
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            //todo: start the movement
            foreach (const int axisNum, axis)
            {
                retValue += checkError(KIM_MoveAbsolute(m_serialNo, WhatChannel(axisNum), m_targetPos[axisNum]), "move absolute");

                //call waitForDone in order to wait until all axes reached their target or a given timeout expired
                //the m_currentPos and m_currentStatus vectors are updated within this function
                retValue += waitForDone(m_params["timeout"].getVal<double>() * 1000.0, axisNum, MoveType::Absolute); //WaitForAnswer(60000, axis);
                
            }

            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate();
                                                  
            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    //if the wait condition has not been released yet, do it now
    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosRel
/*!
starts moving the given axis by the given relative distance

depending on m_async this method directly returns after starting the movement (async = 1) or
only returns if the axis reached the given target position (async = 0)

In some cases only absolute movements are possible, then get the current position, determine the
new absolute target position and call setPosAbs with this absolute target position.
*/
ito::RetVal ThorlabsKCubeIM::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosRel(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosRel
/*!
starts moving the given axes by the given relative distances

depending on m_async this method directly returns after starting the movement (async = 1) or
only returns if all axes reached the given target positions (async = 0)

In some cases only absolute movements are possible, then get the current positions, determine the
new absolute target positions and call setPosAbs with these absolute target positions.
*/
ito::RetVal ThorlabsKCubeIM::setPosRel(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible.").toLatin1().data());
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

        if (!retValue.containsError())
        {
            sendTargetUpdate();

            //set status of all given axes to moving and keep all flags related to the status and switches
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            //todo: start the movement
            cntPos = 0;
            foreach(const int axisNum, axis)
            {
                retValue += checkError(KIM_MoveRelative(m_serialNo, WhatChannel(axisNum), pos[cntPos]), "move relative");

                //call waitForDone in order to wait until all axes reached their target or a given timeout expired
                //the m_currentPos and m_currentStatus vectors are updated within this function
                retValue += waitForDone(m_params["timeout"].getVal<double>() * 1000.0, axisNum, MoveType::Relative); //WaitForAnswer(60000, axis);
                cntPos++;
            }

            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate();

            //release the wait condition now, if async is false (itom waits until now if async is false, hence in the synchronous mode)
            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    //if the wait condition has not been released yet, do it now
    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the position after ever positioning command.
            It sends the current postion and the status to the world.

    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ThorlabsKCubeIM::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    if (sendCurrentPos)
    {
        QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);
        for (int axis = 0; axis < m_numaxis; axis++)
        {
            retval += getPos(axis, sharedpos, NULL);
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
ito::RetVal ThorlabsKCubeIM::waitForDone(const int timeoutMS, const int axis, const int flags)
{
    ito::RetVal retVal;
    retVal += waitForDone(timeoutMS, QVector<int>(1, axis), flags);
    return retVal;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
//! method must be overwritten from ito::AddInActuator
/*!
WaitForDone should wait for a moving motor until the indicated axes (or all axes of nothing is indicated) have stopped or a timeout or user interruption
occurred. The timeout can be given in milliseconds, or -1 if no timeout should be considered. The flag-parameter can be used for your own purpose.
*/
ito::RetVal ThorlabsKCubeIM::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    char motor;
    QTime timer;
    QMutex waitMutex;
    QWaitCondition waitCondition;
    
    long delay = 100; //[ms]

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
        //todo: obtain the current position, status... of all given axes
        
        foreach(const int &i, axis)
        {
            QSharedPointer<double> pos_(new double);
            retVal += getPos(i, pos_, NULL);
            m_currentPos[i] = *pos_;
        }

        done = true; //assume all axes at target
        motor = 0;
        foreach(const int &i, axis)
        {
            if ((std::abs(m_targetPos[i] - m_currentPos[i]) < 0.05) && (flags == MoveType::Absolute || flags == -1))
            {
                setStatus(m_currentStatus[i], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                done = true; //not done yet
            }
            else if (std::abs(m_targetPos[i] - m_currentPos[i]) < 0.05 && flags == MoveType::Relative)
            {
                setStatus(m_currentStatus[i], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                done = true;
            }
            else
            {
                setStatus(m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                done = false;
            }
        }

        //emit actuatorStatusChanged with both m_currentStatus and m_currentPos as arguments
        sendStatusUpdate(false);

        //now check if the interrupt flag has been set (e.g. by a button click on its dock widget)
        if (!done && isInterrupted())
        {
            //todo: force all axes to stop

            //set the status of all axes from moving to interrupted (only if moving was set before)
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            sendStatusUpdate(true);

            retVal += ito::RetVal(ito::retError, 0, "interrupt occurred");
            done = true;
            return retVal;
        }

        //short delay
        waitMutex.lock();
        waitCondition.wait(&waitMutex, delay);
        waitMutex.unlock();

        //raise the alive flag again, this is necessary such that itom does not drop into a timeout if the
        //positioning needs more time than the allowed timeout time.
        setAlive();

        if (timeoutMS > -1)
        {
            if (timer.elapsed() > timeoutMS) timeout = true;
        }
    }

    if (timeout)
    {
        //timeout occured, set the status of all currently moving axes to timeout
        replaceStatus(_axis, ito::actuatorMoving, ito::actuatorTimeout);
        retVal += ito::RetVal(ito::retError, 9999, "timeout occurred");
        sendStatusUpdate(true);
    }

    return retVal;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void ThorlabsKCubeIM::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *w = getDockWidget()->widget();

        if (w)
        {
            if (visible)
            {
                connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), w, SLOT(parametersChanged(QMap<QString, ito::Param>)));
                connect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), w, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
                connect(this, SIGNAL(targetChanged(QVector<double>)), w, SLOT(targetChanged(QVector<double>)));
                emit parametersChanged(m_params);
                sendTargetUpdate();
                sendStatusUpdate(false);
            }
            else
            {
                disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), w, SLOT(parametersChanged(QMap<QString, ito::Param>)));
                disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), w, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
                disconnect(this, SIGNAL(targetChanged(QVector<double>)), w, SLOT(targetChanged(QVector<double>)));
            }
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal ThorlabsKCubeIM::checkError(short value, const char* message)
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
            return ito::RetVal::format(ito::retError, 1, "%s: The FTDI functions have not been initialized.", message);
        case 2:
            return ito::RetVal::format(ito::retError, 1, "%s: The device could not be found.", message);
        case 3:
            return ito::RetVal::format(ito::retError, 1, "%s: The device must be opened before it can be accessed.", message);
        case 37:
            return ito::RetVal::format(ito::retError, 1, "%s: The device cannot perform the function until it has been homed (call calib() before).", message);
        case 38:
            return ito::RetVal::format(ito::retError, 1, "%s: The function cannot be performed as it would result in an illegal position.", message);
        case 39:
            return ito::RetVal::format(ito::retError, 1, "%s: An invalid velocity parameter was supplied. The velocity must be greater than zero. ", message);
        default:
            return ito::RetVal::format(ito::retError, value, "%s: unknown error %i.", message, value);
        }
    }
}
