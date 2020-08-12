/* ********************************************************************
Plugin "ThorlabsFF" for itom software
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

#include "ThorlabsFF.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#define NOMINMAX // https://stackoverflow.com/questions/22744262/cant-call-stdmax-because-minwindef-h-defines-max

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>

#include "dockWidgetThorlabsFF.h"

#include "Thorlabs.MotionControl.FilterFlipper.h"

QList<QByteArray> ThorlabsFF::openedDevices = QList<QByteArray>();

//----------------------------------------------------------------------------------------------------------------------------------
ThorlabsFFInterface::ThorlabsFFInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("ThorlabsFF");

    m_description = QObject::tr("ThorlabsFF");

    m_detaildescription = QObject::tr("ThorlabsFF is an plugin to controll the Thorlabs Filter Flipper: \n\
\n\
* Filter Flipper (MFF101) \n\
\n\
It requires the new Kinesis driver package from Thorlabs and implements the interface Thorlabs.MotionControl.IntegratedStepperMotors.\n\
\n\
Please install the Kinesis driver package in advance with the same bit-version (32/64bit) than itom. \n\
\n\
This plugin has been tested with the flipper MFF101.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION); 

    m_initParamsOpt.append(ito::Param("serialNo", ito::ParamBase::String, "", tr("Serial number of the device to be loaded, if empty, the first device that can be opened will be opened").toLatin1().data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
ThorlabsFFInterface::~ThorlabsFFInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsFFInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(ThorlabsFF) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsFFInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(ThorlabsFF) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
ThorlabsFF::ThorlabsFF() : AddInDataIO(),
m_opened(false)
{
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ThorlabsFF", tr("name of the plugin").toLatin1().data()));
    m_params.insert("deviceName", ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("description of the device").toLatin1().data()));
    m_params.insert("serialNumber", ito::Param("serialNumber", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("serial number of the device").toLatin1().data()));
    
    m_params.insert("pollingInterval", ito::Param("pollingInterval", ito::ParamBase::Int | ito::ParamBase::Readonly, 200, new ito::IntMeta(1, 10000, 1, "pollingInterval"), tr("device polling interval in ms").toLatin1().data()));

    m_params.insert("position", ito::Param("position", ito::ParamBase::Int, FF_Positions::Position1, \
        new ito::IntMeta(FF_Positions::Position1, FF_Positions::Position2, FF_Positions::Position1, "position"), \
        tr("position of the device (position1: %2, position2: %3)").arg(FF_Positions::Position1).arg(FF_Positions::Position2).toLatin1().data()));

    m_params.insert("transitTime", ito::Param("transitTime", ito::ParamBase::Int, 300, new ito::IntMeta(300, 2800, 1, "transitTime"), tr("transit time of the device in ms").toLatin1().data()));

    m_params.insert("firmwareVersion", ito::Param("firmwareVersion", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, tr("firmware version of the connected device").toLatin1().data()));
    m_params.insert("softwareVersion", ito::Param("softwareVersion", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, tr("software version of the connected device").toLatin1().data()));

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetThorlabsFF *dockWidget = new DockWidgetThorlabsFF(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
    }
    memset(m_serialNo, '\0', sizeof(m_serialNo));
}

//----------------------------------------------------------------------------------------------------------------------------------
ThorlabsFF::~ThorlabsFF()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsFF::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    QByteArray serialNo = paramsOpt->at(0).getVal<char*>();

    retValue += checkError(TLI_BuildDeviceList(), "build device list");
    QByteArray existingSerialNo("", 256);
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
            retValue += checkError(TLI_GetDeviceListExt(existingSerialNo.data(), existingSerialNo.size()), "get device list");
        }
    }

    if (!retValue.containsError())
    {
        int idx = existingSerialNo.indexOf("\0");
        if (idx > 0)
        {
            existingSerialNo = existingSerialNo.left(idx); //serial number found 
        }

        QList<QByteArray> serialNumbers = existingSerialNo.split(','); // Thorlabs really!?

        if (serialNo == "") // no serial number input
        {
            bool found = false;
            for (int cnt = 0; cnt < serialNumbers.size(); cnt++)
            {
                if (!openedDevices.contains(serialNumbers[cnt]))
                {
                    serialNo = serialNumbers[cnt];
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                retValue += ito::RetVal(ito::retError, 0, "no free Thorlabs device found.");
            }

        }
        else // serial number input given
        {
            bool found = false;
            for each (const QByteArray &s in serialNumbers)
            {
                if (s == serialNo && s != "")
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                retValue += ito::RetVal::format(ito::retError, 0, "no device with the serial number '%s' found", serialNo.data());
            }
            else if (openedDevices.contains(serialNo))
            {
                retValue += ito::RetVal::format(ito::retError, 0, "Thorlabs device with the serial number '%s' already in use.", serialNo.data());
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
            m_params["serialNumber"].setVal<char*>(serialNo.data()); // bug: deviceInfo.serialNo is sometimes wrong if more than one of the same devices are connected
            setIdentifier(QLatin1String(serialNo.data())); // bug: deviceInfo.serialNo is sometimes wrong if more than one of the same devices are connected
            m_params["deviceName"].setVal<char*>(deviceInfo.description);
        }
    }

    if (!retValue.containsError()) //open device
    {
        if (deviceInfo.isKnownType && (deviceInfo.typeID == 37 /*Filter Flipper*/))
        {
            memcpy(m_serialNo, serialNo.data(), std::min((size_t)serialNo.size(), sizeof(m_serialNo)));
            retValue += checkError(FF_Open(m_serialNo), "open device");

            if (!retValue.containsError())
            {
                m_opened = true;
                openedDevices.append(m_serialNo);
            }

            FF_StartPolling(m_serialNo, m_params["pollingInterval"].getVal<int>());//start device polling at pollingInterval intervals 
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, "the type of the device is not among the supported devices (Long Travel Stage, Labjack, Cage Rotator)");
        }
    }

    if (!retValue.containsError()) // get current parameters
    {
        m_params["position"].setVal<int>(FF_GetPosition(m_serialNo));
        m_params["firmwareVersion"].setVal<int>((int)FF_GetFirmwareVersion(m_serialNo));
        m_params["softwareVersion"].setVal<int>((int)FF_GetSoftwareVersion(m_serialNo));

        retValue += checkError(FF_SetTransitTime(m_serialNo, (unsigned int)m_params["transitTime"].getMin()), "setting transit time");

        m_params["transitTime"].setVal<int>((int)FF_GetTransitTime(m_serialNo));
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

    setInitialized(true); //init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsFF::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    FF_StopPolling(m_serialNo);
    FF_Close(m_serialNo);

    m_opened = false;
    openedDevices.removeOne(m_serialNo);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsFF::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (key == "transitTime")
        {
            val->setVal<int>((unsigned int)FF_GetTransitTime(m_serialNo));
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsFF::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "position")
        {
            FF_MoveToPosition(m_serialNo, val->getVal<int>() == 1 ? FF_Positions::Position2 : FF_Positions::Position1);

            int position = FF_GetPosition(m_serialNo);
            if (position == FF_Positions::FF_PositionError)
            {
                retValue += ito::RetVal(ito::retError, 0, "position error");
            }
            else
            {
                it->setVal<int>(position);
            }
            
        }
        else if (key == "transitTime")
        {
            unsigned int time = (unsigned int)val->getVal<int>();

            retValue += checkError(FF_SetTransitTime(m_serialNo, time), "setting transit time");
            m_params["transitTime"].setVal<int>((int)FF_GetTransitTime(m_serialNo));
        }
        else
        {
            retValue += it->copyValueFrom( &(*val) );
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
void ThorlabsFF::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *widget = getDockWidget()->widget();
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal ThorlabsFF::checkError(short value, const char* message)
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
