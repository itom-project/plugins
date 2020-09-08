/* ********************************************************************
Plugin "OphirLMMeasurement" for itom software
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


#include <Windows.h>

#include <iostream>
#include <iomanip>

#include "OphirPlugin.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>

#include "dockWidgetOphir.h"

#include <qaxbase.h>
#include <qaxobject.h>
#include <quuid.h>
#include <qobject.h>

#define BUFFER_SIZE 100

QList<std::wstring> OphirPlugin::openedDevices = QList<std::wstring>();



//----------------------------------------------------------------------------------------------------------------------------------
OphirPluginInterface::OphirPluginInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("OphirLMMeasurement");

    m_description = QObject::tr("OphirLMMeasurement");

    m_detaildescription = QObject::tr("OphirLMMeasurement is an plugin to connect to Ophir USB speaking devices: \n\
\n\
* StarLite \n\
* StarBright \n\
* Juno \n\
* Nova-II \n\
* Pulsar models \n\
* USBI \n\
* Vega \n\
\n\
This Plugin creates a COM object to communicate with the device.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION); 

    m_initParamsOpt.append(ito::Param("serialNo", ito::ParamBase::String, "", tr("Serial number of the device to be loaded, if empty, the first device that can be opened will be opened.").toLatin1().data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
OphirPluginInterface::~OphirPluginInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPluginInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(OphirPlugin) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPluginInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(OphirPlugin) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
OphirPlugin::OphirPlugin() : AddInDataIO(),
m_opened(false),
m_handle(0)
{
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ThorlabsFF", tr("name of the plugin").toLatin1().data()));
    m_params.insert("deviceName", ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("description of the device").toLatin1().data()));
    m_params.insert("serialNumber", ito::Param("serialNumber", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("serial number of the device").toLatin1().data()));
    m_params.insert("romVersion", ito::Param("romVersion", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("rom version of the device").toLatin1().data()));

    m_params.insert("headName", ito::Param("headName", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("head name connected to the device").toLatin1().data()));
    m_params.insert("headType", ito::Param("headType", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("head type connected to the device").toLatin1().data()));
    m_params.insert("headSerialNumber", ito::Param("headSerialNumber", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("heyd serial number connected to the device").toLatin1().data()));
    
    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetOphir *dockWidget = new DockWidgetOphir(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
    }
    
    m_charBuffer = (char *)malloc(BUFFER_SIZE);   

}

//----------------------------------------------------------------------------------------------------------------------------------
OphirPlugin::~OphirPlugin()
{  
    CoUninitialize();
}


//---------------------------------------------------------------------------------------------------------------------------------- 
std::function<void()> OphirPlugin::plugAndPlayCallback()
{
    std::cout << "Device has been removed from the USB. \n" << std::endl;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
std::function<void(long handle, long channel)> OphirPlugin::dataReadyCallback()
{
    std::vector<double> values;
    std::vector<double> timestamps;
    std::vector<OphirLMMeasurement::Status> statuses;

    m_OphirLM.GetData(m_handle, 0, values, timestamps, statuses);
    for (size_t i = 0; i < values.size(); ++i)
        std::wcout << L"Timestamp: " << std::fixed << std::setprecision(3) << timestamps[i]
        << L" Reading: " << std::scientific << values[i] << L" Status: " << m_OphirLM.StatusString(statuses[i]) << L"\n";
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPlugin::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    
    QByteArray serialNoOptional = paramsOpt->at(0).getVal<char*>();
    std::vector<std::wstring> serialsFound;
    
    QAxObject m_axObject("OphirLMMeasurement.CoLMMeasurement");

    QString str = m_axObject.generateDocumentation();

    QVariant v = m_axObject.dynamicCall("CloseAll()");

    QVariantList params;
    QVariant v2;
    params << v2;
    m_axObject.dynamicCall("ScanUSB(QVariant&)", params);
    std::vector<std::wstring> data = params[0].value<std::vector<std::wstring>>();

    //QVarianList params;
    //QVariant res = object.dynamicCall("GetFlyerTemperature(double&, double&, int&)", params);

    params.clear();
    params << QVariant();
    m_axObject.dynamicCall("GetVersion(int&)", params);
    std::cout << "version: " <<  params[0].toInt() << std::endl;

    params.clear();
    params << QVariant();
    m_axObject.dynamicCall("GetDriverVersion(QString&)", params);
    std::cout << "driver serion: " << params[0].toString().constData() << std::endl;

    std::vector<std::wstring> serialNumbers;

    OphirLMMeasurement m_OphirLM;

    m_OphirLM.ScanUSB(serialsFound);

    if (serialsFound.size() > 0) // found connected devices
    {
        if (serialNoOptional.size() == 0) // optional serial not given, connect to first device
        {
            m_serialNo = serialsFound[0];
            openedDevices.append(m_serialNo);
        }
        else
        {
            bool found = false;
            for (int idx = 0; idx < serialsFound.size(); idx++)
            {
                
                QByteArray data = QByteArray::fromRawData(wCharToChar(serialsFound[idx].c_str()), sizeof(m_charBuffer));

                if (serialNoOptional.contains(m_charBuffer)) // option serial found
                {
                    m_serialNo = serialsFound[idx];
                    found = true;
                    break;
                }
            }
            
            if (!found)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("The given input serial %1 has not been found in serial number of connected devices").arg((serialNoOptional).data()).toLatin1().data());
            }
            
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, "no connected Ophir device detected");
    }

    if (!retValue.containsError()) // open device
    {
        try
        {
            m_OphirLM.OpenUSBDevice(m_serialNo, m_handle);
        }
        catch (int &e)
        {
            retValue += checkError(e, "Open usb device");
        }
        
        m_opened = true;

        long channel;
        bool exists;
        m_OphirLM.IsSensorExists(m_handle, channel, exists);

        if (!exists)
        {
            retValue += ito::RetVal(ito::retError, 0, "no sensor connected to the device");
        }
    }

    if (!retValue.containsError()) // get device infos
    {
        std::wstring deviceName, romVersion, serialNumber;
        std::wstring info, headSN, headType, headName, version;
        m_OphirLM.GetDeviceInfo(m_handle, deviceName, romVersion, serialNumber);      
        m_params["deviceName"].setVal<char>(*wCharToChar(deviceName.c_str()));
        m_params["romVersion"].setVal<char>(*wCharToChar(romVersion.c_str()));
        m_params["serialNumber"].setVal<char>(*wCharToChar(serialNumber.c_str()));

        m_OphirLM.GetSensorInfo(m_handle, 0, headSN, headType, headName);
        m_params["headSerialNumber"].setVal<char>(*wCharToChar(headSN.c_str()));
        m_params["headType"].setVal<char>(*wCharToChar(headType.c_str()));
        m_params["headName"].setVal<char>(*wCharToChar(headName.c_str()));
    }

    if(!retValue.containsError())
    {
        m_OphirLM.RegisterPlugAndPlay(plugAndPlayCallback());
        m_OphirLM.RegisterDataReady(dataReadyCallback());
        m_OphirLM.StartStream(m_handle, 0);
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
ito::RetVal OphirPlugin::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    if (m_opened)
    {
        m_OphirLM.StopAllStreams(); //stop measuring
        m_OphirLM.CloseAll(); //close device
        m_opened = false;
        openedDevices.removeOne(m_serialNo);
    }

    // Free multibyte character buffer
    if (m_charBuffer)
    {
        free(m_charBuffer);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPlugin::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
        
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPlugin::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
            
        }
        else if (key == "transitTime")
        {
            
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
void OphirPlugin::dockWidgetVisibilityChanged(bool visible)
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
ito::RetVal OphirPlugin::checkError(const int &e, const char* message)
{
    switch (e)
    {
    case 0x00000000:
        return ito::retOk;
        break;
    case 0x80070057:
        return ito::RetVal::format(ito::retError, e, "%s: Invalid argument %i.", message, e);
        break;
    case 0x80040201:
        return ito::RetVal::format(ito::retError, e, "%s: Device already opened %i.", message, e);
        break;
    case 0x80040300:
        return ito::RetVal::format(ito::retError, e, "%s: Device failed %i.", message, e);
        break;
    case 0x80040301:
        return ito::RetVal::format(ito::retError, e, "%s: Device firmware is incorrect %i.", message, e);
        break;
    case 0x80040302:
        return ito::RetVal::format(ito::retError, e, "%s: Sensor failed %i.", message, e);
        break;
    case 0x80040303:
        return ito::RetVal::format(ito::retError, e, "%s: Sensor firmware is incorrect %i.", message, e);
        break;
    case 0x80040306:
        return ito::RetVal::format(ito::retError, e, "%s: This sensor is not supported %i.", message, e);
        break;
    case 0x80040308:
        return ito::RetVal::format(ito::retError, e, "%s: The device is no longer available %i.", message, e);
        break;
    case 0x80040405:
        return ito::RetVal::format(ito::retError, e, "%s: command failed %i.", message, e);
        break;
    case 0x80040501:
        return ito::RetVal::format(ito::retError, e, "%s: A channel is in stream mode %i.", message, e);
        break;

    default:
        return ito::RetVal::format(ito::retError, e, "%s: unknown error %i.", message, e);
    }
}

//---------------------------------------------------------------------------------------------------------------------------------- 
char* OphirPlugin::wCharToChar(const wchar_t *input)
{
    size_t  i;
    // Conversion
    wcstombs_s(&i, m_charBuffer, (size_t)BUFFER_SIZE, input, (size_t)BUFFER_SIZE);
    return m_charBuffer;
}

