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

#include "OphirLMMeasurement.h"

#include <iostream>
#include <iomanip>

#include <Windows.h>

#include "OphirPlugin.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>

#include "dockWidgetOphir.h"

QList<QByteArray> OphirPlugin::openedDevices = QList<QByteArray>();

struct CoInitializer
{
    CoInitializer() { CoInitialize(nullptr); }
    ~CoInitializer() { CoUninitialize(); }
};


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
m_opened(false)
{
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ThorlabsFF", tr("name of the plugin").toLatin1().data()));
    m_params.insert("deviceName", ito::Param("deviceName", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("description of the device").toLatin1().data()));
    m_params.insert("serialNumber", ito::Param("serialNumber", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("serial number of the device").toLatin1().data()));
    
    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetOphir *dockWidget = new DockWidgetOphir(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
    }
    memset(m_serialNo, '\0', sizeof(m_serialNo));
}

//----------------------------------------------------------------------------------------------------------------------------------
OphirPlugin::~OphirPlugin()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPlugin::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    QByteArray serialNo = paramsOpt->at(0).getVal<char*>();

    CoInitializer initializer; // must call for COM initialization and deinitialization
    OphirLMMeasurement OphirLM;

    std::vector<std::wstring> serialConnected;

    OphirLM.ScanUSB(serialConnected);

    QByteArray serialToCompare;
    for (int idx = 0; idx < serialConnected.size(); idx++)
    {
        serialToCompare.append(reinterpret_cast<const char>(serialConnected[idx].c_str()));
    }

    
    if (serialConnected.size() > 0) // connected device found
    {

        if (serialNo.size() > 0) // serialnumber optional input 
        {
            if (serialNo.contains(serialToCompare)) // input serial found
            {

            }
        }
        else // connect to first found device
        {

        }
    }
    else // no connected device found
    {
        retValue += ito::RetVal(ito::retError, 0, "no Ophir device detected");
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
ito::RetVal OphirPlugin::checkError(short value, const char* message)
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
