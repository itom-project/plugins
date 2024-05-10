/* ********************************************************************
    Plugin "AndorSDK3" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Institut für Technische Optik, Universität Stuttgart

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

// main-include
#include "AndorSDK3Interface.h"

// standard-includes

// qt-core stuff
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QtPlugin>
#include <QtCore/QMetaObject>


// project-includes
#include "pluginVersion.h"
#include "gitVersion.h"
#include "AndorSDK3.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AndorSDK3Interface::AndorSDK3Interface(QObject *parent)
{
    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("AndorSDK3");

    m_description = QObject::tr("Andor cameras via its SDK3 (Neo, Zyla).");


/*    char docstring[] = \
"This plugin supports Andor cameras that can be run using the SDK3 from Andor (e.g. Neo and Zyla series). It has been tested with the following models: \n\
\n\
- Zyla 5.5 (Dual Camera Link) \n\
\n\
The plugin has been compiled using the Andor SDK 3.8 \n\
\n\
In order to run your camera, please purchase and install the Andor SDK 3.8 or higher and make sure that the necessary libraries are accessible \n\
by the Windows path environment variable (e.g. append \"C:/Program Files/Andor SDK3\" to the path variable).";
    m_detaildescription = tr(docstring);*/
    m_detaildescription = tr(
"This plugin supports Andor cameras that can be run using the SDK3 from Andor (e.g. Neo and Zyla series). It has been tested with the following models: \n\
\n\
- Zyla 5.5 (Dual Camera Link) \n\
\n\
The plugin has been compiled using the Andor SDK 3.8 \n\
\n\
In order to run your camera, please purchase and install the Andor SDK 3.8 or higher and make sure that the necessary libraries are accessible \n\
by the Windows path environment variable (e.g. append \"C:/Program Files/Andor SDK3\" to the path variable).");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param param( "camera_idx", ito::ParamBase::Int | ito::ParamBase::In, 0, 31, 0, tr("camera index that should be opened. The first camera is 0, the second 1...").toLatin1().data());
    m_initParamsMand.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
AndorSDK3Interface::~AndorSDK3Interface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AndorSDK3Interface::getAddInInst( ito::AddInBase **addInInst )
{
    NEW_PLUGININSTANCE(AndorSDK3)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AndorSDK3Interface::closeThisInst( ito::AddInBase **addInInst )
{
   REMOVE_PLUGININSTANCE(AndorSDK3) //-> crashed here???
   /*if (*addInInst)
   {
      delete ((AndorSDK3 *)*addInInst);
      m_InstList.removeOne(*addInInst);
   } */
   return ito::retOk;
}
