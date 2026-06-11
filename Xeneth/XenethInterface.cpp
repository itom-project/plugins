/* ********************************************************************
    Plugin "Xeneth" for itom software
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
#include "XenethInterface.h"

// standard-includes

// qt-core stuff
#include <qstring.h>
#include <QtCore/QStringList>
#include <QtCore/QtPlugin>
#include <QtCore/QMetaObject>


// project-includes
#include "pluginVersion.h"
#include "gitVersion.h"
#include "Xeneth.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
XenethInterface::XenethInterface(QObject *parent)
{
    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("Xeneth");

    m_description = QObject::tr("Xeneth camera family Bobcat");


    char docstring[] = \
 "...";
    m_detaildescription = tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param param( "device", ito::ParamBase::String | ito::ParamBase::In, "soft://0", tr("camera device name to be loaded.").toLatin1().data());
    m_initParamsOpt.append(param);

    /*param = ito::Param("color_mode", ito::ParamBase::String, "auto", tr("initial color model of camera ('gray', 'color' or 'auto' (default)). 'color' is only possible for color cameras").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "auto");
    sm->addItem("gray");
    sm->addItem("color");
    param.setMeta(sm,true);
    m_initParamsOpt.append(param);

    param = ito::Param( "debug_mode", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If debug_mode is 1, message boxes from the uEye driver will appear in case of an error (default: off, 0)").toLatin1().data());
    m_initParamsOpt.append(param);*/
}

//----------------------------------------------------------------------------------------------------------------------------------
XenethInterface::~XenethInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal XenethInterface::getAddInInst( ito::AddInBase **addInInst )
{
    NEW_PLUGININSTANCE(Xeneth)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal XenethInterface::closeThisInst( ito::AddInBase **addInInst )
{
   REMOVE_PLUGININSTANCE(Xeneth)
   return ito::retOk;
}
