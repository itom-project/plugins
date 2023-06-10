/* ********************************************************************
    Plugin "CommonVisionBlox" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2015, Institut fuer Technische Optik, Universitaet Stuttgart

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
#include "commonVisionBloxInterface.h"

// standard-includes

// qt-core stuff
#include <qstring.h>
#include <QtCore/QStringList>
#include <QtCore/QtPlugin>
#include <QtCore/QMetaObject>


// project-includes
#include "pluginVersion.h"
#include "gitVersion.h"
#include "commonVisionBlox.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CVBInterface::CVBInterface(QObject *parent)
{
    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("CommonVisionBlox");

    m_description = QObject::tr("GenICam cameras via Common Vision Blox from Stemmer");


/*    char docstring[] = \
 "This plugin can connect to various cameras via the GenICam interface of the commercial tool Common Vision Blox from company Stemmer. \n\
 \n\
 Until now, the plugin is only implemented for monochrome pixel formats mono8, mono10, mono12, mono14 and mono16. Besides the ROI and \n\
 exposure time, all parameters need to be read and set using the parameter raw:suffix where suffix is the real GenICam parameter, obtained \n\
 via the Stemmer configuration tool. If a bitdepth > 8 bit is chosen, an error might occur during acquisition. Then check the indicated ini file \n\
 from Stemmer GenICam and don't set the pixelFormat property to auto but Mono16. \n\
 \n\
 In case of a slow connection, check the communication center of Stemmer for hints or bugs in the connection, e.g. use the filter driver for GigE connections. \n\
 \n\
 This plugin has been tested with DALSA Genie HM1400 and Xenics Bobcat 640 GigE.";
    m_detaildescription = tr(docstring);*/
    m_detaildescription = tr(
 "This plugin can connect to various cameras via the GenICam interface of the commercial tool Common Vision Blox from company Stemmer. \n\
 \n\
 Until now, the plugin is only implemented for monochrome pixel formats mono8, mono10, mono12, mono14 and mono16. Besides the ROI and \n\
 exposure time, all parameters need to be read and set using the parameter raw:suffix where suffix is the real GenICam parameter, obtained \n\
 via the Stemmer configuration tool. If a bitdepth > 8 bit is chosen, an error might occur during acquisition. Then check the indicated ini file \n\
 from Stemmer GenICam and don't set the pixelFormat property to auto but Mono16. \n\
 \n\
 In case of a slow connection, check the communication center of Stemmer for hints or bugs in the connection, e.g. use the filter driver for GigE connections. \n\
 \n\
 This plugin has been tested with DALSA Genie HM1400 and Xenics Bobcat 640 GigE.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = tr("Licensed under LGPL, Stemmer Common Vision Blox under its own license.");
    m_aboutThis = tr(GITVERSION);

    ito::Param param("scanForCameras", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If 1 scan for new cameras, else take the last opened camera (default). If you scan for new cameras, the configuration file (ini) created in CommonVisionBlox for GenICam or other cameras will be reset to the default values.").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 16, new ito::IntMeta(8,16,2), tr("desired monochromatic bitdepth. Dependent on this parameter PixelFormat is set to mono8, mono10, mono12, mono14 or mono16. Make sure the bitdepth is supported by your camera").toLatin1().data());
    m_initParamsOpt.append(param);

    /*param = ito::Param("color_mode", ito::ParamBase::String, "auto", tr("initial color model of camera ('gray', 'color' or 'auto' (default)). 'color' is only possible for color cameras").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "auto");
    sm->addItem("gray");
    sm->addItem("color");
    param.setMeta(sm,true);
    m_initParamsOpt.append(param);

    param = ito::Param("debug_mode", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If debug_mode is 1, message boxes from the uEye driver will appear in case of an error (default: off, 0)").toLatin1().data());
    m_initParamsOpt.append(param);*/
}

//----------------------------------------------------------------------------------------------------------------------------------
CVBInterface::~CVBInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal CVBInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(CommonVisionBlox)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal CVBInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(CommonVisionBlox)
   return ito::retOk;
}
