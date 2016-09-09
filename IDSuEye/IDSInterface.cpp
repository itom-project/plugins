/* ********************************************************************
    Plugin "IDSuEye" for itom software
    URL: http://www.bitbucket.org/itom/plugins
    Copyright (C) 2014, Pulsar Photonics GmbH, Aachen
    Copyright (C) 2016, Institut fuer Technische Optik, Universitaet Stuttgart

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

#define _SCL_SECURE_NO_WARNINGS (1)
#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

// main-include
#include "IDSInterface.h"

// standard-includes

// qt-core stuff
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QtPlugin>
#include <QtCore/QMetaObject>

// ids-includes
#if linux
    #include "ueye.h"
#else
    #include "uEye.h"
    #include "version.h"
#endif

// project-includes
#include "pluginVersion.h"
#include "IDSuEye.h"

namespace
{
    /**
     * @brief   Extracts the given number of bits from the given position
     * @tparam  bitPosition The start position of the bit sequence.
     * @tparam  bits        The number of bits.   
     * @param   x           The value from which bits shall be extracted.
     * @returns The extracted bits shifted to lowest position, all other stuff is erased.
     **/
    template < size_t bitPosition, size_t bits >
    unsigned int get( unsigned int x )
    {
#if linux
#else
        static_assert( (bitPosition+bits) <= (sizeof(x)*8), "Accessing invalid bits!" );
#endif

        return ( x >> bitPosition ) & ( (0x1<<bits) - 1 );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
IDSInterface::IDSInterface(QObject *parent)
{
    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("IDSuEye");

    m_description = QObject::tr("IDS uEye grabber.");

    const unsigned int libVersion = UEYE_VERSION_CODE;

    int major = get<24,8>(libVersion);
    int minor = get<16,8>(libVersion);
    

/*    char docstring[] = \
 "This plugin supports IDS uEye cameras and has currently been tested with the following models: \n\
- UI145xSE-C (colored, USB2) \n\
- UI124xSE-M (monochrome, USB2). \n\
- UI224xSE-M (monochrome, USB2). \n\
- UI337xCP-C (colored, USB3) \n\
\n\
The plugin has been compiled using the IDS library version %1.%2. \n\
\n\
In order to run your camera, please install the SDK imaging software in the right version such that the necessary drivers are installed. \n\
\n\
The first draft of this plugin has been implemented by Pulsar Photonics GmbH; further work has been done by ITO, University of Stuttgart."; 
    m_detaildescription = tr(docstring).arg(major).arg(minor);*/
    m_detaildescription = tr(
"This plugin supports IDS uEye cameras and has currently been tested with the following models: \n\
* UI145xSE-C (colored, USB2) \n\
* UI124xSE-M (monochrome, USB2) \n\
* UI224xSE-M (monochrome, USB2) \n\
* UI337xCP-C (colored, USB3) \n\
* UI318xCP-M (monochrome, USB3) \n\
* UI318xCP-C (colored, USB3) \n\
\n\
The plugin has been compiled using the IDS library version %1.%2. You can run it with an installed driver version %3.%4x.xx. \n\
\n\
In order to run your camera, please install the SDK imaging software in the right version such that the necessary drivers are installed. \n\
\n\
The first draft of this plugin has been implemented by Pulsar Photonics GmbH; further work has been done by ITO, University of Stuttgart.").arg(major).arg(minor).arg(major).arg(minor / 10);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = tr("Licensed under LGPL");
    m_aboutThis = tr( "N.A." );  

    ito::Param param( "camera_id", ito::ParamBase::Int | ito::ParamBase::In, 0, 254, 0, tr("Camera ID (user-definable in IDS camera manager) of the camera to open (0: the next free camera will opened [default], 1-254: specific camera ID)").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("color_mode", ito::ParamBase::String, "auto", tr("initial color model of camera ('gray', 'color' or 'auto' (default)). 'color' is only possible for color cameras").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "auto");
    sm->addItem("gray");
    sm->addItem("color");
    param.setMeta(sm,true);
    m_initParamsOpt.append(param);

    param = ito::Param( "debug_mode", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If debug_mode is 1, message boxes from the uEye driver will appear in case of an error (default: off, 0)").toLatin1().data());
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
IDSInterface::~IDSInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IDSInterface::getAddInInst( ito::AddInBase **addInInst )
{
    if ( !addInInst )
    {
        return ito::retError;
    }

    ito::RetVal retval = checkVersionConsistency();
    if (!retval.containsError())
    {
        NEW_PLUGININSTANCE(IDSuEye)
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IDSInterface::closeThisInst( ito::AddInBase **addInInst )
{
   REMOVE_PLUGININSTANCE(IDSuEye)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IDSInterface::checkVersionConsistency()
{
    //check consistency of library version vs. dll-version:
    const unsigned int dllVersion = is_GetDLLVersion();
    const unsigned int libVersion = UEYE_VERSION_CODE;

    //the IDS support said that the binary compatibility between different driver versions is
    //given if the major and the 'ten' digit of the minor is the same. The build can be ignored.
    if ((get<24,8>(dllVersion) != get<24,8>(libVersion)) || (static_cast<int>(get<16,8>(dllVersion) / 10) != static_cast<int>(get<16,8>(libVersion) / 10)))
    {
        return ito::RetVal::format(ito::retError, 0, "IDS library version mismatch. Expected version %i.%i.xx, got %i.%i.xx", \
            get<24,8>(libVersion), get<16,8>(libVersion), \
            get<24,8>(dllVersion), get<16,8>(dllVersion));
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
// this macro registers the class IDSInterface with the name IDSuEye as plugin for the Qt-System (see Qt-DOC)
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(IDSuEye, IDSInterface)
#endif
