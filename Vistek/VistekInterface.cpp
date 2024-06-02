/* ********************************************************************
    Plugin "Vistek" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

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

#include "Vistek.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declarations of PI constant
#include "math.h"
#include "pluginVersion.h"
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

#include <qdockwidget.h>
#include <qpushbutton.h>
#include <qmetaobject.h>
#include "dockWidgetVistek.h"
#include "gitVersion.h"

//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class VistekInterface
    \brief Small interface class for class Vistek. This class contains basic information about Vistek and can
        create one or more new instances of Vistek.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! Creates a new instance of Vistek and returns the instance-pointer.
/*!
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created Vistek-instance is stored in *addInInst
    \return ito::RetVal retOk
    \sa Vistek
*/
ito::RetVal VistekInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(Vistek)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Deletes an instance of Vistek. The instance is given by parameter addInInst.
/*!
    \param [in] double pointer to the instance which should be deleted.
    \return ito::RetVal retOk
    \sa Vistek
*/
ito::RetVal VistekInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(Vistek)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor for VistekInterface
/*!
    Defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real plugin (here: Vistek) should or must
    be initialized (e.g. by a Python call) with mandatory or optional parameters, please initialize both vectors m_initParamsMand
    and m_initParamsOpt within this constructor.

    \param [in] parent is the plugin interface's parent object
*/
VistekInterface::VistekInterface(QObject *parent)
{
    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("Vistek");

    m_description = QObject::tr("SVS Vistek GigE grabber.");

    m_detaildescription = QObject::tr("itom plugin for GigE cameras from SVS Vistek. Every camera is simply initialized by the serial number of the connected SVS Vistek camera. \
(see camera housing). \n\
\n\
Some files of the SVGigE SDK are shipped within this plugin (currently 1.5.2). Please check the SVS Vistek website for newer versions of the SDK \
and replace the files if desired. Additionally, it is stated that SVS Vistek does not provide any support for this specific plugin wrapping the \
official SDK of SVS Vistek. \n\
\n\
This plugin requires the necessary libraries from the SVS Vistek SDK (SVGigE.dll, SVGigETLFilter.dll, SVGigETLWinsock.dll or 64bit versions). Please check the right version \
and make these libraries available for itom (PATH environment variable, system directory...). \n\
\n\
For a robust data communication please install the SVGigE FilterDriver and enable Jumbo frames at your network adapter. \n\
\n\
Please notice: Currently, this plugin only works for Vistek drivers up to version 1.5.2. If you want to use a 2.x series of the Vistek drivers, \
use the GenICam plugin of itom that is able to communicate with Vistek USB3 and GigE cameras.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsMand.clear();
    m_initParamsOpt.clear();

    ito::Param p = ito::Param("CameraSerialNo", ito::ParamBase::String | ito::ParamBase::In, "", tr("Serial Number of the SVS Vistek camera (see camera housing)").toLatin1().data());
    ito::StringMeta *m = new ito::StringMeta(ito::StringMeta::RegExp, "^[0-9]*$");
    p.setMeta(m, true);
    m_initParamsOpt << p;

    p = ito::Param("streamingPacketSize", ito::ParamBase::Int | ito::ParamBase::In, -1, 16000, -1, tr("used streaming packet size (-1: use maximal available packet size, else value in bytes). Try to enable jumbo-frames at your network adapter in order to realize higher packet sizes").toLatin1().data());
    m_initParamsOpt << p;

    p = ito::Param("streamingBuffers", ito::ParamBase::Int | ito::ParamBase::In, 1, 300, 3, tr("number of streaming buffers. Increase if you get data losses, decrease if you want to consume less memory").toLatin1().data());
    m_initParamsOpt << p;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor for VistekInterface
/*!
    Clears m_initParamsMand and m_initParamsOpt.
*/
VistekInterface::~VistekInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
// this macro registers the class VistekInterface with the name Vistekinterface as plugin for the Qt-System (see Qt-DOC)
