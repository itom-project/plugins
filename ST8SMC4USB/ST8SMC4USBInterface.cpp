/* ********************************************************************
    Plugin "Newport ST8SMC4USB" for itom software
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

#include "ST8SMC4USBInterface.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#include <QtCore/QtPlugin>

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of ST8SMC4USBInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created ST8SMC4USBInterface-instance is stored in *addInInst
    \return retOk
    \sa ST8SMC4USB
*/
ito::RetVal ST8SMC4USBInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(ST8SMC4USB)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of ST8SMC4USBInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa ST8SMC4USB
*/
ito::RetVal ST8SMC4USBInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(ST8SMC4USB)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
ST8SMC4USBInterface::ST8SMC4USBInterface()
{
    m_type = ito::typeActuator;

    setObjectName("Standa8SMC4USB");

    m_description = QObject::tr("Standa8SMC4USB");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"Standa8SMC4USB is an itom-plugin, which can be used to communicate with the STANDA controllers 8SMC4-USB.\
\n\
It has been tested with one connected controller 8SMC4-USB-B8-1 and one axis.";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr("Standa8SMC4USB is an itom-plugin, which can be used to communicate with the STANDA controllers 8SMC4-USB.\
\n\
It has been tested with one connected controller 8SMC4-USB-B8-1 and one axis.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsMand.append(ito::Param("unitsPerStep", ito::ParamBase::Double, 0.01, new ito::DoubleMeta(0.0,100000.0), tr("units (deg or mm) per step of axis, e.g. full step resolution of data sheet of actuator").toLatin1().data()));
    m_initParamsMand.append(ito::Param("unit", ito::ParamBase::Int, 0, new ito::IntMeta(0,1), tr("unit of axis, 0: degree (default), 1: mm").toLatin1().data()));

    m_initParamsOpt.append(ito::Param("deviceNum", ito::ParamBase::Int, 0, 10, 0, tr("The current number of this specific device, if there are more than one devices connected. (0 = first device)").toLatin1().data()));
    m_initParamsOpt.append(ito::Param("microSteps", ito::ParamBase::Int, 1, 256, 1, tr("micro steps for motor [1,2,4,8,16,32,64,128,256]").toLatin1().data()));
}
