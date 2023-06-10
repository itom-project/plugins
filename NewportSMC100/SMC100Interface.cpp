/* ********************************************************************
    Plugin "Newport SMC100" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#include "SMC100Interface.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#include <QtCore/QtPlugin>

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of SMC100Interface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created SMC100Interface-instance is stored in *addInInst
    \return retOk
    \sa SMC100
*/
ito::RetVal SMC100Interface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(SMC100)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of SMC100Interface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa SMC100
*/
ito::RetVal SMC100Interface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(SMC100)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
SMC100Interface::SMC100Interface()
{
    m_type = ito::typeActuator;

    setObjectName("NewportSMC100");

    m_description = QObject::tr("Newport SMC100CC/PP");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"NewportSMC100 is an itom-plugin, which can be used to communicate with the Newport controllers SMC100CC and SMC100PP.\
\n\
It has been tested with two connected controllers SMC100CC. This system needs a serial port, which differs depending on the controller type. \
The parameters of the serial port (besides port number) are set automatically during initialization.";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr(
"NewportSMC100 is an itom-plugin, which can be used to communicate with the Newport controllers SMC100CC and SMC100PP.\
\n\
It has been tested with two connected controllers SMC100CC. This system needs a serial port, which differs depending on the controller type. \
The parameters of the serial port (besides port number) are set automatically during initialization.");

    m_author = "M. Hoppe, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("serial", ito::ParamBase::HWRef, NULL, tr("An opened serial port (the right communcation parameters will be set by this piezo-controller).").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    m_initParamsMand.append(ito::Param("numAxes", ito::ParamBase::Int, 1, 31, 1, tr("number of connected axes").toLatin1().data()));

    paramVal = ito::Param("keepSerialConfig", ito::ParamBase::Int, 0, 1, 0, tr("If 1 the current configuration of the given serial port is kept, else 0 [default].").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}
