/* ********************************************************************
    Plugin "PI_GCS2" for itom software
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

#include "PIPiezoCtrlInterface.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#include <QtCore/QtPlugin>

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of PIPiezoCtrlInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created PIPiezoCtrlInterface-instance is stored in *addInInst
    \return retOk
    \sa PIPiezoCtrl
*/
ito::RetVal PIPiezoCtrlInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(PIPiezoCtrl)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of PIPiezoCtrlInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa PIPiezoCtrl
*/
ito::RetVal PIPiezoCtrlInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(PIPiezoCtrl)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
PIPiezoCtrlInterface::PIPiezoCtrlInterface()
{
    m_type = ito::typeActuator;

    setObjectName("PIPiezoCtrl");

    m_description = QObject::tr("PI Piezos E-662, E-816, E-621, E-625, E-665, C-663");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"The PIPiezoCtrl is an itom-plugin, which can be used to communicate with PI piezo-controllers.\
Different PI-Piezo Controller (E-816, E-621, E-625, E-665 or E662) are implemented.\n\
\n\
It has been tested with different Piefocs and Piezo-stages. This system needs a serial port, which differs depending on the controller type. \
The parameters of the serial port (besides port number) are set automatically during initialization. \n\
\n\
WARNING: The calibration between applied voltage and desired position is depending on every single PI device and is stored in the corresponding \
PI controller. Therefore don't mix stages and controllers but only use the original, calibrated combination.";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr("The PIPiezoCtrl is an itom-plugin, which can be used to communicate with PI piezo-controllers.\
Different PI-Piezo Controller (E-816, E-621, E-625, E-665, E-662 or C-663) are implemented.\n\
\n\
It has been tested with different Piefocs and Piezo-stages. This system needs a serial port, which differs depending on the controller type. \
The parameters of the serial port (besides port number) are set automatically during initialization. \n\
\n\
WARNING: The calibration between applied voltage and desired position is depending on every single PI device and is stored in the corresponding \
PI controller. Therefore don't mix stages and controllers but only use the original, calibrated combination. \n\
\n\
WARNING: The maximum position for PI controller typ E815/E625 is set to 100 micrometer. It is not possible to ask the hardware for the maximum avaiable position.\
In the case of a higher maximum position, set the plugin parameter \"posLimitHigh\". ");

    m_author = "W. Lyda, M. Gronle, J. Krauter, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("serial", ito::ParamBase::HWRef, NULL, tr("An opened serial port (the right communcation parameters will be set by this piezo-controller).").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("keepSerialConfig", ito::ParamBase::Int, 0, 1, 0, tr("If 1 the current configuration of the given serial port is kept, else 0 [default].").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class PIPiezoCtrlInterface with the name PIPiezoCtrlInterface as plugin for the Qt-System (see Qt-DOC)
