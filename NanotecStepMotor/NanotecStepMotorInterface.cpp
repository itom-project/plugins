/* ********************************************************************
    Plugin "Newport NanotecStepMotor" for itom software
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

#include "NanotecStepMotorInterface.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#include <QtCore/QtPlugin>

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of NanotecStepMotorInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created NanotecStepMotorInterface-instance is stored in *addInInst
    \return retOk
    \sa NanotecStepMotor
*/
ito::RetVal NanotecStepMotorInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(NanotecStepMotor)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of NanotecStepMotorInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa NanotecStepMotor
*/
ito::RetVal NanotecStepMotorInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(NanotecStepMotor)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
NanotecStepMotorInterface::NanotecStepMotorInterface()
{
    m_type = ito::typeActuator;

    setObjectName("NanotecStepMotor");

    m_description = QObject::tr("NanotecStepMotor");

    //for the docstring, please don't set any spaces at the beginning of the line.
    m_detaildescription = QObject::tr("NanotecStepMotor is an itom-plugin, which can be used to communicate with the Nanotec controllers SMCP.\nIt has been tested with one connected controller SMCP33 and 4 axis.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param param(ito::Param("serial", ito::ParamBase::HWRef, NULL, tr("An initialized SerialIO").toLatin1().data()));
    param.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(param);

    param = ito::Param("axisID", ito::ParamBase::IntArray, NULL, tr("internal ID of axis (default 1, 2, 3, ...), range: 1..254").toLatin1().data());
    ito::IntArrayMeta iam(1, 254, 1, 1, 255, 1);
    param.setMeta(&iam, false);
    m_initParamsMand.append(param);

    param = ito::Param("axisSteps", ito::ParamBase::DoubleArray, NULL, tr("number of full steps per unit (deg or mm) of axis").toLatin1().data());
    ito::DoubleArrayMeta ias(0.0, std::numeric_limits<double>::max(), 0.0, 1, 255, 1);
    param.setMeta(&ias, false);
    m_initParamsMand.append(param);

    param = ito::Param("units", ito::ParamBase::IntArray, NULL, tr("unit of axis, 0: degree [default], 1: mm").toLatin1().data());
    ito::IntArrayMeta iau(0, 1, 1, 1, 255, 1);
    param.setMeta(&iau, false);
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
// this macro registers the class NanotecStepMotorInterface with the name NanotecStepMotorInterface as plugin for the Qt-System (see Qt-DOC)
