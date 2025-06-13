/* ********************************************************************
    Plugin "ThorlabsElliptec" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2025, Institut für Technische Optik (ITO),
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

#include "dockWidgetThorlabsElliptec.h"
#include "motorAxisController.h"
#include <iostream>

//------------------------------------------------------------------------------
DockWidgetThorlabsElliptec::DockWidgetThorlabsElliptec(int uniqueID, ito::AddInActuator* actuator) :
    AbstractAddInDockWidget(actuator), m_pActuator(actuator), m_inEditing(false), m_firstRun(true)
{
    ui.setupUi(this);
    ui.lblIdentifier->setText(QString::number(uniqueID));
}

//------------------------------------------------------------------------------
void DockWidgetThorlabsElliptec::parametersChanged(QMap<QString, ito::Param> params)
{
    int axisType = params["axisType"].getVal<int>();

    switch (axisType)
    {
    case 0:
        ui.axisController->setAxisType(0, MotorAxisController::TypeRotational);
        ui.axisController->setAxisUnit(0, MotorAxisController::UnitDeg);
        break;
    case 1:
        ui.axisController->setAxisType(0, MotorAxisController::TypeLinear);
        ui.axisController->setAxisUnit(0, MotorAxisController::UnitMm);
        break;
    default:
        ui.axisController->setEnabled(false); // indexed not supported here
        break;
    }
}

//------------------------------------------------------------------------------
void DockWidgetThorlabsElliptec::identifierChanged(const QString& identifier)
{
    ui.lblIdentifier->setText(identifier);
}

//------------------------------------------------------------------------------
void DockWidgetThorlabsElliptec::enableWidget(bool enabled)
{
    ui.axisController->setEnabled(enabled);
}

//------------------------------------------------------------------------------
void DockWidgetThorlabsElliptec::dockWidgetVisibilityChanged(bool visible)
{
    if (visible)
    {
        // to connect the signals
        QPointer<ito::AddInActuator> actuator(m_pActuator);
        ui.axisController->setActuator(actuator);
        ui.axisController->setNumAxis(1);
        ui.axisController->setAxisType(0, MotorAxisController::TypeRotational);
    }
    else
    {
        ui.axisController->setActuator(QPointer<ito::AddInActuator>());
    }
}
