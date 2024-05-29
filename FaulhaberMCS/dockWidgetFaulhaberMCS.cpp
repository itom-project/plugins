/* ********************************************************************
    Plugin "FaulhaberMCS" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
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

#include "dockWidgetFaulhaberMCS.h"
#include "motorAxisController.h"
#include <iostream>
//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetFaulhaberMCS::DockWidgetFaulhaberMCS(int uniqueID, ito::AddInActuator* actuator) :
    AbstractAddInDockWidget(actuator), m_pActuator(actuator), m_inEditing(false), m_firstRun(true)
{
    ui.setupUi(this);
    ui.lblIdentifier->setText(QString::number(uniqueID));
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetFaulhaberMCS::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.radioButtonReadyToSwitchOn->setChecked(params["readyToSwitchOn"].getVal<int>());
    ui.radioButtonSwitchOn->setChecked(params["switchedOn"].getVal<int>());
    ui.radioButtonOperationEnabled->setChecked(params["operationEnabled"].getVal<int>());
    ui.radioButtonFault->setChecked(params["fault"].getVal<int>());
    ui.radioButtonVoltageEnabled->setChecked(params["voltageEnabled"].getVal<int>());
    ui.radioButtonQuickStop->setChecked(params["quickStop"].getVal<int>());
    ui.radioButtonSwitchOnDisabled->setChecked(params["switchOnDisabled"].getVal<int>());
    ui.radioButtonWarning->setChecked(params["warning"].getVal<int>());
    ui.radioButtonTargetReached->setChecked(params["targetReached"].getVal<int>());
    ui.radioButtonInternalLimitActive->setChecked(params["internalLimitActive"].getVal<int>());
    ui.radioButtonSetPointAcknowledged->setChecked(params["setPointAcknowledged"].getVal<int>());
    ui.radioButtonFollowingError->setCheckable(params["followingError"].getVal<int>());

    ui.lblSerialNo->setText(params["serialNumber"].getVal<char*>());
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetFaulhaberMCS::identifierChanged(const QString& identifier)
{
    ui.lblIdentifier->setText(identifier);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetFaulhaberMCS::enableWidget(bool enabled)
{
    ui.axisController->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetFaulhaberMCS::dockWidgetVisibilityChanged(bool visible)
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
