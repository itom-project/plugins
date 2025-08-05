/* ********************************************************************
    Plugin "SmarActMCS2" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2025, TRUMPF Lasersystems for Semiconductor Manufacturing SE,´Germany

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

#include "dockWidgetSmarActMCS2.h"
#include "motorAxisController.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetSmarActMCS2::DockWidgetSmarActMCS2(ito::AddInActuator* actuator) :
    AbstractAddInDockWidget(actuator), m_inEditing(false), m_pActuator(actuator), 
    m_firstRun(true)
{
    ui.setupUi(this);

    enableWidgets(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSmarActMCS2::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.lblSerialNo->setText(params["serialNumber"].getVal<char*>());

    if (m_firstRun)
    {
        ui.axisController->setNumAxis(params["noOfChannels"].getVal<int>());
        for (int i = 0; i < params["noOfChannels"].getVal<int>(); i++)
        {
            switch (params["baseUnit"].getVal<int*>()[i])
            {
                case 1:
                ui.axisController->setAxisUnit(i, MotorAxisController::AxisUnit::UnitMm);
                    break;
                case 2:
                    ui.axisController->setAxisUnit(i, MotorAxisController::AxisUnit::UnitAU);
                    ui.axisController->setArbitraryUnit("deg");
                    break;
                default:
                    ui.axisController->setAxisUnit(i, MotorAxisController::AxisUnit::UnitAU);
                    break;
            }

            if (params["sensorPresent"].getVal<int*>()[i] == 0)
            {
                ui.axisController->axisEnabled(true);
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSmarActMCS2::enableWidgets(bool enabled)
{
    ui.axisController->setEnabled(enabled);
}


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSmarActMCS2::identifierChanged(const QString& identifier)
{
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSmarActMCS2::dockWidgetVisibilityChanged(bool visible)
{
    if (visible)
    {
        // to connect the signals
        QPointer<ito::AddInActuator> actuator(m_pActuator);
        ui.axisController->setActuator(actuator);
        ui.axisController->setNumAxis(0);
        ui.axisController->setDefaultRelativeStepSize(0.001);
        ui.axisController->setDefaultDecimals(3);
    }
    else
    {
        ui.axisController->setActuator(QPointer<ito::AddInActuator>());
    }
}