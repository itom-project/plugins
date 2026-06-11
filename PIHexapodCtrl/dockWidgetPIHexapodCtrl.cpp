/* ********************************************************************
Plugin "DummyMotor" for itom software
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

#include "dockWidgetPIHexapodCtrl.h"

#include "motorAxisController.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail The constructor by the constructor of the DummyMotor during initialisation of the DummyMotor-Instance.
*
*\param[in] params        m_params-Variable containing the parameters of the DummyMotor
*\param[in] uniqueID    The unique Id of the DummyMotor-Instance connected to this dialog
*
*\sa DummyMotor
*/
DockWidgetPIHexapodCtrl::DockWidgetPIHexapodCtrl(int uniqueID, ito::AddInActuator * myPlugin) :
    ito::AbstractAddInDockWidget(myPlugin),
    m_pActuator(myPlugin),
    m_isVisible(false),
    m_numaxis(-1)
{
    ui.setupUi(this);

    ui.lblID->setText(QString::number(uniqueID));

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This Slot checks all parameters, checks the axis-numbers and enables the corresponding GUI-elements
*
*\param[in] params        m_params-Variable containing the parameters of the DummyMotor
*
*/
void DockWidgetPIHexapodCtrl::parametersChanged(QMap<QString, ito::Param> params)
{
    bool newNumaxis = (m_numaxis != params["numaxis"].getVal<int>());
    if (newNumaxis)
    {
        m_numaxis = params["numaxis"].getVal<int>();
        QStringList axesNames = QString(QLatin1String(params["axesNames"].getVal<const char*>())).split(";");
        ui.lblAxis->setText(QString::number(m_numaxis));
        ui.axisController->setNumAxis(m_numaxis);

        for (int i = 0; i < m_numaxis; i++)
        {
            ui.axisController->setAxisEnabled(i, true);
            ui.axisController->setAxisName(i, axesNames[i]);
            ui.axisController->setDefaultRelativeStepSize(1);

            if (axesNames[i] == "U" || axesNames[i] == "V" || axesNames[i] == "W")
            {
                ui.axisController->setAxisType(i, MotorAxisController::TypeRotational);
            }
            else
            {
                ui.axisController->setAxisType(i, MotorAxisController::TypeLinear);
            }
        }
    }

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIHexapodCtrl::enableWidget(bool enabled)
{
    ui.axisController->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIHexapodCtrl::dockWidgetVisibilityChanged(bool visible)
{
    if (visible)
    {
        //to connect the signals
        QPointer<ito::AddInActuator> actuator(m_pActuator);
        ui.axisController->setActuator(actuator);
    }
    else
    {
        ui.axisController->setActuator(QPointer<ito::AddInActuator>());
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIHexapodCtrl::actuatorStatusChanged(QVector<int> status, QVector<double> positions)
{
    bool running = false;

    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] & ito::actuatorMoving)
        {
            running = true;
        }
    }
}
