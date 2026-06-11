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

/**\file dockWidgetDummyMotor.cpp
* \brief In this file the functions of the non modal dialog for the DummyMotor are specified
*
*    This file defines the functions of the DockWidgetDummyMotor-Class defined in the file "dockWidgetDummyMotor.h"
*
*\sa dockWidgetDummyMotor, DummyMotor
*\author Wolfram Lyda
*\date    Oct2011
*/

#include "dockWidgetDummyMotor.h"

#include "motorAxisController.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail The constructor by the constructor of the DummyMotor during initialisation of the DummyMotor-Instance.
*
*\param[in] params        m_params-Variable containing the parameters of the DummyMotor
*\param[in] uniqueID    The unique Id of the DummyMotor-Instance connected to this dialog
*
*\sa DummyMotor
*/
DockWidgetDummyMotor::DockWidgetDummyMotor(int uniqueID, ito::AddInActuator * myPlugin) :
    ito::AbstractAddInDockWidget(myPlugin),
    m_pActuator(myPlugin),
    m_isVisible(false),
    m_numaxis(-1)
{
    ui.setupUi(this);

    ui.lblID->setText(QString::number(uniqueID));

    m_checkEnabled.append(ui.checkBox_enablex);
    m_checkEnabled.append(ui.checkBox_enabley);
    m_checkEnabled.append(ui.checkBox_enablez);
    m_checkEnabled.append(ui.checkBox_enablea);
    m_checkEnabled.append(ui.checkBox_enableb);
    m_checkEnabled.append(ui.checkBox_enablec);
    foreach(QCheckBox* btn, m_checkEnabled)
    {
        connect(btn, SIGNAL(clicked(bool)), this, SLOT(checkEnabledClicked(bool)));
    }

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This Slot checks all parameters, checks the axis-numbers and enables the corresponding GUI-elements
*
*\param[in] params        m_params-Variable containing the parameters of the DummyMotor
*
*/
void DockWidgetDummyMotor::parametersChanged(QMap<QString, ito::Param> params)
{
    bool newNumaxis = m_numaxis != params["numaxis"].getVal<int>();
    if (newNumaxis)
    {
        m_numaxis = params["numaxis"].getVal<int>();
        ui.lblAxis->setText(QString::number(m_numaxis));
        ui.axisController->setNumAxis(m_numaxis);

        for (int i = 0; i < m_checkEnabled.size(); i++)
        {
            m_checkEnabled[i]->setVisible(m_numaxis > i);
            if (m_numaxis > i)
            {
                ui.axisController->setAxisEnabled(i, m_checkEnabled[i]->isChecked());

                switch (i)
                {
                case 0:
                    ui.axisController->setAxisName(i, "x");
                    ui.axisController->setAxisType(i, MotorAxisController::TypeLinear);
                    break;
                case 1:
                    ui.axisController->setAxisName(i, "y");
                    ui.axisController->setAxisType(i, MotorAxisController::TypeLinear);
                    break;
                case 2:
                    ui.axisController->setAxisName(i, "z");
                    ui.axisController->setAxisType(i, MotorAxisController::TypeLinear);
                    break;
                case 3:
                    ui.axisController->setAxisName(i, "a");
                    ui.axisController->setAxisType(i, MotorAxisController::TypeRotational);
                    break;
                case 4:
                    ui.axisController->setAxisName(i, "b");
                    ui.axisController->setAxisType(i, MotorAxisController::TypeRotational);
                    break;
                case 5:
                    ui.axisController->setAxisName(i, "c");
                    ui.axisController->setAxisType(i, MotorAxisController::TypeRotational);
                    break;
                }
            }
        }
    }

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::checkEnabledClicked(bool checked) //slot if any "enabled"-checkbox is clicked
{
    if (qobject_cast<QCheckBox*>(sender()))
    {
        int idx = m_checkEnabled.indexOf(qobject_cast<QCheckBox*>(sender()));

        if (idx >= 0)
        {
            ui.axisController->setAxisEnabled(idx, checked);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::enableWidget(bool enabled)
{
    for (int i = 0; i < m_checkEnabled.size(); i++)
    {
        m_checkEnabled[i]->setEnabled(enabled && m_numaxis > i);
    }

    ui.axisController->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::dockWidgetVisibilityChanged(bool visible)
{
    if (visible)
    {
        //to connect the signals
        QPointer<ito::AddInActuator> actuator(m_pActuator);
        ui.axisController->setActuator(actuator);
        // setting higher resolution, so we can move nanometers if we want
        ui.axisController->setDefaultDecimals(6);
    }
    else
    {
        ui.axisController->setActuator(QPointer<ito::AddInActuator>());
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::actuatorStatusChanged(QVector<int> status, QVector<double> positions)
{
    bool running = false;

    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] & ito::actuatorMoving)
        {
            running = true;
        }
    }

    ui.groupProperties->setEnabled(!running);
}
