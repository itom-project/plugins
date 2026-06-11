/* ********************************************************************
    Plugin "ThorlabsBP" for itom software
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

#include "DockWidgetThorlabsBP.h"

#include "common/addInInterface.h"

#include "motorAxisController.h"
#include <qlayout.h>
#include <qpointer.h>
#include "Thorlabs.MotionControl.Benchtop.Piezo.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetThorlabsBP::DockWidgetThorlabsBP(ito::AddInActuator *actuator) : ito::AbstractAddInDockWidget(actuator),
    m_firstRun(true),
    m_pActuator(actuator)
{
    ui.setupUi(this);
    ui.motorAxisController->setDefaultAxisType(MotorAxisController::TypeLinear);
    ui.motorAxisController->setArbitraryUnit("V");

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsBP::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.lblDevice->setText(params["deviceName"].getVal<char*>());
    ui.lblSerial->setText(params["serialNumber"].getVal<char*>());
    ui.motorAxisController->setNumAxis(params["numaxis"].getVal<int>());

    if (m_firstRun)
    {
        m_firstRun = false;
    }

    const int *enabled = params["enabled"].getVal<int*>();
    const int *controlMode = params["controlMode"].getVal<int*>();
    const int *hasFeedback = params["hasFeedback"].getVal<int*>();
    const int *zeroed = params["zeroed"].getVal<int*>();
    const int *channel = params["channel"].getVal<int*>();
    const ito::float64* maximumTravelRange = params["maximumTravelRange"].getVal<ito::float64*>();
    const int* maximumVoltage = params["maximumVoltage"].getVal<int*>();

    for (int i = 0; i < ui.motorAxisController->numAxis(); ++i)
    {
        ui.motorAxisController->setAxisEnabled(i, enabled[i] > 0);

        if (zeroed[i])
        {
            ui.motorAxisController->setAxisName(i, QString("Ch %1, zeroed").arg(channel[i]));
        }
        else
        {
            ui.motorAxisController->setAxisName(i, QString("Ch %1, not zeroed").arg(channel[i]));
        }

        if (controlMode[i] == 1 && hasFeedback[i] > 0)
        {
            ui.motorAxisController->setAxisUnit(i, MotorAxisController::UnitMum);
            ui.motorAxisController->setTargetInterval(i, ito::AutoInterval(0.0, maximumTravelRange[i]));
        }
        else
        {
            ui.motorAxisController->setAxisUnit(i, MotorAxisController::UnitAU);
            ui.motorAxisController->setTargetInterval(i, ito::AutoInterval(-maximumVoltage[i], maximumVoltage[i]));
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsBP::enableWidget(bool enabled)
{
    ui.motorAxisController->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsBP::dockWidgetVisibilityChanged(bool visible)
{
    if (visible)
    {
        //to connect the signals
        QPointer<ito::AddInActuator> actuator(m_pActuator);
        ui.motorAxisController->setActuator(actuator);
    }
    else
    {
        ui.motorAxisController->setActuator(QPointer<ito::AddInActuator>());
    }
}
