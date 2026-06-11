/* ********************************************************************
    Plugin "ThorlabsISM" for itom software
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

#include "DockWidgetThorlabsISM.h"

#include "common/addInInterface.h"

#include "motorAxisController.h"
#include <qlayout.h>
#include <qpointer.h>
#include "Thorlabs.MotionControl.IntegratedStepperMotors.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetThorlabsISM::DockWidgetThorlabsISM(ito::AddInActuator *actuator) : ito::AbstractAddInDockWidget(actuator),
    m_firstRun(true),
    m_pActuator(actuator)
{
    ui.setupUi(this);

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsISM::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.lblDevice->setText(params["deviceName"].getVal<char*>());
    ui.lblSerial->setText(params["serialNumber"].getVal<char*>());
    ui.motorAxisController->setNumAxis(params["numaxis"].getVal<int>());

    if (m_firstRun)
    {
        if (params["travelMode"].getVal<int>() == MOT_Rotational)
        {
            for (int i = 0; i < ui.motorAxisController->numAxis(); ++i)
            {
                ui.motorAxisController->setAxisType(i, MotorAxisController::TypeRotational);
                ui.motorAxisController->setAxisUnit(i, MotorAxisController::UnitDeg);
            }
        }
        else
        {
            for (int i = 0; i < ui.motorAxisController->numAxis(); ++i)
            {
                ui.motorAxisController->setAxisType(i, MotorAxisController::TypeLinear);
                ui.motorAxisController->setAxisUnit(i, MotorAxisController::UnitMm);
            }
        }

        m_firstRun = false;
    }

    ui.motorAxisController->setEnabled(params["enabled"].getVal<int>() > 0);

    if (params["homed"].getVal<int>() > 0)
    {
        ui.lblHomed->setText("The device is homed.");
    }
    else
    {
        ui.lblHomed->setText("The device is currently not homed.");
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsISM::enableWidget(bool enabled)
{
    ui.motorAxisController->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsISM::dockWidgetVisibilityChanged(bool visible)
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
