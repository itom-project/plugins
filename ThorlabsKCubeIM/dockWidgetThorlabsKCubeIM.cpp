/* ********************************************************************
    Plugin "ThorlabsISM" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2020, Institut fuer Technische Optik (ITO),
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

#include "DockWidgetThorlabsKCubeIM.h"

#include "common/addInInterface.h"

#include "motorAxisController.h"
#include <qlayout.h>
#include <qpointer.h>
#include "Thorlabs.MotionControl.KCube.InertialMotor.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetThorlabsKCubeIM::DockWidgetThorlabsKCubeIM(ito::AddInActuator *actuator) : ito::AbstractAddInDockWidget(actuator),
    m_firstRun(true),
    m_pActuator(actuator),
    m_numaxis(0)
{
    ui.setupUi(this); 
    
    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsKCubeIM::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.lblDevice->setText(params["deviceName"].getVal<char*>());
    ui.lblSerial->setText(params["serialNumber"].getVal<char*>());
    ui.motorAxisController->setNumAxis(params["numaxis"].getVal<int>());
    m_numaxis = params["numaxis"].getVal<int>();

    if (m_firstRun)
    {
        for (int i = 0; i < ui.motorAxisController->numAxis(); ++i)
        {
            ui.motorAxisController->setAxisType(i, MotorAxisController::TypeLinear);
            ui.motorAxisController->setAxisUnit(i, MotorAxisController::UnitAU);
            requestActuatorStatusAndPositions(true, true);
        }

        m_firstRun = false;
    }
    enableWidget(true);

}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsKCubeIM::enableWidget(bool enabled)
{
    ui.motorAxisController->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsKCubeIM::dockWidgetVisibilityChanged(bool visible)
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

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsKCubeIM::targetChanged(QVector<double> targetPos)
{
    int i = 0;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsKCubeIM::actuatorStatusChanged(QVector<int> status, QVector<double> positions)
{
    bool running = false;

    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] & ito::actuatorMoving)
        {
            running = true;
        }
    }

    ui.motorAxisController->setEnabled(!running);
}

//---------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsKCubeIM::on_btnCalib_clicked()
{
    if (m_pActuator)
    {
        int i;
        QVector<int> axis;

        for (i = 0; i < m_numaxis; i++)
        {
            axis << i;
        }

        enableWidget(false);
        
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pActuator, "calib", Q_ARG(QVector<int>, axis), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        ui.btnCalib->setVisible(false);
        observeInvocation(locker.getSemaphore(), ito::AbstractAddInDockWidget::msgLevelWarningAndError);
        enableWidget(true);
        ui.btnCalib->setVisible(true);
    }
}
