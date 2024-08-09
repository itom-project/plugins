/* ********************************************************************
    Plugin "PIPiezoControl" for itom software
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

#include "DockWidgetST8SMC4USB.h"

#include "common/addInInterface.h"

#include <qmessagebox.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetST8SMC4USB::DockWidgetST8SMC4USB(ito::AddInActuator *actuator) : ito::AbstractAddInDockWidget(actuator)
{
    ui.setupUi(this);
    enableWidget(true);
    ui.btnStart->setEnabled(true);
    ui.btnCancel->setVisible(false);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetST8SMC4USB::parametersChanged(QMap<QString, ito::Param> params)
{
    QString tmp;
    tmp.setNum(params["device_num"].getVal<int>());
    ui.lblDeviceNo->setText(tmp);
    ui.lblPort->setText(params["device_port"].getVal<char*>());

    QString suffix;
    if (params["unit"].getVal<int>() == 0)
    {
        suffix = QLatin1String(" °");
    }
    else
    {
        suffix = QLatin1String(" mm");
    }

    ui.spinBoxStepSize->setSuffix(suffix);
    ui.spinBoxActPos->setSuffix(suffix);
    ui.spinBoxTargetPos->setSuffix(suffix);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetST8SMC4USB::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition) //!< slot to receive information about status and position changes.
{
    ui.spinBoxTargetPos->setEnabled(status[0] & ito::actuatorEnabled);

    if (actPosition.size() > 0)
    {
        ui.spinBoxActPos->setValue(actPosition[0]);
    }

    bool running = false;
    QString style;

    if (status[0] & ito::actuatorMoving)
    {
        style = "background-color: yellow";
        running = true;
    }
    else if (status[0] & ito::actuatorInterrupted)
    {
        style = "background-color: red";
    }
    else if (status[0] & ito::actuatorTimeout)
    {
        style = "background-color: #FFA3FD";
    }
    else
    {
        style = "background-color: ";
    }
    ui.spinBoxActPos->setStyleSheet(style);

    enableWidget(!running);
    ui.btnStart->setEnabled(!running);
    ui.btnCancel->setVisible(running);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetST8SMC4USB::targetChanged(QVector<double> targetPositions)
{
    if (targetPositions.size() > 0)
    {
        ui.spinBoxTargetPos->setValue(targetPositions[0]);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetST8SMC4USB::enableWidget(bool enabled)
{
    ui.spinBoxTargetPos->setEnabled(enabled);
    ui.btnUp->setEnabled(enabled);
    ui.btnDown->setEnabled(enabled);
    ui.btnRefresh->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetST8SMC4USB::on_btnUp_clicked()
{
    setActuatorPosition(0, ui.spinBoxStepSize->value(), true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetST8SMC4USB::on_btnDown_clicked()
{
    setActuatorPosition(0, -ui.spinBoxStepSize->value(), true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetST8SMC4USB::on_btnStart_clicked()
{
    setActuatorPosition(0, ui.spinBoxTargetPos->value(), false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetST8SMC4USB::on_btnCancel_clicked()
{
    setActuatorInterrupt();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetST8SMC4USB::on_btnRefresh_clicked()
{
    requestActuatorStatusAndPositions(true, true);
}
