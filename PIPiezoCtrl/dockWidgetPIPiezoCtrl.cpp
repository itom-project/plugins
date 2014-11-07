/* ********************************************************************
    Plugin "PIPiezoControl" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

#include "dockWidgetPIPiezoCtrl.h"

#include "common/addInInterface.h"

#include <qmessagebox.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
 DockWidgetPIPiezoCtrl::DockWidgetPIPiezoCtrl(int uniqueID, ito::AddInActuator *actuator) : ito::AbstractAddInDockWidget(actuator)
 {
    ui.setupUi(this); 

    ui.lblID->setText(QString::number(uniqueID));

    enableWidget(true);
 }
 //-------------------------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetPIPiezoCtrl::parametersChanged(QMap<QString, ito::Param> params)
 {
    ui.lblDevice1->setText(params["ctrlType"].getVal<char*>());
    ui.lblDevice2->setText(params["ctrlName"].getVal<char*>());
    ui.lblPiezo->setText(params["piezoName"].getVal<char*>());

    bool hasMode = params["hasLocalRemote"].getVal<int>() > 0;
    ui.groupBoxMode->setVisible(hasMode);

    if (params["local"].getVal<int>() > 0)
    {
        ui.radioLocal->setChecked(true);
    }
    else
    {
        ui.radioRemote->setChecked(true);
    }
 }

 //-------------------------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetPIPiezoCtrl::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition) //!< slot to receive information about status and position changes.
 {
    ui.spinBoxTargetPos->setEnabled(status[0] & ito::actuatorEnabled);

    if (actPosition.size() > 0)
    {
        ui.spinBoxActPos->setValue(actPosition[0] * 1000);
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
 }

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::targetChanged(QVector<double> targetPositions)
{
    if (targetPositions.size() > 0)
    {
        ui.spinBoxTargetPos->setValue(targetPositions[0] * 1000);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::enableWidget(bool enabled)
{
    ui.spinBoxTargetPos->setEnabled(enabled);
    ui.btnUp->setEnabled(enabled);
    ui.btnDown->setEnabled(enabled);
    ui.groupBoxMode->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::on_radioLocal_clicked()
{
    QSharedPointer<ito::ParamBase> param;
    if (ui.radioLocal->isChecked())
    {
        param = QSharedPointer<ito::ParamBase>(new ito::ParamBase("local",ito::ParamBase::Int,1.0));
    }
    else
    {
        param = QSharedPointer<ito::ParamBase>(new ito::ParamBase("local",ito::ParamBase::Int,0.0));
    }

    setPluginParameter(param, ito::AbstractAddInDockWidget::msgLevelWarningAndError);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::on_btnUp_clicked()
{
    setActuatorPosition(0, ui.spinBoxStepSize->value() / 1000.0, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::on_btnDown_clicked()
{
    setActuatorPosition(0, -ui.spinBoxStepSize->value() / 1000.0, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::on_btnStart_clicked()
{
    setActuatorPosition(0, ui.spinBoxTargetPos->value() / 1000.0, false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::on_btnRefresh_clicked()
{
    requestActuatorStatusAndPositions(true, true);
}
