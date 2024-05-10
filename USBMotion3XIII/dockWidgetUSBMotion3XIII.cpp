/* ********************************************************************
    Plugin "USBMotion3XIII" for itom software
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

/**\file dockWidgetUSBMotion3XIII.cpp
* \brief In this file the functions of the non modal dialog for the USBMotion3XIII are specified
*
*    This file defines the functions of the DockWidgetDummyMotor-Class defined in the file "dockWidgetUSBMotion3XIII.h"
*
*/

#include "dockWidgetUSBMotion3XIII.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail The constructor by the constructor of the DummyMotor during initialisation of the DummyMotor-Instance.
*
*\param[in] params        m_params-Variable containing the parameters of the DummyMotor
*\param[in] uniqueID    The unique Id of the DummyMotor-Instance connected to this dialog
*
*\sa DummyMotor
*/
DockWidgetUSBMotion3XIII::DockWidgetUSBMotion3XIII(ito::AddInActuator *actuator) :
    ito::AbstractAddInDockWidget(actuator)
{
    ui.setupUi(this);

    m_axisEnabled[0] = false;
    m_axisEnabled[1] = false;
    m_axisEnabled[2] = false;

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::identifierChanged(const QString &identifier)
{
    ui.lblID->setText(identifier);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::basicInformationChanged(const QString &axis, const int *axisUnits)
{
    ui.lblAxis->setText(axis);

    if ((axisUnits[0] != axisUnits[1]) || (axisUnits[0] != axisUnits[2]))
    {
        ui.spinStepSize->setSuffix(QLatin1String("\u00b0/mm")); //degree-sign/mm
    }
    else
    {
        ui.spinStepSize->setSuffix(axisUnits[0] == 0 ? QLatin1String("\u00b0").latin1() : "mm"); /*degree-sign*/
    }

    ui.doubleSpinBox_actpos_x->setSuffix(axisUnits[0] == 0 ? QLatin1String("\u00b0").latin1() : "mm"); /*degree-sign*/
    ui.doubleSpinBox_actpos_y->setSuffix(axisUnits[1] == 0 ? QLatin1String("\u00b0").latin1() : "mm"); /*degree-sign*/
    ui.doubleSpinBox_actpos_z->setSuffix(axisUnits[2] == 0 ? QLatin1String("\u00b0").latin1() : "mm"); /*degree-sign*/

    ui.doubleSpinBox_tarpos_x->setSuffix(axisUnits[0] == 0 ? QLatin1String("\u00b0").latin1() : "mm"); /*degree-sign*/
    ui.doubleSpinBox_tarpos_y->setSuffix(axisUnits[1] == 0 ? QLatin1String("\u00b0").latin1() : "mm"); /*degree-sign*/
    ui.doubleSpinBox_tarpos_z->setSuffix(axisUnits[2] == 0 ? QLatin1String("\u00b0").latin1() : "mm"); /*degree-sign*/
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::parametersChanged(QMap<QString, ito::Param> params)
{
    m_axisEnabled[0] = params["axisSteps1"].getVal<double>() > 0;
    m_axisEnabled[1] = params["axisSteps2"].getVal<double>() > 0;
    m_axisEnabled[2] = params["axisSteps3"].getVal<double>() > 0;

    enableWidget(ui.btnStartAbsolute->isVisible());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::on_btnStartAbsolute_clicked()
{
    QVector<int> axes;
    QVector<double> pos;

    if (m_axisEnabled[0])
    {
        axes << 0;
        pos << ui.doubleSpinBox_tarpos_x->value();
    }

    if (m_axisEnabled[1])
    {
        axes << 1;
        pos << ui.doubleSpinBox_tarpos_y->value();
    }

    if (m_axisEnabled[2])
    {
        axes << 2;
        pos << ui.doubleSpinBox_tarpos_z->value();
    }

    setActuatorPosition(axes, pos, false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::on_btn_relPlus1_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    setActuatorPosition(0, stepDeg, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::on_btn_relMinus1_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    setActuatorPosition(0, -stepDeg, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::on_btn_relPlus2_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    setActuatorPosition(1, stepDeg, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::on_btn_relMinus2_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    setActuatorPosition(1, -stepDeg, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::on_btn_relPlus3_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    setActuatorPosition(2, stepDeg, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::on_btn_relMinus3_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    setActuatorPosition(2, -stepDeg, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::on_btnStop_clicked()
{
    setActuatorInterrupt();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::targetChanged(QVector<double> targetPositions)
{
    int i = targetPositions.size();

    if (i >= 0)
    {
        ui.doubleSpinBox_tarpos_x->setValue(targetPositions[0]);
    }
    if (i >= 1)
    {
        ui.doubleSpinBox_tarpos_y->setValue(targetPositions[1]);
    }
    if (i >= 2)
    {
        ui.doubleSpinBox_tarpos_z->setValue(targetPositions[2]);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition)
{
    ui.doubleSpinBox_tarpos_x->setEnabled(status[0] & ito::actuatorEnabled);
    ui.doubleSpinBox_tarpos_y->setEnabled(status[1] & ito::actuatorEnabled);
    ui.doubleSpinBox_tarpos_z->setEnabled(status[2] & ito::actuatorEnabled);

    if (actPosition.size() > 0)
    {
        ui.doubleSpinBox_actpos_x->setValue(actPosition[0]);
        ui.doubleSpinBox_actpos_y->setValue(actPosition[1]);
        ui.doubleSpinBox_actpos_z->setValue(actPosition[2]);
    }

    bool running = false;
    QString style;

    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] & ito::actuatorMoving)
        {
            style = "background-color: yellow";
            running = true;
        }
        else if (status[i] & ito::actuatorInterrupted)
        {
            style = "background-color: red";
        }
        else if (status[i] & ito::actuatorTimeout)
        {
            style = "background-color: #FFA3FD";
        }
        else
        {
            style = "background-color: ";
        }

         switch(i)
         {
         case 0:
             ui.doubleSpinBox_actpos_x->setStyleSheet(style);
             break;
         case 1:
            ui.doubleSpinBox_actpos_y->setStyleSheet(style);
            break;
         case 2:
            ui.doubleSpinBox_actpos_z->setStyleSheet(style);
            break;
         }
     }

     enableWidget(!running);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUSBMotion3XIII::enableWidget(bool enabled)
{
    ui.btn_relPlus1->setEnabled(enabled && m_axisEnabled[0]);
    ui.btn_relPlus2->setEnabled(enabled && m_axisEnabled[1]);
    ui.btn_relPlus3->setEnabled(enabled && m_axisEnabled[2]);
    ui.btn_relMinus1->setEnabled(enabled && m_axisEnabled[0]);
    ui.btn_relMinus2->setEnabled(enabled && m_axisEnabled[1]);
    ui.btn_relMinus3->setEnabled(enabled && m_axisEnabled[2]);
    ui.doubleSpinBox_tarpos_x->setEnabled(enabled && m_axisEnabled[0]);
    ui.doubleSpinBox_tarpos_y->setEnabled(enabled && m_axisEnabled[1]);
    ui.doubleSpinBox_tarpos_z->setEnabled(enabled && m_axisEnabled[2]);

    ui.btnStartAbsolute->setVisible(enabled);
    ui.btnStop->setVisible(!enabled);
}
