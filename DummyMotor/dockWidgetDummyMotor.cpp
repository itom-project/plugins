/* ********************************************************************
    Plugin "DummyMotor" for itom software
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

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail The constructor by the constructor of the DummyMotor during initialisation of the DummyMotor-Instance.
*
*\param[in] params        m_params-Variable containg the parameters of the DummyMotor
*\param[in] uniqueID    The unique Id of the DummyMotor-Instance connected to this dialog
*
*\sa DummyMotor
*/
DockWidgetDummyMotor::DockWidgetDummyMotor(int uniqueID, ito::AddInActuator * myPlugin) :
    ito::AbstractAddInDockWidget(myPlugin),
    m_isVisible(false),
    m_numaxis(-1)
{   
    ui.setupUi(this);

    ui.lblID->setText(QString::number(uniqueID));

    m_btnRelDec.append(ui.pushButton_xm);
    m_btnRelDec.append(ui.pushButton_ym);
    m_btnRelDec.append(ui.pushButton_zm);
    m_btnRelDec.append(ui.pushButton_am);
    m_btnRelDec.append(ui.pushButton_bm);
    m_btnRelDec.append(ui.pushButton_cm);
    foreach(QPushButton* btn, m_btnRelDec)
    {
        connect(btn, SIGNAL(clicked()), this, SLOT(btnRelDecClicked()));
    }

    m_btnRelInc.append(ui.pushButton_xp);
    m_btnRelInc.append(ui.pushButton_yp);
    m_btnRelInc.append(ui.pushButton_zp);
    m_btnRelInc.append(ui.pushButton_ap);
    m_btnRelInc.append(ui.pushButton_bp);
    m_btnRelInc.append(ui.pushButton_cp);
    foreach(QPushButton* btn, m_btnRelInc)
    {
        connect(btn, SIGNAL(clicked()), this, SLOT(btnRelIncClicked()));
    }

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

    m_spinCurrentPos.append(ui.doubleSpinBox_actpos_x);
    m_spinCurrentPos.append(ui.doubleSpinBox_actpos_y);
    m_spinCurrentPos.append(ui.doubleSpinBox_actpos_z);
    m_spinCurrentPos.append(ui.doubleSpinBox_actpos_a);
    m_spinCurrentPos.append(ui.doubleSpinBox_actpos_b);
    m_spinCurrentPos.append(ui.doubleSpinBox_actpos_c);

    m_spinTargetPos.append(ui.doubleSpinBox_tarpos_x);
    m_spinTargetPos.append(ui.doubleSpinBox_tarpos_y);
    m_spinTargetPos.append(ui.doubleSpinBox_tarpos_z);
    m_spinTargetPos.append(ui.doubleSpinBox_tarpos_a);
    m_spinTargetPos.append(ui.doubleSpinBox_tarpos_b);
    m_spinTargetPos.append(ui.doubleSpinBox_tarpos_c);

    m_labels.append(ui.label_xachse);
    m_labels.append(ui.label_yachse);
    m_labels.append(ui.label_zachse);
    m_labels.append(ui.label_aachse);
    m_labels.append(ui.label_bachse);
    m_labels.append(ui.label_cachse);

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This Slot checks all parameters, checks the axis-numbers and enables the corresponding GUI-elements
*
*\param[in] params        m_params-Variable containg the parameters of the DummyMotor
*
*/
void DockWidgetDummyMotor::parametersChanged(QMap<QString, ito::Param> params)
{
    bool newNumaxis = m_numaxis != params["numaxis"].getVal<int>();
    if (newNumaxis)
    {
        m_numaxis = params["numaxis"].getVal<int>();
        ui.lblAxis->setText(QString::number(m_numaxis));

        visibleWidget();
    }

    enableWidget(true);

    for (int i = 0; i < m_checkEnabled.size(); i++)
    {
        m_checkEnabled[i]->setChecked(m_numaxis > i);
        m_btnRelDec[i]->setEnabled(m_numaxis > i);
        m_btnRelInc[i]->setEnabled(m_numaxis > i);
        m_spinTargetPos[i]->setEnabled(m_numaxis > i);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::targetChanged(QVector<double> targetPos)
{
    for (int i = 0; i < targetPos.size(); i++)
    {
        m_spinTargetPos[i]->setValue(targetPos[i]);
    }
 }

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::actuatorStatusChanged(QVector<int> status, QVector<double> positions)
{
    bool running = false;
    QString style;

    for (int i = 0; i < std::min(status.size(), m_spinCurrentPos.size()); i++)
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
        /*else if (status[i] & ito::actuatorTimeout) //timeout is bad for dummyMotor, since the waitForDone-method always drops into a timeout
        {
            style = "background-color: green";
        }*/
        else
        {
            style = "background-color: ";
        }

        m_spinCurrentPos[i]->setStyleSheet(style);
    }

    enableWidget(!running);

    for (int i = 0; i < std::min(positions.size(), m_spinCurrentPos.size()); i++)
    {
        m_spinCurrentPos[i]->setValue(positions[i]);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::btnRelDecClicked()                //slot if any button for a relative, negative movement is clicked
{
    double dpos = ui.spinStepSize->value() / -1e3;

    if (qobject_cast<QPushButton*>(sender()))
    {
        int idx = m_btnRelDec.indexOf(qobject_cast<QPushButton*>(sender()));

        if (idx >= 0)
        {
            setActuatorPosition(idx, dpos, true);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::btnRelIncClicked()                //slot if any button for a relative, positive movement is clicked
{
    double dpos = ui.spinStepSize->value() / 1e3;

    if (qobject_cast<QPushButton*>(sender()))
    {
        int idx = m_btnRelInc.indexOf(qobject_cast<QPushButton*>(sender()));

        if (idx >= 0)
        {
            setActuatorPosition(idx, dpos, true);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::checkEnabledClicked(bool checked) //slot if any "enabled"-checkbox is clicked
{
    if (qobject_cast<QCheckBox*>(sender()))
    {
        int idx = m_checkEnabled.indexOf(qobject_cast<QCheckBox*>(sender()));

        if (idx >= 0)
        {
            m_btnRelDec[idx]->setEnabled(checked);
            m_btnRelInc[idx]->setEnabled(checked);
            m_spinTargetPos[idx]->setEnabled(checked);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_stop_clicked()
{
    setActuatorInterrupt();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_start_clicked()
{
    QVector<int> axis;
    QVector<double> dpos;

    for (int i = 0; i < m_checkEnabled.size(); ++i)
    {
        if (m_checkEnabled[i]->isChecked() == 1)
        {
            axis << i;
            dpos << m_spinTargetPos[i]->value();
        }
    }

    setActuatorPosition(axis, dpos, false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_refresh_clicked()
{
    requestActuatorStatusAndPositions(true, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::visibleWidget()
{
    for (int i = 0; i < m_checkEnabled.size(); i++)
    {
        m_checkEnabled[i]->setVisible(m_numaxis > i);
        m_btnRelDec[i]->setVisible(m_numaxis > i);
        m_btnRelInc[i]->setVisible(m_numaxis > i);
        m_spinCurrentPos[i]->setVisible(m_numaxis > i);
        m_spinTargetPos[i]->setVisible(m_numaxis > i);
        m_labels[i]->setVisible(m_numaxis > i);
    }

    ui.label_actpos_t->setVisible(m_numaxis > 3);
    ui.label_tarpos_r->setVisible(m_numaxis > 3);

    if (m_numaxis < 4)
    {
        this->setMinimumHeight(278);
    }
    else
    {
        this->setMinimumHeight(366);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::enableWidget(bool enabled)
{
    for (int i = 0; i < m_checkEnabled.size(); i++)
    {
        m_checkEnabled[i]->setEnabled(enabled && m_numaxis > i);
        m_btnRelDec[i]->setEnabled(enabled && m_numaxis > i);
        m_btnRelInc[i]->setEnabled(enabled && m_numaxis > i);
    }

    ui.pushButton_start->setVisible(enabled);
    ui.pushButton_stop->setVisible(!enabled);

    ui.spinStepSize->setFocus();
}