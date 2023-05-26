/* ********************************************************************
    Plugin "AerotechEnsemble" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

/**\file dockWidgetDummyMotor.cpp
* \brief In this file the functions of the non modal dialog for the DummyMotor are specified
*
*    This file defines the functions of the DockWidgetDummyMotor-Class defined in the file "dockWidgetDummyMotor.h"
*
*\sa dockWidgetDummyMotor, DummyMotor
*/

#include "dockWidgetAerotechEnsemble.h"
#include "common/addInInterface.h"
#include <qmessagebox.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail The constructor by the constructor of the DummyMotor during initialisation of the DummyMotor-Instance.
*
*\param[in] params        m_params-Variable containg the parameters of the DummyMotor
*\param[in] uniqueID    The unique Id of the DummyMotor-Instance connected to this dialog
*
*\sa DummyMotor
*/
DockWidgetAerotechEnsemble::DockWidgetAerotechEnsemble(QMap<QString, ito::Param> params, int uniqueID, ito::AddInActuator *actuator) :
    m_actuator(actuator),
    m_pSignalMapperEnabled(NULL),
    m_pSignalPosInc(NULL),
    m_pSignalPosDec(NULL),
    m_initialized(false),
    m_numaxis(-1)
{
    ui.setupUi(this);

    m_pSignalMapperEnabled = new QSignalMapper(this);
    m_pSignalPosInc = new QSignalMapper(this);
    m_pSignalPosDec = new QSignalMapper(this);

    connect(m_pSignalMapperEnabled, SIGNAL(mapped(const int &)), this, SLOT(checkEnabledClicked(const int &)));
    connect(m_pSignalPosInc, SIGNAL(mapped(const int &)), this, SLOT(posIncrementClicked(const int &)));
    connect(m_pSignalPosDec, SIGNAL(mapped(const int &)), this, SLOT(posDecrementClicked(const int &)));

    ui.lblID->setText(QString::number(uniqueID));

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::CheckAxisNums(QMap<QString, ito::Param> params)
{
//    ui.cb0_Name->setChecked(params["axisEnable"].getVal<bool>());
//    on_cb0_Name_clicked();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This Slot checks all parameters, currently only calling CheckAxisNums
*
*\param[in] params        m_params-Variable containg the parameters of the DummyMotor
*
*/
void DockWidgetAerotechEnsemble::valuesChanged(QMap<QString, ito::Param> params)
{
     CheckAxisNums(params);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::init(QMap<QString, ito::Param> params, QStringList axisNames)
{
    if (m_initialized)
    {
        return; //already initialized
    }

    m_numaxis = params["numAxis"].getVal<int>();
    ui.lblAxis->setText(QString::number(m_numaxis));

    QCheckBox* widEnabled;
    QDoubleSpinBox* widActual;
    QDoubleSpinBox* widTarget;
    QPushButton* widPosInc;
    QPushButton* widPosDec;

    for (int i = 0; i < m_numaxis; ++i)
    {
        widEnabled = new QCheckBox(ui.groupPositioning);
        widEnabled->setChecked(true);
        widEnabled->setText(axisNames[i]);
        m_pWidgetEnabled.append(widEnabled);
        ui.gridLayout->addWidget(widEnabled, 2 + i, 0, 1, 1);
        connect(widEnabled, SIGNAL(clicked()), m_pSignalMapperEnabled, SLOT(map()));
        m_pSignalMapperEnabled->setMapping(widEnabled, i);

        widActual = new QDoubleSpinBox(ui.groupPositioning);
        widActual->setEnabled(false);
        QSizePolicy sizePolicy6(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy6.setHorizontalStretch(0);
        sizePolicy6.setVerticalStretch(0);
        sizePolicy6.setHeightForWidth(widActual->sizePolicy().hasHeightForWidth());
        widActual->setSizePolicy(sizePolicy6);
        widActual->setMinimumSize(QSize(0, 0));
        widActual->setMaximumSize(QSize(16777215, 16777215));
        widActual->setDecimals(6);
        widActual->setMinimum(-1e+08);
        widActual->setMaximum(1e+08);
        widActual->setSingleStep(0.001);
        widActual->setSuffix(tr("mm"));
        m_pWidgetActual.append(widActual);
        ui.gridLayout->addWidget(widActual, 2 + i, 1, 1, 1);

        widTarget = new QDoubleSpinBox(ui.groupPositioning);
        sizePolicy6.setHeightForWidth(widTarget->sizePolicy().hasHeightForWidth());
        widTarget->setSizePolicy(sizePolicy6);
        widTarget->setMinimumSize(QSize(0, 0));
        widTarget->setMaximumSize(QSize(16777215, 16777215));
        widTarget->setDecimals(6);
        widTarget->setMinimum(-1e+08);
        widTarget->setMaximum(1e+08);
        widTarget->setSingleStep(0.5);
        widTarget->setSuffix(tr("mm"));
        widTarget->setValue(0);
        m_pWidgetTarget.append(widTarget);
        ui.gridLayout->addWidget(widTarget, 2 + i, 2, 1, 1);

        widPosInc = new QPushButton(ui.groupPositioning);
        widPosInc->setMaximumSize(QSize(20, 16777215));
        widPosInc->setText("+");
        m_pWidgetPosInc.append(widPosInc);
        ui.gridLayout->addWidget(widPosInc, 2 + i, 3, 1, 1);
        connect(widPosInc, SIGNAL(clicked()), m_pSignalPosInc, SLOT(map()));
        m_pSignalPosInc->setMapping(widPosInc, i);

        widPosDec = new QPushButton(ui.groupPositioning);
        widPosDec->setMaximumSize(QSize(20, 16777215));
        widPosDec->setText("-");
        m_pWidgetPosDec.append(widPosDec);
        ui.gridLayout->addWidget(widPosDec, 2 + i, 4, 1, 1);
        connect(widPosDec, SIGNAL(clicked()), m_pSignalPosDec, SLOT(map()));
        m_pSignalPosDec->setMapping(widPosDec, i);
    }

    //ui.gridLayout->addWidget(ui.horizontalLayout_6, 2 + m_numaxis, 0, 1, 1);
    //ui.gridLayout->itemAt(2 + m_numaxis)->layout(addWidget(ui.horizontalLayout_6));
    //now disable the templates from the ui file
    delete ui.cb0_Name;
    ui.cb0_Name = NULL;
    delete ui.dsb0_Actual;
    ui.dsb0_Actual = NULL;
    delete ui.dsb0_Target;
    ui.dsb0_Target = NULL;
    delete ui.pb0_Add;
    ui.pb0_Add = NULL;
    delete ui.pb0_Sub;
    ui.pb0_Sub = NULL;

    m_initialized = true;

    setMinimumHeight((29 * m_numaxis) + 184);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::targetChanged(QVector<double> targetPositions)
{
    if (!m_initialized)
    {
        return;
    }

    for (int i = 0; i < targetPositions.size(); ++i)
    {
        m_pWidgetTarget[i]->setValue(targetPositions[i]);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition)
{
    int i = 0;

    for (i = 0; i < m_numaxis; i++)
    {
        m_pWidgetTarget[i]->setEnabled(status[i] & ito::actuatorEnabled);
    }

    if (actPosition.size() > 0)
    {
        for (i = 0; i < m_numaxis; i++)
        {
            m_pWidgetActual[i]->setValue(actPosition[i]);
        }
    }

    bool running = false;
    QString style;

    for (i = 0; i < status.size(); i++)
    {
        if (status[i] & ito::actuatorMoving)
        {
            style = "background-color: yellow";
            running = true;
            m_pWidgetPosInc[i]->setEnabled(false);
            m_pWidgetPosDec[i]->setEnabled(false);
        }
        else if (status[i] & ito::actuatorInterrupted)
        {
            style = "background-color: red";
            m_pWidgetPosInc[i]->setEnabled(true);
            m_pWidgetPosDec[i]->setEnabled(true);
        }
        else if (status[i] & ito::actuatorTimeout)
        {
            style = "background-color: #FFA3FD";
            m_pWidgetPosInc[i]->setEnabled(true);
            m_pWidgetPosDec[i]->setEnabled(true);
        }
        else
        {
            style = "background-color: ";
            m_pWidgetPosInc[i]->setEnabled(true);
            m_pWidgetPosDec[i]->setEnabled(true);
        }

        m_pWidgetActual[i]->setStyleSheet(style);
     }

     enableWidget(!running);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::checkEnabledClicked(const int &index)
{
    if (!m_initialized)
    {
        return;
    }

    m_pWidgetTarget[index]->setEnabled(m_pWidgetEnabled[index]->isChecked());
    m_pWidgetPosDec[index]->setEnabled(m_pWidgetEnabled[index]->isChecked());
    m_pWidgetPosInc[index]->setEnabled(m_pWidgetEnabled[index]->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/*void DockWidgetAerotechEnsemble::MoveRelative(const int &index, const double dpos)
{
    ItomSharedSemaphore *waitCond = new ItomSharedSemaphore();
    QMetaObject::invokeMethod(m_actuator, "setPosRel", Q_ARG(int, index), Q_ARG(double, dpos), Q_ARG(ItomSharedSemaphore*, waitCond));

    if (waitCond->waitAndProcessEvents(60000)) //no timeout
    {
        if (waitCond->returnValue.containsWarningOrError())
        {
            QMessageBox msgBox(this);
            if (waitCond->returnValue.errorMessage() != NULL)
            {
                QString errStr = waitCond->returnValue.errorMessage();
                msgBox.setText(errStr);
            }
            else
            {
                msgBox.setText(tr("unknown error or warning when moving any axis"));
            }
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox::critical(this, tr("Timeout"), tr("timeout when moving any axis"));
    }

    waitCond->release();
    waitCond->deleteSemaphore();
}*/

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::Move(const QVector<int> axis, const QVector<double> dpos, const char* func)
{
    ItomSharedSemaphore *waitCond = new ItomSharedSemaphore();
    QMetaObject::invokeMethod(m_actuator, func, Q_ARG(QVector<int>, axis), Q_ARG(QVector<double>, dpos), Q_ARG(ItomSharedSemaphore*, waitCond));

    if (waitCond->waitAndProcessEvents(60000)) //no timeout
    {
        if (waitCond->returnValue.containsWarningOrError())
        {
            QMessageBox msgBox(this);
            if (waitCond->returnValue.hasErrorMessage())
            {
                msgBox.setText(QLatin1String(waitCond->returnValue.errorMessage()));
            }
            else
            {
                msgBox.setText(tr("unknown error or warning when moving any axis"));
            }
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox::critical(this, tr("Timeout"), tr("timeout when moving any axis"));
    }

    waitCond->release();
    waitCond->deleteSemaphore();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::posIncrementClicked(const int &index)
{
    if (!m_initialized)
    {
        return;
    }

    double dpos = ui.dsb_StepSize->value();
//    MoveRelative(index, dpos);
    Move(QVector<int>(1, index), QVector<double>(1, dpos), "setPosRel");
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::posDecrementClicked(const int &index)
{
    if (!m_initialized)
    {
        return;
    }

    double dpos = -ui.dsb_StepSize->value();
//    MoveRelative(index, dpos);
    Move(QVector<int>(1, index), QVector<double>(1, dpos), "setPosRel");
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::enableWidget(bool enabled)
{
/*    ui.btn_relPlus1->setEnabled(enabled);
    ui.btn_relPlus2->setEnabled(enabled);
    ui.btn_relPlus3->setEnabled(enabled);
    ui.btn_relMinus1->setEnabled(enabled);
    ui.btn_relMinus2->setEnabled(enabled);
    ui.btn_relMinus3->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_x->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_y->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_z->setEnabled(enabled);*/

    ui.pb_Start->setVisible(enabled);
    ui.pb_Stop->setVisible(!enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb_Start_clicked()
{
    QVector<int> axis;
    QVector<double> dpos;

    for (int i = 0; i < m_numaxis; i++)
    {
        if (m_pWidgetEnabled[i]->isChecked())
        {
            axis << i;
            dpos << m_pWidgetTarget[i]->value();
        }
    }

//    emit MoveAbsolute(axis, dpos, 0);
    Move(axis, dpos, "setPosAbs");
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb_Stop_clicked()
{
    if (m_actuator) m_actuator->setInterrupt();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb_Refresh_clicked()
{
    emit MotorTriggerStatusRequest(true, false);
}
