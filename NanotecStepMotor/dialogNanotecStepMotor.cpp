/* ********************************************************************
    Plugin "Standa NanotecStepMotor" for itom software
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

#include "dialogNanotecStepMotor.h"
#include "NanotecStepMotor.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogNanotecStepMotor::DialogNanotecStepMotor(ito::AddInBase *actuator) :
    AbstractAddInConfigDialog(actuator),
    m_firstRun(true),
    m_isChanging(false)
{
    ui.setupUi(this);

    freshStarted = true;

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogNanotecStepMotor::~DialogNanotecStepMotor()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogNanotecStepMotor::parametersChanged(QMap<QString, ito::Param> params)
{
    m_isChanging = true;

    if (freshStarted)
    {
        m_numAxis = params["numaxis"].getVal<int>();
        m_microStepsInitialStatus = QVector<int>(m_numAxis, 0);
        m_speedInitialStatus = QVector<int>(m_numAxis, 0);
        m_accelInitialStatus = QVector<int>(m_numAxis, 0);
        m_decelInitialStatus = QVector<int>(m_numAxis, 0);
        m_axisToInitialize   = QVector<bool>(m_numAxis, false);
    }

    // get Initial Parameters
    memcpy(m_microStepsInitialStatus.data(), params["microSteps"].getVal<int*>(), sizeof(int) * m_numAxis);
    memcpy(m_speedInitialStatus.data(), params["speed"].getVal<int*>(), sizeof(int) * m_numAxis);
    memcpy(m_accelInitialStatus.data(), params["accel"].getVal<int*>(), sizeof(int) * m_numAxis);
    memcpy(m_decelInitialStatus.data(), params["decel"].getVal<int*>(), sizeof(int) * m_numAxis);

    if (freshStarted)
    {

        // Execute those lines only once to create the list entries

        for (int i = 0; i < m_numAxis; ++i)
        {
            createUiListEntry(i);
        }
        freshStarted = false;

        // further set numAxis Label and ID-Label
        ui.idLabel->setText(QString(params["deviceID"].getVal<char*>()));
        ui.numAxisLabel->setText(QString::number(m_numAxis));
    }

    // Set Spinboxes to currwent values
    for (int i = 0; i < m_numAxis; ++i)
    {
        m_pMicroStepsSpin[i]->setValue(m_microStepsInitialStatus[i]);
        m_pAccelSpin[i]->setValue(m_accelInitialStatus[i]);
        m_pSpeedSpin[i]->setValue(m_speedInitialStatus[i]);
        m_pDecelSpin[i]->setValue(m_decelInitialStatus[i]);
    }

    // set ui to new parameters
    ui.checkAsync->setChecked(params["async"].getVal<int>());

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);

    m_currentParameters = params;

    m_isChanging = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogNanotecStepMotor::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    int *valArray = new int[m_numAxis];

    // This is a general option that does not need the config mode
    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("async", ito::ParamBase::Int, ui.checkAsync->isChecked())));

    for (int i = 0; i < m_numAxis; ++i)
    {
        valArray[i] = m_pMicroStepsSpin[i]->value();
    }

    if (memcmp(valArray, m_currentParameters["microSteps"].getVal<int*>(), sizeof(int) * m_numAxis) != 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("microSteps", ito::ParamBase::IntArray, m_numAxis, valArray)));
    }

    for (int i = 0; i < m_numAxis; ++i)
    {
        valArray[i] = m_pSpeedSpin[i]->value();
    }

    if (memcmp(valArray, m_currentParameters["speed"].getVal<int*>(), sizeof(int) * m_numAxis) != 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("speed", ito::ParamBase::IntArray, m_numAxis, valArray)));
    }

    for (int i = 0; i < m_numAxis; ++i)
    {
        valArray[i] = m_pAccelSpin[i]->value();
    }

    if (memcmp(valArray, m_currentParameters["accel"].getVal<int*>(), sizeof(int) * m_numAxis) != 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("accel", ito::ParamBase::IntArray, m_numAxis, valArray)));
    }

    for (int i = 0; i < m_numAxis; ++i)
    {
        valArray[i] = m_pDecelSpin[i]->value();
    }

    if (memcmp(valArray, m_currentParameters["decel"].getVal<int*>(), sizeof(int) * m_numAxis) != 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("decel", ito::ParamBase::IntArray, m_numAxis, valArray)));
    }

    delete[] valArray;

    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNanotecStepMotor::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); //close dialog with reject
    }
    else if (role == QDialogButtonBox::AcceptRole)
    {
        accept(); //AcceptRole
    }
    else
    {
        applyParameters(); //ApplyRole
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNanotecStepMotor::enableDialog(bool enabled)
{

}

//---------------------------------------------------------------------------------------------------------------------
void DialogNanotecStepMotor::spinboxChanged(int value)
{
    if (!m_isChanging)
    {
        QFrame *senderFrame = qobject_cast<QFrame*>(QObject::sender()->parent());
        int i = senderFrame->objectName().toInt();
        senderFrame->setStyleSheet("#" + senderFrame->objectName() + " {background-color: DeepSkyBlue}");
        m_axisToInitialize[i] = true;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DialogNanotecStepMotor::resetButtonClicked()
{
    QFrame *senderFrame = qobject_cast<QFrame*>(QObject::sender()->parent());
    int i = senderFrame->objectName().toInt();

    m_pMicroStepsSpin[i]->setValue(m_microStepsInitialStatus[i]);
    m_pSpeedSpin[i]->setValue(m_speedInitialStatus[i]);
    m_pAccelSpin[i]->setValue(m_accelInitialStatus[i]);
    m_pDecelSpin[i]->setValue(m_decelInitialStatus[i]);
    m_axisToInitialize[i] = false;

    senderFrame->setStyleSheet("");
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DialogNanotecStepMotor::createUiListEntry(const int i)
{
    // Create outer elements
    QFrame *frame = new QFrame(ui.scrollAreaWidgetContents);
    ui.scrollAreaWidgetContents->layout()->setContentsMargins(0, 0, 0, 0);
    QHBoxLayout *layout = new QHBoxLayout(frame);
    layout->setContentsMargins(0, 0, 0, 0);
    frame->setLayout(layout);
    frame->setObjectName(QString::number(i));

    // Create innner elements and set option
    QLabel *nrLabel = new QLabel(QString::number(i), frame);
    nrLabel->setMaximumWidth(25);
    nrLabel->setAlignment(Qt::AlignCenter);

    QSpinBox *microStepsSpin = new QSpinBox(frame);
    microStepsSpin->setToolTip(tr("MicroSteps"));
    microStepsSpin->setMaximum(64);
    microStepsSpin->setMinimum(1);
    microStepsSpin->setSuffix(tr(" step/unit"));

    QSpinBox *speedSpin = new QSpinBox(frame);
    speedSpin->setToolTip(tr("Speed"));
    speedSpin->setMaximum(1000000);
    speedSpin->setMinimum(1);
    speedSpin->setSuffix(tr(" units/sec"));

    QSpinBox *accelSpin = new QSpinBox(frame);
    accelSpin->setToolTip(tr("Acceleration"));
    accelSpin->setMaximum(65535);
    accelSpin->setMinimum(1);
    accelSpin->setSuffix(tr(" units/sec"));

    QSpinBox *decelSpin = new QSpinBox(frame);
    decelSpin->setToolTip(tr("Deceleration"));
    decelSpin->setMaximum(65535);
    decelSpin->setMinimum(0);
    decelSpin->setSuffix(tr(" units/sec"));

    QPushButton *resetButton = new QPushButton(frame);
    resetButton->setToolTip(tr("revert the changed configurations of this axis."));
    resetButton->setIcon(QIcon(":/dialogue/reset.png"));
    resetButton->setIconSize(QSize(20, 20));
    resetButton->setMaximumHeight(22);
    resetButton->setMaximumWidth(22);
    resetButton->setContentsMargins(0, 0, 0, 0);

    // insert elements in Layout
    layout->insertWidget(0, nrLabel);
    layout->insertWidget(1, microStepsSpin);
    layout->insertWidget(2, speedSpin);
    layout->insertWidget(3, accelSpin);
    layout->insertWidget(4, decelSpin);
    layout->insertWidget(5, resetButton);

    frame->setLayout(layout);
    ui.verticalListLayout->insertWidget(ui.verticalListLayout->count() - 1, frame);

    //// Map signals from each line to corresponding slot
    connect(microStepsSpin, SIGNAL(valueChanged(int)), this, SLOT(spinboxChanged(int)));
    connect(speedSpin, SIGNAL(valueChanged(int)), this, SLOT(spinboxChanged(int)));
    connect(accelSpin, SIGNAL(valueChanged(int)), this, SLOT(spinboxChanged(int)));
    connect(decelSpin, SIGNAL(valueChanged(int)), this, SLOT(spinboxChanged(int)));
    connect(resetButton, SIGNAL(clicked()), this, SLOT(resetButtonClicked()));

    // store Pointer to each spin box in a qvector for later occurring use
    m_pListElements.append(frame);
    m_pMicroStepsSpin.append(microStepsSpin);
    m_pSpeedSpin.append(speedSpin);
    m_pAccelSpin.append(accelSpin);
    m_pDecelSpin.append(decelSpin);
    m_pResetBtn.append(resetButton);
}
