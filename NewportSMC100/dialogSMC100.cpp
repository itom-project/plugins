/* ********************************************************************
    Plugin "Newport SMC100" for itom software
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

#include "dialogSMC100.h"
#include "SMC100.h"


#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>


//----------------------------------------------------------------------------------------------------------------------------------
DialogSMC100::DialogSMC100(ito::AddInBase *actuator) :
    AbstractAddInConfigDialog(actuator),
    m_firstRun(true),
    m_pPlugin(actuator)
{
    ui.setupUi(this);

    freshStarted = true;

    m_calibStatusNames.append(tr("MZ and encoder"));
    m_calibStatusNames.append(tr("Current Pos as Home"));
    m_calibStatusNames.append(tr("MZ only"));
    m_calibStatusNames.append(tr("EoR and encoder"));
    m_calibStatusNames.append(tr("EoR only"));

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogSMC100::~DialogSMC100()
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSMC100::parametersChanged(QMap<QString, ito::Param> params)
{
    if (freshStarted)
    {
        m_numAxis = params["numaxis"].getVal<int>();
        m_calibInitialStatus = QVector<int>(m_numAxis, 0);
        m_speedInitialStatus = QVector<double>(m_numAxis, 0.0);
        m_accelInitialStatus = QVector<double>(m_numAxis, 0.0);
        m_axisToInitialize   = QVector<bool>(m_numAxis, false);
        m_InitialStatus      = QVector<int>(m_numAxis, false);
    }

    // get Initial Parameters
    memcpy(m_calibInitialStatus.data(), params["calib_mode"].getVal<int*>(), sizeof(int) * m_numAxis);
    memcpy(m_InitialStatus.data(), params["current_status"].getVal<double*>(), sizeof(double) * m_numAxis);
    memcpy(m_speedInitialStatus.data(), params["speed"].getVal<double*>(), sizeof(double) * m_numAxis);
    memcpy(m_accelInitialStatus.data(), params["accel"].getVal<double*>(), sizeof(double) * m_numAxis);

    if (freshStarted)
    { // Execute those lines only once to create the list entries

        for (int i = 0; i < m_numAxis; ++i)
        {
            createUiListEntry(i);
        }
        freshStarted = false;

        // further set numAxis Label and ID-Label
        ui.idLabel->setText(QString(params["name"].getVal<char*>()));
        ui.numAxisLabel->setText(QString::number(m_numAxis));
    }

    // Disconnect signals of ui elements while changing their values
    for (int i = 0; i < m_pComboBoxes.size(); ++i)
    {
        disconnect(m_pComboBoxes[i], SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxChanged(int)));
        disconnect(m_pSpeedSpin[i], SIGNAL(valueChanged(double)), this, SLOT(spinboxChanged(double)));
        disconnect(m_pAccelSpin[i], SIGNAL(valueChanged(double)), this, SLOT(spinboxChanged(double)));
    }

    // Set Spinboxes to currwent values
    for (int i = 0; i < m_numAxis; ++i)
    {
        m_pAccelSpin[i]->setValue(m_accelInitialStatus[i]);
        m_pSpeedSpin[i]->setValue(m_speedInitialStatus[i]);
    }

    // Set Comboboxes to currentStatus
    for (int i = 0; i < m_numAxis; ++i)
    {
        int j = 0;
        while (true)
        {
            if (m_pComboBoxes[i]->itemData(j) == m_calibInitialStatus[i])
            {
                m_pComboBoxes[i]->setCurrentIndex(j);
                break;
            }
            ++j;
        }
    }

    // set ui to new parameters
    ui.checkAsync->setChecked(params["async"].getVal<int>());

    // Reconnect them for reaction on upcoming changes
    for (int i = 0; i < m_pComboBoxes.size(); ++i)
    {
        connect(m_pComboBoxes[i], SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxChanged(int)));
        connect(m_pSpeedSpin[i], SIGNAL(valueChanged(double)), this, SLOT(spinboxChanged(double)));
        connect(m_pAccelSpin[i], SIGNAL(valueChanged(double)), this, SLOT(spinboxChanged(double)));
    }
    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogSMC100::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

    // This is a general option that does not need the config mode
    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("async", ito::ParamBase::Int, ui.checkAsync->isChecked())));

    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogSMC100::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogSMC100::on_calibrateBtn_clicked()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    ItomSharedSemaphore* waitCond = NULL;

    // Check in m_axisToInitialize which parameters have to be applied to values apply them
    QVector<int> axis;
    QVector<double> speedV;
    QVector<double> accelV;
    QVector<int> calibV;
    QVector<int> configV;
    QVector<int> configVInv;
    for (int i = 0; i < m_axisToInitialize.size(); ++i)
    {
        speedV.append(m_pSpeedSpin[i]->value());
        accelV.append(m_pAccelSpin[i]->value());
        axis.append(i);
        if (m_axisToInitialize[i] == true)
        {
            configV.append(1);
            configVInv.append(0);
            calibV.append(m_pComboBoxes[i]->itemData(m_pComboBoxes[i]->currentIndex()).toInt());
        }
        else
        {
            configV.append(0);
            configVInv.append(0);
            calibV.append(m_calibInitialStatus[i]); // don't care, keep old mode
        }
    }

    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("config_state", ito::ParamBase::IntArray, configV.size(), configV.data())));
    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("speed", ito::ParamBase::DoubleArray, speedV.size(), speedV.data())));
    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("accel", ito::ParamBase::DoubleArray, accelV.size(), accelV.data())));
    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("calib_mode", ito::ParamBase::IntArray, calibV.size(), calibV.data())));
    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("config_state", ito::ParamBase::IntArray, configVInv.size(), configVInv.data())));
    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("exec_calib", ito::ParamBase::IntArray, configV.size(), configV.data())));


    if (!retValue.containsError())
    {
        ui.calibrateBtn->setEnabled(false);
        // Apply the parameters new values
        retValue += setPluginParameters(values, msgLevelWarningAndError);
        if (!retValue.containsError())
        {
            foreach(QFrame *f, m_pListElements)
            {
                f->setStyleSheet("");
            }
            m_axisToInitialize.fill(false);
        }
        ui.calibrateBtn->setEnabled(true);
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogSMC100::enableDialog(bool enabled)
{

}

//---------------------------------------------------------------------------------------------------------------------
void DialogSMC100::comboBoxChanged(int itemIdx)
{
    QFrame *senderFrame = qobject_cast<QFrame*>(QObject::sender()->parent());
    int i = senderFrame->objectName().toInt();
    senderFrame->setStyleSheet("#" + senderFrame->objectName() + " {background-color: DeepSkyBlue}");
    m_axisToInitialize[i] = true;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DialogSMC100::spinboxChanged(double value)
{
    QFrame *senderFrame = qobject_cast<QFrame*>(QObject::sender()->parent());
    int i = senderFrame->objectName().toInt();
    senderFrame->setStyleSheet("#" + senderFrame->objectName() + " {background-color: DeepSkyBlue}");
    m_axisToInitialize[i] = true;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DialogSMC100::resetButtonClicked()
{
    QFrame *senderFrame = qobject_cast<QFrame*>(QObject::sender()->parent());
    int i = senderFrame->objectName().toInt();
    senderFrame->setStyleSheet("");

    // Disconnect signals of ui elements while changing their values
    disconnect(m_pComboBoxes[i], SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxChanged(int)));
    disconnect(m_pSpeedSpin[i], SIGNAL(valueChanged(double)), this, SLOT(spinboxChanged(double)));
    disconnect(m_pAccelSpin[i], SIGNAL(valueChanged(double)), this, SLOT(spinboxChanged(double)));

    m_pSpeedSpin[i]->setValue(m_speedInitialStatus[i]);
    m_pAccelSpin[i]->setValue(m_accelInitialStatus[i]);
    for (int j = 0; j < m_pComboBoxes[i]->count(); ++j)
    {
        if (m_pComboBoxes[i]->itemData(j) == m_calibInitialStatus[i])
        {
            m_pComboBoxes[i]->setCurrentIndex(m_pComboBoxes[i]->itemData(j).toInt());
            break;
        }
    }
    m_axisToInitialize[i] = false;

    // Reconnect them for reaction on upcoming changes
    connect(m_pComboBoxes[i], SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxChanged(int)));
    connect(m_pSpeedSpin[i], SIGNAL(valueChanged(double)), this, SLOT(spinboxChanged(double)));
    connect(m_pAccelSpin[i], SIGNAL(valueChanged(double)), this, SLOT(spinboxChanged(double)));
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DialogSMC100::createUiListEntry(const int i)
{
    // Create outer elements
    QFrame *frame = new QFrame(ui.scrollAreaWidgetContents);
    ui.scrollAreaWidgetContents->layout()->setContentsMargins(0, 0, 0, 0);
    QHBoxLayout *layout = new QHBoxLayout(frame);
    layout->setContentsMargins(0, 0, 0, 0);
    frame->setLayout(layout);
    frame->setObjectName(QString::number(i));

    // Create innner elements and set option
    QLabel *nrLabel = new QLabel(QString::number(i),frame);
    nrLabel->setMaximumWidth(25);
    nrLabel->setAlignment(Qt::AlignCenter);

    QComboBox *statusCombo = new QComboBox(frame);
    statusCombo->setToolTip(tr("Calibration mode"));
    for (int i = 0; i < m_calibStatusNames.size(); ++i)
    {
        statusCombo->addItem(QIcon(), QString::number(i) + " " + m_calibStatusNames[i], QVariant(i));
    }

    QDoubleSpinBox *speedSpin = new QDoubleSpinBox(frame);
    speedSpin->setToolTip(tr("Speed"));
    // TODO hier muessen noch die Werte ausgelesen werden
    speedSpin->setMaximum(100);
    speedSpin->setMinimum(0);
    speedSpin->setSuffix(tr(" units/sec"));

    QDoubleSpinBox *accelSpin = new QDoubleSpinBox(frame);
    accelSpin->setToolTip(tr("Acceleration"));
    accelSpin->setMaximum(100);
    accelSpin->setMinimum(0);
    accelSpin->setSuffix(tr(" units/sec^2"));

    QPushButton *resetButton = new QPushButton(frame);
    resetButton->setToolTip(tr("revert the changed configurations of this axis."));
    resetButton->setIcon(QIcon(":/dialogue/reset.png"));
    resetButton->setIconSize(QSize(20, 20));
    resetButton->setMaximumHeight(22);
    resetButton->setMaximumWidth(22);
    resetButton->setContentsMargins(0, 0, 0, 0);

    // insert elements in Layout
    layout->insertWidget(0, nrLabel);
    layout->insertWidget(1, speedSpin);
    layout->insertWidget(2, accelSpin);
    layout->insertWidget(3, statusCombo);
    layout->insertWidget(4, resetButton);

    frame->setLayout(layout);
    ui.verticalListLayout->insertWidget(ui.verticalListLayout->count() - 1, frame);

    //// Map signals from each line to corresponding slot
    connect(statusCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxChanged(int)));
    connect(speedSpin, SIGNAL(valueChanged(double)), this, SLOT(spinboxChanged(double)));
    connect(accelSpin, SIGNAL(valueChanged(double)), this, SLOT(spinboxChanged(double)));
    connect(resetButton, SIGNAL(clicked()), this, SLOT(resetButtonClicked()));

    // store Pointer to each spin box in a qvector for later occurring use
    m_pListElements.append(frame);
    m_pSpeedSpin.append(speedSpin);
    m_pAccelSpin.append(accelSpin);
    m_pComboBoxes.append(statusCombo);
    m_pResetBtn.append(resetButton);
}
