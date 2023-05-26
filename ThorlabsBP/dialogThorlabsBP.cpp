/* ********************************************************************
    Plugin "ThorlabsBP" for itom software
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

#include "dialogThorlabsBP.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsBP::DialogThorlabsBP(ito::AddInActuator *actuator) :
    AbstractAddInConfigDialog(actuator),
    m_firstRun(true),
    m_pAia(actuator),
    m_currentAxis(-1)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsBP::~DialogThorlabsBP()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBP::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;
    temporaryParams = params;
    int numaxis = params["numaxis"].getVal<int>();

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.lblDevice->setText(params["deviceName"].getVal<char*>());
        ui.lblSerial->setText(QString("%1").arg(params["serialNumber"].getVal<char*>()));

        ui.comboAxisSelector->clear();

        const int *channels = params["channel"].getVal<int*>();
        for (int i = 0; i < numaxis; ++i)
        {
            ui.comboAxisSelector->addItem(QString("Axis %1, Channel %2").arg(i).arg(channels[i]));
        }

        m_firstRun = false;
    }

    ui.checkAsync->setChecked(params["async"].getVal<int>());
    ui.spinTimeout->setValue(params["timeout"].getVal<int>());
    ui.spinTimeout->setMaximum(params["timeout"].getMax());

    if (numaxis > 0)
    {
        ui.comboAxisSelector->setCurrentIndex(0);
        currentAxisChanged(0);
    }

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogThorlabsBP::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    currentAxisChanged(m_currentAxis);

    int async = ui.checkAsync->isChecked() ? 1 : 0;
    if (async != m_currentParameters["async"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("async", ito::ParamBase::Int, async)));
    }

    double timeout = ui.spinTimeout->value();
    if (std::abs(timeout - m_currentParameters["timeout"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout", ito::ParamBase::Double, timeout)));
    }

    if (m_currentParameters["enabled"] != temporaryParams["enabled"])
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase(temporaryParams["enabled"])));
    }

    if (m_currentParameters["controlMode"] != temporaryParams["controlMode"])
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase(temporaryParams["controlMode"])));
    }

    if (m_currentParameters["maximumVoltage"] != temporaryParams["maximumVoltage"])
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase(temporaryParams["maximumVoltage"])));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBP::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogThorlabsBP::enableDialog(bool enabled)
{
    ui.groupGeneral->setEnabled(enabled);
    ui.groupAxis->setEnabled(enabled);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBP::on_btnCalib_clicked()
{
    if (m_pAia)
    {
        enableDialog(false);
        ui.buttonBox->setEnabled(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pAia, "calib", Q_ARG(int, m_currentAxis), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
		ui.btnCalib->setEnabled(false);
        observeInvocation(locker.getSemaphore(), ito::AbstractAddInConfigDialog::msgLevelWarningAndError);
        enableDialog(true);
        ui.buttonBox->setEnabled(true);
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBP::on_comboAxisSelector_currentIndexChanged(int index)
{
    currentAxisChanged(index);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBP::currentAxisChanged(int newAxis)
{
    if (m_currentAxis >= 0)
    {
        temporaryParams["enabled"].getVal<int*>()[m_currentAxis] = ui.checkEnabled->isChecked() ? 1 : 0;
        temporaryParams["controlMode"].getVal<int*>()[m_currentAxis] = ui.checkControlMode->isChecked() ? 1 : 0;
        switch (ui.comboMaximumVoltage->currentIndex())
        {
        case 0:
            temporaryParams["maximumVoltage"].getVal<int*>()[m_currentAxis] = 75;
            break;
        case 1:
            temporaryParams["maximumVoltage"].getVal<int*>()[m_currentAxis] = 100;
            break;
        case 2:
            temporaryParams["maximumVoltage"].getVal<int*>()[m_currentAxis] = 150;
            break;
        }
    }

    ui.checkEnabled->setChecked(temporaryParams["enabled"].getVal<int*>()[newAxis] > 0);
    ui.checkControlMode->setChecked(temporaryParams["controlMode"].getVal<int*>()[newAxis] > 0);
    ui.checkControlMode->setEnabled(temporaryParams["hasFeedback"].getVal<int*>()[newAxis] > 0);
    switch (temporaryParams["maximumVoltage"].getVal<int*>()[newAxis])
    {
    case 75:
        ui.comboMaximumVoltage->setCurrentIndex(0);
        break;
    case 100:
        ui.comboMaximumVoltage->setCurrentIndex(1);
        break;
    case 150:
        ui.comboMaximumVoltage->setCurrentIndex(2);
        break;
    }
    ui.lblTravelRange->setText(QString(QLatin1String("%1 µm")).arg(temporaryParams["maximumTravelRange"].getVal<double*>()[newAxis] * 1000.0));

    if (temporaryParams["zeroed"].getVal<int*>()[newAxis])
    {
        ui.btnCalib->setText("This axis is already zeroed.");
        ui.btnCalib->setEnabled(false);
    }
    else
    {
        ui.btnCalib->setText("Calibrate and zero this axis (requires few seconds).");
        ui.btnCalib->setEnabled(true);
    }

    m_currentAxis = newAxis;


}
