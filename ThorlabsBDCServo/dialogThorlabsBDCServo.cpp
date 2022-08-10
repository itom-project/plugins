/* ********************************************************************
    Plugin "ThorlabsBDCServo" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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

#include "dialogThorlabsBDCServo.h"

#include <qdialogbuttonbox.h>
#include <qmessagebox.h>
#include <qmetaobject.h>
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsBDCServo::DialogThorlabsBDCServo(ito::AddInActuator* actuator) :
    AbstractAddInConfigDialog(actuator), m_firstRun(true), m_pAia(actuator), m_currentAxis(-1)
{
    ui.setupUi(this);

    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsBDCServo::~DialogThorlabsBDCServo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBDCServo::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;
    temporaryParams = params;
    int numaxis = params["numaxis"].getVal<int>();

    if (m_firstRun)
    {
        setWindowTitle(
            QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.lblDevice->setText(params["deviceName"].getVal<char*>());
        ui.lblSerial->setText(QString("%1").arg(params["serialNumber"].getVal<char*>()));

        ui.comboAxisSelector->clear();

        const int* channels = params["channel"].getVal<int*>();
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

    // now activate group boxes, since information is available now (at startup, information is not
    // available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogThorlabsBDCServo::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase>> values;
    currentAxisChanged(m_currentAxis);

    int async = ui.checkAsync->isChecked() ? 1 : 0;
    if (async != m_currentParameters["async"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("async", ito::ParamBase::Int, async)));
    }

    double timeout = ui.spinTimeout->value();
    if (std::abs(timeout - m_currentParameters["timeout"].getVal<double>()) >
        std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("timeout", ito::ParamBase::Double, timeout)));
    }

    if (m_currentParameters["enabled"] != temporaryParams["enabled"])
    {
        values.append(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase(temporaryParams["enabled"])));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBDCServo::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); // close dialog with reject
    }
    else if (role == QDialogButtonBox::AcceptRole)
    {
        accept(); // AcceptRole
    }
    else
    {
        applyParameters(); // ApplyRole
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBDCServo::enableDialog(bool enabled)
{
    ui.groupGeneral->setEnabled(enabled);
    ui.groupAxis->setEnabled(enabled);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBDCServo::on_btnCalib_clicked()
{
    if (m_pAia)
    {
        enableDialog(false);
        ui.buttonBox->setEnabled(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(
            m_pAia,
            "calib",
            Q_ARG(int, m_currentAxis),
            Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        ui.btnCalib->setEnabled(false);
        observeInvocation(
            locker.getSemaphore(), ito::AbstractAddInConfigDialog::msgLevelWarningAndError);
        enableDialog(true);
        ui.buttonBox->setEnabled(true);
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBDCServo::on_comboAxisSelector_currentIndexChanged(int index)
{
    currentAxisChanged(index);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsBDCServo::currentAxisChanged(int newAxis)
{
    if (m_currentAxis >= 0)
    {
        temporaryParams["enabled"].getVal<int*>()[m_currentAxis] =
            ui.checkEnabled->isChecked() ? 1 : 0;
    }

    ui.checkEnabled->setChecked(temporaryParams["enabled"].getVal<int*>()[newAxis] > 0);

    if (temporaryParams["homed"].getVal<int*>()[newAxis])
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
