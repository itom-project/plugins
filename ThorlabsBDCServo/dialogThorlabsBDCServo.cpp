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
    int numaxis = params["numaxis"].getVal<ito::int32>();
    int currentAxisShown = ui.comboAxisSelector->currentIndex();

    if (m_firstRun)
    {
        setWindowTitle(
            QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.lblDevice->setText(params["deviceName"].getVal<char*>());
        ui.lblSerial->setText(QString("%1").arg(params["serialNumber"].getVal<char*>()));

        ui.comboAxisSelector->clear();

        const int* channels = params["channel"].getVal<ito::int32*>();
        for (int i = 0; i < numaxis; ++i)
        {
            ui.comboAxisSelector->addItem(QString("Axis %1, Channel %2").arg(i).arg(channels[i]));
        }

        m_firstRun = false;
    }

    ui.checkAsync->setChecked(params["async"].getVal<ito::int32>());
    ui.spinTimeout->setValue(params["timeout"].getVal<double>());
    ui.spinTimeout->setMaximum(params["timeout"].getMax());

    ui.doubleSpinBoxAcceleration->setMaximum(params["acceleration"].getMax());
    ui.doubleSpinBoxVelocity->setMaximum(params["velocity"].getMax());
    ui.doubleSpinBoxBacklash->setMaximum(params["backlash"].getMax());

    ui.doubleSpinBoxAcceleration->setMinimum(params["acceleration"].getMin());
    ui.doubleSpinBoxVelocity->setMinimum(params["velocity"].getMin());
    ui.doubleSpinBoxBacklash->setMinimum(params["backlash"].getMin());

    ui.doubleSpinBoxAcceleration->setValue(
        params["acceleration"].getVal<ito::float64*>()[currentAxisShown]);
    ui.doubleSpinBoxBacklash->setValue(
        params["backlash"].getVal<ito::float64*>()[currentAxisShown]);
    ui.doubleSpinBoxVelocity->setValue(
        params["velocity"].getVal<ito::float64*>()[currentAxisShown]);

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
    int currentAxisShown = ui.comboAxisSelector->currentIndex();
    int numberAxis = m_currentParameters["numaxis"].getVal<ito::int32>();

    int async = ui.checkAsync->isChecked() ? 1 : 0;
    if (async != m_currentParameters["async"].getVal<ito::int32>())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("async", ito::ParamBase::Int, async)));
    }

    double timeout = ui.spinTimeout->value();
    if (std::abs(timeout - m_currentParameters["timeout"].getVal<ito::float64>()) >
        std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("timeout", ito::ParamBase::Double, timeout)));
    }

    int checked = int(ui.checkEnabled->isChecked());
    if (m_currentParameters["enabled"].getVal<ito::int32*>()[currentAxisShown] != checked)
    {
        int enabled[] = {0, 0};
        memcpy(
            enabled,
            m_currentParameters["enabled"].getVal<ito::int32*>(),
            numberAxis * sizeof(ito::int32));
        enabled[currentAxisShown] = checked;

        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("enabled", ito::ParamBase::IntArray, 2, enabled)));
    }

    double accel = ui.doubleSpinBoxAcceleration->value();
    if (std::abs(accel - m_currentParameters["acceleration"].getVal<ito::float64*>()[currentAxisShown]) >
        std::numeric_limits<double>::epsilon())
    {
        double accelerations[] = {0.0, 0.0};
        memcpy(
            accelerations,
            m_currentParameters["acceleration"].getVal<ito::float64*>(),
            numberAxis * sizeof(ito::float64));
        accelerations[currentAxisShown] = accel;

        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("acceleration", ito::ParamBase::DoubleArray, 2, accelerations)));
    }

    double vel = ui.doubleSpinBoxVelocity->value();
    if (std::abs(vel - m_currentParameters["velocity"].getVal<ito::float64*>()[currentAxisShown]) >
        std::numeric_limits<double>::epsilon())
    {
        double velocity[] = {0.0, 0.0};
        memcpy(
            velocity,
            m_currentParameters["velocity"].getVal<ito::float64*>(),
            numberAxis * sizeof(ito::float64*));
        velocity[currentAxisShown] = vel;

        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("velocity", ito::ParamBase::DoubleArray, 2, velocity)));
    }

    double back = ui.doubleSpinBoxVelocity->value();
    if (std::abs(back - m_currentParameters["backlash"].getVal<ito::float64*>()[currentAxisShown]) >
        std::numeric_limits<double>::epsilon())
    {
        double backlash[] = {0.0, 0.0};
        memcpy(
            backlash,
            m_currentParameters["backlash"].getVal<ito::float64*>(),
            numberAxis * sizeof(ito::float64*));
        backlash[currentAxisShown] = back;

        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("backlash", ito::ParamBase::DoubleArray, 2, backlash)));
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
        m_currentParameters["enabled"].getVal<ito::int32*>()[m_currentAxis] =
            ui.checkEnabled->isChecked() ? 1 : 0;
    }

    ui.checkEnabled->setChecked(m_currentParameters["enabled"].getVal<ito::int32*>()[newAxis] > 0);

    ui.doubleSpinBoxAcceleration->setValue(
        m_currentParameters["acceleration"].getVal<ito::float64*>()[newAxis]);
    ui.doubleSpinBoxVelocity->setValue(
        m_currentParameters["velocity"].getVal<ito::float64*>()[newAxis]);
    ui.doubleSpinBoxBacklash->setValue(
        m_currentParameters["backlash"].getVal<ito::float64*>()[newAxis]);

    if (m_currentParameters["homed"].getVal<ito::int32*>()[newAxis])
    {
        ui.btnCalib->setText("This axis is already homed.");
        ui.btnCalib->setEnabled(false);
    }
    else
    {
        ui.btnCalib->setText("homing axes.");
        ui.btnCalib->setEnabled(true);
    }

    m_currentAxis = newAxis;
}
