///* ********************************************************************
//    Plugin "ThorlabsISM" for itom software
//    URL: http://www.uni-stuttgart.de/ito
//    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
//    Universitaet Stuttgart, Germany
//
//    This file is part of a plugin for the measurement software itom.
//
//    This itom-plugin is free software; you can redistribute it and/or modify it
//    under the terms of the GNU Library General Public Licence as published by
//    the Free Software Foundation; either version 2 of the Licence, or (at
//    your option) any later version.
//
//    itom and its plugins are distributed in the hope that it will be useful, but
//    WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
//    General Public Licence for more details.
//
//    You should have received a copy of the GNU Library General Public License
//    along with itom. If not, see <http://www.gnu.org/licenses/>.
//*********************************************************************** */

#include "dialogThorlabsKCubeDCServo.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>

#include "Thorlabs.MotionControl.KCube.InertialMotor.h"

//-------------------------------------------------------------------------------------
DialogThorlabsKCubeDCServo::DialogThorlabsKCubeDCServo(ito::AddInActuator *actuator) :
    AbstractAddInConfigDialog(actuator),
    m_firstRun(true),
    m_pActuator(actuator)
{
    ui.setupUi(this);
    ui.btnHomeCancel->setVisible(false);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//-------------------------------------------------------------------------------------
DialogThorlabsKCubeDCServo::~DialogThorlabsKCubeDCServo()
{
}

//-------------------------------------------------------------------------------------
void DialogThorlabsKCubeDCServo::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.lblDevice->setText(params["deviceName"].getVal<char*>());
        ui.lblSerial->setText(params["serialNumber"].getVal<char*>());
        ui.lblFirmware->setText(params["firmwareVersion"].getVal<char*>());
        ui.lblHardware->setText(params["hardwareVersion"].getVal<char*>());

        double accel = params["acceleration"].getVal<double>();
        double accelMin = params["acceleration"].getMin();
        double accelMax = params["acceleration"].getMax();

        double speed = params["speed"].getVal<double>();
        double speedMin = params["speed"].getMin();
        double speedMax = params["speed"].getMax();

        ui.spinAccel->setMaximum(accelMax);
        ui.spinAccel->setMinimum(accelMin);
        ui.spinAccel->setValue(accel);

        ui.spinSpeed->setMaximum(speedMax);
        ui.spinSpeed->setMinimum(speedMin);
        ui.spinSpeed->setValue(speed);

        ui.checkEnableAxis->setChecked(params["enableAxis"].getVal<int>() > 0);

        if (params["homingAvailable"].getVal<int>() == 0)
        {
            ui.btnHome->setVisible(false);
        }

        m_firstRun = false;
    }

    ui.spinTimeout->setValue(params["timeout"].getVal<int>());

    ui.checkAsync->setChecked(params["async"].getVal<int>());

    ui.checkFrontPanel->setChecked(params["lockFrontPanel"].getVal<int>());

    if (params["lockFrontPanel"].getFlags() & ito::ParamBase::Readonly)
    {
        ui.checkFrontPanel->setEnabled(false);
    }

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//-------------------------------------------------------------------------------------
ito::RetVal DialogThorlabsKCubeDCServo::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

    int frontPanel = ui.checkFrontPanel->isChecked() ? 1 : 0;
    if (frontPanel != m_currentParameters["lockFrontPanel"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("lockFrontPanel", ito::ParamBase::Int, frontPanel)));
    }

    int async = ui.checkAsync->isChecked() ? 1 : 0;
    if (async != m_currentParameters["async"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("async", ito::ParamBase::Int, async)));
    }

    int enableAxis = ui.checkEnableAxis->isChecked() ? 1 : 0;
    if (enableAxis != m_currentParameters["enableAxis"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("enableAxis", ito::ParamBase::Int, enableAxis)));
    }

    double timeout = ui.spinTimeout->value();
    if (std::abs(timeout - m_currentParameters["timeout"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout", ito::ParamBase::Double, timeout)));
    }

    double accel = ui.spinAccel->value();
    if (std::abs(timeout - m_currentParameters["acceleration"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("acceleration", ito::ParamBase::Double, accel)));
    }

    double speed = ui.spinSpeed->value();
    if (std::abs(timeout - m_currentParameters["speed"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("speed", ito::ParamBase::Double, speed)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsKCubeDCServo::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogThorlabsKCubeDCServo::enableDialog(bool enabled)
{
    ui.groupSettings->setEnabled(enabled);
    ui.groupMode->setEnabled(enabled);
    ui.btnSetOrigin->setEnabled(enabled);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsKCubeDCServo::on_btnSetOrigin_clicked()
{
    if (m_pActuator)
    {
        QVector<int> axis;
        axis << 0;

        enableDialog(false);
        ui.buttonBox->setEnabled(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pActuator, "setOrigin", Q_ARG(QVector<int>, axis), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        ui.btnSetOrigin->setVisible(false);
        observeInvocation(locker.getSemaphore(), ito::AbstractAddInConfigDialog::msgLevelWarningAndError);
        enableDialog(true);
        ui.buttonBox->setEnabled(true);
        ui.btnSetOrigin->setVisible(true);
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsKCubeDCServo::on_btnHome_clicked()
{
    if (m_pActuator)
    {
        QVector<int> axis;
        axis << 0;

        enableDialog(false);
        ui.btnHomeCancel->setVisible(true);

        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pActuator, "calib", Q_ARG(QVector<int>, axis), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        ui.btnHome->setVisible(false);
        observeInvocation(locker.getSemaphore(), ito::AbstractAddInConfigDialog::msgLevelWarningAndError);
        enableDialog(true);
        ui.btnHomeCancel->setVisible(false);
        ui.btnHome->setVisible(true);
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsKCubeDCServo::on_btnHomeCancel_clicked()
{
    if (m_pActuator)
    {
        m_pActuator->setInterrupt();
    }
}
