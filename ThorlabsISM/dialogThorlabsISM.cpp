/* ********************************************************************
    Plugin "ThorlabsISM" for itom software
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

#include "dialogThorlabsISM.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>

#include "Thorlabs.MotionControl.IntegratedStepperMotors.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsISM::DialogThorlabsISM(ito::AddInActuator *actuator) :
    AbstractAddInConfigDialog(actuator),
    m_firstRun(true),
    m_pAia(actuator)
{
    ui.setupUi(this);
    ui.btnCalibInterrupt->setVisible(false);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);

    QString m_sec2Char = QLatin1String("s_");
    m_sec2Char.replace("_", QLatin1String("\u00B2")); //power of two symbol

    QString m_degreeChar = QLatin1String("\u00B0"); //degree symbol
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsISM::~DialogThorlabsISM()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsISM::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.lblDevice->setText(params["deviceName"].getVal<char*>());
        ui.lblSerial->setText(QString("%1").arg(params["serialNumber"].getVal<char*>()));

        if (params["travelMode"].getVal<int>() == MOT_Linear)
        {
            ui.spinSpeed->setSuffix(" mm/s");
            QString suffix(" mm/s_");
            suffix.replace("_", QLatin1String("\u00B2")); //power of two symbol
            ui.spinAccel->setSuffix(suffix);
        }
        else
        {
            QString suffix(" _/s");
            suffix.replace("_", QLatin1String("\u00B0")); //degree symbol
            ui.spinSpeed->setSuffix(suffix);

            suffix.append("_");
            suffix.replace("_", QLatin1String("\u00B2")); //power of two symbol
            ui.spinAccel->setSuffix(suffix);
        }

        m_firstRun = false;
    }

    ui.spinMoveCurrent->setValue(params["moveCurrent"].getVal<int>());
    ui.spinRestCurrent->setValue(params["restCurrent"].getVal<int>());
    ui.spinTimeout->setValue(params["timeout"].getVal<int>());

    ui.spinSpeed->setValue(params["speed"].getVal<double>());
    ui.spinAccel->setValue(params["accel"].getVal<double>());

    ui.checkAsync->setChecked(params["async"].getVal<int>());
    ui.checkEnabled->setChecked(params["enabled"].getVal<int>());

    ui.btnCalib->setEnabled(params["homingAvailable"].getVal<int>() > 0);

    if (params["homed"].getVal<int>() > 0)
    {
        ui.lblHomed->setText("The device is homed.");
    }
    else
    {
        ui.lblHomed->setText("The device is currently not homed.");
    }

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogThorlabsISM::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

    int enabled = ui.checkEnabled->isChecked() ? 1 : 0;
    if (enabled != m_currentParameters["enabled"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("enabled", ito::ParamBase::Int, enabled)));
    }

    int async = ui.checkAsync->isChecked() ? 1 : 0;
    if (async != m_currentParameters["async"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("async", ito::ParamBase::Int, async)));
    }

    int current = ui.spinMoveCurrent->value();
    if (current != m_currentParameters["moveCurrent"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("moveCurrent", ito::ParamBase::Int, current)));
    }

    current = ui.spinRestCurrent->value();
    if (current != m_currentParameters["restCurrent"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("restCurrent", ito::ParamBase::Int, current)));
    }

    double timeout = ui.spinTimeout->value();
    if (std::abs(timeout - m_currentParameters["timeout"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout", ito::ParamBase::Double, timeout)));
    }

    double speed = ui.spinSpeed->value();
    if (std::abs(speed - m_currentParameters["speed"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("speed", ito::ParamBase::Double, speed)));
    }

    double accel = ui.spinAccel->value();
    if (std::abs(accel - m_currentParameters["accel"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("accel", ito::ParamBase::Double, accel)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsISM::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogThorlabsISM::enableDialog(bool enabled)
{
    ui.groupSettings->setEnabled(enabled);
    ui.groupMotorPower->setEnabled(enabled);
    ui.groupMode->setEnabled(enabled);
    ui.btnCalib->setEnabled(enabled);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsISM::on_btnCalib_clicked()
{
    if (m_pAia)
    {
        enableDialog(false);
        ui.buttonBox->setEnabled(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pAia, "calib", Q_ARG(int, 0), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        ui.btnCalib->setVisible(false);
        ui.btnCalibInterrupt->setVisible(true);
        observeInvocation(locker.getSemaphore(), ito::AbstractAddInConfigDialog::msgLevelWarningAndError);
        enableDialog(true);
        ui.buttonBox->setEnabled(true);
        ui.btnCalib->setVisible(true);
        ui.btnCalibInterrupt->setVisible(false);
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsISM::on_btnCalibInterrupt_clicked()
{
    if (m_pAia)
    {
        m_pAia->setInterrupt();
    }
}
