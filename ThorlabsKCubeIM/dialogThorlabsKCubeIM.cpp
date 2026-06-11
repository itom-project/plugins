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

#include "dialogThorlabsKCubeIM.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>

#include "Thorlabs.MotionControl.KCube.InertialMotor.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsKCubeIM::DialogThorlabsKCubeIM(ito::AddInActuator *actuator) :
    AbstractAddInConfigDialog(actuator),
    m_firstRun(true),
    m_actuator(actuator),
    m_numaxis(0)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsKCubeIM::~DialogThorlabsKCubeIM()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsKCubeIM::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.lblDevice->setText(params["deviceName"].getVal<char*>());
        ui.lblSerial->setText(params["serialNumber"].getVal<char*>());
        ui.lblFirmware->setText(params["firmwareVersion"].getVal<char*>());
        ui.lblSoftware->setText(params["softwareVersion"].getVal<char*>());

        m_numaxis = params["numaxis"].getVal<int>();

        int *vol = params["maxVoltage"].getVal<int*>();
        int minVol = params["maxVoltage"].getMin();
        int maxVol = params["maxVoltage"].getMax();

        ui.spinBoxAxis0MaxVol->setMaximum(maxVol);
        ui.spinBoxAxis0MaxVol->setMinimum(minVol);
        ui.spinBoxAxis0MaxVol->setValue(vol[0]);

        ui.spinBoxAxis1MaxVol->setMaximum(maxVol);
        ui.spinBoxAxis1MaxVol->setMinimum(minVol);
        ui.spinBoxAxis1MaxVol->setValue(vol[1]);

        ui.spinBoxAxis2MaxVol->setMaximum(maxVol);
        ui.spinBoxAxis2MaxVol->setMinimum(minVol);
        ui.spinBoxAxis2MaxVol->setValue(vol[2]);

        ui.spinBoxAxis3MaxVol->setMaximum(maxVol);
        ui.spinBoxAxis3MaxVol->setMinimum(minVol);
        ui.spinBoxAxis3MaxVol->setValue(vol[3]);

        int *stepRate = params["stepRate"].getVal<int*>();
        int minStep = params["stepRate"].getMin();
        int maxStep = params["stepRate"].getMax();

        ui.spinBoxAxis0StepRate->setMaximum(maxStep);
        ui.spinBoxAxis0StepRate->setMinimum(minStep);
        ui.spinBoxAxis0StepRate->setValue(stepRate[0]);

        ui.spinBoxAxis1StepRate->setMaximum(maxStep);
        ui.spinBoxAxis1StepRate->setMinimum(minStep);
        ui.spinBoxAxis1StepRate->setValue(stepRate[1]);

        ui.spinBoxAxis2StepRate->setMaximum(maxStep);
        ui.spinBoxAxis2StepRate->setMinimum(minStep);
        ui.spinBoxAxis2StepRate->setValue(stepRate[2]);

        ui.spinBoxAxis3StepRate->setMaximum(maxStep);
        ui.spinBoxAxis3StepRate->setMinimum(minStep);
        ui.spinBoxAxis3StepRate->setValue(stepRate[3]);

        int *accel = params["acceleration"].getVal<int*>();
        int minAccel = params["acceleration"].getMin();
        int maxAccel = params["acceleration"].getMax();

        ui.spinBoxAxis0Accel->setMaximum(maxAccel);
        ui.spinBoxAxis0Accel->setMinimum(minAccel);
        ui.spinBoxAxis0Accel->setValue(accel[0]);

        ui.spinBoxAxis1Accel->setMaximum(maxAccel);
        ui.spinBoxAxis1Accel->setMinimum(minAccel);
        ui.spinBoxAxis1Accel->setValue(accel[1]);

        ui.spinBoxAxis2Accel->setMaximum(maxAccel);
        ui.spinBoxAxis2Accel->setMinimum(minAccel);
        ui.spinBoxAxis2Accel->setValue(accel[2]);

        ui.spinBoxAxis3Accel->setMaximum(maxAccel);
        ui.spinBoxAxis3Accel->setMinimum(minAccel);
        ui.spinBoxAxis3Accel->setValue(accel[3]);

        m_firstRun = false;
    }

    ui.spinTimeout->setValue(params["timeout"].getVal<int>());

    ui.checkAsync->setChecked(params["async"].getVal<int>());
    ui.checkDualChannel->setChecked(params["dualChannel"].getVal<int>());
    ui.checkFrontPanel->setChecked(params["lockFrontPanel"].getVal<int>());

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogThorlabsKCubeIM::applyParameters()
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

    int dualChannel = ui.checkDualChannel->isChecked() ? 1 : 0;
    if (dualChannel != m_currentParameters["dualChannel"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("dualChannel", ito::ParamBase::Int, dualChannel)));
    }

    double timeout = ui.spinTimeout->value();
    if (std::abs(timeout - m_currentParameters["timeout"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout", ito::ParamBase::Double, timeout)));
    }

    int newMaxVol[] = { 0, 0, 0, 0 };
    int newStepRate[] = { 0, 0, 0, 0 };
    int newAccel[] = { 0, 0, 0, 0 };
    for (int axis = 0; axis < m_numaxis; axis++)
    {
        switch (axis)
        {
        case 0:
            newMaxVol[axis] = ui.spinBoxAxis0MaxVol->value();
            newStepRate[axis] = ui.spinBoxAxis0StepRate->value();
            newAccel[axis] = ui.spinBoxAxis0Accel->value();
            break;
        case 1:
            newMaxVol[axis] = ui.spinBoxAxis1MaxVol->value();
            newStepRate[axis] = ui.spinBoxAxis1StepRate->value();
            newAccel[axis] = ui.spinBoxAxis1Accel->value();
            break;
        case 2:
            newMaxVol[axis] = ui.spinBoxAxis2MaxVol->value();
            newStepRate[axis] = ui.spinBoxAxis2StepRate->value();
            newAccel[axis] = ui.spinBoxAxis2Accel->value();
            break;
        case 3:
            newMaxVol[axis] = ui.spinBoxAxis3MaxVol->value();
            newStepRate[axis] = ui.spinBoxAxis3StepRate->value();
            newAccel[axis] = ui.spinBoxAxis3Accel->value();
            break;
        default:
            break;
        }

    }

    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("maxVoltage", ito::ParamBase::IntArray, 4, newMaxVol)));
    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stepRate", ito::ParamBase::IntArray, 4, newStepRate)));
    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("acceleration", ito::ParamBase::IntArray, 4, newAccel)));

    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsKCubeIM::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogThorlabsKCubeIM::enableDialog(bool enabled)
{
    ui.groupSettings->setEnabled(enabled);
    ui.groupMode->setEnabled(enabled);
    ui.btnCalib->setEnabled(enabled);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsKCubeIM::on_btnCalib_clicked()
{
    if (m_actuator)
    {
        int i;
        QVector<int> axis;

        for (i = 0; i<m_numaxis; i++)
        {
            axis << i;
        }

        enableDialog(false);
        ui.buttonBox->setEnabled(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_actuator, "calib", Q_ARG(QVector<int>, axis), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        ui.btnCalib->setVisible(false);
        observeInvocation(locker.getSemaphore(), ito::AbstractAddInConfigDialog::msgLevelWarningAndError);
        enableDialog(true);
        ui.buttonBox->setEnabled(true);
        ui.btnCalib->setVisible(true);
    }
}
