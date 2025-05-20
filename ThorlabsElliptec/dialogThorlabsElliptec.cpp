/* ********************************************************************
    Plugin "ThorlabsElliptec" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2025, Institut für Technische Optik (ITO),
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

#include "dialogThorlabsElliptec.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qvector.h>
#include <qmessagebox.h>

#include "paramEditorWidget.h"

//------------------------------------------------------------------------------
DialogThorlabsElliptec::DialogThorlabsElliptec(
    ito::AddInActuator* actuator, bool allowCleaning, bool allowOptimization) :
    AbstractAddInConfigDialog(actuator), m_firstRun(true), m_pluginPointer(actuator)
{
    ui.setupUi(this);
    enableDialog(false);
    ui.paramEditor->setPlugin(m_pluginPointer);
    ui.cmdCancelCleaning->setEnabled(false);
    ui.progressBar->setVisible(false);
    ui.lblProgress->setVisible(false);
    ui.cmdCleanMechanics->setVisible(allowCleaning);
    ui.cmdOptimizeMotors->setVisible(allowOptimization);
};

//------------------------------------------------------------------------------
DialogThorlabsElliptec::~DialogThorlabsElliptec() {};

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(
            QString((params)["name"].getVal<const char*>()) + " - " + tr("Configuration Dialog"));
        m_firstRun = false;
        enableDialog(true);

        QString modelName = params["model"].getVal<const char*>();
        int numMotors = params["numMotors"].getVal<int>();

        ui.btnSearch1->setVisible(numMotors >= 1);
        ui.btnSearch2->setVisible(numMotors >= 2);
    }
}

//------------------------------------------------------------------------------
ito::RetVal DialogThorlabsElliptec::applyParameters()
{
    QVector<QSharedPointer<ito::ParamBase>> values = ui.paramEditor->getAndResetChangedParameters();
    return setPluginParameters(values, msgLevelWarningAndError);
}

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::on_buttonBox_clicked(QAbstractButton* btn)
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

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::on_cmdOptimizeMotors_clicked()
{
    ito::RetVal retValue;
    QSharedPointer<QVector<ito::ParamBase>> _dummy;

    enableDialog(false);
    ui.cmdCancelCleaning->setEnabled(true);
    ui.lblProgress->setText("Cleaning and optimizing motors is running. Please wait for 20-40 minutes.");
    ui.progressBar->setVisible(true);
    ui.lblProgress->setVisible(true);
    ui.buttonBox->setDisabled(true);

    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    QMetaObject::invokeMethod(
        m_pluginPointer.data(),
        "execFunc",
        Q_ARG(QString, "optimizeMotors"),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

    retValue += observeInvocation(locker.getSemaphore());

    ui.buttonBox->setEnabled(true);
    enableDialog(true);
    ui.cmdCancelCleaning->setEnabled(false);
    ui.progressBar->setVisible(false);
    ui.lblProgress->setVisible(false);
}

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::on_cmdCleanMechanics_clicked()
{
    ito::RetVal retValue;
    QSharedPointer<QVector<ito::ParamBase>> _dummy;

    enableDialog(false);
    ui.cmdCancelCleaning->setEnabled(true);
    ui.lblProgress->setText("Cleaning mechanics is running. Please wait for 20-40 minutes.");
    ui.progressBar->setVisible(true);
    ui.lblProgress->setVisible(true);
    ui.buttonBox->setDisabled(true);

    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    QMetaObject::invokeMethod(
        m_pluginPointer.data(),
        "execFunc",
        Q_ARG(QString, "cleanMechanics"),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

    retValue += observeInvocation(locker.getSemaphore());

    ui.buttonBox->setEnabled(true);
    enableDialog(true);
    ui.cmdCancelCleaning->setEnabled(false);
    ui.progressBar->setVisible(false);
    ui.lblProgress->setVisible(false);
}

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::on_cmdCancelCleaning_clicked()
{
    QSharedPointer<QVector<ito::ParamBase>> _dummy;
    ito::AddInActuator* actuator = qobject_cast<ito::AddInActuator*>(m_pluginPointer.data());

    if (actuator)
    {
        ui.lblProgress->setText("Cancellation in progress. Please wait for 2-3 seconds...");
        actuator->setInterrupt();
    }
}

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::on_cmdSaverUserData_clicked()
{
    QSharedPointer<QVector<ito::ParamBase>> _dummy;
    m_pluginPointer->execFunc("saveUserData", _dummy, _dummy, _dummy, nullptr);
}

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::enableDialog(bool enabled)
{
    ui.paramEditor->setEnabled(enabled);
    ui.cmdOptimizeMotors->setEnabled(enabled);
    ui.cmdSaveUserData->setEnabled(enabled);
    ui.cmdCleanMechanics->setEnabled(enabled);
    ui.cmdHome->setEnabled(enabled);
}

//------------------------------------------------------------------------------
ito::RetVal DialogThorlabsElliptec::observeInvocation(ItomSharedSemaphore* waitCond) const
{
    ito::RetVal retval;
    bool timeout = false;
    int aliveCounter = 20; // check alive every 10 seconds
    bool alive;

    while (!timeout && waitCond->waitAndProcessEvents(500) == false)
    {
        aliveCounter--;

        // alive is interally reset once it is requested. Therefore the or operator...
        alive |= (m_pluginPointer->isAlive() > 0); 
        
        if (aliveCounter <= 0 && alive == false) 
        {
            retval += ito::RetVal(
                ito::retError,
                0,
                tr("Timeout while waiting for answer from plugin instance.").toLatin1().data());
            timeout = true;
        }
        else if (aliveCounter < 0)
        {
            aliveCounter = 20;
            alive = false;
        }
    }

    if (!timeout)
    {
        retval += waitCond->returnValue;
    }

    if (retval.containsError())
    {
        QMessageBox msgBox;
        msgBox.setText(tr("Error while execution"));
        if (retval.hasErrorMessage())
        {
            msgBox.setInformativeText(QLatin1String(retval.errorMessage()));
        }
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.exec();
    }
    else if (retval.containsWarning())
    {
        QMessageBox msgBox;
        msgBox.setText(tr("Warning while execution"));
        if (retval.hasErrorMessage())
        {
            msgBox.setInformativeText(QLatin1String(retval.errorMessage()));
        }
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();
    }

    return retval;
}

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::on_cmdHome_clicked()
{
    ito::RetVal retValue;
    QSharedPointer<QVector<ito::ParamBase>> _dummy;

    enableDialog(false);
    ui.lblProgress->setText("Drive motor to home (zero) position.");
    ui.progressBar->setVisible(true);
    ui.lblProgress->setVisible(true);
    ui.buttonBox->setDisabled(true);

    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    QMetaObject::invokeMethod(
        m_pluginPointer.data(),
        "calib",
        Q_ARG(int, 0),
        Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

    retValue += observeInvocation(locker.getSemaphore());

    ui.buttonBox->setEnabled(true);
    enableDialog(true);
    ui.progressBar->setVisible(false);
    ui.lblProgress->setVisible(false);
}

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::on_btnResetDefaults_clicked()
{
    ito::RetVal retValue;
    QSharedPointer<QVector<ito::ParamBase>> _dummy;

    enableDialog(false);
    ui.lblProgress->setText("Reset motor settings to defaults.");
    ui.progressBar->setVisible(true);
    ui.lblProgress->setVisible(true);
    ui.buttonBox->setDisabled(true);

    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    QMetaObject::invokeMethod(
        m_pluginPointer.data(),
        "execFunc",
        Q_ARG(QString, "resetDefaults"),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

    retValue += observeInvocation(locker.getSemaphore());

    ui.buttonBox->setEnabled(true);
    enableDialog(true);
    ui.progressBar->setVisible(false);
    ui.lblProgress->setVisible(false);
}

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::on_btnSearch2_clicked()
{
    ito::RetVal retValue;
    QSharedPointer<QVector<ito::ParamBase>> _dummy;
    QSharedPointer<QVector<ito::ParamBase>> mand(new QVector<ito::ParamBase>());
    mand->append(ito::ParamBase("motorIdx", ito::ParamBase::Int, 1));

    enableDialog(false);
    ui.lblProgress->setText("Search optimal motor frequencies for motor stage 2.");
    ui.progressBar->setVisible(true);
    ui.lblProgress->setVisible(true);
    ui.buttonBox->setDisabled(true);

    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    QMetaObject::invokeMethod(
        m_pluginPointer.data(),
        "execFunc",
        Q_ARG(QString, "searchFrequencies"),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, mand),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

    retValue += observeInvocation(locker.getSemaphore());

    ui.buttonBox->setEnabled(true);
    enableDialog(true);
    ui.progressBar->setVisible(false);
    ui.lblProgress->setVisible(false);
}

//------------------------------------------------------------------------------
void DialogThorlabsElliptec::on_btnSearch1_clicked()
{
    ito::RetVal retValue;
    QSharedPointer<QVector<ito::ParamBase>> _dummy;
    QSharedPointer<QVector<ito::ParamBase>> mand(new QVector<ito::ParamBase>());
    mand->append(ito::ParamBase("motorIdx", ito::ParamBase::Int, 0));

    enableDialog(false);
    ui.lblProgress->setText("Search optimal motor frequencies for motor stage 1.");
    ui.progressBar->setVisible(true);
    ui.lblProgress->setVisible(true);
    ui.buttonBox->setDisabled(true);

    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    QMetaObject::invokeMethod(
        m_pluginPointer.data(),
        "execFunc",
        Q_ARG(QString, "searchFrequencies"),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, mand),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(QSharedPointer<QVector<ito::ParamBase>>, _dummy),
        Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

    retValue += observeInvocation(locker.getSemaphore());

    ui.buttonBox->setEnabled(true);
    enableDialog(true);
    ui.progressBar->setVisible(false);
    ui.lblProgress->setVisible(false);
}
