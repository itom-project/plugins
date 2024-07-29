/* ********************************************************************
    Plugin "DummyMotor" for itom software
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

/**\file dialogDummyMotor.cpp
* \brief In this file the functions of the modal dialog for the DummyMotor are specified
*
*    This file defines the functions of the dialogDummyMotor-Class defined in the file "dialogDummyMotor.h"
*
*\sa dialogDummyMotor, DummyMotor
*\author Wolfram Lyda
*\date    Oct2011
*/

#include "dialogDummyMotor.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qmetaobject.h>

#include "paramEditorWidget.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogDummyMotor::DialogDummyMotor(ito::AddInBase* actuator) :
    AbstractAddInConfigDialog(actuator), m_actuator(actuator), m_firstRun(true), m_numaxis(0),
    m_pluginPointer(actuator)
{
    ui.setupUi(this);
    memset(m_enable, 0, 10 * sizeof(int));

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableGUI(false);
    ui.paramEditor->setPlugin(m_pluginPointer);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogDummyMotor::~DialogDummyMotor()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyMotor::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        m_numaxis = params["numaxis"].getVal<int>();
        setWindowTitle(QString(params["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        m_firstRun = false;
    }

    switch (m_numaxis)
    {
    case 6:
        ui.checkBox_EnableC->setEnabled(1);
        ui.checkBox_EnableC->setChecked(1);
    case 5:
        ui.checkBox_EnableB->setEnabled(1);
        ui.checkBox_EnableB->setChecked(1);
    case 4:
        ui.checkBox_EnableA->setEnabled(1);
        ui.checkBox_EnableA->setChecked(1);
    case 3:
        ui.checkBox_EnableZ->setEnabled(1);
        ui.checkBox_EnableZ->setChecked(1);
    case 2:
        ui.checkBox_EnableY->setEnabled(1);
        ui.checkBox_EnableY->setChecked(1);
    case 1:
        ui.checkBox_EnableX->setEnabled(1);
        ui.checkBox_EnableX->setChecked(1);
    }

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableGUI(true);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogDummyMotor::applyParameters()
{
    QVector<QSharedPointer<ito::ParamBase>> values = ui.paramEditor->getAndResetChangedParameters();
    return setPluginParameters(values, msgLevelWarningAndError);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyMotor::on_pushButtonCalib_clicked()
{
    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    int i;
    QVector<int> axis;

    for(i=0;i<m_numaxis;i++)
    {
        if (m_enable[i])
            axis << i;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);
    enableGUI(false);
    ui.buttonBox->setDisabled(true);
    QMetaObject::invokeMethod(m_actuator,"calib",Q_ARG(QVector<int>,axis),Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));

    observeInvocation(locker.getSemaphore(), msgLevelWarningAndError);
    ui.buttonBox->setEnabled(true);
    enableGUI(true);
    QApplication::restoreOverrideCursor();
}
//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyMotor::on_checkBox_EnableX_clicked()
{
    m_enable[0]=ui.checkBox_EnableX->isChecked();
}
//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyMotor::on_checkBox_EnableY_clicked()
{
    m_enable[1]=ui.checkBox_EnableY->isChecked();
}
//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyMotor::on_checkBox_EnableZ_clicked()
{
    m_enable[2] = ui.checkBox_EnableZ->isChecked();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyMotor::on_buttonBox_clicked(QAbstractButton* btn)
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

//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyMotor::enableGUI(bool enabled)
{
    ui.paramEditor->setEnabled(enabled);
}
