/* ********************************************************************
    Plugin "MyCobot280Pi" for itom software
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

/**\file dialogMyCobot280Pi.cpp
* \brief In this file, the functions of the modal dialog for the MyCobot280Pi are specified
*
*    This file defines the functions of the DialogMyCobot280Pi class defined in the file "dialogMyCobot280Pi.h"
*
*\sa DialogMyCobot280Pi, MyCobot280Pi
*\author Wolfram Lyda
*\date    Oct2011
*/

#include "dialogMyCobot280Pi.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qmetaobject.h>

#include "paramEditorWidget.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogMyCobot280Pi::DialogMyCobot280Pi(ito::AddInBase* actuator) :
    AbstractAddInConfigDialog(actuator), m_actuator(actuator), m_firstRun(true), m_numaxis(0),
    m_pluginPointer(actuator)
{
    ui.setupUi(this);
    memset(m_enable, 0, 10 * sizeof(int));

    // Disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableGUI(false);
    ui.paramEditor->setPlugin(m_pluginPointer);
}

//----------------------------------------------------------------------------------------------------------------------------------
DialogMyCobot280Pi::~DialogMyCobot280Pi()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogMyCobot280Pi::parametersChanged(QMap<QString, ito::Param> params)
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
        ui.checkBox_EnableC->setEnabled(true);
        ui.checkBox_EnableC->setChecked(true);
    case 5:
        ui.checkBox_EnableB->setEnabled(true);
        ui.checkBox_EnableB->setChecked(true);
    case 4:
        ui.checkBox_EnableA->setEnabled(true);
        ui.checkBox_EnableA->setChecked(true);
    case 3:
        ui.checkBox_EnableZ->setEnabled(true);
        ui.checkBox_EnableZ->setChecked(true);
    case 2:
        ui.checkBox_EnableY->setEnabled(true);
        ui.checkBox_EnableY->setChecked(true);
    case 1:
        ui.checkBox_EnableX->setEnabled(true);
        ui.checkBox_EnableX->setChecked(true);
    }

    // Now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableGUI(true);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogMyCobot280Pi::applyParameters()
{
    QVector<QSharedPointer<ito::ParamBase>> values = ui.paramEditor->getAndResetChangedParameters();
    return setPluginParameters(values, msgLevelWarningAndError);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogMyCobot280Pi::on_pushButtonCalib_clicked()
{
    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    int i;
    QVector<int> axis;

    for(i = 0; i < m_numaxis; i++)
    {
        if (m_enable[i])
            axis << i;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);
    enableGUI(false);
    ui.buttonBox->setDisabled(true);
    QMetaObject::invokeMethod(m_actuator, "calib", Q_ARG(QVector<int>, axis), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

    observeInvocation(locker.getSemaphore(), msgLevelWarningAndError);
    ui.buttonBox->setEnabled(true);
    enableGUI(true);
    QApplication::restoreOverrideCursor();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogMyCobot280Pi::on_checkBox_EnableX_clicked()
{
    m_enable[0] = ui.checkBox_EnableX->isChecked();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogMyCobot280Pi::on_checkBox_EnableY_clicked()
{
    m_enable[1] = ui.checkBox_EnableY->isChecked();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogMyCobot280Pi::on_checkBox_EnableZ_clicked()
{
    m_enable[2] = ui.checkBox_EnableZ->isChecked();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogMyCobot280Pi::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); // Close dialog with reject
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

//----------------------------------------------------------------------------------------------------------------------------------
void DialogMyCobot280Pi::enableGUI(bool enabled)
{
    ui.paramEditor->setEnabled(enabled);
}
