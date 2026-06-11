/* ********************************************************************
Plugin "GenICam" for itom software
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

#include "dialogGenicam.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

#include "common/addInInterface.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogGenicam::DialogGenicam(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true),
    m_pluginPointer(grabber)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);

    ui.paramEditorWidget->setPlugin(m_pluginPointer);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogGenicam::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
        m_firstRun = false;
    }

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogGenicam::applyParameters()
{
    QVector<QSharedPointer<ito::ParamBase> > values = ui.paramEditorWidget->getAndResetChangedParameters();

    return setPluginParameters(values, msgLevelWarningAndError);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogGenicam::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogGenicam::enableDialog(bool enabled)
{
    ui.paramEditorWidget->setEnabled(enabled);
}
