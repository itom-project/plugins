/* ********************************************************************
    Plugin "NewportConexLDS" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
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

#include "dialogNewportConexLDS.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qvector.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogNewportConexLDS::DialogNewportConexLDS(ito::AddInBase* rawIO) :
    AbstractAddInConfigDialog(rawIO), m_firstRun(true), m_pluginPointer(rawIO)
{
    ui.setupUi(this);

    // disable dialog, since no parameters are known yet. Parameters will immediately be sent by the
    // slot parametersChanged.
    enableDialog(false);

    ui.paramEditorWidget->setPlugin(m_pluginPointer);
}

//----------------------------------------------------------------------------------------------------------------------------------
DialogNewportConexLDS::~DialogNewportConexLDS()
{
    QSharedPointer<ito::ParamBase> p(
        new ito::ParamBase("enableConfiguration", ito::ParamBase::Int, 0));
    setPluginParameter(p, msgLevelWarningAndError);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogNewportConexLDS::parametersChanged(QMap<QString, ito::Param> params)
{
    // save the currently set parameters to m_currentParameters
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(
            QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        QString config = params["configurationState"].getVal<char*>();
        m_firstRun = false;

        if (config != "CONFIGURATION")
        {
            enableDialog(false);
        }
        else
        {
            enableDialog(true);
        }
    }

    // set the status of all widgets depending on the values of params
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogNewportConexLDS::applyParameters()
{
    QVector<QSharedPointer<ito::ParamBase>> values =
        ui.paramEditorWidget->getAndResetChangedParameters();

    return setPluginParameters(values, msgLevelWarningAndError);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNewportConexLDS::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogNewportConexLDS::enableDialog(bool enabled)
{
    // e.g.
    ui.paramEditorWidget->setEnabled(enabled);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNewportConexLDS::on_btnConfig_clicked()
{
    QSharedPointer<ito::ParamBase> p1(
        new ito::ParamBase("laserPowerState", ito::ParamBase::Int, 0));
    setPluginParameter(p1, msgLevelWarningAndError);
    QSharedPointer<ito::ParamBase> p2(
        new ito::ParamBase("enableConfiguration", ito::ParamBase::Int, 1));
    setPluginParameter(p2, msgLevelWarningAndError);

    enableDialog(true);
}
