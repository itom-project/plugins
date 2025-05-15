/* ********************************************************************
    Plugin "ThorlabsDMH" for itom software
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

#include "dialogThorlabsDMH.h"

#include "common/addInInterface.h"

#include "paramEditorWidget.h"
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qvector.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsDMH::DialogThorlabsDMH(ito::AddInActuator* actuator) :
    AbstractAddInConfigDialog(actuator), m_firstRun(true), m_pluginPointer(actuator)
{
    ui.setupUi(this);
    enableDialog(false);
    ui.paramEditor->setPlugin(m_pluginPointer);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsDMH::parametersChanged(QMap<QString, ito::Param> params)
{
    // save the currently set parameters to m_currentParameters
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(
            QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
        m_firstRun = false;
        enableDialog(true);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogThorlabsDMH::applyParameters()
{
    QVector<QSharedPointer<ito::ParamBase>> values = ui.paramEditor->getAndResetChangedParameters();
    return setPluginParameters(values, msgLevelWarningAndError);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsDMH::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogThorlabsDMH::enableDialog(bool enabled)
{
    ui.paramEditor->setEnabled(enabled);
}
