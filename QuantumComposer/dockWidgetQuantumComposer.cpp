/* ********************************************************************
    Plugin "QuantumComposer" for itom software
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

#include "dockWidgetQuantumComposer.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetQuantumComposer::DockWidgetQuantumComposer(ito::AddInDataIO* dataIO) :
    AbstractAddInDockWidget(dataIO),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetQuantumComposer::parametersChanged(QMap<QString, ito::Param> params)
 {
    ui.lblManu->setText(params["manufacturer"].getVal<char*>());
    ui.lblModel->setText(params["model"].getVal<char*>());
    ui.lblVersion->setText(params["version"].getVal<char*>());
    ui.lblSerialNo->setText(params["serialNumber"].getVal<char*>());

    if (m_firstRun)
    {
        ui.radioButtonOutput->setChecked(bool(params["state"].getVal<int>()));
        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;
        int state = params["state"].getVal<int>();
        bool boolState = bool(state);
        ui.radioButtonOutput->setChecked(boolState);

        m_inEditing = false;
    }
 }

//----------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetQuantumComposer::identifierChanged(const QString& identifier)
{
    ui.lblID->setText(identifier);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetQuantumComposer::on_radioButtonOutput_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(
            new ito::ParamBase("state", ito::ParamBase::Int, int(checked)));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}
