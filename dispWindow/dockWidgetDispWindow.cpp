/* ********************************************************************
    Plugin "dispWindow" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut für Technische Optik (ITO),
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

#include "dockWidgetDispWindow.h"
#include "dispWindow.h"

#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------
DockWidgetDispWindow::DockWidgetDispWindow(ito::AddInDataIO* dispWindow) :
    AbstractAddInDockWidget(dispWindow), m_inEditing(false), m_firstRun(true),
    m_curNumPhaseShifts(-1), m_curNumGrayCodes(-1), m_numimgChangeInProgress(false)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------
void DockWidgetDispWindow::parametersChanged(QMap<QString, ito::Param> params)
{
    int tempNumPhaseShifts = params["phaseshift"].getVal<int>();
    int tempNumGrayCodes = params["numgraybits"].getVal<int>();

    if (m_firstRun || tempNumPhaseShifts != m_curNumPhaseShifts ||
        tempNumGrayCodes != m_curNumGrayCodes)
    {
        m_curNumPhaseShifts = tempNumPhaseShifts;
        m_curNumGrayCodes = tempNumGrayCodes;

        m_firstRun = false;

        m_curNumPhaseShifts = params["phaseshift"].getVal<int>();
        m_curNumGrayCodes = params["numgraybits"].getVal<int>();

        ui.comboBox->clear();

        for (int x = 0; x < tempNumPhaseShifts; x++)
        {
            ui.comboBox->addItem(tr("phase shift %1").arg(x + 1), 0);
        }

        ui.comboBox->addItem(tr("black"), 0);
        ui.comboBox->addItem(tr("white"), 0);

        for (int x = 0; x < tempNumGrayCodes; x++)
        {
            ui.comboBox->addItem(tr("gray images %1").arg(x + 1), 0);
        }
    }

    if (!m_inEditing)
    {
        m_inEditing = true;
        // check the value of all given parameters and adjust your widgets according to them (value
        // only should be enough)

        ui.comboBox->setCurrentIndex(params["numimg"].getVal<int>());

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------
void DockWidgetDispWindow::on_comboBox_currentIndexChanged(int index)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("numimg", ito::ParamBase::Int, index));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------
void DockWidgetDispWindow::identifierChanged(const QString& identifier)
{
    ui.lblID->setText(identifier);
}
