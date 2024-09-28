/* ********************************************************************
Plugin "ThorlabsFF" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2020, Institut für Technische Optik (ITO),
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

#include "dockWidgetThorlabsFF.h"
#include "Thorlabs.MotionControl.FilterFlipper.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetThorlabsFF::DockWidgetThorlabsFF(ito::AddInDataIO *flipper) :
    AbstractAddInDockWidget(flipper),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsFF::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        ui.lblDeviceName->setText(params["deviceName"].getVal<char*>());

        ui.sBoxTransitTime->setMaximum(params["transitTime"].getMax());
        ui.sBoxTransitTime->setMinimum(params["transitTime"].getMin());
        ui.sBoxTransitTime->setSingleStep(1);
        ui.sBoxTransitTime->setValue(params["transitTime"].getVal<int>());

        m_firstRun = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsFF::on_sBoxTransitTime_valueChanged(int i)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("transitTime",ito::ParamBase::Int, i));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsFF::identifierChanged(const QString &identifier)
{
    ui.lblIdentifier->setText(identifier);
}


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsFF::on_btnPos1_clicked()
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("position", ito::ParamBase::Int, FF_Positions::Position1));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsFF::on_btnPos2_clicked()
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("position", ito::ParamBase::Int, FF_Positions::Position2));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}
