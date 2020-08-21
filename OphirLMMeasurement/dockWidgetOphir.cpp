/* ********************************************************************
Plugin "OphirLMMeasurement" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2020, Institut fuer Technische Optik (ITO),
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

#include "dockWidgetOphir.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetOphir::DockWidgetOphir(ito::AddInDataIO *flipper) :
    AbstractAddInDockWidget(flipper),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphir::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        ui.lblDeviceName->setText(params["deviceName"].getVal<char*>());

        m_firstRun = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphir::identifierChanged(const QString &identifier)
{
    ui.lblIdentifier->setText(identifier);
}
