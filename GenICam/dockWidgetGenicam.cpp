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

#include "dockWidgetGenicam.h"


#include <qmetaobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetGenicam::DockWidgetGenicam(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);

    QPointer<ito::AddInBase> plugin(grabber);

    if (ui.editorWidget->property("collapsed").isValid())
    {
        ui.editorWidget->setProperty("collapsed", true);
    }

    ui.editorWidget->setPlugin(plugin);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetGenicam::identifierChanged(const QString &identifier)
{
    ui.label_Identifier->setText(identifier);
    ui.editorWidget->refresh();
}
