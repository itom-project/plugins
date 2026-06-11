/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
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

#include "dockWidgetNI-DAQmx.h"


#include <qmetaobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetNIDAQmx::DockWidgetNIDAQmx(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);

    QPointer<ito::AddInBase> plugin(grabber);

    if (ui.editorWidget->property("collapsed").isValid())
    {
        ui.editorWidget->setProperty("collapsed", false);
    }

    ui.editorWidget->setPlugin(plugin);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNIDAQmx::identifierChanged(const QString &identifier)
{
    ui.lblName->setText(identifier);
    ui.editorWidget->refresh();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNIDAQmx::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.lblType->setText(params["taskType"].getVal<const char*>());
    ui.lblMode->setText(params["taskMode"].getVal<const char*>());

    ui.listChannels->clear();
    QStringList channels = QString(params["channels"].getVal<const char*>()).split(";");

    foreach(const QString &c, channels)
    {
        ui.listChannels->addItem(c.split(",")[0]);
    }

    ui.ledRunning->setChecked(params["taskStarted"].getVal<int>() > 0);
    ui.ledConfigured->setChecked(params["taskConfigured"].getVal<int>() > 0);
    ui.ledLogging->setChecked(params["loggingActive"].getVal<int>() > 0);

    QByteArray taskType = params["taskType"].getVal<const char*>();
    ui.ledLogging->setEnabled(taskType.endsWith("Input"));
    ui.lblLogging->setEnabled(taskType.endsWith("Input"));
}
