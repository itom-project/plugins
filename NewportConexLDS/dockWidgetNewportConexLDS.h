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

#ifndef DOCKWIDGETMYGRABBER_H
#define DOCKWIDGETMYGRABBER_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>
#include <qwidget.h>

#include "ui_dockWidgetNewportConexLDS.h"

class DockWidgetNewportConexLDS : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

protected:
    void timerEvent(QTimerEvent* event);

public:
    DockWidgetNewportConexLDS(int uniqueID, ito::AddInDataIO* rawIO);
    ~DockWidgetNewportConexLDS();

private:
    Ui::DockWidgetNewportConexLDS ui;
    ito::AddInDataIO* m_plugin;
    bool m_inEditing;
    bool m_firstRun;

    int m_timerId;
    bool m_timerIsRunning;

public slots:
    void parametersChanged(QMap<QString, ito::Param> params);
    void identifierChanged(const QString& identifier);

private slots:
    // add here slots connected to changes of any widget
    // example:
    // void on_contrast_valueChanged(int i);
    void on_btnLaserPower_toggled(bool state);
};

#endif
