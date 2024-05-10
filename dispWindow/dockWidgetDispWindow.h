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

#ifndef DOCKWIDGETDISPWINDOW_H
#define DOCKWIDGETDISPWINDOW_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>
#include <qwidget.h>

#include "ui_dockWidgetDispWindow.h"


class DockWidgetDispWindow : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

public:
    DockWidgetDispWindow(ito::AddInDataIO* dispWindow);
    ~DockWidgetDispWindow()
    {
    }

private:
    Ui::DockWidgetDispWindow ui;

    int m_curNumPhaseShifts;
    int m_curNumGrayCodes;
    bool m_numimgChangeInProgress;

    bool m_inEditing;
    bool m_firstRun;

signals:

public slots:
    void parametersChanged(QMap<QString, ito::Param> params);
    void identifierChanged(const QString& identifier);

private slots:
    void on_comboBox_currentIndexChanged(int index);
};

#endif
