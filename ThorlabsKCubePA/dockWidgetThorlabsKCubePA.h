/* ********************************************************************
Plugin "ThorlabsKCubePA" for itom software
Copyright (C) 2018, TRUMPF Laser- & Systemtechnik GmbH, Ditzingen

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

#ifndef DOCKWIDGETTHORLABSISM_H
#define DOCKWIDGETTHORLABSISM_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetThorlabsKCubePA.h"

class DockWidgetThorlabsKCubePA : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetThorlabsKCubePA(ito::AddInDataIO *grabber);
        ~DockWidgetThorlabsKCubePA() {};

    private:
        Ui::DockWidgetThorlabsKCubePA ui;
        bool m_inEditing;
        bool m_firstRun;

    public slots:
        void identifierChanged(const QString &identifier);
        void parametersChanged(QMap<QString, ito::Param> params) {}


};

#endif
