/* ********************************************************************
    Plugin "PCOCamera" for itom software
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

#ifndef DOCKWIDGETPCOCAMERA_H
#define DOCKWIDGETPCOCAMERA_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetPCOCamera.h"

class DockWidgetPCOCamera : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetPCOCamera(ito::AddInDataIO *grabber);
        ~DockWidgetPCOCamera() {};

    private:
        Ui::DockWidgetPCOCamera ui;
        bool m_inEditing;
        bool m_firstRun;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:
        void on_spinBox_offset_valueChanged(int d);
        void on_spinBox_gain_valueChanged(int d);
        void on_doubleSpinBox_integration_time_valueChanged(double d);
};

#endif //DOCKWIDGETPCOCAMERA_H
