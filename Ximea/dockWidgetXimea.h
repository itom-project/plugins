/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
    Copyright (C) 2018, Institut für Technische Optik, Universität Stuttgart

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

#ifndef DOCKWIDGETXIMEA_H
#define DOCKWIDGETXIMEA_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"
#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>
#include <qabstractbutton.h>
#include <qsharedpointer.h>
#include <qmetaobject.h>

#include "ui_dockWidgetXimea.h"

class DockWidgetXimea : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetXimea(int uniqueID, ito::AddInDataIO *grabber);
        ~DockWidgetXimea() {};

    private:
        Ui::DockWidgetXimea ui;
        void enableWidget(bool enabled);

        bool m_inEditing;
        bool m_firstRun;
        QMap<QString, ito::Param> m_currentParams;

        inline double msecToSec(double musec) { return (double)musec * 1.0e-3; }
        inline double secToMsec(double sec) { return (double)(sec * 1.0e3); }

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:
        void on_sliderWidget_offset_valueChanged(double value);
        void on_sliderWidget_gain_valueChanged(double value);
        void on_sliderWidget_integrationtime_valueChanged(double value);
};

#endif
