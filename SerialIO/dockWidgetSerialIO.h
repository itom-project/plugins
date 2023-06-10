/* ********************************************************************
    Plugin "SerialIO" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#ifndef DOCKWIDGETSERIALIO_H
#define DOCKWIDGETSERIALIO_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include "ui_dockWidgetSerialIO.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>
#include <qbytearray.h>

class DockWidgetSerialIO : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetSerialIO(ito::AddInDataIO *dataIO);
        ~DockWidgetSerialIO() {};

    private:
        Ui::DockWidgetSerialIO ui;
        bool m_inEditing;

    signals:

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
//        void valuesChanged(QMap<QString, ito::Param> params);
//        void uniqueIDChanged(const int uniqueID);
        void serialLog(QByteArray data, QByteArray endline, const char InOutChar);
        void identifierChanged(const QString &identifier);

    private slots:
        void on_ClrButton_clicked();
        void on_checkIgnoreEmpty_clicked();
};

#endif
