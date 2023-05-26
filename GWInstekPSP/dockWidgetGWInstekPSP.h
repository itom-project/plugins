/* ********************************************************************
    Plugin "GWInstekPSP" for itom software
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

#ifndef DOCKWIDGETGWINSTEKPSP_H
#define DOCKWIDGETGWINSTEKPSP_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qsharedpointer.h>
#include <qevent.h>

#include "ui_dockWidgetGWInstekPSP.h"

class DockWidgetGWInstekPSP : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetGWInstekPSP(QMap<QString, ito::Param> params, int uniqueID);
        ~DockWidgetGWInstekPSP() {};

    protected:
        void timerEvent(QTimerEvent *event);

    private:
        Ui::DockWidgetGWInstekPSP ui;
        int m_timerID;

    signals:
        void setParamVoltage(const double voltage);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_cbAuto_clicked();
        void on_pbStart_clicked();
};

#endif
