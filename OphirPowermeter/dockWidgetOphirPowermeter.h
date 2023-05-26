/* ********************************************************************
    Plugin "OphirSerialPlugin" for itom software
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

#ifndef DOCKWIDGETOPHIRPOWERMETER_H
#define DOCKWIDGETOPHIRPOWERMETER_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/param.h"
#include "common/retVal.h"

#include <qmap.h>
#include <qstring.h>
#include <qabstractbutton.h>

#include "ui_dockWidgetOphirPowermeter.h"

#include <QtGui>
#include <qwidget.h>
#include <qsharedpointer.h>
#include <qmetaobject.h>
#include <qstackedwidget.h>

class DockWidgetOphirPowermeter : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    protected:
        void timerEvent(QTimerEvent *event);

    public:
        DockWidgetOphirPowermeter(int uniqueID, ito::AddInDataIO *adda);
        ~DockWidgetOphirPowermeter();

    private:
        void enableWidget(bool enabled);

        Ui::dockWidgetOphirPowermeter ui;
        ito::AddInDataIO *m_plugin;

        bool m_inEditing;
        bool m_firstRun;

        int m_timerId;
        bool m_timerIsRunning;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);
        void manageTimer(const bool &val);

    private slots:
        void on_checkAutograbbing_stateChanged(int val);
        void on_spinBoxWavelength_valueChanged(double val);
        void on_comboBoxWavelength_currentIndexChanged(int val);
        void on_comboBoxRange_currentIndexChanged(int val);
        void on_comboBoxMeasurementType_currentIndexChanged(int val);
};

#endif
