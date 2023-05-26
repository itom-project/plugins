/* ********************************************************************
    Plugin "USBMotion3XIII" for itom software
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

#ifndef DOCKWIDGETUSBMOTION3XIII_H
#define DOCKWIDGETUSBMOTION3XIII_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetUSBMotion3XIII.h"

class DockWidgetUSBMotion3XIII : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetUSBMotion3XIII(ito::AddInActuator *actuator);
        ~DockWidgetUSBMotion3XIII() {};

    private:
        Ui::DockWidgetUSBMotion3XIII ui;
        bool m_inEditing;
        bool m_firstRun;

        void enableWidget(bool enabled);

        bool m_axisEnabled[3];

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);
        void basicInformationChanged(const QString &axis, const int *axisUnits);

        void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition);
        void targetChanged(QVector<double> targetPositions);

    private slots:
        void on_btn_relPlus1_clicked();
        void on_btn_relMinus1_clicked();
        void on_btn_relPlus2_clicked();
        void on_btn_relMinus2_clicked();
        void on_btn_relPlus3_clicked();
        void on_btn_relMinus3_clicked();
        void on_btnStartAbsolute_clicked();
        void on_btnStop_clicked();
};

#endif
