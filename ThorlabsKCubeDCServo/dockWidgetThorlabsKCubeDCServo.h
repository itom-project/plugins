///* ********************************************************************
//    Plugin "ThorlabsKCubeDCServo" for itom software
//    URL: http://www.uni-stuttgart.de/ito
//    Copyright (C) 2021, Institut fuer Technische Optik (ITO),
//    Universitaet Stuttgart, Germany
//
//    This file is part of a plugin for the measurement software itom.
//
//    This itom-plugin is free software; you can redistribute it and/or modify it
//    under the terms of the GNU Library General Public Licence as published by
//    the Free Software Foundation; either version 2 of the Licence, or (at
//    your option) any later version.
//
//    itom and its plugins are distributed in the hope that it will be useful, but
//    WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
//    General Public Licence for more details.
//
//    You should have received a copy of the GNU Library General Public License
//    along with itom. If not, see <http://www.gnu.org/licenses/>.
//*********************************************************************** */

#pragma once

#define NOMINMAX // https://stackoverflow.com/questions/22744262/cant-call-stdmax-because-minwindef-h-defines-max

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetThorlabsKCubeDCServo.h"

class DockWidgetThorlabsKCubeDCServo : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetThorlabsKCubeDCServo(ito::AddInActuator * myPlugin);
        ~DockWidgetThorlabsKCubeDCServo() {};

    private:
        void enableWidget(bool enabled);
        bool m_firstRun;
        ito::AddInActuator *m_pActuator;

        Ui::DockWidgetThorlabsKCubeDCServo ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier) { };
        void dockWidgetVisibilityChanged(bool visible);

        void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition);
        void targetChanged(QVector<double> targetPositions);
        void on_btnHome_clicked();
        void on_btnHomeCancel_clicked();
};
