/* ********************************************************************
    Plugin "LeicaMotorFocus" for itom software
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

#ifndef DOCKWIDGETLEICAMOTORFOCUS_H
#define DOCKWIDGETLEICAMOTORFOCUS_H

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetLeicaMotorFocus.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

namespace ito {
    class AddInActuator;
}

class DockWidgetLeicaMotorFocus : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetLeicaMotorFocus(QMap<QString, ito::Param> params, int uniqueID,ito::AddInActuator *actuator);
        ~DockWidgetLeicaMotorFocus() {};

    private:
        Ui::DockWidgetLeicaMotorFocus ui;

        ito::AddInActuator *m_actuator;

        void setMotorStatus(bool busy);

    signals:
        void MoveRelative(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        void MoveAbsolute(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        void MotorTriggerStatusRequest(bool sendActPosition, bool sendTargetPos);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);

        void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition); //!< slot to receive information about status and position changes.
        void targetChanged(QVector<double> targetPositions);

    private slots:
        void on_btnDown_clicked();
        void on_btnUp_clicked();
        void on_btnStart_clicked();
        void on_btnStop_clicked();
        void on_btnRefresh_clicked();

};

#endif
