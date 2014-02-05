/* ********************************************************************
    Plugin "PIPiezoControl" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

#ifndef DOCKWIDGETPIPIEZOCTRL_H
#define DOCKWIDGETPIPIEZOCTRL_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qsharedpointer.h>

#include "ui_dockWidgetPIPiezoCtrl.h"

namespace ito
{
    class AddInActuator; //forward declaration
}

class DockWidgetPIPiezoCtrl : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetPIPiezoCtrl(int uniqueID, ito::AddInActuator *actuator);
        ~DockWidgetPIPiezoCtrl() {};

    private:
        void enableWidget(bool enabled);
        void waitForDoneAndCheckRetVal(ItomSharedSemaphore *waitCond);

        ito::AddInActuator *m_pPlugin;

        Ui::DockWidgetPIPiezoCtrl ui;

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);
        void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition); //!< slot to receive information about status and position changes.
        void targetChanged(QVector<double> targetPositions);

    private slots:
        void on_radioLocal_clicked();
        void on_radioRemote_clicked() { on_radioLocal_clicked(); }
        void on_btnUp_clicked();
        void on_btnDown_clicked();
        void on_btnStart_clicked();
        void on_btnRefresh_clicked();
};

#endif
