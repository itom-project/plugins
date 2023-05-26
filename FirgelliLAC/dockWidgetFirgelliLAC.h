/* ********************************************************************
    Plugin "FirgelliLAC" for itom software
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

#ifndef DOCKWIDGETFIRGELLILAC_H
#define DOCKWIDGETFIRGELLILAC_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetFirgelliLAC.h"

class DockWidgetFirgelliLAC : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetFirgelliLAC(ito::AddInActuator * myPlugin);
        ~DockWidgetFirgelliLAC() {};

    private:
        void enableWidget(bool enabled);

        Ui::DockWidgetFirgelliLAC ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier) { ui.lblID->setText(identifier); };

        void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition);
        void targetChanged(QVector<double> targetPositions);

    private slots:
        void on_btnUp_clicked();
        void on_btnDown_clicked();
        void on_btnStart_clicked();
        void on_btnRefresh_clicked();
        void on_btnCancel_clicked();
};

#endif
