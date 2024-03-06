/* ********************************************************************
    Plugin "FaulhaberMCS" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
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

#ifndef DOCKWIDGETMYACTUATOR_H
#define DOCKWIDGETMYACTUATOR_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>
#include <qwidget.h>

#include "ui_dockWidgetFaulhaberMCS.h"

class DockWidgetFaulhaberMCS : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

public:
    DockWidgetFaulhaberMCS(ito::AddInActuator* actuator);
    ~DockWidgetFaulhaberMCS(){};

private:
    Ui::DockWidgetFaulhaberMCS ui; //! Handle to the ui
    bool m_inEditing;
    bool m_firstRun;

    void enableWidgets(bool enabled);

    QVector<QPushButton*> m_btnRelDec;
    QVector<QPushButton*> m_btnRelInc;
    QVector<QDoubleSpinBox*> m_spinCurrentPos;
    QVector<QDoubleSpinBox*> m_spinTargetPos;
    QVector<QLabel*> m_labels;


public slots:
    void parametersChanged(QMap<QString, ito::Param> params);
    void identifierChanged(const QString& identifier);

    void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition);
    void targetChanged(QVector<double> targetPositions);

private slots:

    void btnRelDecClicked(); // slot if any button for a relative, negative movement is clicked
    void btnRelIncClicked(); // slot if any button for a relative, positive movement is clicked

    void on_btnRefresh_clicked(); // slot if the refresh button is clicked
    void on_btnStart_clicked(); // slot if the start button is clicked
    void on_btnStop_clicked(); // slot if the stop button is clicked
};

#endif
