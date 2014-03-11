/* ********************************************************************
    Plugin "DummyGrabber" for itom software
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

#ifndef DOCKWIDGETMSMEDIAFOUNDATION_H
#define DOCKWIDGETMSMEDIAFOUNDATION_H

#include "common/sharedStructures.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetMSMediaFoundation.h"

class DockWidgetMSMediaFoundation : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetMSMediaFoundation(QMap<QString, ito::Param> params, int uniqueID);
        ~DockWidgetMSMediaFoundation() {};

    private:
        Ui::DockWidgetMSMediaFoundation ui;

    signals:
        void GainOffsetPropertiesChanged(double gain, double offset);
        void IntegrationPropertiesChanged(double integrationtime);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);


    private slots:
        void on_spinBox_offset_valueChanged(int d);
        void on_spinBox_gain_valueChanged(int d);
        void on_doubleSpinBox_integration_time_valueChanged(double d);
};

#endif
