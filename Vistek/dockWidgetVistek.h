/* ********************************************************************
    Plugin "Vistek" for itom software
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

#ifndef DOCKWIDGETVISTEK_H
#define DOCKWIDGETVISTEK_H

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetVistek.h"
#include "common/sharedStructures.h"

#include "Vistek.h"

class DockWidgetVistek : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetVistek();
        ~DockWidgetVistek() {};

    private:
        Ui::DockWidgetVistek ui;
        bool m_inEditing;
        float m_exposureStep;

    signals:
        void GainPropertyChanged(double gain);
        void OffsetPropertyChanged(double gain);
        void ExposurePropertyChanged(double gain);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);
        void propertiesChanged(float gainIncrement, float exposureIncrement, VistekFeatures features);

    private slots:
        void on_exposureSpinBox_valueChanged(double val);
        void on_gainSpinBox_valueChanged(double val);
        void on_offsetSpinBox_valueChanged(double val);
};

#endif
