/* ********************************************************************
    Plugin "cmu1394" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#ifndef DOCKWIDGETCMU1394_H
#define DOCKWIDGETCMU1394_H

#include "common/sharedStructures.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetCMU1394.h"

class DockWidgetCMU1394 : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetCMU1394(QMap<QString, ito::Param> params, int uniqueID);
        ~DockWidgetCMU1394() {};

    private:
        Ui::DockWidgetCMU1394 ui;
        char updating;

    signals:
//        void changeParameters(QMap<QString, ito::ParamBase> params);
        void OffsetPropertiesChanged(double offset);
        void GainPropertiesChanged(double gain);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);


    private slots:
        void on_doubleSpinBoxGain_editingFinished();
        void on_doubleSpinBoxOffset_editingFinished();
        void on_horizontalSliderGain_valueChanged(int value);
        void on_horizontalSliderOffset_valueChanged(int value);
};

#endif
