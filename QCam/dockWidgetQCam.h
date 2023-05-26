/* ********************************************************************
    Plugin "QCam" for itom software
    URL: ???

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

#ifndef DOCKWIDGETQCAM_H
#define DOCKWIDGETQCAM_H

#include "common/sharedStructures.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetQCam.h"

class DockWidgetQCam : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetQCam();
        ~DockWidgetQCam() {};

    private:
        Ui::DockWidgetQCam ui;

    signals:
        void changeParameters(QMap<QString, ito::ParamBase> params);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);


    private slots:
        void on_doubleSpinBoxGain_editingFinished();
        void on_doubleSpinBoxOffset_editingFinished();
        void on_horizontalSliderGain_valueChanged(int value);
        void on_horizontalSliderOffset_valueChanged(int value);
};

#endif
