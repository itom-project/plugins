/* ********************************************************************
    Plugin "V4L2" for itom software
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

#ifndef DOCKWIDGETV4L2_H
#define DOCKWIDGETV4L2_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"
#include "itomWidgets/sliderWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>
#include <QCheckBox>

#include "ui_dockWidgetV4L2.h"

#include "v4l2_itom_api.h"

class DockWidgetV4L2 : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetV4L2(ito::AddInDataIO *grabber);
        ~DockWidgetV4L2() {};

    private:
        Ui::DockWidgetV4L2 ui;
        bool m_inEditing;
        bool m_firstRun;
        Device* m_device;
        QMap<QString, SliderWidget*> m_sliders;
        QMap<QString, QCheckBox*> m_checkbox;
        void setup_widget(const QMap<QString, ito::Param> &params);

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);
        void initialize(Device* device);

    private slots:
        void on_slider_valueChanged(double d);
        void on_cB_toggled(bool checked);

};

#endif
