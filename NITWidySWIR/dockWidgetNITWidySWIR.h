/* ********************************************************************
Plugin "NITWidySWIR" for itom software
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

#ifndef DOCKWIDGETNITWIDYSWIR_H
#define DOCKWIDGETNITWIDYSWIR_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetNITWidySWIR.h"

class dockWidgetNITWidySWIR : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        dockWidgetNITWidySWIR(ito::AddInDataIO *grabber);
        ~dockWidgetNITWidySWIR() {};

    private:
        Ui::dockWidgetNITWidySWIR ui;
        bool m_inEditing;
        bool m_firstRun;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:
        //add here slots connected to changes of any widget
        //example:
        //void on_contrast_valueChanged(int i);
        void on_sliderWidget_offset_valueChanged(double value);
        void on_sliderWidget_gain_valueChanged(double value);
        void on_sliderWidget_integrationTime_valueChanged(double value);

};

#endif
