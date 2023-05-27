/* ********************************************************************
    Plugin "IDSuEye" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Pulsar Photonics GmbH, Aachen
    Copyright (C) 2017, Institut fuer Technische Optik, Universitaet Stuttgart

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

#ifndef DOCKWIDGETIDSUEYE_H
#define DOCKWIDGETIDSUEYE_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetIDS.h"
#include <qabstractbutton.h>
#include <QtGui>
#include <qwidget.h>
#include <qsharedpointer.h>
#include <qmetaobject.h>

class DockWidgetIDS : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetIDS(ito::AddInDataIO *grabber);
        ~DockWidgetIDS() {};

    private:
        Ui::DockWidgetIDS ui;
        bool m_inEditing;
        bool m_firstRun;
        QMap<QString, ito::Param> m_currentParams;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier) {};

    private slots:
        void on_sliderExposure_valueChanged(double value);
        void on_sliderGain_valueChanged(double value);
        void on_sliderGainRed_valueChanged(double value);
        void on_sliderGainGreen_valueChanged(double value);
        void on_sliderGainBlue_valueChanged(double value);
        void on_sliderOffset_valueChanged(double value);
		void on_comboExposureUnit_currentIndexChanged(int index);
};

#endif
