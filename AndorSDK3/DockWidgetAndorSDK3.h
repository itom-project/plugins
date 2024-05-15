/* ********************************************************************
    Plugin "AndorSDK3" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Institut für Technische Optik, Universität Stuttgart

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

#ifndef DOCKWIDGETANDORSDK3_H
#define DOCKWIDGETANDORSDK3_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_DockWidgetAndorSDK3.h"

class DockWidgetSDK3 : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetSDK3(ito::AddInDataIO *grabber);
        ~DockWidgetSDK3() {};

    private:
        Ui::DockWidgetAndorSDK3 ui;
        bool m_inEditing;
        bool m_firstRun;
        QMap<QString, ito::Param> m_currentParams;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier) {};

    private slots:
        void on_sliderExposure_valueChanged(double value);
        void on_sliderGain_valueChanged(double value);
};

#endif
