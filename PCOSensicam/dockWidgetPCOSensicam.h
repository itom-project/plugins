/* ********************************************************************
    Plugin "PCOSensicam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut für Technische Optik (ITO),
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

#ifndef DOCKWIDGETPCOSENSICAM_H
#define DOCKWIDGETPCOSENSICAM_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_DockWidgetPCOSensicam.h"

class DockWidgetPCOSensicam : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetPCOSensicam(ito::AddInDataIO *grabber);
        ~DockWidgetPCOSensicam() {};

    private:
        Ui::DockWidgetPCOSensicam ui;
        bool m_inEditing;
        bool m_firstRun;
        double exposureToSecFactor;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:
        void on_checkFastMode_toggled(bool checked);
        void on_comboGainMode_currentIndexChanged(int index);
        void on_slider_exposure_valueChanged(double d);
        void on_slider_delay_valueChanged(double d);
};

#endif //DOCKWIDGETPCOSENSICAM_H
