/* ********************************************************************
    Plugin "MsMediaFoundation" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

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

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetMSMediaFoundation.h"

class DockWidgetMSMediaFoundation : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetMSMediaFoundation(ito::AddInDataIO *grabber);
        ~DockWidgetMSMediaFoundation() {};

    private:
        Ui::DockWidgetMSMediaFoundation ui;
        bool m_inEditing;
        bool m_firstRun;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:
        void on_sW_Brightness_valueChanged(double d);
        void on_sW_Contrast_valueChanged(double d);
        void on_sW_Gain_valueChanged(double d);
        void on_sW_Saturation_valueChanged(double d);
        void on_sW_Sharpness_valueChanged(double d);
        void on_combo_IntegrationTime_currentIndexChanged(int index);

        void on_cB_Brightness_toggled(bool checked);
        void on_cB_Contrast_toggled(bool checked);
        void on_cB_Gain_toggled(bool checked);
        void on_cB_Saturation_toggled(bool checked);
        void on_cB_Sharpness_toggled(bool checked);
        void on_cB_IntegrationTime_toggled(bool checked);
};

#endif
