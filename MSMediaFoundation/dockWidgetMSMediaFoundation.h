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
#include "common/sharedStructuresQt.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetMSMediaFoundation.h"
#include "common/addInInterface.h"

class DockWidgetMSMediaFoundation : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetMSMediaFoundation(ito::AddInDataIO *grabber);
        ~DockWidgetMSMediaFoundation() {};

    private:
        Ui::DockWidgetMSMediaFoundation ui;
        bool m_inEditing;
        bool m_firstRun;
        ito::AddInDataIO *m_pMSMediaFoundation;

    signals:
//        void dockWidgetValueChanged(int type, double value);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);
        void propertiesChanged(QString identifier);

    private slots:
        void on_sW_Brightness_valueChanged(double d);
        void on_sW_Contrast_valueChanged(double d);
        void on_sW_Gain_valueChanged(double d);
        void on_sW_Saturation_valueChanged(double d);
        void on_sW_Sharpness_valueChanged(double d);

        void on_cB_Brightness_stateChanged(int state);
        void on_cB_Contrast_stateChanged(int state);
        void on_cB_Gain_stateChanged(int state);
        void on_cB_Saturation_stateChanged(int state);
        void on_cB_Sharpness_stateChanged(int state);

        void sendParameters(const int type, const double d);
};

#endif
