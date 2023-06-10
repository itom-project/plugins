/* ********************************************************************
itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2018, Institut fuer Technische Optik (ITO),,
Universität Stuttgart, Germany

This file is part of itom and its software development toolkit (SDK).

itom is free software; you can redistribute it and/or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or (at
your option) any later version.

In addition, as a special exception, the Institut für Technische
Optik (ITO) gives you certain additional rights.
These rights are described in the ITO LGPL Exception version 1.0,
which can be found in the file LGPL_EXCEPTION.txt in this package.

itom is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef DOCKWIDGETMYGRABBER_H
#define DOCKWIDGETMYGRABBER_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>
#include <qtimer.h>
#include "DataObject/dataobj.h"
#include "ui_dockWidgetThorlabsPowerMeter.h"


class DockWidgetThorlabsPowerMeter : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetThorlabsPowerMeter(ito::AddInDataIO *grabber);
        ~DockWidgetThorlabsPowerMeter() {};

    private:
        Ui::ThorlabsPowerMeter ui;
        bool m_inEditing;
        bool m_firstRun;
        ito::AddInDataIO *m_plugin;
        int m_timerId;
        bool m_timerIsRunning;
        void calculateUnit(const ito::float64 &val, QPair<double, QString> &result);

    protected:
        void timerEvent(QTimerEvent *event);



    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);
        void manageTimer(const bool &val);

    private slots:
        void on_dspinWavelength_valueChanged(double val);
        void on_spinAverage_valueChanged(int val);
        void on_dspinAttenuation_valueChanged(double val);
        void on_spinLineFrequency_valueChanged(int val);
        void on_checkBoxAutoRange_stateChanged(int val);
        void on_checkAutograbbing_stateChanged(int val);
        void on_sliderPowerRange_valueChanged(double val);
        void on_comboBandwidth_currentIndexChanged(int val);
        void on_btnZero_clicked();

        //add here slots connected to changes of any widget
        //example:
        //void on_contrast_valueChanged(int i);
};

#endif
