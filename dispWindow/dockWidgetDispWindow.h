/* ********************************************************************
    Plugin "dispWindow" for itom software
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

#ifndef DOCKWIDGETDISPWINDOW_H
#define DOCKWIDGETDISPWINDOW_H

#include "projWindow.h"
//#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetDispWindow.h"

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

namespace ito {
    class AddInActuator;
}

class DispWindow; //forward declaration

class DockWidgetDispWindow : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetDispWindow(const QString &identifier, PrjWindow *prjWindow, DispWindow *dispWindow);
        ~DockWidgetDispWindow() {}
        PrjWindow *m_pPrjWindow;
        DispWindow *m_pDispWindow;
        //QMap<QString, ito::Param> *m_pParams;

    private:
        Ui::DockWidgetDispWindow ui;

        int m_curNumPhaseShifts;
        int m_curNumGrayCodes;
        bool m_numimgChangeInProgress;

    signals:

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);
        
    private slots:
        void on_comboBox_currentIndexChanged(int index);
};

#endif
