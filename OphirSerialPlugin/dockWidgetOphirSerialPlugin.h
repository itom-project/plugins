/* ********************************************************************
    Plugin "OphirSerialPlugin" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2020, Institut fuer Technische Optik (ITO),
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

#ifndef DOCKWIDGETOPHIRSERIALPLUGIN_H
#define DOCKWIDGETOPHIRSERIALPLUGIN_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/param.h"
#include "common/retVal.h"

#include <qmap.h>
#include <qstring.h>
#include <qabstractbutton.h>

#include "ui_dockWidgetOphirSerialPlugin.h"

#include <QtGui>
#include <qwidget.h>
#include <qsharedpointer.h>
#include <qmetaobject.h>
#include <qstackedwidget.h>

class DockWidgetOphirSerialPlugin : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetOphirSerialPlugin(int uniqueID, ito::AddInBase *actuator);
        ~DockWidgetOphirSerialPlugin() {};

    private:
        void enableWidget(bool enabled);

        Ui::dockWidgetOphirSerialPlugin ui;

        bool m_inEditing;
        bool m_firstRun;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:

};

#endif
