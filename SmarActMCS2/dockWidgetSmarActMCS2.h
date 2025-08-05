/* ********************************************************************
    Plugin "SmarActMCS2" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2025, TRUMPF Lasersystems for Semiconductor Manufacturing SE,´Germany

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

#ifndef DOCKWIDGETSMARACTMCS2_H
#define DOCKWIDGETSMARACTMCS2_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetSmarActMCS2.h"

class DockWidgetSmarActMCS2 : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetSmarActMCS2(ito::AddInActuator *actuator);
        ~DockWidgetSmarActMCS2() {};

    private:
        Ui::DockWidgetSmarActMCS2 ui; //! Handle to the ui
        bool m_inEditing;
        bool m_firstRun;

        void enableWidgets(bool enabled);

        ito::AddInActuator* m_pActuator;


    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString& identifier);
        void dockWidgetVisibilityChanged(bool visible);
};

#endif
