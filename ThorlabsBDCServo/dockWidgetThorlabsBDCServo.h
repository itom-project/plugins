/* ********************************************************************
    Plugin "ThorlabsBDCServo" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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

#ifndef DOCKWIDGETFIRGELLILAC_H
#define DOCKWIDGETFIRGELLILAC_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetThorlabsBDCServo.h"

class DockWidgetThorlabsBDCServo : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

public:
    DockWidgetThorlabsBDCServo(ito::AddInActuator* myPlugin);
    ~DockWidgetThorlabsBDCServo(){};

private:
    void enableWidget(bool enabled);
    bool m_firstRun;
    ito::AddInActuator* m_pActuator;

    Ui::DockWidgetThorlabsBDCServo ui;

public slots:
    void parametersChanged(QMap<QString, ito::Param> params);
    void identifierChanged(const QString& identifier){};
    void dockWidgetVisibilityChanged(bool visible);
};

#endif
