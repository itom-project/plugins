/* ********************************************************************
    Plugin "HidApi" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#ifndef DOCKWIDGETHIDAPI_H
#define DOCKWIDGETHIDAPI_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "ui_dockWidgetHidApi.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>
#include <qbytearray.h>

class DockWidgetHidApi : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetHidApi(QMap<QString, ito::Param> params, int uniqueID);
        ~DockWidgetHidApi() {};

    private:
        Ui::DockWidgetHidApi ui;

    signals:

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);
        void serialLog(QByteArray data, const char InOutChar);

    private slots:
        void on_ClrButton_clicked();
};

#endif
