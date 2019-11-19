/* ********************************************************************
Plugin "IntelRealSense" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2019, Institut fuer Technische Optik (ITO),
Universitaet Stuttgart, Germany

This file is part of a plugin for the measurement software itom.

This itom-plugin is free software; you can redistribute it and/or modify it
under the terms of the Apache Licence as published INTEL.

itom and its plugins are distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef DIALOGINTELREALSENSE_H
#define DIALOGINTELREALSENSE_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogIntelRealSense.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogIntelRealSense : public ito::AbstractAddInConfigDialog 
{
    Q_OBJECT

    public:
        DialogIntelRealSense(ito::AddInBase *grabber);
        ~DialogIntelRealSense() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;

        Ui::DialogIntelRealSense ui;
		//QPointer<ito::AddInBase> m_pluginPointer(grabber);

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
};

#endif
