///* ********************************************************************
//    Plugin "ThorlabsKCubeDCServo" for itom software
//    URL: http://www.uni-stuttgart.de/ito
//    Copyright (C) 2021, Institut fuer Technische Optik (ITO),
//    Universitaet Stuttgart, Germany
//
//    This file is part of a plugin for the measurement software itom.
//
//    This itom-plugin is free software; you can redistribute it and/or modify it
//    under the terms of the GNU Library General Public Licence as published by
//    the Free Software Foundation; either version 2 of the Licence, or (at
//    your option) any later version.
//
//    itom and its plugins are distributed in the hope that it will be useful, but
//    WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
//    General Public Licence for more details.
//
//    You should have received a copy of the GNU Library General Public License
//    along with itom. If not, see <http://www.gnu.org/licenses/>.
//*********************************************************************** */

#pragma once

#include "common/abstractAddInConfigDialog.h"
#include "common/addInInterface.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "ui_dialogThorlabsKCubeDCServo.h"

#include <qabstractbutton.h>
#include <qdialog.h>
#include <qmap.h>
#include <qsharedpointer.h>
#include <qstring.h>

namespace ito {
class AddInBase; // forward declaration
}

class DialogThorlabsKCubeDCServo : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

public:
    DialogThorlabsKCubeDCServo(ito::AddInActuator* actuator);
    ~DialogThorlabsKCubeDCServo();

    ito::RetVal applyParameters();

private:
    void enableDialog(bool enabled);

    Ui::DialogThorlabsKCubeDCServo ui;
    bool m_firstRun;
    ito::AddInActuator* m_pActuator;

public slots:
    void parametersChanged(QMap<QString, ito::Param> params);

private slots:
    void on_buttonBox_clicked(QAbstractButton* btn);
    void on_btnSetOrigin_clicked();
    void on_btnHome_clicked();
    void on_btnHomeCancel_clicked();
};
