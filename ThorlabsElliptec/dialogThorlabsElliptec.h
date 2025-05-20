/* ********************************************************************
    Plugin "ThorlabsElliptec" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2025, Institut für Technische Optik (ITO),
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

#pragma once

#include "common/abstractAddInConfigDialog.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "common/addInInterface.h"
#include "ui_dialogThorlabsElliptec.h"

#include <qabstractbutton.h>
#include <qmap.h>
#include <qstring.h>

namespace ito {
class AddInBase; // forward declaration
}

class DialogThorlabsElliptec : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

public:
    DialogThorlabsElliptec(ito::AddInActuator* actuator, bool allowCleaning, bool allowOptimization);
    ~DialogThorlabsElliptec();

    ito::RetVal applyParameters();

protected:
    ito::RetVal observeInvocation(ItomSharedSemaphore* waitCond) const;

private:
    void enableDialog(bool enabled);
    bool m_firstRun;

    QPointer<ito::AddInBase> m_pluginPointer;
    Ui::DialogThorlabsElliptec ui;

public slots:
    void parametersChanged(QMap<QString, ito::Param> params);

private slots:
    void on_buttonBox_clicked(QAbstractButton* btn);
    void on_cmdOptimizeMotors_clicked();
    void on_cmdCleanMechanics_clicked();
    void on_cmdCancelCleaning_clicked();
    void on_cmdSaverUserData_clicked();
    void on_cmdHome_clicked();
    void on_btnResetDefaults_clicked();
    void on_btnSearch2_clicked();
    void on_btnSearch1_clicked();
};
