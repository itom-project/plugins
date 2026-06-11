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

#ifndef DIALOGSMARACTMCS2_H
#define DIALOGSMARACTMCS2_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "common/addInInterface.h"
#include "ui_dialogSmarActMCS2.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogSmarActMCS2 : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
    DialogSmarActMCS2(ito::AddInActuator* actuator);
        ~DialogSmarActMCS2() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;

        QPointer<ito::AddInBase> m_pluginPointer;
        Ui::DialogSmarActMCS2 ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
};

#endif
